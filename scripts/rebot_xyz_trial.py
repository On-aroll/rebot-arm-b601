"""Explicit nearby XYZ trial; reuses the existing IK, trajectory and Seeed SDK.

Read/plan never enable. Execute is a foreground, one-use onsite operation.
J7 is excluded: opening the gripper is unrelated to moving the TCP.
"""
import argparse
import copy
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import time

from rebot_mit_hardware import Envelope, HardwareExecutor, MotionIntent, OnsitePermit
from rebot_mit_mock import Trajectory, TrajectoryBudgetError

AXES=tuple(range(1,7))


def hardware_motion_options(raw):
    config=Envelope.parse(raw)
    return dict(speed_min_rad_s=.005,speed_max_rad_s=.15,default_speed_rad_s=min(a.v_max for a in config.axes),
                timeout_min_s=5.,timeout_max_s=120.,default_timeout_s=config.policy.task_timeout,
                automatic_margin_s=5.,default_timeout_mode='auto')


def configure_motion(raw, start, target, settings):
    """Build an in-memory task envelope from visible operator-selected timing.

    Device parameters, saved gains and the original proposal are untouched.
    Feedback, command and lease deadlines do not follow the motion duration.
    """
    from rebot_mit_mock import finite
    options=hardware_motion_options(raw)
    if not isinstance(settings,dict) or not {'speed_rad_s','timeout_mode'}<=set(settings) or set(settings)-{'speed_rad_s','timeout_mode','timeout_s','feedforward_mode'}:
        raise ValueError('请在运动设置中明确选择实机关节速度和任务时限')
    speed=settings['speed_rad_s'];mode=settings['timeout_mode']
    if not finite(speed) or not options['speed_min_rad_s']<=speed<=options['speed_max_rad_s']:
        raise ValueError('实机关节速度必须在 0.005–0.15 rad/s 之间')
    if mode not in ('auto','manual'):
        raise ValueError('任务时限请选择按轨迹自动计算或手动设置')
    task_raw=copy.deepcopy(raw)
    ff=settings.get('feedforward_mode','none')
    from rebot_native_xyz import PROFILES
    if ff in PROFILES:
        from rebot_native_xyz import proposal,validate_target,profile
        _,seconds=profile(ff)
        if speed!=.02 or mode=='manual' and settings.get('timeout_s')!=seconds:
            raise ValueError(f'本次原生 XYZ 方案固定为 0.02 rad/s、{seconds:g} 秒；请选择此速度和自动时限')
        task_raw,_=proposal([(a['q_min'],a['q_max']) for a in raw['axes']],mode=ff)
        config=Envelope.parse(task_raw);validate_target(start,target,config)
        return task_raw,config
    if ff not in ('none','seeed-rs-gravity-v1','seeed-rs-slow-position-v1'):
        raise ValueError('请选择无前馈或 Seeed 官方重力前馈')
    if ff!='none':
        from rebot_official_motion import policy
        task_raw['feedforward']=policy()
        if ff=='seeed-rs-slow-position-v1':
            from rebot_official_motion import apply_slow_gains
            apply_slow_gains(task_raw)
    else: task_raw.pop('feedforward',None)
    for axis in task_raw['axes']: axis['v_max']=speed
    if mode=='manual':
        timeout=settings.get('timeout_s')
        if not finite(timeout) or not options['timeout_min_s']<=timeout<=options['timeout_max_s']:
            raise ValueError('手动任务时限必须在 5–120 秒之间')
    else:
        task_raw['policy']['task_timeout']=options['timeout_max_s']
        try:
            trajectory=Trajectory.create([float(v) for v in start],[[float(v) for v in target]],Envelope.parse(task_raw),
                                         start_tolerance=task_raw.get('start_boundary_tolerance',0.))
            timeout=math.ceil(trajectory.duration+options['automatic_margin_s'])
            if timeout>options['timeout_max_s']:
                raise ValueError(f'自动任务时限需 {timeout} 秒（轨迹 {trajectory.duration:.1f} 秒 + 5 秒余量），'
                                 '超过当前 120 秒上限；请缩小位移或调整实机速度后重新预演')
        except TrajectoryBudgetError:
            # Keep the maximum and let the pure preflight report required time.
            timeout=options['timeout_max_s']
    task_raw['policy']['task_timeout']=timeout
    return task_raw,Envelope.parse(task_raw)


def read_start(config, bus):
    readings=[]
    checker=HardwareExecutor(config,bus,clock=bus.clock)
    try:
        bus.open_readonly()
        for _ in range(3):
            samples=bus.read(AXES,config.policy)
            readings.append({str(i):asdict(s) for i,s in samples.items()})
            checker.validate_feedback(samples,AXES)
            if any(abs(s.qd)>config.policy.start_speed for s in samples.values()):
                raise ValueError('六轴未停稳；不自动放宽速度阈值')
            if any(abs(s.q-readings[0][str(i)]['q'])>config.policy.arrival_q for i,s in samples.items()):
                raise ValueError('只读准备期间姿态变化，需重新准备')
        return dict(ok=True,expected_start=[samples[i].q for i in AXES],readings=readings,
                    config_hash=config.fingerprint,read_at=time.time(),hardware_writes=0,
                    velocity_source='host position differences; raw mechVel preserved for effort checks')
    except Exception as e:
        return dict(ok=False,reason=str(e),readings=readings,hardware_writes=0)
    finally:
        bus.close()


def plan_nearby(robot, start, config):
    import numpy as np
    q=np.asarray(start,dtype=float)
    if q.shape!=(6,) or not np.all(np.isfinite(q)):
        raise ValueError('需要六轴有限实测起点')
    margin=config.start_boundary_tolerance
    if np.any(q<robot.lower-margin) or np.any(q>robot.upper+margin):
        raise ValueError('实测起点越限；未裁剪或修改零位')
    if robot.contacts(q): raise ValueError('实测起点在模型中有碰撞')
    xyz=robot.fk(q).translation
    candidates=[]; rejected=[]
    for axis in range(3):
        for sign in (-1,1):
            goal=xyz.copy(); goal[axis]+=sign*.005
            try:
                target=robot.ik(goal,seed=q,position_only=True)
                delta=target-q
                if np.any(np.abs(delta)>=math.radians(60)):
                    raise ValueError('存在关节变化达到 60°')
                trajectory=Trajectory.create(q.tolist(),[target.tolist()],config,start_tolerance=margin)
                if trajectory.duration+config.policy.settle_time>config.policy.task_timeout:
                    raise ValueError('不能在本轮试验时限内完成')
                robot.check_path(q,target)
                candidates.append(dict(goal_xyz_m=goal.tolist(),target=target.tolist(),
                    delta_deg=np.rad2deg(delta).tolist(),joint_distance_rad=float(np.linalg.norm(delta)),
                    duration_s=trajectory.duration,position_error_mm=float(np.linalg.norm(robot.fk(target).translation-goal)*1000)))
            except ValueError as e:
                rejected.append(dict(axis=axis,sign=sign,reason=str(e)))
    if not candidates: raise ValueError('未找到 5 mm 邻近、限位/碰撞/试验时序均通过的候选点')
    selected=min(candidates,key=lambda item:item['joint_distance_rad'])
    assessment=assess_motion(robot,q,selected['target'],config)
    return dict(ok=True,expected_start=q.tolist(),start_xyz_m=xyz.tolist(),**selected,
                controlled_axes=list(AXES),gripper_commanded=False,orientation_mode='position',
                selection='smallest joint distance among six seeded 5 mm axis candidates; not a global optimum',
                candidate_count=len(candidates),rejected=rejected,config_hash=config.fingerprint,
                start_boundary_tolerance_deg=math.degrees(margin),
                boundary_recovery_axes=[i+1 for i in range(6) if not robot.lower[i]<=q[i]<=robot.upper[i]],
                **assessment,planned_at=time.time(),hardware_writes=0)


def assess_motion(robot, start, target, config):
    """Pure preflight shared by the terminal trial and the webpage. No CAN."""
    from rebot_native_xyz import PROFILES
    if config.version in PROFILES:
        from rebot_native_xyz import assess_motion as assess_native
        return assess_native(robot,start,target,config)
    import numpy as np
    q=np.asarray(start,dtype=float)
    target=np.asarray(target,dtype=float)
    if q.shape!=(6,) or target.shape!=(6,) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(target)):
        raise ValueError('需要完整的六轴实测起点与目标')
    if np.any(np.abs(target-q)>=math.radians(60)):
        raise ValueError('本轮每轴变化必须小于 60°')
    robot.check_path(q,target)
    # Reuse the official pure helper with the project's explicit RS model.
    # Do not call GravityCompensation.start(): it switches modes and enables.
    from reBotArm_control_py.dynamics import compute_generalized_gravity
    data=robot.pm.createData()
    try:
        trajectory=Trajectory.create(q.tolist(),[target.tolist()],config,start_tolerance=config.start_boundary_tolerance)
    except TrajectoryBudgetError as error:
        reason=(f'按当前 {config.axes[0].v_max:g} rad/s 实机试验速度，轨迹至少需 {error.duration:.1f} 秒，'
                f'另需预留 {error.reserve:g} 秒；本次试验时限只有 {error.budget:g} 秒。'
                '请缩小目标位移，或另行确认更长的试验时限。尚未进入出力检查，未启动电机。')
        return dict(execution_ready=False,execution_reasons=[reason],hardware_seconds=error.duration,
                    task_timeout_s=error.budget,
                    gravity_check=dict(status='not-evaluated',reason='duration exceeds trial budget'),
                    static_balance_check=dict(status='not-evaluated',reason='duration exceeds trial budget'))
    gravity_samples=np.asarray([
        compute_generalized_gravity(robot.pm,np.asarray(trajectory.sample(t)[0]),data)[:6]
        for t in np.linspace(0,trajectory.duration,51)
    ])
    if not np.all(np.isfinite(gravity_samples)):
        raise ValueError('官方 RS 重力估算出现非有限值；不生成实机方案')
    peaks=np.max(np.abs(gravity_samples),axis=0)
    gravity_check=dict(source='Seeed compute_generalized_gravity with explicit locked RS URDF; estimate only',
        estimated_Nm=gravity_samples[0].tolist(),sampled_peak_Nm=peaks.tolist(),sample_count=51,
        software_effort_limits_Nm=[a.torque_max for a in config.axes],
        insufficient_axes=[i+1 for i,a in enumerate(config.axes) if peaks[i]>a.torque_max],
        continuous_external_support_verified=False)
    # Raising an abort threshold does not produce holding torque. Check whether
    # the actual configured PD/feedforward could balance the model within the
    # existing tracking-error bound. Friction/payload still require site testing.
    feedforward_samples=np.tile([a.tau_ff for a in config.axes],(len(gravity_samples),1))
    if config.feedforward is not None:
        from rebot_official_motion import OfficialMIT
        official=OfficialMIT(config,model=robot.pm)
        feedforward_samples=np.asarray([np.asarray(official.commands(trajectory.sample(t)[0],trajectory.sample(t)[1],trajectory.sample(t)[0]))[:,4]
            for t in np.linspace(0,trajectory.duration,51)])
    residuals=np.max(np.abs(gravity_samples-feedforward_samples),axis=0)
    required_offsets=np.asarray([residuals[i]/a.kp if a.kp>0 else math.inf for i,a in enumerate(config.axes)])
    static_balance_check=dict(source='gravity residual divided by configured MIT Kp; not physical acceptance',
        required_offset_deg=np.rad2deg(required_offsets).tolist(),
        allowed_error_deg=[math.degrees(a.error_max) for a in config.axes],
        insufficient_axes=[i+1 for i,a in enumerate(config.axes) if required_offsets[i]>a.error_max],
        payload_and_friction_validated=False)
    static_balance_check['nominal_model_only']=True
    static_balance_check['feedforward_mode']='none' if config.feedforward is None else config.feedforward.mode
    estimated_effort=peaks+np.asarray([a.kp*a.error_max+a.kd*a.v_max for a in config.axes])
    reserve_check=dict(estimated_peak_with_tracking_reserve_Nm=estimated_effort.tolist(),
        insufficient_axes=[i+1 for i,a in enumerate(config.axes) if estimated_effort[i]>a.torque_max] if config.feedforward else [])
    reasons=[]
    if trajectory.duration+config.policy.settle_time>config.policy.task_timeout:
        reasons.append(f'按已确认的 {config.axes[0].v_max:g} rad/s 试验速度需 {trajectory.duration:.1f} 秒，超过本轮 {config.policy.task_timeout:g} 秒时限；请选择更近的目标')
    if gravity_check['insufficient_axes']:
        reasons.append('J'+ '/J'.join(map(str,gravity_check['insufficient_axes']))+' 模型重力估算超过当前软件力矩包络')
    if static_balance_check['insufficient_axes']:
        reasons.append('J'+ '/J'.join(map(str,static_balance_check['insufficient_axes']))+' 的本次增益与前馈在当前跟踪误差边界内不足以平衡模型重力；未自动改增益或前馈')
    if reserve_check['insufficient_axes']:
        reasons.append('J'+ '/J'.join(map(str,reserve_check['insufficient_axes']))+' 官方重力前馈加跟踪余量超过本次软件力矩包络')
    return dict(gravity_check=gravity_check,static_balance_check=static_balance_check,
                effort_reserve_check=reserve_check,
                control_source='Seeed RebotArmEndPose._loop_cb: measured q → g(q) → MIT q/dq/kp/kd/tau' if config.feedforward else 'existing zero-feedforward trial',
                execution_ready=not reasons,execution_reasons=reasons,hardware_seconds=trajectory.duration)


def execute(config, plan, evidence):
    from rebot_seeed_transport import SeeedTransport
    if plan.get('ok') is not True or plan.get('controlled_axes')!=list(AXES) or plan.get('hardware_writes')!=0:
        raise ValueError('需要通过的六轴只读起点与邻近 XYZ 预演')
    if plan.get('config_hash')!=config.fingerprint or not 0<=time.time()-plan.get('planned_at',0)<60:
        raise ValueError('预演已过期或配置变化，请重新准备')
    if (plan.get('execution_ready') is not True or plan.get('gravity_check',{}).get('insufficient_axes')!=[]
            or plan.get('static_balance_check',{}).get('insufficient_axes')!=[]):
        raise ValueError('承重控制尚未验证；放宽软件力矩上限不等于已有足够的 PD/前馈力矩，不使能六轴')
    start,target=plan['expected_start'],plan['target']
    if len(start)!=6 or len(target)!=6 or any(abs(a-b)>=math.radians(60) for a,b in zip(start,target)):
        raise ValueError('本轮必须六轴且每轴变化小于 60°')
    permit=OnsitePermit(config.fingerprint,AXES,time.monotonic()+config.policy.task_timeout+10,evidence,AXES,True,True,
                        coordinated_start_checked=True)
    executor=HardwareExecutor(config,SeeedTransport(position_window=True))
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_:executor.cancel())
    executor.heartbeat(executor.session,0,True)
    executor.start(MotionIntent([target],'xyz',f'onsite-xyz-{time.time_ns()}',expected_start=start),permit)
    sequence=0
    try:
        while executor.worker.is_alive():
            sequence+=1; executor.heartbeat(executor.session,sequence,True)
            executor.worker.join(.1)
    finally:
        executor.cancel(); executor.worker.join()
    result=executor.status()
    result.update(backend='seeed-sdk-p4-guarded',plan=plan,command_log=executor.frames,
                  parameter_source='explicit task envelope and pinned official/reference sources; not a device parameter backup')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal',type=Path,required=True)
    actions=parser.add_mutually_exclusive_group(required=True)
    actions.add_argument('--read',action='store_true')
    actions.add_argument('--plan',action='store_true')
    actions.add_argument('--execute',action='store_true')
    parser.add_argument('--input',type=Path)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--onsite-evidence')
    for flag in ('operator-present','power-cut-ready','supported','stop-tested'):
        parser.add_argument('--'+flag,action='store_true')
    args=parser.parse_args()
    raw=json.loads(args.proposal.read_text())
    if raw.get('operation')!='nearby-xyz-five-millimeters': raise ValueError('需要明确的本轮邻近 XYZ 方案')
    config=Envelope.parse(raw['envelope'])
    if args.read:
        from rebot_seeed_transport import SeeedTransport
        result=read_start(config,SeeedTransport(position_window=True))
    elif args.plan:
        from rebot_core import Robot
        read=json.loads(args.input.read_text())
        if read.get('ok') is not True or read.get('config_hash')!=config.fingerprint or not 0<=time.time()-read.get('read_at',0)<60:
            raise ValueError('需要本轮未过期的六轴只读起点')
        result=plan_nearby(Robot(),read['expected_start'],config)
        result['readings']=read['readings']
    else:
        if not all((args.input,args.onsite_evidence and args.onsite_evidence.strip(),args.operator_present,args.power_cut_ready,args.supported,args.stop_tested)):
            parser.error('执行需要当次人员/支撑/独立断电/失能确认；默认不执行')
        result=execute(config,json.loads(args.input.read_text()),args.onsite_evidence)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('readings','command_log','plan')},ensure_ascii=False))
    return int(result.get('ok') is False or result.get('state') in ('FAULT','CANCELLED'))


if __name__=='__main__':
    raise SystemExit(main())
