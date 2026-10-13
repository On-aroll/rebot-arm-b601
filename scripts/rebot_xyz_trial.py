"""Explicit nearby XYZ trial; reuses the existing IK, trajectory and Seeed SDK.

Read/plan never enable. Execute is a foreground, one-use onsite operation.
J7 is excluded: opening the gripper is unrelated to moving the TCP.
"""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import signal
import time

from rebot_mit_hardware import Envelope, HardwareExecutor, MotionIntent, OnsitePermit
from rebot_mit_mock import Trajectory

AXES=tuple(range(1,7))


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
    # Reuse the official pure helper with the project's explicit RS model.
    # Do not call GravityCompensation.start(): it switches modes and enables.
    from reBotArm_control_py.dynamics import compute_generalized_gravity
    data=robot.pm.createData()
    trajectory=Trajectory.create(q.tolist(),[selected['target']],config,start_tolerance=margin)
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
    return dict(ok=True,expected_start=q.tolist(),start_xyz_m=xyz.tolist(),**selected,
                controlled_axes=list(AXES),gripper_commanded=False,orientation_mode='position',
                selection='smallest joint distance among six seeded 5 mm axis candidates; not a global optimum',
                candidate_count=len(candidates),rejected=rejected,config_hash=config.fingerprint,
                start_boundary_tolerance_deg=math.degrees(margin),
                boundary_recovery_axes=[i+1 for i in range(6) if not robot.lower[i]<=q[i]<=robot.upper[i]],
                gravity_check=gravity_check,execution_ready=not gravity_check['insufficient_axes'],
                planned_at=time.time(),hardware_writes=0)


def execute(config, plan, evidence):
    from rebot_seeed_transport import SeeedTransport
    if plan.get('ok') is not True or plan.get('controlled_axes')!=list(AXES) or plan.get('hardware_writes')!=0:
        raise ValueError('需要通过的六轴只读起点与邻近 XYZ 预演')
    if plan.get('config_hash')!=config.fingerprint or not 0<=time.time()-plan.get('planned_at',0)<60:
        raise ValueError('预演已过期或配置变化，请重新准备')
    if plan.get('execution_ready') is not True or plan.get('gravity_check',{}).get('insufficient_axes')!=[]:
        raise ValueError('承重控制尚未验证，当前软件力矩包络不足；不要求现场手托，不使能六轴')
    start,target=plan['expected_start'],plan['target']
    if len(start)!=6 or len(target)!=6 or any(abs(a-b)>=math.radians(60) for a,b in zip(start,target)):
        raise ValueError('本轮必须六轴且每轴变化小于 60°')
    permit=OnsitePermit(config.fingerprint,AXES,time.monotonic()+15,evidence,AXES,True,True,
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
                  parameter_source='user 3/1/0; explicit software envelope, not a device parameter backup')
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
