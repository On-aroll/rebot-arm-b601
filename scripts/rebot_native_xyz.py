"""Small XYZ trials using the drive position loops, not a new host PID.

Reuses the verified CSP register adapter, Seeed registration, existing trajectory,
one-use task permit, feedback, cancellation and CAN ownership. Import is inert.
"""
from dataclasses import asdict, replace
import math
from numbers import Real
import time

from rebot_mit_hardware import Envelope, HardwareExecutor, MODELS, SdkTransport
from rebot_mit_mock import Trajectory, finite
from rebot_native_csp import CspTransport, PROTECTED, proposal as single_proposal
from rebot_seeed_transport import SeeedTransport, official_module, OFFICIAL

AXES=tuple(MODELS)
MODE='native-csp-xyz-v1'
PATH_MODE='native-csp-xyz-path-v1'
PROFILES={MODE:(.5,15.),PATH_MODE:(5.,30.)}
CURRENT=(10.,)*6
TORQUE=(1.,3.,9.,3.,1.,1.)
RUNTIME={i:{0x7005:5,0x7017:.02,0x7018:CURRENT[i-1],0x700B:TORQUE[i-1]} for i in AXES}


def profile(version):
    if version not in PROFILES:
        raise ValueError('需要明确的原生 XYZ 小幅或路径调试方案')
    return PROFILES[version]


def proposal(limits, *, mode=MODE):
    delta,seconds=profile(mode)
    raw,_=single_proposal(limits)
    raw['version']=mode
    raw['policy']['task_timeout']=seconds
    return raw,dict(mode=mode,axes=list(AXES),seconds=seconds,max_axis_delta_deg=delta,
        speed_max_rad_s=.02,current_max_a=list(CURRENT),torque_max_nm=list(TORQUE),
        pid_source='live drive parameters; read and preserved, not replaced by MIT gains',
        summary=f'J1–J6 原生位置闭环：每轴变化最多 {delta:g}°，最高 0.02 rad/s，首次使能起最多 {seconds:g} 秒；J7 不参与。',
        stop_policy='任务到位、取消或异常后失能；确认停止状态与持续稳定反馈后恢复临时运行参数，不恢复旧目标。')


def validate_target(start,target,config):
    delta,_=profile(config.version)
    if len(start)!=6 or len(target)!=6 or any(isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v) for v in (*start,*target)):
        raise ValueError('需要完整的六轴有限起点与目标')
    if any(abs(b-a)>math.radians(delta)+1e-12 for a,b in zip(start,target)):
        raise ValueError(f'本次原生 XYZ 方案每轴变化最多 {delta:g}°，请缩小坐标位移')
    if any(not a.q_min<=q<=a.q_max for a,q in zip(config.axes,target)):
        raise ValueError('XYZ 目标必须位于名义关节范围内；不修改限位或零位')


def assess_motion(robot,start,target,config):
    import numpy as np
    from reBotArm_control_py.dynamics import compute_generalized_gravity
    validate_target(start,target,config)
    robot.check_path(np.asarray(start),np.asarray(target))
    trajectory=Trajectory.create([float(v) for v in start],[[float(v) for v in target]],config,start_tolerance=config.start_boundary_tolerance)
    data=robot.pm.createData()
    peaks=np.max(np.abs([compute_generalized_gravity(robot.pm,np.asarray(trajectory.sample(t)[0]),data)[:6]
        for t in np.linspace(0,trajectory.duration,51)]),axis=0)
    if not np.all(np.isfinite(peaks)): raise ValueError('官方重力模型返回非有限值')
    reasons=[]
    if any(g>bound for g,bound in zip(peaks,TORQUE)): reasons.append('模型重力超过本次原生环力矩上限')
    if trajectory.duration+2.>config.policy.task_timeout: reasons.append(f'轨迹与启动余量超过 {config.policy.task_timeout:g} 秒试验上限')
    return dict(execution_ready=not reasons,execution_reasons=reasons,hardware_seconds=trajectory.duration,
        start_deg=[math.degrees(v) for v in start],target_deg=[math.degrees(v) for v in target],
        delta_deg=[math.degrees(b-a) for a,b in zip(start,target)],
        max_axis_delta_deg=max(math.degrees(abs(b-a)) for a,b in zip(start,target)),
        control_source='motorbridge typed CSP registers and existing drive position/velocity/current loops; no host MIT gravity PD',
        gravity_check=dict(source='official RS model, nominal estimate only',sampled_peak_Nm=peaks.tolist(),limits_Nm=list(TORQUE)),
        native_position_control=True,physical_acceptance=False)


def forward_kinematics():
    # Validate the exact existing Seeed checkout before importing its RS model.
    official_module()
    import numpy as np
    import pinocchio as pin
    model=pin.buildModelFromUrdf(str(OFFICIAL/'urdf/RS/urdf/ReBot_Arm_RS.urdf'))
    data=model.createData();frame=model.getFrameId('gripper_end')
    def fk(q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(6,) or not np.all(np.isfinite(q)): raise ValueError('FK 需要六个有限关节角')
        # Match Robot.fk: six arm joints, both gripper DOFs at model zero.
        full=np.zeros(model.nq);full[:6]=q
        pin.forwardKinematics(model,data,full)
        pin.updateFramePlacements(model,data)
        return [float(v) for v in data.oMf[frame].translation*1000]
    return fk


class NativeXYZTransport(SeeedTransport):
    def __init__(self,**kwargs):
        super().__init__(position_window=True,**kwargs)
        self.drives={}

    def read(self,axes,policy,check=lambda:None):
        samples=super().read(axes,policy,check)
        for i,s in samples.items():
            check();self.ownership.check();requested=self.clock()
            value=self.motors[i].robstride_get_param_f32_host_id(0x701A,0xFD,max(1,int(policy.io_timeout*1000)))
            check()
            if not finite(value): raise ValueError(f'J{i} 电流反馈无效')
            samples[i]=replace(s,iq_raw_a=value,diagnostics_requested_at=requested,diagnostics_received_at=self.clock())
        self.latest_samples.update(samples)
        return samples

    def authorize(self,config,permit):
        if permit.axes!=AXES or permit.coordinated_start_checked is not True:
            raise PermissionError('原生 XYZ 需要本次六轴协同许可')
        SdkTransport.authorize(self,config,permit)
        for i in AXES:
            # Inert per-drive register views share one controller and CAN owner.
            # They are never opened or closed independently.
            drive=CspTransport(axis=i,runtime=RUNTIME[i],factory=self.factory,ownership=self.ownership,clock=self.clock)
            drive.coordinated=True
            for key in ('controller','motors','active','permit','envelope','latest_samples','boundary_start','boundary_recovered'):
                setattr(drive,key,getattr(self,key))
            self.drives[i]=drive

    @property
    def register_log(self):
        return sorted([entry for drive in self.drives.values() for entry in drive.register_log],key=lambda e:e['requested_at'])

    def stop_supported(self):
        result=SdkTransport.stop_supported(self);result['disabled_status_by_axis']={}
        for i in result['sdk_status_ack_axes']:
            try: result['disabled_status_by_axis'][str(i)]=self.drives[i].disabled_status()
            except Exception as error: result['errors'][str(i)]=str(error)
        return result

    def restore(self):
        result=dict(ok=True,restored={},errors={},old_target_restored=False)
        for i,drive in self.drives.items():
            try:
                drive.restoration_authorized=True;restored=[]
                if drive.changed:
                    if 0x7016 in drive.changed:
                        drive.write_register(0x7016,self.latest_samples[i].q,lambda:None,phase='disabled-current')
                    for index in (0x7005,0x7017,0x7018,0x700B):
                        if index in drive.changed:
                            drive.write_register(index,drive.original[index],lambda:None,phase='restore');restored.append(hex(index))
                    for index in PROTECTED:
                        if drive.read_register(index)!=drive.original[index]: raise ValueError(f'保护参数 {index:#x} 与真实快照不同；未自动改写')
                result['restored'][str(i)]=restored
            except Exception as error: result['errors'][str(i)]=str(error)
        result['ok']=not result['errors']
        return result

    def frame(self,*args): raise PermissionError('原生 XYZ 不发送 MIT 帧')


class NativeXYZExecutor(HardwareExecutor):
    def __init__(self,config,transport,*,fk=None,**kwargs):
        super().__init__(config,transport,**kwargs)
        raw,_=proposal([(a.q_min,a.q_max) for a in config.axes],mode=config.version)
        if config.fingerprint!=Envelope.parse(raw).fingerprint: raise ValueError('原生 XYZ 仅接受固定小幅慢速方案')
        self.fk=fk or forward_kinematics()
        self.deadline=None;self.timing={};self.arrival_feedback={};self.restoration=None
        self.target=None;self.tcp_error_mm=None;self.goal_xyz_mm=None;self.arrival_xyz_mm=None

    def check(self,permit):
        super().check(permit)
        if self.deadline is not None and self.clock()>=self.deadline:
            raise TimeoutError(f'原生 XYZ 首次使能起 {self.config.policy.task_timeout:g} 秒结束，撤力')

    def validate(self,samples,*,csp=False,quiet=False):
        p=self.config.policy
        for i,s in samples.items():
            a=self.config.axes[i-1];margin=self.config.start_boundary_tolerance
            if s.mode!=(5 if csp else 0): raise ValueError(f'J{i} 原生 XYZ 运行模式不符')
            if not 0<=self.clock()-s.requested_at<=p.feedback_age: raise TimeoutError(f'J{i} 反馈过期')
            if not a.q_min-margin<=s.q<=a.q_max+margin: raise ValueError(f'J{i} 原始位置超出起点容差')
            if abs(s.qd)>(p.start_speed if quiet else a.v_max): raise ValueError(f'J{i} 实测速度超限：{s.qd:.6f} rad/s')
            if s.iq_raw_a is None or abs(s.iq_raw_a)>CURRENT[i-1]: raise ValueError(f'J{i} 电流反馈无效或超过 10 A')

    def send(self,q,samples,permit,phase,axes=AXES):
        self.check(permit);self.validate(samples,csp=True)
        entry=dict(time=self.clock(),phase=phase,commands=[],feedback={str(i):asdict(s) for i,s in samples.items()})
        self.frames.append(entry)
        for i in axes:
            if abs(q[i-1]-samples[i].q)>self.config.axes[i-1].error_max: raise ValueError(f'J{i} 原生环跟踪误差超限')
            command=dict(axis=i,target_rad=q[i-1],status='attempted');entry['commands'].append(command)
            try:
                self.bus.drives[i].write_register(0x7016,q[i-1],lambda:self.check(permit),phase=phase,verify=False)
                command['status']='host-send-complete';self.last_axis_command[i]=self.clock()
            except Exception as error:
                command.update(status='unknown-may-have-reached-drive',error=str(error));raise
        self.check(permit)

    def run(self,intent,permit):
        if self.consumed: raise RuntimeError('一次性原生 XYZ 任务不能重放')
        self.consumed=True;terminal='FAULT';p=self.config.policy
        try:
            permit.validate(self.config,self.clock())
            if permit.axes!=AXES or not permit.coordinated_start_checked or intent.source!='native-csp-xyz' or len(intent.waypoints)!=1:
                raise ValueError('需要本次明确六轴 XYZ 任务')
            expected=intent.expected_start;target=intent.waypoints[0]
            if expected is None: raise ValueError('缺少已预演实测起点')
            validate_target(expected,target,self.config)
            self.target=tuple(target);self.goal_xyz_mm=list(self.fk(target))
            if len(self.goal_xyz_mm)!=3 or any(not finite(v) for v in self.goal_xyz_mm): raise ValueError('目标 FK 坐标无效')
            check=lambda:self.check(permit)
            check();self.bus.open_readonly();samples=self.bus.read(AXES,p,check);self.feedback.update(samples)
            self.validate(samples,quiet=True)
            if any(abs(samples[i].q-expected[i-1])>p.arrival_q for i in AXES): raise ValueError('原生 XYZ 起点已改变，请重新预演')
            self.bus.authorize(self.config,permit)
            for drive in self.bus.drives.values(): check();drive.quiesce();check()
            for drive in self.bus.drives.values(): drive.snapshot_registers(check)
            for i,drive in self.bus.drives.items():
                for index in (0x7005,0x700B,0x7018,0x7017): drive.write_register(index,RUNTIME[i][index],check,phase='prepare')
            # All axes remain disabled. Snapshot/configuration can take longer
            # than the velocity window; obtain two new position sweeps before
            # the first enable instead of differentiating against old samples.
            self.bus.previous_samples.clear()
            start=list(expected);enabled=[]
            for i,drive in self.bus.drives.items():
                samples=self.bus.read(AXES,p,check);self.feedback.update(samples);self.validate(samples,csp=True,quiet=True)
                if any(abs(samples[j].q-expected[j-1])>p.arrival_q for j in AXES if j not in enabled): raise ValueError('未使能轴起点变化，不追赶旧姿态')
                if enabled: self.send(start,samples,permit,'hold',tuple(enabled))
                start[i-1]=samples[i].q
                drive.write_register(0x7016,start[i-1],check,phase='preload-current')
                if self.clock()-samples[i].requested_at>p.command_age: raise TimeoutError('原生 XYZ 首目标过期，未使能')
                if self.deadline is None:
                    self.timing['enable_attempted_at']=self.clock();self.deadline=self.clock()+p.task_timeout
                    self.timing.update(power_deadline=self.deadline,source='host monotonic timestamps; not physical enable duration')
                self.last_axis_command[i]=self.clock();drive.enable(i);check();enabled.append(i)
                self.send(start,samples,permit,'hold',tuple(enabled))
            validate_target(start,target,self.config)
            trajectory=Trajectory.create(start,[target],self.config,start_tolerance=self.config.start_boundary_tolerance)
            self.state='RUNNING';began=None;quiet=None;settled=None;last=self.clock()
            while True:
                cycle=self.clock();check();samples=self.bus.read(AXES,p,check);self.feedback.update(samples);self.validate(samples,csp=True)
                now=self.clock()
                if now-last>p.period+p.jitter_max: raise TimeoutError('原生 XYZ 反馈周期延迟超限')
                if began is None:
                    ready=all(abs(samples[i].q-start[i-1])<=p.arrival_q and abs(samples[i].qd)<=p.start_speed for i in AXES)
                    quiet=now if ready and quiet is None else quiet if ready else None
                    if quiet is not None and now-quiet>=p.settle_time: began=now
                elapsed=0. if began is None else now-began
                q,_,_,_=trajectory.sample(elapsed);self.send(q,samples,permit,'hold' if began is None else 'trajectory');last=now
                xyz=list(self.fk([samples[i].q for i in AXES]))
                if len(xyz)!=3 or any(not finite(v) for v in xyz): raise ValueError('反馈 FK 坐标无效')
                self.tcp_error_mm=math.dist(xyz,self.goal_xyz_mm)
                arrived=began is not None and elapsed>=trajectory.duration and self.tcp_error_mm<=1. and all(
                    abs(samples[i].q-target[i-1])<=p.arrival_q and abs(samples[i].qd)<=p.arrival_qd for i in AXES)
                settled=now if arrived and settled is None else settled if arrived else None
                if settled is not None and now-settled>=p.settle_time:
                    self.motion_arrived=True;self.arrival_feedback=dict(samples);self.arrival_xyz_mm=xyz
                    self.reason='六轴原生 XYZ 关节反馈与模型 TCP 到位并稳定，随后撤力；空间精度未独立测量';terminal='ARRIVED';break
                self.wait(max(0.,p.period-(self.clock()-cycle)))
        except Exception as error:
            self.failure_feedback=dict(self.feedback);self.reason=str(error)
            terminal='CANCELLED' if isinstance(error,InterruptedError) else 'FAULT'
        finally:
            if self.bus.active:
                self.state='STOPPING';self.timing['stop_requested_at']=self.clock()
                self.stop_result=self.bus.stop_supported();self.timing['stop_call_completed_at']=self.clock()
                try:
                    if self.stop_result['errors']: raise ValueError('失能未完全确认，不恢复高限制')
                    axes=tuple(self.stop_result['attempted_axes']);quiet=None;began=self.clock();self.stop_result['observations']=[]
                    while True:
                        samples=self.bus.read(axes,p);self.feedback.update(samples);now=self.clock()
                        self.stop_result['observations'].append({str(i):asdict(s) for i,s in samples.items()})
                        for i,s in samples.items():
                            a=self.config.axes[i-1];margin=self.config.start_boundary_tolerance
                            if not 0<=now-s.requested_at<=p.feedback_age or not a.q_min-margin<=s.q<=a.q_max+margin:
                                raise ValueError('停止后位置越界或反馈过期')
                        stationary=all(abs(s.qd)<=p.arrival_qd for s in samples.values())
                        quiet=now if stationary and quiet is None else quiet if stationary else None
                        if quiet is not None and now-quiet>=p.settle_time: break
                        if now-began>=p.feedback_age+p.settle_time: raise ValueError('停止后未连续停稳，不恢复高限制')
                        if self.wait==self.cancelled.wait: time.sleep(p.period)
                        else: self.wait(p.period)
                    self.stop_result['stationary_sample']=True;self.restoration=self.bus.restore()
                    if not self.restoration['ok']: raise ValueError('部分运行参数恢复未完全确认')
                except Exception as error:
                    self.reason+='；停止清理：'+str(error);terminal='FAULT'
            try: self.bus.close()
            except Exception as error: self.reason+='；连接清理：'+str(error);terminal='FAULT'
            self.state=terminal

    def status(self):
        result=super().status();result.update(control=dict(mode=self.config.version,axes=list(AXES),task_timeout_s=self.config.policy.task_timeout,
            max_axis_delta_deg=profile(self.config.version)[0],
            speed_limit_rad_s=.02,config_fingerprint=self.config.fingerprint,pid_source='live drive PID, not overwritten'),
            arrival_feedback={str(i):asdict(s) for i,s in self.arrival_feedback.items()},restoration=self.restoration,
            register_snapshot={str(i):{hex(k):v for k,v in drive.original.items()} for i,drive in self.bus.drives.items()},
            goal_xyz_mm=self.goal_xyz_mm,arrival_xyz_mm=self.arrival_xyz_mm,tcp_error_mm=self.tcp_error_mm,timing=self.timing)
        return result
