"""Small XYZ trials using the drive position loops, not a new host PID.

Reuses the verified CSP register adapter, Seeed registration, existing trajectory,
one-use task permit, feedback, cancellation and CAN ownership. Import is inert.
"""
from dataclasses import asdict, replace
from collections import deque
import math
from numbers import Real
import time

from .rebot_mit_hardware import Envelope, HardwareExecutor, MODELS, SdkTransport
from .rebot_mit_mock import Trajectory, finite
from .rebot_native_csp import CspTransport, PROTECTED, proposal as single_proposal
from .rebot_seeed_transport import SeeedTransport, official_module, OFFICIAL
from ..csp_official_limits import SPEED_MAX, ACCELERATION_MAX, JERK_MAX, DRIVE_TORQUE, DRIVE_CURRENT, RATED_TORQUE

AXES=tuple(MODELS)
MODE='native-csp-xyz-v1'
PATH_MODE='native-csp-xyz-path-v1'
FULL_MODE='native-csp-twin-full-v1'
SMOOTH_FULL_MODE='native-csp-twin-full-v2'
OFFICIAL_FULL_MODE='native-csp-twin-full-v3-official'
LIVE_FULL_MODE='native-csp-twin-full-v4-command500'
OFFICIAL_FULL_MODES=(OFFICIAL_FULL_MODE,LIVE_FULL_MODE)
PROFILES={MODE:(.5,15.),PATH_MODE:(5.,30.),FULL_MODE:(360.,1800.),SMOOTH_FULL_MODE:(360.,1800.),OFFICIAL_FULL_MODE:(360.,1800.),LIVE_FULL_MODE:(360.,1800.)}
CURRENT=(10.,)*6
TORQUE=(1.,3.,9.,3.,1.,1.)
RUNTIME={i:{0x7005:5,0x7017:.02,0x7018:CURRENT[i-1],0x700B:TORQUE[i-1]} for i in AXES}


def runtime_for_mode(version, *, original=None):
    profile(version)
    if version in OFFICIAL_FULL_MODES:
        return {i:{0x7005:5,0x7017:min(SPEED_MAX,original[0x7017]) if original else SPEED_MAX,
            0x7018:DRIVE_CURRENT[i-1],0x700B:DRIVE_TORQUE[i-1]} for i in AXES}
    speed = .9 if version == SMOOTH_FULL_MODE else .1 if version == FULL_MODE else .02
    return {i:{**RUNTIME[i],0x7017:speed} for i in AXES}


def gravity_limits_for_mode(version):
    return RATED_TORQUE if version in OFFICIAL_FULL_MODES else TORQUE


def profile(version):
    if version not in PROFILES:
        raise ValueError('未知的原生 CSP 控制方案')
    return PROFILES[version]


def proposal(limits, *, mode=MODE):
    delta,seconds=profile(mode)
    raw,_=single_proposal(limits)
    raw['version']=mode
    raw['policy']['task_timeout']=seconds
    # User-requested host command watchdog; preserve historical fingerprints.
    if mode == LIVE_FULL_MODE:
        raw['policy']['command_age']=.5
    speed=runtime_for_mode(mode)[1][0x7017]
    if mode in (FULL_MODE, SMOOTH_FULL_MODE):
        # Version v1 remains immutable for old recovery fingerprints. v2 is
        # the user-requested <=0.9 command ceiling / 1.0 measured-speed guard.
        # Original per-drive limits must cover it; snapshot checks enforce this.
        # PID/current/torque, acceleration and watchdogs remain unchanged.
        for axis in raw['axes']:
            axis.update(v_max=1.0 if mode == SMOOTH_FULL_MODE else .12,a_max=.08,jerk_max=.4,error_max=math.radians(3))
    elif mode in OFFICIAL_FULL_MODES:
        for i,axis in enumerate(raw['axes']):
            axis.update(v_max=SPEED_MAX,a_max=ACCELERATION_MAX,jerk_max=JERK_MAX,
                torque_max=DRIVE_TORQUE[i],error_max=math.radians(3))
    return raw,dict(mode=mode,axes=list(AXES),seconds=seconds,max_axis_delta_deg=delta,
        speed_max_rad_s=speed,current_max_a=list(DRIVE_CURRENT if mode in OFFICIAL_FULL_MODES else CURRENT),
        torque_max_nm=list(DRIVE_TORQUE if mode in OFFICIAL_FULL_MODES else TORQUE),
        pid_source='live drive parameters; read and preserved, not replaced by MIT gains',
        summary=f'J1–J6 原生位置闭环：每轴变化最多 {delta:g}°且不越过模型限位，最高 {speed:g} rad/s，首次使能起最多 {seconds:g} 秒；J7 不参与。',
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
    gravity_limits=gravity_limits_for_mode(config.version)
    if any(g>bound for g,bound in zip(peaks,gravity_limits)): reasons.append('模型重力超过本次原生环力矩上限')
    if trajectory.duration+2.>config.policy.task_timeout: reasons.append(f'轨迹与启动余量超过 {config.policy.task_timeout:g} 秒试验上限')
    return dict(execution_ready=not reasons,execution_reasons=reasons,hardware_seconds=trajectory.duration,
        start_deg=[math.degrees(v) for v in start],target_deg=[math.degrees(v) for v in target],
        delta_deg=[math.degrees(b-a) for a,b in zip(start,target)],
        max_axis_delta_deg=max(math.degrees(abs(b-a)) for a,b in zip(start,target)),
        control_source='motorbridge typed CSP registers and existing drive position/velocity/current loops; no host MIT gravity PD',
        gravity_check=dict(source='official RS model, nominal estimate only',sampled_peak_Nm=peaks.tolist(),limits_Nm=list(gravity_limits)),
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


class FeedbackTimeout(TimeoutError):
    """Fresh telemetry was not available; old motion must remain cancelled."""


class NativeXYZTransport(SeeedTransport):
    def __init__(self,**kwargs):
        super().__init__(position_window=True,**kwargs)
        self.drives={}
        self.drive_backend='csp'
        self.mit_parameters=None
        self.poll_io=deque(maxlen=64)

    def read(self,axes,policy,check=lambda:None,*,on_sample=None):
        axes = tuple(axes)
        # Bootstrap the encoder velocity window with two complete sweeps.
        # Once warm, publish each fully checked axis immediately, so a later
        # axis's retry cannot hide an already received fresh J1 sample.
        cold = any(i not in self.previous_samples for i in axes)
        initial = super().read(axes,policy,check) if cold else None
        samples = {}
        for i in axes:
            s = initial[i] if cold else super().read((i,),policy,check)[i]
            check();self.ownership.check();requested=self.clock()
            value=self.motors[i].robstride_get_param_f32_host_id(0x701A,0xFD,max(1,int(policy.io_timeout*1000)))
            check()
            if not finite(value): raise TimeoutError(f'J{i} 电流反馈无效')
            samples[i]=replace(s,iq_raw_a=value,diagnostics_requested_at=requested,diagnostics_received_at=self.clock())
            self.latest_samples[i] = samples[i]
            if on_sample:
                on_sample({i:samples[i]})
        self.latest_samples.update(samples)
        return samples

    def read_cycle(self, policy, diagnostic_axis, check=lambda:None, *, on_sample=None,on_io=None,interleaved=False,max_fields=None):
        """One owner: all encoders every cycle, mode/current round-robin.

        Positions plus the supplied mode/current pairs replace a 24-query sweep.
        Each field retains its own request timestamp; cached diagnostics never
        become fresh merely because a position was read. No streamed state or
        second CAN reader is used on this firmware.
        """
        if any(i not in self.previous_samples or i not in self.latest_samples for i in AXES):
            raise RuntimeError('交错轮询必须先完成六轴反馈初始化')
        diagnostic_axes = (diagnostic_axis,) if isinstance(diagnostic_axis,int) else tuple(diagnostic_axis)
        if interleaved and diagnostic_axes != AXES:
            raise ValueError('逐轴模式/电流/位置轮询需要完整六轴')
        if max_fields is not None and (not interleaved or type(max_fields) is not int or not 1<=max_fields<=18):
            raise ValueError('有界字段刷新只允许完整六轴的1–18项最旧字段')
        def record(i,index,requested):
            received=self.clock()
            previous = self.poll_io[-1] if self.poll_io else None
            self.poll_io.append(dict(axis=i,index=hex(index),requested_at=requested,
                received_at=received,duration_ms=round((received-requested)*1000,3),
                between_queries_ms=round(max(0.,requested-previous['received_at'])*1000,3) if previous else None))
            return received
        def read_mode(i):
            check(); self.ownership.check(); requested = self.clock()
            mode = self.motors[i].robstride_get_param_i8(0x7005,max(1,int(policy.io_timeout*1000)))
            record(i,0x7005,requested);check()
            if type(mode) is not int:raise FeedbackTimeout(f'J{i} 模式反馈无效')
            self.latest_samples[i] = replace(self.latest_samples[i],mode=mode,mode_requested_at=requested)
            if on_sample:on_sample({i:self.latest_samples[i]})
            if on_io:on_io()
        def read_current(i):
            check(); self.ownership.check(); requested = self.clock()
            current = self.motors[i].robstride_get_param_f32_host_id(0x701A,0xFD,max(1,int(policy.io_timeout*1000)))
            received=record(i,0x701A,requested);check()
            if not finite(current):raise FeedbackTimeout(f'J{i} 电流反馈无效')
            self.latest_samples[i] = replace(self.latest_samples[i],iq_raw_a=current,
                diagnostics_requested_at=requested,diagnostics_received_at=received)
            if on_sample:on_sample({i:self.latest_samples[i]})
            if on_io:on_io()
        samples = {}
        def read_position(i):
            check(); self.ownership.check(); requested = self.clock()
            q = self.motors[i].robstride_get_param_f32_host_id(0x7019,0xFD,max(1,int(policy.io_timeout*1000)))
            record(i,0x7019,requested)
            check(); received = self.clock()
            previous = self.previous_samples[i]
            dt = (requested+received-previous.requested_at-previous.received_at)/2
            if not finite(q) or not 0 < dt <= policy.feedback_age:
                raise FeedbackTimeout(f'J{i} 位置反馈或差分窗口无效/过期')
            cached = self.latest_samples[i]
            sample = replace(cached,q=q,qd=(q-previous.q)/dt,qd_raw=None,
                requested_at=requested,received_at=received,
                mode_requested_at=cached.mode_requested_at if cached.mode_requested_at is not None else cached.requested_at)
            samples[i] = self.latest_samples[i] = self.previous_samples[i] = sample
            if on_sample:on_sample({i:sample})
            if on_io:on_io()
        if interleaved:
            # Same 18 reads and original deadlines, oldest FIELD first. After a
            # hold/J7 preparation, refreshing J1 mode first could leave its older
            # current expiring at the inter-reply reference check (observed 601ms).
            fields=((0x7005,'mode_requested_at',read_mode),
                    (0x701A,'diagnostics_requested_at',read_current),
                    (0x7019,'requested_at',read_position))
            pending=[(i,index,stamp,read) for i in AXES for index,stamp,read in fields]
            initial_stamps={(i,index):getattr(self.latest_samples[i],stamp) for i,index,stamp,_ in pending}
            completed=0
            while pending and (max_fields is None or completed<max_fields):
                def age_key(row):
                    i,index,stamp,_=row;s=self.latest_samples[i]
                    at=getattr(s,stamp)
                    return (s.requested_at if at is None else at,i,index)
                row=min(pending,key=age_key);pending.remove(row)
                # J7's checked I/O may service oldest arm fields on this same
                # owner. Count those real reads instead of querying them a
                # second time when the outer sweep resumes.
                if getattr(self.latest_samples[row[0]],row[2]) != initial_stamps[row[0],row[1]]:continue
                row[3](row[0])
                completed+=1
            # A position may be read before its mode/current in this sweep.
            samples={i:self.latest_samples[i] for i in AXES}
        else:
            for i in diagnostic_axes:read_mode(i);read_current(i)
            for i in AXES:read_position(i)
        return samples

    def authorize(self,config,permit):
        if permit.axes!=AXES or permit.coordinated_start_checked is not True:
            raise PermissionError('原生 XYZ 需要本次六轴协同许可')
        SdkTransport.authorize(self,config,permit)
        for i in AXES:
            # Inert per-drive register views share one controller and CAN owner.
            # They are never opened or closed independently.
            kwargs=dict(axis=i,runtime=runtime_for_mode(config.version)[i],factory=self.factory,ownership=self.ownership,clock=self.clock)
            if self.drive_backend == 'mit':
                from .rebot_twin_mit import MitDrive
                drive=MitDrive(parameters=self.mit_parameters,**kwargs)
            else:
                drive=CspTransport(**kwargs)
            drive.coordinated=True
            drive.preserve_lower_speed = config.version in OFFICIAL_FULL_MODES
            for key in ('controller','motors','active','permit','envelope','latest_samples','boundary_start','boundary_recovered','support_recovery'):
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

    def reset_views(self, originals=None, *, mode=MODE):
        """Inert restore-only views. This grants no permit to enable or move."""
        if self.active:
            raise RuntimeError('先完成当前会话停止，再重置')
        self.drives = {}
        for i in AXES:
            drive = CspTransport(axis=i, runtime=runtime_for_mode(mode,original=originals[i] if originals else None)[i],
                factory=self.factory, ownership=self.ownership, clock=self.clock)
            for key in ('controller','motors','active','latest_samples'):
                setattr(drive,key,getattr(self,key))
            if originals:
                drive.original = dict(originals[i])
            self.drives[i] = drive

    def disable_all_for_reset(self):
        # On a restarted worker active is empty even if a previous process
        # exited badly. A reset therefore explicitly disables J1–J6, not J7.
        result = dict(attempted_axes=[], disabled_status_by_axis={}, errors={})
        for i,drive in self.drives.items():
            result['attempted_axes'].append(i)
            try:
                self.ownership.check()
                self.motors[i].disable()
                result['disabled_status_by_axis'][str(i)] = drive.disabled_status()
            except Exception as exc:
                result['errors'][str(i)] = str(exc)
        return result

    def restore(self, check=lambda:None):
        result=dict(ok=True,restored={},errors={},old_target_restored=False)
        for i,drive in self.drives.items():
            try:
                check()
                drive.restoration_authorized=True;restored=[]
                if drive.changed:
                    if 0x7016 in drive.changed:
                        drive.write_register(0x7016,self.latest_samples[i].q,check,phase='disabled-current')
                    for index in (0x7005,0x7017,0x7018,0x700B):
                        if index in drive.changed:
                            drive.write_register(index,drive.original[index],check,phase='restore');restored.append(hex(index))
                    for index in PROTECTED:
                        check()
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
        if config.fingerprint!=Envelope.parse(raw).fingerprint: raise ValueError('原生 CSP 仅接受已定义的控制方案，不能任意扩大包络')
        self.fk=fk or forward_kinematics()
        self.deadline=None;self.timing={};self.arrival_feedback={};self.restoration=None
        self.target=None;self.tcp_error_mm=None;self.goal_xyz_mm=None;self.arrival_xyz_mm=None
        self.command_targets={};self.target_changed_at={}
        self.command_history={}

    def check(self,permit):
        super().check(permit)
        if self.deadline is not None and self.clock()>=self.deadline:
            raise TimeoutError(f'原生 XYZ 首次使能起 {self.config.policy.task_timeout:g} 秒结束，撤力')

    def validate(self,samples,*,csp=False,quiet=False):
        p=self.config.policy
        for i,s in samples.items():
            a=self.config.axes[i-1];margin=self.config.start_boundary_tolerance
            expected = 5 if csp and getattr(self.bus,'drive_backend','csp') == 'csp' else 0
            if s.mode!=expected:
                raise ValueError(f'J{i} 运行模式不符：读到 {s.mode}，本阶段要求 {expected}；'
                                 '运行模式不等于使能状态。请核查上次停止与参数恢复，未自动切换模式')
            age = self.clock()-s.requested_at
            if not 0<=age<=p.feedback_age:
                raise FeedbackTimeout(f'J{i} 反馈过期：年龄 {age*1000:.0f} ms，允许 0–{p.feedback_age*1000:.0f} ms')
            for label,stamp in (('模式',s.mode_requested_at),('电流',s.diagnostics_requested_at)):
                if stamp is not None and not 0 <= self.clock()-stamp <= p.feedback_age:
                    raise FeedbackTimeout(f'J{i} {label}反馈过期（{(self.clock()-stamp)*1000:.0f} ms > '
                        f'{p.feedback_age*1000:.0f} ms）；新位置不能刷新旧诊断时间')
            recovery=getattr(self.bus,'support_recovery',None)
            if recovery:
                if not recovery.allows_feedback(i,s.q):
                    raise ValueError(f'J{i} 支撑恢复反馈偏离已确认的单调回位走廊')
            elif not a.q_min-margin<=s.q<=a.q_max+margin:
                raise ValueError(f'J{i} 原始位置超出起点容差')
            if abs(s.qd)>(p.start_speed if quiet else a.v_max): raise ValueError(f'J{i} 实测速度超限：{s.qd:.6f} rad/s')
            current_limit=runtime_for_mode(self.config.version)[i][0x7018]
            if s.iq_raw_a is None or abs(s.iq_raw_a)>current_limit:
                raise ValueError(f'J{i} 电流反馈无效或超过 {current_limit:g} A')

    def send(self,q,samples,permit,phase,axes=AXES):
        self.check(permit);self.validate(samples,csp=True)
        entry=dict(time=self.clock(),phase=phase,commands=[],feedback={str(i):asdict(s) for i,s in samples.items()})
        self.frames.append(entry)
        for i in axes:
            previous=self.command_targets.get(i)
            reference=previous if self.config.version in OFFICIAL_FULL_MODES and previous is not None else q[i-1]
            if self.config.version in OFFICIAL_FULL_MODES and self.command_history.get(i):
                # Compare a measurement with the command that existed when its
                # query began, not a newer command sent while polling other axes.
                history=self.command_history[i]
                reference=next((value for stamp,value in reversed(history) if stamp<=samples[i].requested_at),history[0][1])
            if abs(reference-samples[i].q)>self.config.axes[i-1].error_max: raise ValueError(f'J{i} 原生环跟踪误差超限')
            if self.config.version in OFFICIAL_FULL_MODES and previous is not None:
                # Following error is measured against a target already sent.
                # Bound the new reference step separately by device velocity.
                allowance=self.bus.drives[i].runtime[0x7017]*min(.25,max(0.,entry['time']-self.target_changed_at[i]))+1e-6
                if abs(q[i-1]-previous)>allowance: raise ValueError(f'J{i} 新目标变化超过设备限速预算')
            command=dict(axis=i,target_rad=q[i-1],status='attempted');entry['commands'].append(command)
            try:
                self.bus.drives[i].write_register(0x7016,q[i-1],lambda:self.check(permit),phase=phase,verify=False)
                command['status']='host-send-complete';self.last_axis_command[i]=self.clock()
                if previous is None or abs(q[i-1]-previous)>1e-12: self.target_changed_at[i]=entry['time']
                self.command_targets[i]=q[i-1]
                self.command_history.setdefault(i,deque(maxlen=512)).append((self.clock(),q[i-1]))
            except Exception as error:
                command.update(status='unknown-may-have-reached-drive',error=str(error));raise
        self.check(permit)
        return entry['time']

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
                for index in (0x7005,0x700B,0x7018,0x7017): drive.write_register(index,drive.runtime[index],check,phase='prepare')
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
            speed_limit_rad_s=runtime_for_mode(self.config.version)[1][0x7017],config_fingerprint=self.config.fingerprint,pid_source='live drive PID, not overwritten'),
            arrival_feedback={str(i):asdict(s) for i,s in self.arrival_feedback.items()},restoration=self.restoration,
            register_snapshot={str(i):{hex(k):v for k,v in drive.original.items()} for i,drive in self.bus.drives.items()},
            goal_xyz_mm=self.goal_xyz_mm,arrival_xyz_mm=self.arrival_xyz_mm,tcp_error_mm=self.tcp_error_mm,timing=self.timing)
        return result
