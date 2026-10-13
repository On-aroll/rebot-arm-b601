"""Explicit RobStride SDK adapter. Import/construction never opens CAN.

No default motion limits, automatic enable, ensure_mode, generic SDK stop, zero
write, parameter write or automatic target recovery. Tests inject a fake SDK.
Actual motion requires a fresh, one-use onsite permit from an explicitly confirmed trial.
"""
from dataclasses import dataclass, asdict, replace, field
import fcntl
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import secrets
import threading
import time
from rebot_mit_mock import AxisLimits, Trajectory, finite

_PERMIT_LOCK=threading.RLock()
_CONSUMED_PERMITS=set()

MODELS={i:'rs-06' if i<=3 else 'rs-00' for i in range(1,7)}

@dataclass(frozen=True)
class Sample:
    axis: int
    q: float
    qd: float
    mode: int
    requested_at: float
    received_at: float
    qd_raw: float | None = None
    # Host request/response bounds, NOT a drive-provided timestamp or enabled bit.

@dataclass(frozen=True)
class TrialPolicy:
    period: float
    io_timeout: float
    feedback_age: float
    command_age: float
    jitter_max: float
    lease_timeout: float
    start_speed: float
    arrival_q: float
    arrival_qd: float
    settle_time: float
    task_timeout: float

@dataclass(frozen=True)
class Envelope:
    version: str
    axes: tuple
    policy: TrialPolicy
    start_boundary_tolerance: float = 0.

    @classmethod
    def parse(cls, data):
        if data.get('source')!='explicit-onsite-trial-proposal' or not data.get('version'):
            raise ValueError('需要明确的现场试验方案和版本；不能使用 B1 合成配置')
        if len(data.get('axes',[]))!=6: raise ValueError('需要六轴模型/规划包络')
        axes=[]
        for item in data['axes']:
            if set(item)!=set(AxisLimits.__dataclass_fields__) or not all(finite(v) for v in item.values()):
                raise ValueError('试验边界必须完整且为有限数值，不自动补默认值')
            a=AxisLimits(**item)
            if a.q_min>=a.q_max or min(a.v_max,a.a_max,a.jerk_max,a.error_max,a.torque_max)<=0:
                raise ValueError('运动包络无效')
            if (a.kp,a.kd,a.tau_ff)!=(3.,1.,0.): raise ValueError('本轮只接受用户提供的 3/1/0；其他增益需另行审核')
            if a.kp_max<a.kp or a.kd_max<a.kd: raise ValueError('增益超出明确包络')
            # Official v0.5.6 model-specific encoding ranges; NOT safety limits.
            index=len(axes)+1
            vmax,tmax,kpmax,kdmax=(50.,36.,5000.,100.) if index<=3 else (33.,14.,500.,5.)
            if a.q_min < -4*math.pi or a.q_max > 4*math.pi or a.v_max>vmax or a.kp_max>kpmax or a.kd_max>kdmax or a.torque_max>tmax:
                raise ValueError('超出官方 MIT 编码范围')
            axes.append(a)
        p=data.get('policy',{})
        if set(p)!=set(TrialPolicy.__dataclass_fields__) or not all(finite(v) and v>0 for v in p.values()):
            raise ValueError('控制周期、超时和到位阈值需要明确的试验值')
        p=TrialPolicy(**p)
        if p.period>=p.command_age or p.io_timeout>=p.feedback_age or p.settle_time>=p.task_timeout:
            raise ValueError('试验时序约束无效')
        tolerance=data.get('start_boundary_tolerance',0.)
        if not finite(tolerance) or not 0<=tolerance<=math.radians(1):
            raise ValueError('起点边界容差必须明确且不超过 1°')
        return cls(data['version'],tuple(axes),p,tolerance)

    @property
    def fingerprint(self):
        data=asdict(self)
        if not self.start_boundary_tolerance: data.pop('start_boundary_tolerance')
        return hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()

@dataclass
class OnsitePermit:
    """Ephemeral operator evidence, never produced by software config approval.

    Supported-release is the only implemented physical stop policy. It is NOT
    accepted for an unsupported gravity joint, nor enabled automatically.
    """
    config_hash: str
    axes: tuple
    expires_at: float
    evidence: str
    mechanically_supported_axes: tuple
    independent_power_cut_ready: bool
    operator_present: bool
    token: str = field(default_factory=lambda:secrets.token_hex(24))
    used: bool = False
    coordinated_start_checked: bool = False

    def validate(self, config, now):
        if self.used or self.token in _CONSUMED_PERMITS or self.config_hash!=config.fingerprint or not now<self.expires_at:
            raise PermissionError('现场确认已过期、已使用或配置已变化')
        if not self.evidence.strip() or not self.operator_present or not self.independent_power_cut_ready:
            raise PermissionError('缺少当次现场确认及独立断电条件')
        if not self.axes or any(type(i) is not int or i not in MODELS for i in self.axes) or len(set(self.axes))!=len(self.axes):
            raise PermissionError('试验轴必须明确且不包含 J7')
        if not set(self.axes)<=set(self.mechanically_supported_axes):
            raise PermissionError('仅支持已机械支撑轴的去使能停止，禁止承重轴自动掉落')


class CanOwnership:
    """Linux cooperative lock + foreign CAN receive-list detection.

    Does not exclude a malicious TX-only SocketCAN process or privileged user.
    A dedicated host/session with all other controllers stopped is a prerequisite.
    """
    def __init__(self, channel='can0', directory=None):
        if channel!='can0': raise ValueError('本轮仅审核了 can0')
        self.channel=channel
        self.directory=Path(directory or f'/tmp/rebot-can-owner-{os.getuid()}')
        self.handle=None; self.owned=None

    def entries(self):
        entries=[]
        for name in ('all','fil','inv','sff','eff','err'):
            path=Path('/proc/net/can')/('rcvlist_'+name)
            for line in path.read_text().splitlines():
                fields=line.split()
                if fields and fields[0] in (self.channel, 'any') and len(fields)>=5:
                    # Match counters change; pointer/ID/filter identify a subscriber.
                    entries.append((name,*fields[:5],*fields[6:]))
        return sorted(entries)

    def acquire(self):
        self.directory.mkdir(mode=0o700,exist_ok=True)
        self.handle=(self.directory/'can0.lock').open('a+')
        try:
            fcntl.flock(self.handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if self.entries(): raise RuntimeError('CAN 已被其他接收者占用；先退出网关、只读跟随和其他控制器')
        except Exception:
            self.close(); raise

    def claim(self):
        entries=self.entries()
        # Audited v0.5.6 SocketCanBus owns one raw catch-all subscription.
        # Do not absorb a second subscriber appearing during SDK construction.
        if len(entries)!=1 or entries[0][0]!='all' or entries[0][1]!=self.channel:
            raise RuntimeError('SDK 建连期间发现额外/异常 CAN 接收者，拒绝接管')
        self.owned=entries
    def check(self):
        if self.handle is None or self.owned is None or self.entries()!=self.owned:
            raise RuntimeError('CAN 所有权发生变化，拒绝继续运动')
    def close(self):
        if self.handle is not None: self.handle.close(); self.handle=None


class SdkTransport:
    def __init__(self, *, factory=None, ownership=None, clock=time.monotonic):
        self.factory=factory; self.ownership=ownership or CanOwnership(); self.clock=clock
        self.controller=None; self.motors={}; self.active=set(); self.permit=None; self.envelope=None
        self.sent=0
        self.latest_samples={}; self.boundary_start={}; self.boundary_recovered=set()

    def open_readonly(self):
        if self.controller: raise RuntimeError('连接已打开')
        self.ownership.acquire()
        try:
            if self.factory is None:
                if importlib.metadata.version('motorbridge')!='0.5.6':
                    raise RuntimeError('只审核了 motorbridge 0.5.6；版本变化需重新核对协议')
                from motorbridge import Controller
                factory=Controller
            else: factory=self.factory  # Offline test double; production has no factory flag.
            self.controller=factory(channel='can0')
            for i,model in MODELS.items(): self.motors[i]=self.controller.add_robstride_motor(i,0xFD,model)
            self.ownership.claim()
        except Exception:
            self.close(); raise

    def read(self, axes, policy, check=lambda:None):
        samples={}
        for i in axes:
            check(); self.ownership.check(); requested=self.clock(); motor=self.motors[i]
            q=motor.robstride_get_param_f32_host_id(0x7019,0xFD,max(1,int(policy.io_timeout*1000)))
            check()
            v=motor.robstride_get_param_f32_host_id(0x701B,0xFD,max(1,int(policy.io_timeout*1000)))
            check()
            # SDK i8 path may retry for >=150 ms; checked afterward, never hidden.
            mode=motor.robstride_get_param_i8(0x7005,max(1,int(policy.io_timeout*1000)))
            check(); received=self.clock()
            if received-requested>policy.feedback_age: raise TimeoutError(f'J{i} 反馈查询超过试验时限')
            if not finite(q) or not finite(v) or type(mode) is not int: raise ValueError(f'J{i} 无效反馈')
            samples[i]=Sample(i,q,v,mode,requested,received)
        self.latest_samples.update(samples)
        return samples

    def authorize(self, config, permit):
        if self.factory is None and len(permit.axes)!=1 and not (
                permit.axes==tuple(MODELS) and permit.coordinated_start_checked is True):
            raise PermissionError('多轴试验需要明确的六轴协同起动许可；默认仅允许单轴')
        self.ownership.check()
        if config.start_boundary_tolerance:
            if any(i not in self.latest_samples or not 0<=self.clock()-self.latest_samples[i].requested_at<=config.policy.feedback_age for i in permit.axes):
                raise PermissionError('边界容差必须绑定新鲜原始起点')
            self.boundary_start={i:self.latest_samples[i].q for i in permit.axes}
        with _PERMIT_LOCK:
            permit.validate(config,self.clock())
            _CONSUMED_PERMITS.add(permit.token)
            permit.used=True
        self.permit=permit; self.envelope=config

    def enable(self, axis):
        if not self.permit or axis not in self.permit.axes or self.clock()>=self.permit.expires_at:
            raise PermissionError('没有当次使能许可')
        self.ownership.check()
        # May reach drive before SDK times out. Always include attempted axis in stop.
        self.active.add(axis)
        self.motors[axis].enable()

    def frame(self, axis, values):
        if axis not in self.active or self.clock()>=self.permit.expires_at: raise PermissionError('许可失效')
        if len(values)!=5 or not all(finite(v) for v in values): raise ValueError('MIT 帧必须是五个有限数值')
        a=self.envelope.axes[axis-1]
        bounded=a.q_min<=values[0]<=a.q_max
        if not bounded and axis not in self.boundary_recovered:
            start=self.boundary_start.get(axis); tolerance=self.envelope.start_boundary_tolerance
            if start is not None and a.q_min-tolerance<=start<a.q_min:
                bounded=max(a.q_min-tolerance,start-self.envelope.policy.arrival_q)<=values[0]<a.q_min and values[1]>=0
            elif start is not None and a.q_max<start<=a.q_max+tolerance:
                bounded=a.q_max<values[0]<=min(a.q_max+tolerance,start+self.envelope.policy.arrival_q) and values[1]<=0
        if not bounded or abs(values[1])>a.v_max or tuple(values[2:])!=(a.kp,a.kd,a.tau_ff):
            raise ValueError('MIT 帧与已确认的范围/增益不一致')
        self.ownership.check()
        self.motors[axis].send_mit(*values)
        if a.q_min<=values[0]<=a.q_max: self.boundary_recovered.add(axis)
        self.sent+=1  # Host send completion; MIT frame has no per-frame drive ACK.

    def stop_supported(self):
        result=dict(requested=True,policy='explicitly-supported-release',attempted_axes=sorted(self.active),
                    sent_axes=[],sdk_status_ack_axes=[],send_status={},errors={},physical_stopped=False)
        # Stop overrides permit expiry/cancel/ownership change; only axes attempted by us.
        # Never calls SDK controlled_stop (which can substitute gains).
        for i in sorted(self.active):
            try:
                self.motors[i].disable()
                result['sent_axes'].append(i); result['sdk_status_ack_axes'].append(i)
                result['send_status'][str(i)]='host-send-complete-and-sdk-status-observed'
            except Exception as e:
                result['errors'][str(i)]=str(e)
                result['send_status'][str(i)]='unknown-may-have-reached-drive'
        self.active.clear(); self.permit=None; self.envelope=None
        return result

    def close(self):
        if self.active: raise RuntimeError('先完成明确的停止请求，再释放控制句柄')
        try:
            if self.controller: self.controller.close_bus()  # NOT shutdown()/context manager.
        finally:
            for m in self.motors.values(): m.close()
            if self.controller: self.controller.close()
            self.controller=None; self.motors={}; self.latest_samples.clear(); self.boundary_start.clear(); self.boundary_recovered.clear(); self.ownership.close()


class MotionIntent:
    """Common bounded joint-trajectory intent. Future Cartesian/gamepad planning
    must resolve IK upstream and require a live lease; never streams browser CAN.
    """
    def __init__(self, waypoints, source, request_id, expected_start=None):
        if source not in ('onsite-single-axis','onsite-hold','xyz','gamepad-planned') or not request_id:
            raise ValueError('需要明确运动来源及一次性请求编号')
        self.waypoints=tuple(tuple(p) for p in waypoints); self.source=source; self.request_id=request_id
        self.expected_start=tuple(expected_start) if expected_start is not None else None
        if source!='onsite-single-axis' and (self.expected_start is None or len(self.expected_start)!=6 or not all(finite(v) for v in self.expected_start)):
            raise ValueError('XYZ/手柄计划必须绑定已经预演的真实起点')


class HoldIntent(MotionIntent):
    """No browser-supplied destination: hold the newly sampled J1 position."""
    def __init__(self, request_id, expected_start, seconds):
        if seconds != 3: raise ValueError('首次 J1 保持仅允许 3 秒')
        super().__init__([], 'onsite-hold', request_id, expected_start)
        self.hold_seconds = 3.


class HardwareExecutor:
    """One background worker, fresh hardware start, bounded synchronous SDK calls.

    Lease is independent of the browser animation loop. This is soft realtime,
    not a hardware safety controller; process/power failure needs independent stop.
    """
    def __init__(self, config, transport, *, clock=time.monotonic, wait=None):
        self.config=config; self.bus=transport; self.clock=clock
        self.cancelled=threading.Event(); self.wait=wait or self.cancelled.wait
        self.lock=threading.Lock(); self.lease_until=0.; self.lease_seq=-1; self.session=secrets.token_hex(16)
        self.motion_arrived=False; self.last_axis_command={}; self.consumed=False; self.state='IDLE'; self.reason=''; self.stop_result=None; self.feedback={}; self.frames=[]; self.used=set(); self.worker=None

    def heartbeat(self, session, sequence, focused):
        with self.lock:
            if session!=self.session or type(sequence) is not int or sequence<=self.lease_seq: raise PermissionError('过期/重复的操作窗口心跳')
            self.lease_seq=sequence
            if focused is not True: self.cancelled.set(); return
            self.lease_until=self.clock()+self.config.policy.lease_timeout

    def cancel(self): self.cancelled.set()

    def check(self, permit):
        now=self.clock()
        if self.cancelled.is_set(): raise InterruptedError('任务取消或窗口失焦')
        if now>self.lease_until: raise TimeoutError('操作窗口失联，禁止继续发送旧目标')
        if now>=permit.expires_at: raise TimeoutError('现场许可过期')
        if any(now-stamp>self.config.policy.command_age for stamp in self.last_axis_command.values()):
            raise TimeoutError('已使能轴超过命令有效期；停止启动/执行，不补发旧帧')

    def validate_feedback(self, samples, axes):
        now=self.clock(); p=self.config.policy
        for i in axes:
            s=samples[i]; a=self.config.axes[i-1]
            if s.mode!=0: raise ValueError(f'J{i} 实际模式不是 MIT；未自动切换')
            if not 0<=now-s.requested_at<=p.feedback_age: raise TimeoutError(f'J{i} 反馈过期')
            margin=self.config.start_boundary_tolerance
            if not a.q_min-margin<=s.q<=a.q_max+margin: raise ValueError(f'J{i} 原始位置越限；未归一化或裁剪')
            if abs(s.qd)>a.v_max: raise ValueError(f'J{i} 实测速度超过试验边界')
            if s.qd_raw is not None and a.kd*abs(s.qd_raw)>a.torque_max:
                raise ValueError(f'J{i} 原始速度反馈的阻尼估算超过包络')

    def _send(self, q, v, samples, permit):
        issued=self.clock(); p=self.config.policy
        self.validate_feedback(samples,permit.axes)
        frames=[]
        for i in permit.axes:
            self.check(permit)
            if self.clock()-issued>p.command_age: raise TimeoutError('当前帧已过期')
            a=self.config.axes[i-1]; s=samples[i]; k=i-1
            error=q[k]-s.q
            # Conservative absolute software effort envelope, NOT measured torque.
            damping_error=max(abs(v[k]-s.qd), abs(v[k]-s.qd_raw) if s.qd_raw is not None else 0.)
            estimate=a.kp*abs(error)+a.kd*damping_error+abs(a.tau_ff)
            if abs(error)>a.error_max or estimate>a.torque_max: raise ValueError(f'J{i} 跟踪误差/软件转矩包络超限')
            values=(q[k],v[k],a.kp,a.kd,a.tau_ff)
            self.bus.frame(i,values); self.last_axis_command[i]=self.clock(); frames.append(dict(axis=i,values=values,estimated_effort=estimate))
            self.check(permit)
            if self.clock()-issued>p.command_age: raise TimeoutError('SDK 发送完成已超时，停止旧任务')
        self.frames.append(dict(time=issued,commands=frames))

    def start(self, intent, permit):
        with self.lock:
            if self.state!='IDLE': raise RuntimeError('执行器需新会话；任务不能并发或重放')
            permit.validate(self.config,self.clock())
            self.state='PREPARING'
        self.worker=threading.Thread(target=self.run,args=(intent,permit),daemon=False,name='rebot-mit-trial')
        self.worker.start()

    def run(self, intent, permit):
        # Synchronous entry is also exercised with fake clock/SDK; no web frame pump.
        with self.lock:
            if self.consumed: raise RuntimeError('执行会话已消费；重新连接不会重放任务')
            self.consumed=True
        permit=replace(permit,axes=tuple(permit.axes),mechanically_supported_axes=tuple(permit.mechanically_supported_axes))
        try:
            if intent.request_id in self.used: raise ValueError('任务不能重放')
            self.used.add(intent.request_id); permit.validate(self.config,self.clock())
            self.check(permit); self.state='PREPARING'
            self.bus.open_readonly()
            check=lambda:self.check(permit)
            samples=self.bus.read(tuple(MODELS),self.config.policy,check)
            # A supported single-axis trial checks only its commanded drive.
            # Other raw positions stay unchanged; XYZ retains all-six checks.
            checked_axes=permit.axes if intent.source=='onsite-single-axis' else tuple(MODELS)
            self.validate_feedback(samples,checked_axes)
            start=tuple(samples[i].q for i in MODELS)
            if intent.expected_start is not None and any(abs(a-b)>self.config.policy.arrival_q for a,b in zip(start,intent.expected_start)):
                raise ValueError('真实起点与已预演起点不一致，需重新规划')
            if any(abs(samples[i].qd)>self.config.policy.start_speed for i in checked_axes): raise ValueError('受控轴未静止，不接受跳变起点')
            holding=isinstance(intent,HoldIntent)
            if intent.source=='onsite-hold' and (not holding or permit.axes!=(1,)):
                raise ValueError('保持试验仅允许 J1 和后端新鲜目标')
            waypoints=(start,) if holding else intent.waypoints
            reviewed=start if holding or intent.expected_start is None else intent.expected_start
            for point in waypoints:
                if len(point)!=6: raise ValueError('需要六轴目标')
                if any(point[i-1]!=reviewed[i-1] for i in MODELS if i not in permit.axes): raise ValueError('目标包含未获准的轴')
            # Passive slots track the newly sampled start, never a stale reading.
            waypoints=tuple(tuple(point[i-1] if i in permit.axes else start[i-1] for i in MODELS) for point in waypoints)
            if intent.source=='onsite-single-axis' and len(permit.axes)!=1: raise ValueError('首次试验必须只有一个轴')
            scoped_axes=permit.axes if intent.source=='onsite-single-axis' else None
            trajectory=Trajectory.create(start,waypoints,self.config,axes=scoped_axes,start_tolerance=self.config.start_boundary_tolerance)
            if trajectory.duration+self.config.policy.settle_time>self.config.policy.task_timeout: raise ValueError('轨迹无法在试验时限内完成')
            self.bus.authorize(self.config,permit)
            # For each enabled axis read position AGAIN before its first frame.
            # Never first target HOME or the previous task's endpoint.
            startup_samples={}
            for i in permit.axes:
                check(); self.last_axis_command[i]=self.clock(); self.bus.enable(i); check()
                current=self.bus.read((i,),self.config.policy,check)
                self.validate_feedback(current,(i,))
                if abs(current[i].q-start[i-1])>self.config.policy.arrival_q or abs(current[i].qd)>self.config.policy.start_speed:
                    raise ValueError('使能后起点变化，停止并重新预演')
                values=(current[i].q,0.,self.config.axes[i-1].kp,self.config.axes[i-1].kd,self.config.axes[i-1].tau_ff)
                if self.config.axes[i-1].kd*max(abs(current[i].qd),abs(current[i].qd_raw or 0.))>self.config.axes[i-1].torque_max:
                    raise ValueError('首帧阻尼力矩估算超过包络')
                issued=self.clock()
                self.bus.frame(i,values)
                if self.clock()-issued>self.config.policy.command_age: raise TimeoutError('首帧发送超过有效期')
                self.last_axis_command[i]=self.clock(); check()
                startup_samples.update(current)
                if len(permit.axes)>1:
                    # Refresh already started axes during sequential SDK setup.
                    # A slow enable/read still faults before any stale target.
                    started=tuple(startup_samples)
                    hold_q=tuple(startup_samples[j].q if j in startup_samples else start[j-1] for j in MODELS)
                    self._send(hold_q,(0.,)*6,startup_samples,replace(permit,axes=started))
            # Rebase all permitted axes on post-enable feedback, so the curve
            # cannot jump back to a pre-enable pose after its first hold frame.
            samples=self.bus.read(permit.axes,self.config.policy,check)
            self.validate_feedback(samples,permit.axes)
            if any(abs(samples[i].q-start[i-1])>self.config.policy.arrival_q or abs(samples[i].qd)>self.config.policy.start_speed for i in permit.axes):
                raise ValueError('使能后实机起点发生变化，需重新规划')
            start=tuple(samples[i].q if i in samples else start[i-1] for i in MODELS)
            if len(permit.axes)>1:
                self._send(start,(0.,)*6,samples,permit)
            if holding: waypoints=(start,)
            trajectory=Trajectory.create(start,waypoints,self.config,axes=scoped_axes,start_tolerance=self.config.start_boundary_tolerance)
            began=self.clock(); last=began; settled=None; self.state='RUNNING'
            while True:
                check(); now=self.clock(); elapsed=now-began
                if elapsed>self.config.policy.task_timeout: raise TimeoutError('任务超时')
                if holding and elapsed>=intent.hold_seconds:
                    if settled is None or now-settled<self.config.policy.settle_time:
                        raise TimeoutError('3 秒保持已结束，未达到稳定条件；停止发送目标并撤力')
                    self.motion_arrived=True; break
                if now-last>self.config.policy.period+self.config.policy.jitter_max: raise TimeoutError('执行循环延迟超限')
                samples=self.bus.read(permit.axes,self.config.policy,check); self.feedback=samples
                self.validate_feedback(samples,permit.axes); check()
                now=self.clock(); elapsed=now-began
                if elapsed>self.config.policy.task_timeout: raise TimeoutError('任务超时')
                if holding and elapsed>=intent.hold_seconds:
                    if settled is None or now-settled<self.config.policy.settle_time:
                        raise TimeoutError('3 秒保持已结束，未达到稳定条件；停止发送目标并撤力')
                    self.motion_arrived=True; break
                if now-last>self.config.policy.period+self.config.policy.jitter_max: raise TimeoutError('反馈周期延迟超限')
                q,v,_,_=trajectory.sample(elapsed)
                self._send(q,v,samples,permit); last=now
                if elapsed>=trajectory.duration:
                    arrived=all(abs(samples[i].q-q[i-1])<=self.config.policy.arrival_q and abs(samples[i].qd)<=self.config.policy.arrival_qd for i in permit.axes)
                    settled=now if arrived and settled is None else settled if arrived else None
                    if settled is not None and now-settled>=self.config.policy.settle_time and elapsed>=getattr(intent,'hold_seconds',0):
                        self.motion_arrived=True; break
                self.wait(max(0.,self.config.policy.period-(self.clock()-now)))
        except Exception as e:
            self.reason=str(e); self.state='CANCELLED' if isinstance(e,InterruptedError) else 'FAULT'
        finally:
            terminal='ARRIVED' if self.motion_arrived else self.state
            if self.bus.active:
                self.state='STOPPING'
                self.stop_result=self.bus.stop_supported()
                if self.stop_result['errors']:
                    terminal='FAULT'; self.reason+='; 停止确认失败，需要现场独立停止'
                # A single SDK ACK/sample is never physical-stop acceptance.
                # Failure to observe fresh low speed must not look like success.
                try:
                    axes=tuple(self.stop_result['attempted_axes'])
                    stopped=self.bus.read(axes,self.config.policy)
                    self.validate_feedback(stopped,axes)
                    self.stop_result['stationary_sample']=all(abs(s.qd)<=self.config.policy.arrival_qd for s in stopped.values())
                    if not self.stop_result['stationary_sample']:
                        terminal='FAULT'; self.reason+='; 停止后反馈仍有速度，需要现场独立停止'
                except Exception as e:
                    self.stop_result['verification_error']=str(e)
                    terminal='FAULT'; self.reason+='; 停止反馈验证失败：'+str(e)
            try: self.bus.close()
            except Exception as e:
                self.reason+='; connection cleanup: '+str(e); terminal='FAULT'
            self.state=terminal

    def status(self):
        permit=self.bus.permit
        return dict(state=self.state,reason=self.reason,motion_arrived_before_stop=self.motion_arrived,stop=self.stop_result,frames=len(self.frames),
                    feedback={str(i):asdict(s) for i,s in self.feedback.items()},
                    physical_acceptance=False, hardware_permission=bool(permit and self.state in ('PREPARING','RUNNING') and
                        not self.cancelled.is_set() and self.clock()<min(permit.expires_at,self.lease_until)),
                    note='试验执行记录不是动态标定、转矩实测或物理停止验收')
