"""One explicitly approved J3 CSP trial, using the audited motorbridge protocol.

This is a narrow adapter to the drive's existing position loop, not a new PID.
It reuses the task lease, cancellation, CAN ownership and septic trajectory.
Importing it does not connect. No PID, zero, flash-save or generic pos_vel calls.
"""
from dataclasses import asdict
import math
import time

from rebot_mit_hardware import Envelope, HardwareExecutor, MODELS, SdkTransport
from rebot_mit_mock import Trajectory, finite
from rebot_seeed_transport import SeeedTransport

AXIS = 3
DELTA = math.radians(.5)
TASK_TIMEOUT = 15.  # Explicitly approved extension of the same J3 trial.
RUNTIME = {0x7005: 5, 0x7017: .02, 0x7018: 10., 0x700B: 9.}
PROTECTED = (0x701E, 0x701F, 0x7020, 0x702B)


def proposal(limits):
    raw = dict(source='explicit-onsite-trial-proposal', version='native-csp-j3-v2-15s',
        start_boundary_tolerance=math.radians(1),
        axes=[dict(q_min=float(lo), q_max=float(hi), v_max=.02, a_max=.04, jerk_max=.2,
            kp=3., kd=1., tau_ff=0., kp_max=3., kd_max=1.,
            torque_max=(1.,3.,9.,3.,1.,1.)[i], error_max=math.radians(.6))
            for i,(lo,hi) in enumerate(limits)],
        policy=dict(period=.1, io_timeout=.05, feedback_age=.6, command_age=.3,
            jitter_max=.15, lease_timeout=.75, start_speed=.01, arrival_q=math.radians(.1),
            arrival_qd=.01, settle_time=.3, task_timeout=TASK_TIMEOUT))
    Envelope.parse(raw)
    public = dict(title='J3 原生 CSP +0.5° 小幅试验', seconds=TASK_TIMEOUT, axes=[3], delta_deg=.5,
        summary=f'只控制 J3：从新鲜实测起点增加 0.5°，最高 0.02 rad/s；首次使能起最多 {TASK_TIMEOUT:g} 秒，完成或异常均失能。',
        runtime_registers={hex(k):v for k,v in RUNTIME.items()},
        parameter_source='本次明确授权的临时运行参数；PID、零位、保存草稿及掉电保存不写入',
        stop_policy='先失能并观察稳定反馈，再恢复非目标运行参数；不恢复旧位置目标。停止未确认则保留低限制并报告。',
        limitations='原生位置环内的加速度/力矩不由主机轨迹保证；SDK 阻塞可能延迟停止，网页不是物理急停。')
    return raw, public


def validate_samples(config, samples, clock, *, csp=False, quiet=False):
    for i,s in samples.items():
        a=config.axes[i-1]; p=config.policy
        if s.mode != (5 if csp and i==AXIS else 0):
            raise ValueError(f'J{i} 实际运行模式与本次 CSP 阶段不符')
        if not 0 <= clock()-s.requested_at <= p.feedback_age:
            raise TimeoutError(f'J{i} 反馈过期')
        margin=0. if i==AXIS else config.start_boundary_tolerance
        if not a.q_min-margin <= s.q <= a.q_max+margin:
            raise ValueError(f'J{i} 原始位置超出本次范围；未裁剪或改零位')
        limit=p.start_speed if quiet else a.v_max
        if abs(s.qd)>limit:
            raise ValueError(f'J{i} 实测速度 {s.qd:.6f} rad/s 超过 {limit:g} rad/s')
        if i==AXIS and s.iq_raw_a is not None and abs(s.iq_raw_a)>RUNTIME[0x7018]:
            raise ValueError(f'J3 电流反馈 {s.iq_raw_a:.3f} A 超过本次 10 A 限制')


def probe(config, bus):
    readings=[]
    try:
        bus.open_readonly()
        for _ in range(3):
            samples=bus.read(tuple(MODELS),config.policy)
            readings.append({str(i):asdict(s) for i,s in samples.items()})
            validate_samples(config,samples,bus.clock,quiet=True)
            if any(abs(s.q-readings[0][str(i)]['q'])>config.policy.arrival_q for i,s in samples.items()):
                raise ValueError('只读检查期间起点发生变化')
        return dict(ok=True,expected_start=[samples[i].q for i in MODELS],
                    feedback=readings[-1],readings=readings,hardware_writes=0)
    except Exception as e:
        return dict(ok=False,reason=str(e),feedback=readings[-1] if readings else {},readings=readings,hardware_writes=0)
    finally:
        bus.close()


class CspTransport(SeeedTransport):
    def __init__(self, *, axis=AXIS, runtime=None, **kwargs):
        if type(axis) is not int or axis not in MODELS: raise ValueError('需要明确的 J1–J6 轴编号')
        super().__init__(position_window=True, diagnostic_axes=(axis,), **kwargs)
        self.axis=axis; self.runtime=dict(RUNTIME if runtime is None else runtime)
        self.coordinated=False; self.last_boundary_target=None
        self.original={}; self.register_log=[]; self.changed=set()
        self.prepared=set(); self.preloaded_at=None; self.restoration_authorized=False

    def authorize(self, config, permit):
        if permit.axes!=(self.axis,): raise PermissionError('CSP 需要对应的明确单轴许可')
        # Register/feedback comes from Seeed; no MIT group or lifecycle needed.
        SdkTransport.authorize(self,config,permit)

    def read_register(self, index):
        motor=self.motors[self.axis]
        value=(motor.robstride_get_param_i8(index,50) if index==0x7005 else
               motor.robstride_get_param_f32_host_id(index,0xFD,50))
        if not finite(value): raise ValueError(f'J{self.axis} 寄存器 {index:#x} 无效')
        return value

    def snapshot_registers(self, check):
        for index in (*self.runtime,*PROTECTED):
            check(); self.ownership.check()
            self.original[index]=self.read_register(index); check()
        if self.original[0x7005]!=0: raise ValueError(f'J{self.axis} 起始模式不是 MIT，拒绝覆盖未知运行状态')
        if any(self.original[i]<v for i,v in self.runtime.items() if i!=0x7005):
            raise ValueError('设备现有限制低于本次提案，不能自动提高')

    def write_register(self, index, value, check, *, phase, verify=True):
        if index not in (*self.runtime,0x7016) or not finite(value):
            raise PermissionError('仅允许本次明确的 CSP 运行寄存器')
        if phase in ('restore','disabled-current'):
            if not self.restoration_authorized or self.active:
                raise PermissionError('恢复参数须先确认失能及持续稳定反馈')
            if phase=='restore' and (index not in self.original or value!=self.original[index]):
                raise PermissionError('恢复值必须来自本次真实设备快照')
            if phase=='disabled-current' and (index!=0x7016 or value!=self.latest_samples[self.axis].q):
                raise PermissionError('失能后只装载新鲜当前位置，不恢复旧目标')
        else:
            scope=self.permit and (self.permit.axes==(self.axis,) or self.coordinated and self.permit.axes==tuple(MODELS) and self.permit.coordinated_start_checked)
            if not scope or self.axis not in self.active or self.clock()>=self.permit.expires_at:
                raise PermissionError('没有当次 CSP 寄存器写入许可')
            if phase=='prepare':
                if index not in self.runtime or value!=self.runtime[index]: raise PermissionError('只能写入已确认的四项运行值')
            elif phase in ('preload-current','hold','trajectory'):
                a=self.envelope.axes[self.axis-1]
                bounded=a.q_min<=value<=a.q_max
                if not bounded and self.coordinated and self.axis not in self.boundary_recovered:
                    start=self.boundary_start.get(self.axis); margin=self.envelope.start_boundary_tolerance
                    if start is not None and a.q_min-margin<=start<a.q_min:
                        bounded=max(a.q_min-margin,start-self.envelope.policy.arrival_q)<=value<a.q_min
                        if phase=='trajectory': bounded=bounded and value>= (self.last_boundary_target if self.last_boundary_target is not None else start)-1e-12
                    elif start is not None and a.q_max<start<=a.q_max+margin:
                        bounded=a.q_max<value<=min(a.q_max+margin,start+self.envelope.policy.arrival_q)
                        if phase=='trajectory': bounded=bounded and value<= (self.last_boundary_target if self.last_boundary_target is not None else start)+1e-12
                    if phase=='preload-current': bounded=bounded and value==self.latest_samples[self.axis].q
                if index!=0x7016 or not bounded: raise PermissionError('CSP 目标越界')
            else: raise PermissionError('未知 CSP 写入阶段')
        check(); self.ownership.check()
        entry=dict(axis=self.axis,index=hex(index),value=value,phase=phase,requested_at=self.clock(),status='attempted')
        self.register_log.append(entry); self.changed.add(index)
        try:
            motor=self.motors[self.axis]
            if index==0x7005: motor.robstride_write_param_i8(index,int(value))
            else: motor.robstride_write_param_f32(index,value)
            check()
            if verify:
                actual=self.read_register(index); check(); entry['readback']=actual
                # This firmware reports loc_ref at encoder resolution (and
                # follows mechPos while disabled). Keep a tight explicit 0.02°
                # comparison; speed/current/torque/mode readbacks stay exact.
                tolerance=math.radians(.02) if index==0x7016 else 1e-6
                if not math.isclose(actual,value,rel_tol=1e-6,abs_tol=tolerance):
                    raise ValueError(f'J{self.axis} {index:#x} 写后读回不一致：{actual} != {value}')
            entry.update(status='readback-matched' if verify else 'host-send-complete',completed_at=self.clock())
            if phase=='prepare': self.prepared.add(index)
            if phase=='preload-current': self.preloaded_at=entry['requested_at']
            if index==0x7016 and phase in ('preload-current','hold','trajectory'): self.last_boundary_target=value
            if index==0x7016 and phase in ('hold','trajectory') and self.envelope.axes[self.axis-1].q_min<=value<=self.envelope.axes[self.axis-1].q_max:
                self.boundary_recovered.add(self.axis)
        except Exception as e:
            entry.update(status='unknown-may-have-reached-drive',error=str(e)); raise

    def disabled_status(self):
        """Called only immediately after SDK disable's new-status wait."""
        state=self.motors[self.axis].get_state()
        if state is None: raise ValueError('失能后的状态帧缺失')
        arbitration=int(state.arbitration_id)
        if (arbitration>>24)&0x1F != 2 or (arbitration>>8)&0xFF != self.axis:
            raise ValueError('失能后的状态帧类型或电机 ID 不符')
        mode=(arbitration>>22)&3; faults=(arbitration>>16)&0x3F
        if mode!=0 or faults:
            raise ValueError(f'失能状态未确认：模式位 {mode}，故障位 {faults:#x}')
        return dict(arbitration_id=arbitration,mode_state=mode,fault_bits=faults,
                    source='SDK status after disable; no independent drive timestamp')

    def quiesce(self):
        scope=self.permit and (self.permit.axes==(self.axis,) or self.coordinated and self.permit.axes==tuple(MODELS) and self.permit.coordinated_start_checked)
        if not scope: raise PermissionError('没有对应轴的当次停止/切模式许可')
        self.ownership.check(); self.active.add(self.axis)
        self.motors[self.axis].disable()
        return self.disabled_status()

    def enable(self, axis):
        if axis!=self.axis or self.prepared!=set(self.runtime) or self.preloaded_at is None:
            raise PermissionError('先完成 J3 运行参数读回与当前位置装载，再使能')
        if self.clock()-self.preloaded_at>self.envelope.policy.command_age:
            raise TimeoutError('已装载的当前位置超过首帧期限')
        super().enable(axis)

    def stop_supported(self):
        result=super().stop_supported()
        if self.axis in result['sdk_status_ack_axes']:
            try: result['disabled_status']=self.disabled_status()
            except Exception as e: result['errors'][str(self.axis)]=str(e)
        return result

    def frame(self, *args):
        raise PermissionError('CSP 试验不发送 MIT 帧')


class CspJ3Trial(HardwareExecutor):
    def __init__(self, config, transport, **kwargs):
        super().__init__(config,transport,**kwargs)
        if transport.axis!=AXIS or transport.runtime!=RUNTIME or transport.coordinated:
            raise ValueError('本入口仅允许既定 J3 单轴运行参数')
        expected,_=proposal([(a.q_min,a.q_max) for a in config.axes])
        if config.fingerprint!=Envelope.parse(expected).fingerprint:
            raise ValueError(f'CSP 试验只能使用已确认的固定 0.5°/0.02 rad/s/{TASK_TIMEOUT:g} 秒方案')
        self.restore_result=None; self.power_deadline=None; self.arrival_feedback={}
        self.enable_attempted_at=None; self.stop_requested_at=None; self.stop_call_completed_at=None

    def check(self, permit):
        super().check(permit)
        if self.power_deadline is not None and self.clock()>=self.power_deadline:
            raise TimeoutError(f'CSP 首次使能起 {TASK_TIMEOUT:g} 秒时限已到，撤力')

    def send_position(self, q, samples, permit, phase):
        self.check(permit); validate_samples(self.config,samples,self.clock,csp=True)
        s=samples[AXIS]; a=self.config.axes[AXIS-1]
        if not a.q_min<=q<=a.q_max or abs(q-s.q)>a.error_max:
            raise ValueError(f'J3 CSP 跟踪误差或目标越界：误差 {math.degrees(q-s.q):.3f}°')
        entry=dict(time=self.clock(),phase=phase,target_rad=q,feedback={str(i):asdict(s) for i,s in samples.items()})
        self.frames.append(entry)
        self.bus.write_register(0x7016,q,lambda:self.check(permit),phase=phase,verify=False)
        self.last_axis_command[AXIS]=self.clock(); self.check(permit)

    def run(self, intent, permit):
        if self.consumed: raise RuntimeError('CSP 试验会话不能重放')
        self.consumed=True; terminal='FAULT'; p=self.config.policy
        try:
            permit.validate(self.config,self.clock())
            if permit.axes!=(AXIS,) or intent.source!='native-csp-j3' or len(intent.waypoints)!=1:
                raise ValueError('只允许已批准的 J3 CSP 小幅试验')
            expected=intent.expected_start; target=intent.waypoints[0]
            if expected is None or len(expected)!=6 or any(not finite(v) for v in expected):
                raise ValueError('缺少已检查的六轴起点')
            if any(abs(b-a-(DELTA if i==2 else 0.))>1e-9 for i,(a,b) in enumerate(zip(expected,target))):
                raise ValueError('目标必须为 J3 +0.5°，其他轴不改变')
            check=lambda:self.check(permit)
            check(); self.bus.open_readonly()
            samples=self.bus.read(tuple(MODELS),p,check); self.feedback.update(samples)
            validate_samples(self.config,samples,self.clock,quiet=True)
            if any(abs(samples[i].q-expected[i-1])>p.arrival_q for i in MODELS):
                raise ValueError('实测起点已改变，请重新检查')
            self.bus.authorize(self.config,permit)
            self.bus.quiesce(); check()
            self.bus.snapshot_registers(check)
            for index in (0x7005,0x700B,0x7018,0x7017):
                self.bus.write_register(index,RUNTIME[index],check,phase='prepare')
            samples=self.bus.read(tuple(MODELS),p,check); self.feedback.update(samples)
            validate_samples(self.config,samples,self.clock,csp=True,quiet=True)
            if any(abs(samples[i].q-expected[i-1])>p.arrival_q for i in MODELS):
                raise ValueError('使能前起点发生变化；不自动追赶旧目标')
            start=tuple(samples[i].q for i in MODELS)
            # Keep passive slots at the measured start. Only J3 is commanded.
            goal=tuple(target[2] if i==3 else start[i-1] for i in MODELS)
            trajectory=Trajectory.create(start,[goal],self.config,axes=(AXIS,))
            self.bus.write_register(0x7016,start[2],check,phase='preload-current')
            if self.clock()-samples[AXIS].requested_at>p.command_age:
                raise TimeoutError('首个位置目标已过期；未使能')
            self.enable_attempted_at=self.clock()
            self.power_deadline=self.enable_attempted_at+p.task_timeout
            self.last_axis_command[AXIS]=self.clock(); self.bus.enable(AXIS); check()
            began=self.clock(); last=began; settled=None; self.state='RUNNING'
            while True:
                cycle_started=self.clock()
                check(); samples=self.bus.read(tuple(MODELS),p,check); self.feedback.update(samples)
                validate_samples(self.config,samples,self.clock,csp=True); check()
                if any(abs(samples[i].q-start[i-1])>p.arrival_q for i in MODELS if i!=AXIS):
                    raise ValueError('未控制轴发生位移，停止 J3 小幅试验')
                now=self.clock()
                if now-last>p.period+p.jitter_max: raise TimeoutError('CSP 反馈周期延迟超限')
                # A short hold verifies the preloaded start before advancing.
                elapsed=max(0.,now-began-p.settle_time)
                if now-began<p.settle_time and abs(samples[AXIS].q-start[2])>p.arrival_q:
                    raise ValueError('CSP 启动保持偏移，轨迹未开始')
                q,_,_,_=trajectory.sample(elapsed)
                self.send_position(q[2],samples,permit,'hold' if elapsed==0 else 'trajectory'); last=now
                arrived=elapsed>=trajectory.duration and abs(samples[AXIS].q-goal[2])<=p.arrival_q and abs(samples[AXIS].qd)<=p.arrival_qd
                settled=now if arrived and settled is None else settled if arrived else None
                if settled is not None and now-settled>=p.settle_time:
                    self.motion_arrived=True; self.arrival_feedback=dict(samples)
                    self.reason='J3 CSP 反馈到位并稳定，随后撤力；六轴 XYZ 尚未验收'; terminal='ARRIVED'; break
                # Account for the entire synchronous feedback sweep. Sleeping
                # a full period after reading six axes double-counts that time.
                self.wait(max(0.,p.period-(self.clock()-cycle_started)))
        except Exception as e:
            self.failure_feedback=dict(self.feedback); self.reason=str(e)
            terminal='CANCELLED' if isinstance(e,InterruptedError) else 'FAULT'
        finally:
            if self.bus.active:
                self.state='STOPPING'; self.stop_requested_at=self.clock()
                self.stop_result=self.bus.stop_supported(); self.stop_call_completed_at=self.clock()
                try:
                    if self.stop_result['errors']: raise ValueError('失能状态未确认，保留当前低限制')
                    quiet=None; began=self.clock(); self.stop_result['observations']=[]
                    while True:
                        samples=self.bus.read((AXIS,),p); self.feedback.update(samples); now=self.clock()
                        self.stop_result['observations'].append({str(i):asdict(s) for i,s in samples.items()})
                        s=samples[AXIS]
                        if not 0<=now-s.requested_at<=p.feedback_age: raise TimeoutError('失能后反馈过期')
                        quiet=now if abs(s.qd)<=p.arrival_qd and quiet is None else quiet if abs(s.qd)<=p.arrival_qd else None
                        if quiet is not None and now-quiet>=p.settle_time: break
                        if now-began>=p.feedback_age+p.settle_time: raise ValueError('失能后未持续停稳，保留当前低限制')
                        if self.wait==self.cancelled.wait: time.sleep(p.period)
                        else: self.wait(p.period)
                    self.stop_result['stationary_sample']=True
                    self.restore_result=dict(ok=False,restored=[],old_target_restored=False)
                    self.bus.restoration_authorized=True
                    if self.bus.changed:
                        # Never restore an old loc_ref. Mode first, then limits,
                        # only after explicit disable and quiet fresh feedback.
                        if 0x7016 in self.bus.changed:
                            self.bus.write_register(0x7016,s.q,lambda:None,phase='disabled-current')
                        for index in (0x7005,0x7017,0x7018,0x700B):
                            if index in self.bus.changed:
                                self.bus.write_register(index,self.bus.original[index],lambda:None,phase='restore')
                                self.restore_result['restored'].append(hex(index))
                        for index in PROTECTED:
                            if self.bus.read_register(index)!=self.bus.original[index]:
                                raise ValueError(f'保护参数 {index:#x} 与起始快照不一致；没有自动改回')
                    self.restore_result['ok']=True
                except Exception as e:
                    self.reason+='；清理未完全确认：'+str(e); terminal='FAULT'
                    self.restore_result=dict(self.restore_result or {},ok=False,error=str(e))
            try: self.bus.close()
            except Exception as e: self.reason+='；连接清理失败：'+str(e); terminal='FAULT'
            self.state=terminal

    def status(self):
        result=super().status()
        result.update(control=dict(mode='native-csp-j3-v1',axis=3,delta_deg=.5,
            speed_limit_rad_s=.02,task_timeout_s=TASK_TIMEOUT,runtime_registers={hex(k):v for k,v in RUNTIME.items()},
            pid_source='live drive parameters, not overwritten',config_fingerprint=self.config.fingerprint),
            register_snapshot={hex(k):v for k,v in self.bus.original.items()},
            restoration=self.restore_result,arrival_feedback={str(i):asdict(s) for i,s in self.arrival_feedback.items()},
            timing=dict(enable_attempted_at=self.enable_attempted_at,power_deadline=self.power_deadline,
                stop_requested_at=self.stop_requested_at,stop_call_completed_at=self.stop_call_completed_at,
                source='host monotonic timestamps; not independently measured physical enable duration'))
        return result
