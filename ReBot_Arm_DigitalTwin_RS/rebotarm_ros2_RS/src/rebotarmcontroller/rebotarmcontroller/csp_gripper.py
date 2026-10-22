"""Explicit J7 scope on the existing CAN worker; no second controller or PID."""
from dataclasses import asdict, replace
import json
import math
import os
import time

from .csp_runtime.rebot_mit_hardware import Envelope, OnsitePermit, Sample
from .csp_runtime.rebot_native_csp import CspTransport, PROTECTED
from .twin_program import number
from .twin_motion import RuckigVelocity
from .twin_grasp import GraspMonitor, MAX_REFERENCE_LEAD


# Keep commands inside the nominal endpoints so encoder noise does not cross
# the unchanged strict 0–5 rad feedback guard. This is below the 0.1° arrival
# tolerance; the requested endpoint and actual command remain distinguishable.
ENDPOINT_MARGIN = .001


class GripperPermit(OnsitePermit):
    def allowed_axes(self, config):
        return (7,) if config.version == 'native-csp-twin-j7-v2-official' else ()


class CspGripper:
    def __init__(self, hardware):
        self.hw = hardware
        self.clock = hardware.clock
        self.view = None
        self.sample = None
        self.request_probe = False
        self.authorized = False
        self.preparing = False
        self.reading = False
        self.pending = None
        self.manual = None
        self.manual_velocity = 0.
        self.manual_ramp = RuckigVelocity(1)
        self.curve = None
        self.pending_grasp = None
        self.grasp = None
        self._grasp_observation = None
        self.reference = None
        self.arrived = False
        self.error = ''
        self.last_poll = -10.
        self.last_step = -10.
        self.speed_max = 0.
        self.history = []
        self.journal = hardware.output / 'csp-gripper-state.json' if hardware.output else None
        self.recovery = None
        if self.journal and self.journal.exists():
            try:
                row = json.loads(self.journal.read_text())
                if row.get('pending'):self.recovery = row
            except (ValueError, OSError):
                self.recovery = {'pending': True, 'error': 'J7恢复记录不可读'}

    def feedback_time(self):
        s = self.sample
        return min(s.requested_at, s.mode_requested_at if s.mode_requested_at is not None else s.requested_at,
                   s.diagnostics_requested_at if s.diagnostics_requested_at is not None else s.requested_at) if s else None

    def state(self):
        stamp = self.feedback_time()
        fresh = stamp is not None and 0 <= self.clock()-stamp <= .6
        initialized = bool(self.view and self.view.active and self.reference is not None and not self.preparing)
        return dict(available=bool(self.sample and self.speed_max > 0), fresh=fresh, authorized=self.authorized,
                    position=self.sample.q if self.sample else None, velocity=self.sample.qd if self.sample else None,
                    current_a=self.sample.iq_raw_a if self.sample else None, mode=self.sample.mode if self.sample else None,
                    active=initialized, preparing=self.preparing, target=self.curve['goal'] if self.curve else self.reference,
                    arrived=self.arrived, error=self.error, recovery_required=bool(self.recovery and not initialized and not self.preparing),
                    speed_max=self.speed_max, official_speed_max=5., range_rad=[0., 5.],
                    command_reference=self.reference, endpoint_margin_rad=ENDPOINT_MARGIN,
                    sample_at=self.sample.requested_at if self.sample else None,
                    grasp_supported=True, grasp=self.grasp.state() if self.grasp else {},
                    source='Seeed RS ID7 rs-00; motor radians, not jaw millimeters', calibrated=False)

    def _motor(self):
        bus = self.hw.bus
        bus.ownership.check()
        if 7 not in bus.motors:
            bus.motors[7] = bus.controller.add_robstride_motor(7, 0xFD, 'rs-00')
        return bus.motors[7]

    def _query(self, index, read):
        requested = self.clock()
        try:return read()
        finally:
            received = self.clock();io = getattr(self.hw.bus, 'poll_io', None)
            if io is not None:
                previous = io[-1] if io else None
                io.append(dict(axis=7,index=hex(index),requested_at=requested,received_at=received,
                    duration_ms=round((received-requested)*1000,3),
                    between_queries_ms=round(max(0.,requested-previous['received_at'])*1000,3) if previous else None))

    def read_field(self, check=lambda: None):
        """One oldest field on the shared CAN owner; retain all other timestamps."""
        s = self.sample
        if s is None or not (self.view and self.view.active) and self.clock()-s.requested_at > .6:
            # Arm preparation can leave an inactive J7 unpolled for seconds.
            # Rebuild that read-only sample with the original full read; never
            # bridge a long gap with differentiated velocity on an active J7.
            return self._read(check)
        fields = ((s.requested_at,0x7019),
                  (s.mode_requested_at if s.mode_requested_at is not None else s.requested_at,0x7005),
                  (s.diagnostics_requested_at if s.diagnostics_requested_at is not None else s.requested_at,0x701A))
        _,index = min(fields,key=lambda row:row[0])
        m = self._motor();check();requested = self.clock()
        self.reading = True
        try:
            value = self._query(index,lambda:m.robstride_get_param_i8(index,50) if index == 0x7005 else
                                m.robstride_get_param_f32_host_id(index,0xFD,50))
            received = self.clock()
            if index == 0x7005:
                if type(value) is not int:raise TimeoutError('J7模式反馈无效')
                s = replace(s,mode=value,mode_requested_at=requested)
            elif not math.isfinite(value):raise TimeoutError('J7反馈无效')
            elif index == 0x701A:
                s = replace(s,iq_raw_a=value,diagnostics_requested_at=requested,diagnostics_received_at=received)
            else:
                dt = (requested+received-s.requested_at-s.received_at)/2
                if not 0 < dt <= .6:raise TimeoutError('J7位置差分窗口无效或过期')
                s = replace(s,q=value,qd=(value-s.q)/dt,requested_at=requested,received_at=received)
            self.sample = s;self.hw.bus.latest_samples[7] = s
            check()
        finally:self.reading = False
        return s

    def _read(self, check=lambda: None):
        m = self._motor();check();began = self.clock()
        self.reading = True
        try:
            q = self._query(0x7019,lambda:m.robstride_get_param_f32_host_id(0x7019, 0xFD, 50))
            q_received = self.clock();check()
            mode_at = self.clock()
            mode = self._query(0x7005,lambda:m.robstride_get_param_i8(0x7005, 50));check()
            current_at = self.clock()
            current = self._query(0x701A,lambda:m.robstride_get_param_f32_host_id(0x701A, 0xFD, 50))
            current_received = self.clock();check()
        finally:self.reading = False
        if not all(math.isfinite(x) for x in (q, current)) or type(mode) is not int:
            raise TimeoutError('J7反馈无效')
        old = self.sample;dt = began-old.requested_at if old else 0
        qd = (q-old.q)/dt if old and 0 < dt <= .6 else 0.
        self.sample = Sample(7, q, qd, mode, began, q_received, iq_raw_a=current,
                             diagnostics_requested_at=current_at, diagnostics_received_at=current_received,
                             mode_requested_at=mode_at)
        self.hw.bus.latest_samples[7] = self.sample
        return self.sample

    def _save(self, pending, error=''):
        if self.journal:
            self.journal.parent.mkdir(parents=True, exist_ok=True)
            row = dict(version='native-csp-twin-j7-v2-official', pending=pending, error=error,
                       original={hex(k): v for k, v in (self.view.original if self.view else {}).items()},
                       runtime={hex(k): v for k, v in (self.view.runtime if self.view else {}).items()},
                       register_log=self.view.register_log[-2000:] if self.view else [], at=time.time())
            # Retain the recovery lock if any durable-write step fails.
            if pending:self.recovery = row
            temp = self.journal.with_suffix('.tmp')
            with temp.open('w') as stream:
                json.dump(row, stream, ensure_ascii=False, indent=2)
                stream.flush();os.fsync(stream.fileno())
            temp.replace(self.journal)
            directory_fd = os.open(self.journal.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:os.fsync(directory_fd)
            finally:os.close(directory_fd)
            self.recovery = row if pending else None

    def _new_view(self, runtime):
        h = self.hw
        v = CspTransport(axis=7, runtime=runtime, include_gripper=True, ownership=h.bus.ownership,
                         factory=h.bus.factory, clock=self.clock)
        v.controller = h.bus.controller;v.motors = h.bus.motors;v.latest_samples = h.bus.latest_samples
        return v

    def _start(self, check):
        if self.recovery:raise RuntimeError('J7上次参数恢复未确认，请重置后再控制')
        self.preparing = True
        try:self._prepare_start(check)
        finally:self.preparing = False

    def _prepare_start(self, check):
        self.reference = None;self.history = []
        s = self._read(check)
        if s.mode != 0 or not 0 <= s.q <= 5 or abs(s.qd) > .01:
            raise ValueError('J7起始模式、静止状态或0–5rad范围不符，未使能')
        def startup_check():
            check()
            if not self.authorized:raise RuntimeError('J7启动授权已撤销，禁止继续配置或使能')
            if self.view and self.view.permit:self.view.permit.expires_at = self.clock()+2
            # Snapshot/configuration can outlast one J7 feedback window.
            # Refresh on this CAN owner; _read receives the outer check so
            # this startup-only helper never calls itself recursively.
            if self.clock()-self.sample.requested_at >= .25:
                self._read(check)
        runtime = {0x7005: 5, 0x7017: self.speed_max, 0x7018: 16., 0x700B: 14.}
        # Preserve lower original current/torque limits as well as speed.
        m = self._motor()
        for index in (0x7018, 0x700B):
            original = m.robstride_get_param_f32_host_id(index, 0xFD, 50);startup_check()
            if not math.isfinite(original) or original <= 0:raise ValueError('J7原始出力限制无效')
            runtime[index] = min(runtime[index], original)
        self.view = self._new_view(runtime)
        axis = replace(self.hw.config.axes[5], q_min=0., q_max=5., v_max=5., torque_max=runtime[0x700B])
        config = Envelope('native-csp-twin-j7-v2-official', (*self.hw.config.axes, axis), self.hw.config.policy)
        permit = GripperPermit(config.fingerprint, (7,), self.clock()+2, '本页面显式授权J7', (7,), True, True)
        self.view.authorize(config, permit)
        self.view.snapshot_registers(startup_check)
        self._save(True)  # durable lineage before disable, mode or enable
        self.view.quiesce();startup_check()
        for index, value in runtime.items():self.view.write_register(index, value, startup_check, phase='prepare')
        s = self._read(check)
        if abs(s.qd) > .01:raise ValueError('J7准备期间移动，取消使能')
        self.view.write_register(0x7016, s.q, startup_check, phase='preload-current')
        self.view.enable(7)
        self.reference = s.q;self.history = [(self.clock(), s.q)]

    def pause(self):
        self.pending = self.curve = self.manual = None;self.arrived = False
        self.pending_grasp = None
        if self.grasp and self.grasp.status not in ('holding', 'failed', 'cancelled'):self.grasp.cancel()
        self.manual_velocity = 0.
        self.manual_ramp.reset()

    def observe_hold(self, check):
        if not self.view or not self.view.active:return
        if self.reference is None:
            raise TimeoutError('J7启动未完成，不能确认夹爪保持；请支撑后失能并核对恢复')
        s = self._read(check)
        if s.mode != 5 or not 0 <= s.q <= 5 or abs(s.qd) > 5 or abs(s.iq_raw_a) > self.view.runtime[0x7018]:
            raise ValueError('J7保持期间模式、限位、速度或电流不符')
        if self.reference is not None and abs(s.q-self.reference) > math.radians(3):
            raise ValueError('J7保持位置误差超过3°')
        self._observe_grasp(s)

    def _observe_grasp(self, sample):
        if not self.grasp:return
        times = (sample.requested_at,
                 sample.mode_requested_at if sample.mode_requested_at is not None else sample.requested_at,
                 sample.diagnostics_requested_at if sample.diagnostics_requested_at is not None else sample.requested_at)
        previous_observation = self._grasp_observation
        if previous_observation and previous_observation[0] is self.grasp and any(
                new <= old for new,old in zip(times,previous_observation[1])):return
        self._grasp_observation = (self.grasp,times)
        previous = self.grasp.status
        # A new current/mode reply alone cannot create another contact sample.
        # The minimum timestamp advances only after all three fields refresh.
        self.grasp.update(position=sample.q, velocity=sample.qd, current=sample.iq_raw_a,
                          stamp=min(times), now=self.clock(), reference=self.reference)
        if previous != 'failed' and self.grasp.status == 'failed' and getattr(self.hw, 'session', ''):
            # Freeze arm references on this CAN owner before the ROS/UI tick.
            self.hw.teleop_pause(self.hw.session)

    def resume_hold(self):
        if self.view and self.view.active:
            if self.reference is None:raise RuntimeError('J7启动未完成，禁止恢复夹爪控制')
            self.view.permit.expires_at = self.clock()+2
            self.history = [(self.clock(),self.reference)]
            # The CAN owner has just revalidated hold feedback and accepted a
            # new lease. The prior interruption is no longer an active error.
            self.error = ''

    def step(self, check=lambda: None, *, poll=True, one_field=False):
        if not (self.sample or self.request_probe or self.pending or self.pending_grasp or self.view and self.view.active):return
        active = bool(self.view and self.view.active)
        interval = .18 if active else .09 if self.pending or self.pending_grasp or self.manual else .25
        read_due = poll and (one_field or self.sample is None or self.request_probe or self.clock()-self.last_poll >= interval)
        if not read_due and not (active and self.clock()-self.last_step >= .02):return
        dt = min(.1, max(0., self.clock()-self.last_step))
        try:
            probing = bool(read_due and self.request_probe)
            check()
            if read_due:
                self.request_probe = False;self.last_poll = self.clock()
                s = self.read_field(check) if one_field else self._read(check)
            else:
                s = self.sample  # Real timestamp retained; the old 600 ms guard still applies.
                if s is None:raise TimeoutError('J7缺少反馈样本，禁止发送目标')
            if probing:
                limit = self._motor().robstride_get_param_f32_host_id(0x7017, 0xFD, 50);check()
                self.speed_max = min(5., number(limit, 'J7设备限速', .005, 100))
                if not self.recovery:self.error = ''
            if self.manual and self.clock()-self.manual['at'] > .25:
                self.manual = None;self.manual_velocity = 0.;self.manual_ramp.reset()
            if self.pending or self.pending_grasp or self.manual:
                if not self.authorized or not self.hw.motion_ready:raise RuntimeError('J7需要有效租约、单独授权及机械臂已使能保持')
                if not self.view or not self.view.active:
                    self._start(check)
                    s = self._read(check)  # Read the enabled mode after preparation.
                if self.pending_grasp:
                    job = self.pending_grasp;self.pending_grasp = None
                    profile = job['profile']
                    if profile['abort_current_a'] > self.view.runtime[0x7018]:
                        self.grasp.fail('夹持中止电流超过本次保留的设备限流，未开始闭合')
                    else:
                        try:
                            self.grasp = GraspMonitor(profile, s.q, self.clock(), job['id'])
                            self.pending = (profile['empty_position'], job['speed'])
                        except ValueError as exc:
                            self.grasp.fail(str(exc))
                if self.pending:
                    target, speed = self.pending;self.pending = None
                    delta = abs(target-self.reference)
                    duration = max(.5, 1.875*delta/speed, math.sqrt(5.78*delta/4), (60*delta/30)**(1/3))
                    self.manual_ramp.reset()
                    self.curve = dict(start=self.reference, goal=target, duration=duration, elapsed=0.)
                self.arrived = False
                # Startup/register reads may outlast the original deadman input.
                # Only a newly refreshed input can move the newly enabled J7.
                if self.manual and self.clock()-self.manual['at'] > .25:
                    self.manual = None;self.manual_velocity = 0.;self.manual_ramp.reset()
            if self.view and self.view.active:
                if self.reference is None:raise RuntimeError('J7启动未完成，禁止下发夹爪目标')
                if not self.authorized:raise RuntimeError('J7控制许可已撤回')
                if self.clock()-self.feedback_time() > .6 or self.history and self.clock()-self.history[-1][0] > self.hw.config.policy.command_age:
                    raise TimeoutError('J7反馈或命令时限已过，停止控制')
                self.view.permit.expires_at = self.clock()+2
                if s.mode != 5 or not 0 <= s.q <= 5 or abs(s.qd) > 5 or abs(s.iq_raw_a) > self.view.runtime[0x7018]:
                    raise ValueError('J7运行模式、限位、速度或电流反馈不符')
                previous = next((q for stamp, q in reversed(self.history) if stamp < s.requested_at),
                                self.history[0][1] if self.history else self.reference)
                if abs(previous-s.q) > math.radians(3):raise ValueError('J7跟踪误差超过3°')
                self._observe_grasp(s)
                grasp_closing = self.grasp and self.grasp.status == 'closing'
                if self.grasp and not grasp_closing:self.curve = None
                if self.curve:
                    p = self.curve
                    p['elapsed'] += dt  # Never catch up a delayed SDK interval.
                    t = min(1., p['elapsed']/p['duration'])
                    self.reference = p['start']+(p['goal']-p['start'])*t**3*(10-15*t+6*t*t)
                    if grasp_closing:
                        self.reference = max(self.reference, s.q-MAX_REFERENCE_LEAD)
                    self.arrived = t >= 1 and abs(s.q-p['goal']) <= math.radians(.1)
                elif self.manual:
                    # Fresh deadman input only; never integrate a delayed gap.
                    desired = self.manual['velocity']
                    try:
                        # The shared generator supports one axis too. Substeps
                        # preserve its 50 ms integration cap without dropping
                        # half of the normal ~90 ms gripper cycle.
                        count=max(1,math.ceil(dt/.05))
                        candidate=self.reference
                        for _ in range(count):
                            candidate=float(self.manual_ramp.step([candidate],[desired],dt/count,
                                self.speed_max,4.,[0.],[5.],jerk=30.)[0])
                        self.reference=candidate
                        self.manual_velocity=float(self.manual_ramp.velocity[0])
                    except (RuntimeError,ValueError,TypeError) as exc:
                        raise TimeoutError(f'J7平滑参考计算异常，保留原目标：{exc}') from exc
                if self.curve or self.manual:
                    self.reference = min(5.-ENDPOINT_MARGIN,max(ENDPOINT_MARGIN,self.reference))
                self.view.write_register(0x7016, self.reference, check, phase='trajectory', verify=False)
                self.last_step = self.clock()
                self.history.append((self.clock(), self.reference));self.history = self.history[-200:]
        except Exception as exc:
            self.error = str(exc);self.pending = self.curve = self.manual = None
            if self.view and self.view.active:
                raise  # arm worker's existing finally stops both scopes

    def finish(self, *, recover=False):
        self.authorized = False;self.pause()
        if self.grasp:self.grasp.cancel()
        if not self.view and recover and self.recovery:
            row = self.recovery
            if row.get('version') != 'native-csp-twin-j7-v2-official':raise ValueError('J7恢复记录版本不符')
            original = {int(k, 16): v for k, v in row.get('original', {}).items()}
            runtime = {int(k, 16): v for k, v in row.get('runtime', {}).items()}
            if set(original) != {0x7005, 0x7017, 0x7018, 0x700B, *PROTECTED} or set(runtime) != {0x7005, 0x7017, 0x7018, 0x700B}:
                raise ValueError('J7缺少完整真实快照，禁止覆盖参数')
            if original[0x7005] != 0 or runtime[0x7005] != 5 or not all(math.isfinite(x) for x in original.values()):
                raise ValueError('J7恢复快照无效')
            self._motor();self.view = self._new_view(runtime);self.view.original = original
            self.view.changed = set(runtime)  # target is intentionally never recovered
        v = self.view
        if not v or not v.changed and not v.active:return
        try:
            # Explicit disable even after a process restart with empty active.
            self._motor().disable();v.disabled_status();v.active.clear()
            deadline = self.clock()+2;quiet = None
            while self.clock() < deadline:
                s = self._read();now = self.clock()
                quiet = (quiet if quiet is not None else now) if abs(s.qd) <= .01 else None
                if quiet is not None and now-quiet >= .3:break
                time.sleep(.02)
            else:raise RuntimeError('J7失能后未连续确认停稳')
            for index in (*v.runtime, *PROTECTED):
                current = v.read_register(index)
                allowed = [v.original[index]] + ([v.runtime[index]] if index in v.runtime else [])
                if not any(math.isclose(current, value, rel_tol=1e-6, abs_tol=1e-6) for value in allowed):
                    raise ValueError('J7当前参数不属于原始/本次临时值，禁止恢复')
            v.restoration_authorized = True
            if 0x7016 in v.changed:v.write_register(0x7016, s.q, lambda: None, phase='disabled-current')
            for index in (0x7005, 0x7017, 0x7018, 0x700B):
                if index in v.changed:v.write_register(index, v.original[index], lambda: None, phase='restore')
            self._read();self._save(False);self.view = None;self.error = ''
            self.reference = None;self.history = []
        except Exception as exc:
            self.error = str(exc);self._save(True, self.error);raise
