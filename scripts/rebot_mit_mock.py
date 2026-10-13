"""B1 deterministic six-axis MIT executor. Pure memory; no CAN/network adapter.

All limits/gains are mandatory synthetic fixtures, not robot recommendations.
Nothing here reads ConfigStore, supplies physical defaults, or changes a motor.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import threading

AXES = 6
UNITS = {'q': 'rad', 'qd': 'rad/s', 'a': 'rad/s^2', 'jerk': 'rad/s^3',
         'kp': 'Nm/rad', 'kd': 'Nm/(rad/s)', 'tau': 'Nm', 'time': 's'}


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def vector(value, name):
    if not isinstance(value, (tuple, list)) or len(value) != AXES or not all(map(finite, value)):
        raise ValueError(f'{name}: expected six finite numbers')
    return tuple(float(x) for x in value)


@dataclass(frozen=True)
class AxisLimits:
    q_min: float
    q_max: float
    v_max: float
    a_max: float
    jerk_max: float
    kp: float
    kd: float
    tau_ff: float
    kp_max: float
    kd_max: float
    torque_max: float
    error_max: float


@dataclass(frozen=True)
class Policy:
    period: float
    jitter_max: float
    feedback_age: float
    feedback_skew: float
    command_age: float
    ack_timeout: float
    stop_ack_timeout: float
    start_speed: float
    arrival_q: float
    arrival_qd: float
    plan_ttl: float
    task_timeout: float


@dataclass(frozen=True)
class MockConfig:
    version: str
    device: str
    axes: tuple
    policy: Policy

    @classmethod
    def parse(cls, raw):
        if raw.get('source') != 'synthetic-test-fixture' or raw.get('approval_scope') != 'mock-only':
            raise ValueError('Only explicitly synthetic mock-only fixtures are accepted')
        if raw.get('units') != UNITS or not isinstance(raw.get('version'), str) or not raw['version']:
            raise ValueError('Explicit units and version required')
        if not isinstance(raw.get('device'), str) or not raw['device'].startswith('virtual:'):
            raise ValueError('Virtual identity required')
        if not isinstance(raw.get('axes'), list) or len(raw['axes']) != AXES:
            raise ValueError('Exactly J1-J6; J7 is not supported')
        axes = []
        for data in raw['axes']:
            if set(data) != set(AxisLimits.__dataclass_fields__) or not all(map(finite, data.values())):
                raise ValueError('All limits and gains must be explicit finite numbers')
            a = AxisLimits(**data)
            if a.q_min >= a.q_max or min(a.v_max, a.a_max, a.jerk_max, a.torque_max, a.error_max) <= 0:
                raise ValueError('Invalid position/derivative/effort limits')
            if not (0 <= a.kp <= a.kp_max and 0 <= a.kd <= a.kd_max and abs(a.tau_ff) <= a.torque_max):
                raise ValueError('Gain or feedforward exceeds explicit mock envelope')
            axes.append(a)
        data = raw.get('policy', {})
        if set(data) != set(Policy.__dataclass_fields__) or not all(finite(x) and x > 0 for x in data.values()):
            raise ValueError('All time/tolerance limits must be explicit positive numbers')
        p = Policy(**data)
        if p.period >= p.command_age or p.ack_timeout >= p.command_age:
            raise ValueError('Command lifetime must exceed period and acknowledgement deadline')
        return cls(raw['version'], raw['device'], tuple(axes), p)

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True, allow_nan=False).encode()).hexdigest()


def shape(u):
    """Septic rest-to-rest polynomial, C3 at each waypoint, no angle wrapping."""
    if u <= 0:
        return 0., 0., 0., 0.
    if u >= 1:
        return 1., 0., 0., 0.
    return (35*u**4-84*u**5+70*u**6-20*u**7,
            140*u**3*(1-u)**3,
            420*u**2*(1-u)**2*(1-2*u),
            840*u-5040*u**2+8400*u**3-4200*u**4)


@dataclass(frozen=True)
class Trajectory:
    points: tuple
    durations: tuple

    @classmethod
    def create(cls, start, waypoints, config, *, axes=None, start_tolerance=0.):
        if not finite(start_tolerance) or not 0<=start_tolerance<=math.radians(1):
            raise ValueError('Explicit start tolerance must be between zero and one degree')
        points = (vector(start, 'start'),) + tuple(vector(p, 'waypoint') for p in waypoints)
        if len(points) < 2:
            raise ValueError('At least one waypoint required')
        selected = tuple(range(1, AXES+1)) if axes is None else tuple(axes)
        if not selected or len(set(selected)) != len(selected) or any(type(i) is not int or not 1 <= i <= AXES for i in selected):
            raise ValueError('Explicit valid controlled axes required')
        durations = []
        for index,q in enumerate(points):
            if any(q[i-1] != points[0][i-1] for i in range(1, AXES+1) if i not in selected):
                raise ValueError('Uncommanded axis must remain at its raw start')
            margin=start_tolerance if index==0 else 0.
            if any(not config.axes[i-1].q_min-margin <= q[i-1] <= config.axes[i-1].q_max+margin for i in selected):
                raise ValueError('Raw continuous position outside mock limits; no wrap/clip')
        for left, right in zip(points, points[1:]):
            required = config.policy.period
            for i in selected:
                x, y, a = left[i-1], right[i-1], config.axes[i-1]
                d = abs(y-x)
                # Exact extrema of the unit septic's first three derivatives.
                required = max(required, d*(35/16)/a.v_max,
                               math.sqrt(d*(84*math.sqrt(5)/25)/a.a_max),
                               (d*52.5/a.jerk_max)**(1/3))
            cycles = required/config.policy.period
            if not finite(cycles) or required+config.policy.command_age >= config.policy.task_timeout:
                raise ValueError('Trajectory duration is unrepresentable or exceeds task budget')
            duration = math.ceil(cycles)*config.policy.period
            cubic_time = duration*duration*duration
            if not finite(cubic_time) or cubic_time == 0:
                raise ValueError('Trajectory time scale is unrepresentable')
            durations.append(duration)
        if not finite(sum(durations)) or sum(durations) + config.policy.command_age >= config.policy.task_timeout:
            raise ValueError('Trajectory exceeds explicit task time budget')
        return cls(points, tuple(durations))

    @property
    def duration(self):
        return sum(self.durations)

    def sample(self, elapsed):
        remaining = max(0., elapsed)
        for i, duration in enumerate(self.durations):
            if remaining <= duration or i == len(self.durations)-1:
                if remaining >= duration:
                    return (self.points[i+1], (0.,)*AXES, (0.,)*AXES, (0.,)*AXES)
                derivatives = shape(remaining/duration)
                delta = tuple(y-x for x, y in zip(self.points[i], self.points[i+1]))
                return tuple(tuple((self.points[i][j] if k == 0 else 0.) + d*derivatives[k]/duration**k
                                   for j, d in enumerate(delta)) for k in range(4))
            remaining -= duration
        raise AssertionError('Empty trajectory')


@dataclass(frozen=True)
class Feedback:
    q: tuple
    qd: tuple
    sampled_at: tuple
    sequence: int
    device: str
    modes: tuple
    valid: tuple


@dataclass(frozen=True)
class RuntimeCommand:
    task: int
    sequence: int
    config_hash: str
    issued_at: float
    expires_at: float
    q_des: tuple
    qd_des: tuple
    kp: tuple
    kd: tuple
    tau_ff: tuple


@dataclass(frozen=True)
class Receipt:
    due_at: float | None
    ok: bool


@dataclass(frozen=True)
class Plan:
    token: int
    task_version: str
    config_hash: str
    feedback: Feedback
    created_at: float
    expires_at: float
    trajectory: Trajectory


class VirtualClock:
    def __init__(self):
        self.now = 0.

    def advance(self, seconds):
        self.now += seconds  # tests may deliberately inject clock faults


class HardwareDisabledTransport:
    def __init__(self, *args, **kwargs):
        raise PermissionError('LOCKED: B1 has no hardware transport')


class MockMotorTransport:
    """Ideal six-axis follower with explicit fault injection; not a physics model."""
    def __init__(self, clock, device, q):
        if type(clock) is not VirtualClock or not device.startswith('virtual:'):
            raise ValueError('Virtual clock/identity required')
        self.clock, self.device = clock, device
        self.q, self.qd = vector(q, 'initial'), (0.,)*AXES
        self.modes, self.valid = (0,)*AXES, (True,)*AXES
        self.connected = True
        self.follow = True
        self.fail_read = self.fail_write = self.fail_stop = False
        self.ack_delay = self.stop_ack_delay = 0.
        self.drop_ack = self.drop_stop_ack = False
        self.ack_ok = self.stop_ack_ok = True
        self.frozen_at = {}  # axis -> sample time; never affects other axes
        self.sequence = 0
        self.freeze_sequence = False
        self.on_read = None
        self.frames, self.stops = [], []

    def feedback(self):
        if not self.connected or self.fail_read:
            raise ConnectionError('feedback communication failure')
        if not self.freeze_sequence:
            self.sequence += 1
        result = Feedback(self.q, self.qd, tuple(self.frozen_at.get(i, self.clock.now) for i in range(AXES)),
                          self.sequence, self.device, self.modes, self.valid)
        if self.on_read:
            self.on_read()
        return result

    def send(self, frame):
        if not self.connected or self.fail_write:
            raise ConnectionError('motion communication failure')
        if self.clock.now > frame.expires_at:
            raise TimeoutError('expired frame')
        self.frames.append(frame)
        if self.follow:
            self.q, self.qd = frame.q_des, frame.qd_des
        return Receipt(None if self.drop_ack else self.clock.now+self.ack_delay, self.ack_ok)

    def stop(self):
        self.stops.append(self.clock.now)  # attempt is distinct from acknowledged stop
        if not self.connected or self.fail_stop:
            raise ConnectionError('stop communication failure')
        self.qd = (0.,)*AXES
        return Receipt(None if self.drop_stop_ack else self.clock.now+self.stop_ack_delay, self.stop_ack_ok)


class MockExecutor:
    """Tick from a virtual scheduler only. Stop preempts feedback and motion ACKs.

    Public status never grants hardware permission. Configuration updates must go
    through replace_config; every pending plan binds the complete fingerprint.
    """
    def __init__(self, config, transport, clock):
        if type(transport) is not MockMotorTransport or type(clock) is not VirtualClock or transport.clock is not clock:
            raise PermissionError('LOCKED: only the concrete in-memory transport is allowed')
        # Revalidate even directly constructed frozen dataclasses.
        config = MockConfig.parse({**asdict(config), 'axes': [asdict(a) for a in config.axes],
                                  'source': 'synthetic-test-fixture', 'approval_scope': 'mock-only', 'units': UNITS})
        self.config, self.transport, self.clock = config, transport, clock
        self.state, self.reason = 'READY_FOR_MOCK', None
        self.events, self.plans = [], {}
        self._counter = 0
        self._lock, self._cancel = threading.RLock(), threading.Event()
        self._last_feedback = None
        self._last_clock = clock.now
        self._last_tick = None
        self._last_frame = None
        self._pending = None
        self._stop_receipt = None
        self._plan = None
        self._stop_reason = None
        self._stop_at = None
        self._stop_sent = self._stop_acknowledged = False

    def status(self):
        return dict(state=self.state, reason=self.reason, hardware=False, hardware_state='LOCKED',
                    source='synthetic-test-fixture', physical_stop_verified=False,
                    stop_requested=self._stop_at is not None,
                    stop_sent=self._stop_sent,
                    stop_acknowledged=self._stop_acknowledged,
                    motion_frames=len(self.transport.frames))

    def _event(self, kind, **data):
        self.events.append(dict(time=self.clock.now if finite(self.clock.now) else None, kind=kind, **data))

    def _time(self):
        now = self.clock.now
        if not finite(now) or now < self._last_clock:
            raise ValueError('clock invalid or reversed')
        self._last_clock = now
        return now

    def _feedback(self, now):
        f = self.transport.feedback()
        vector(f.q, 'feedback q'); vector(f.qd, 'feedback qd'); vector(f.sampled_at, 'sample timestamps')
        if f.device != self.config.device or f.modes != (0,)*AXES or f.valid != (True,)*AXES:
            raise ValueError('feedback identity/mode/validity mismatch')
        if type(f.sequence) is not int or f.sequence <= 0:
            raise ValueError('feedback sequence invalid')
        if any(t > now or now-t > self.config.policy.feedback_age for t in f.sampled_at):
            raise TimeoutError('feedback stale/future on at least one axis')
        if max(f.sampled_at)-min(f.sampled_at) > self.config.policy.feedback_skew:
            raise ValueError('six-axis feedback sampling skew exceeded')
        if self._last_feedback and (f.sequence <= self._last_feedback.sequence or
                                  any(t < old for t, old in zip(f.sampled_at, self._last_feedback.sampled_at))):
            raise ValueError('feedback sequence/time reversed')
        if any(not a.q_min <= q <= a.q_max or abs(v) > a.v_max for q, v, a in zip(f.q, f.qd, self.config.axes)):
            raise ValueError('feedback position/speed outside envelope')
        self._last_feedback = f
        return f

    def prepare(self, waypoints, task_version):
        with self._lock:
            if self.state not in ('READY_FOR_MOCK', 'PREVIEWED') or not isinstance(task_version, str) or not task_version:
                raise ValueError('Explicit fresh task required; executor is busy or needs reset')
            self.plans.clear()
            self.state = 'READY_FOR_MOCK'  # failed replacement must not leave an old preview executable
            now = self._time()
            f = self._feedback(now)
            if any(abs(v) > self.config.policy.start_speed for v in f.qd):
                raise ValueError('Only a stationary mock start is supported')
            trajectory = Trajectory.create(f.q, waypoints, self.config)
            self._counter += 1
            p = Plan(self._counter, task_version, self.config.fingerprint, f, now,
                     now+self.config.policy.plan_ttl, trajectory)
            self.plans = {p.token: p}
            self.state = 'PREVIEWED'
            self._event('previewed', token=p.token)
            return p

    def start(self, plan, task_version):
        with self._lock:
            if self.state != 'PREVIEWED' or self.plans.pop(plan.token, None) is not plan:
                raise ValueError('Plan unknown/consumed or executor busy')
            self.state = 'READY_FOR_MOCK'  # a failed start consumes the plan, no fallback
            now = self._time()
            if self._cancel.is_set() or not plan.created_at <= now < plan.expires_at:
                raise ValueError('Plan expired or cancellation pending')
            if plan.config_hash != self.config.fingerprint or plan.task_version != task_version:
                raise ValueError('Plan configuration/task version changed')
            f = self._feedback(now)
            if f.q != plan.feedback.q or any(abs(v) > self.config.policy.start_speed for v in f.qd):
                raise ValueError('Start moved; replan from new raw feedback')
            self._plan, self._start_at = plan, now
            self._last_tick = self._last_frame = self._pending = None
            self.state = 'RUNNING_MOCK'
            self._event('started', token=plan.token)
            # First target is generated synchronously from fresh feedback, never restored.
            self._cycle(now, f)

    def request_cancel(self):
        self._cancel.set()  # non-blocking; checked before reads, ACKs and every send

    def replace_config(self, config):
        with self._lock:
            if self.state in ('RUNNING_MOCK', 'STOP_REQUESTED'):
                self._stop('configuration changed during execution')
                raise ValueError('Stop and reset before configuration change')
            # Constructing a validator prevents bypass via directly created dataclasses.
            checked = MockExecutor(config, self.transport, self.clock).config
            self.config = checked
            self.plans.clear()
            if self.state == 'PREVIEWED':
                self.state = 'READY_FOR_MOCK'

    def _stop(self, reason):
        if self.state == 'STOP_REQUESTED':
            return
        self.state, self.reason = 'STOP_REQUESTED', reason
        self.plans.clear()
        self._pending = None
        self._stop_reason, self._stop_at = reason, self.clock.now
        self._event('stop_requested', reason=reason)
        try:
            self._stop_receipt = self.transport.stop()
            self._stop_sent = True
            self._event('stop_sent')
        except (ConnectionError, ValueError, TimeoutError) as exc:
            self.state = 'FAULT'
            self.reason = f'{reason}; stop send failed: {exc}'
            self._event('stop_failed', reason=self.reason)
            return
        self._poll_stop()

    def _poll_stop(self):
        now, receipt = self.clock.now, self._stop_receipt
        if not finite(now) or now < self._stop_at:
            self.state, self.reason = 'FAULT', f'{self._stop_reason}; stop clock invalid'
        elif now-self._stop_at > self.config.policy.stop_ack_timeout:
            self.state, self.reason = 'FAULT', f'{self._stop_reason}; stop acknowledgement timeout'
        elif receipt and receipt.due_at is not None and now >= receipt.due_at:
            if receipt.ok:
                self._stop_acknowledged = True
                self._event('stop_acknowledged', physical_stop_verified=False)
                self.state = 'STOPPED' if self._stop_reason == 'cancelled' else 'FAULT'
            else:
                self.state, self.reason = 'FAULT', f'{self._stop_reason}; stop negative acknowledgement'
        if self.state == 'FAULT':
            self._event('fault', reason=self.reason)

    def tick(self):
        with self._lock:
            if self.state == 'STOP_REQUESTED':
                self._poll_stop()
                return
            if self.state != 'RUNNING_MOCK':
                return
            # Cancellation must not wait for missing motor feedback or motion ACKs.
            if self._cancel.is_set():
                self._stop('cancelled')
                return
            try:
                now = self._time()
                p = self.config.policy
                if self._last_tick is not None and now-self._last_tick > p.period+p.jitter_max+1e-10:
                    raise TimeoutError('scheduler cycle overrun')
                if self._last_tick is not None and now-self._last_tick < p.period-1e-10:
                    raise ValueError('scheduler cycle too early')
                self._last_tick = now
                if now-self._start_at > p.task_timeout:
                    raise TimeoutError('task timeout')
                if self._plan.config_hash != self.config.fingerprint:
                    raise ValueError('configuration changed')
                if self._last_frame and now > self._last_frame.expires_at:
                    raise TimeoutError('command expired')
                f = self._feedback(now)
                if self._cancel.is_set():
                    self._stop('cancelled')
                    return
                if self._pending:
                    receipt, sent = self._pending
                    if now-sent > p.ack_timeout+1e-10:
                        raise TimeoutError('motion acknowledgement timeout')
                    if receipt.due_at is None or now < receipt.due_at:
                        return
                    if not receipt.ok:
                        raise ConnectionError('motion negative acknowledgement')
                    self._pending = None
                    self._event('motion_acknowledged', sequence=self._last_frame.sequence)
                self._cycle(now, f)
            except (ValueError, TimeoutError, ConnectionError) as exc:
                self._stop(str(exc))

    def _cycle(self, now, f):
        if self._cancel.is_set():
            self._stop('cancelled')
            return
        try:
            if self._last_frame and any(abs(x-y) > a.error_max for x, y, a in zip(self._last_frame.q_des, f.q, self.config.axes)):
                raise ValueError('tracking error exceeded')
            end_sent = self._last_frame and self._last_frame.issued_at-self._start_at >= self._plan.trajectory.duration
            if end_sent:
                if (all(abs(x-y) <= self.config.policy.arrival_q for x, y in zip(f.q, self._plan.trajectory.points[-1]))
                    and all(abs(v) <= self.config.policy.arrival_qd for v in f.qd)):
                    self.state = 'COMPLETED'
                    self._event('arrival_confirmed', physical_stop_verified=False)
                    return
                # Keep final target valid until feedback confirms arrival or task times out.
            q, v, acceleration, jerk = self._plan.trajectory.sample(now-self._start_at)
            for x, y, z, j, actual, velocity, a in zip(q, v, acceleration, jerk, f.q, f.qd, self.config.axes):
                if not (a.q_min <= x <= a.q_max and abs(y) <= a.v_max+1e-9 and abs(z) <= a.a_max+1e-9 and abs(j) <= a.jerk_max+1e-9):
                    raise ValueError('trajectory envelope exceeded')
                effort = a.kp*(x-actual)+a.kd*(y-velocity)+a.tau_ff
                if not finite(effort) or abs(effort) > a.torque_max:
                    raise ValueError('estimated MIT effort exceeds envelope; no silent clipping')
            if self._last_frame and now-self._last_frame.issued_at > self.config.policy.period+self.config.policy.jitter_max+1e-10:
                raise TimeoutError('command cadence overrun; no catch-up burst')
            frame = RuntimeCommand(self._plan.token, 1 if not self._last_frame else self._last_frame.sequence+1,
                                   self.config.fingerprint, now, now+self.config.policy.command_age, q, v,
                                   tuple(a.kp for a in self.config.axes), tuple(a.kd for a in self.config.axes),
                                   tuple(a.tau_ff for a in self.config.axes))
            if self._cancel.is_set():
                self._stop('cancelled')
                return
            receipt = self.transport.send(frame)
            self._last_frame, self._pending = frame, (receipt, now)
            if self._last_tick is None:
                self._last_tick = now
            self._event('motion_sent', sequence=frame.sequence)
        except (ValueError, TimeoutError, ConnectionError) as exc:
            self._stop(str(exc))

    def reset(self):
        with self._lock:
            if self.state in ('RUNNING_MOCK', 'STOP_REQUESTED'):
                raise ValueError('Cannot reset an active task')
            self.plans.clear()
            self._cancel.clear()
            self._last_frame = self._pending = self._plan = self._stop_receipt = None
            self._last_feedback = self._last_tick = self._stop_at = self._stop_reason = None
            self._stop_sent = self._stop_acknowledged = False
            self.state, self.reason = 'READY_FOR_MOCK', None
            self._event('manual_mock_reset')
