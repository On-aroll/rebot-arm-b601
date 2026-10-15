"""Fixed J7 motor-angle trial, sharing the validated CSP execution lifecycle.

Seeed's pinned RS mapping is authoritative for ID7/rs-00. The upstream 0..5
motor-radian endpoints are a software guard, NOT measured jaw/TCP calibration.
No official group enable, mode shortcut, PID write, homing or flash save.
"""
from dataclasses import asdict
import math

from rebot_mit_hardware import Envelope, OnsitePermit
from rebot_mit_mock import AxisLimits, Trajectory, finite
from rebot_native_csp import CspAxisTrial, CspTransport, DELTA, proposal as arm_proposal

VERSION = 'native-csp-j7-v1-15s'
RUNTIME = {0x7005: 5, 0x7017: .02, 0x7018: 10., 0x700B: .2}
AXES = tuple(range(1, 8))
GRIPPER_LIMIT = AxisLimits(0., 5., .02, .04, .2, 3., 1., 0., 3., 1., .2, math.radians(.6))


class GripperEnvelope(Envelope):
    @classmethod
    def parse(cls, raw):
        if raw.get('version') != VERSION or len(raw.get('axes', [])) != 7:
            raise ValueError('J7 需要独立且明确的七轴只读/单夹爪控制方案')
        if raw['axes'][6] != asdict(GRIPPER_LIMIT):
            raise ValueError('J7 仅支持固定低速小幅提案；不自动改变边界或力矩')
        arm = Envelope.parse(dict(raw, axes=raw['axes'][:6]))
        if arm.feedforward or arm.gain_review:
            raise ValueError('J7 不使用六轴重力前馈或 MIT 增益方案')
        return cls(arm.version, arm.axes + (GRIPPER_LIMIT,), arm.policy,
                   arm.start_boundary_tolerance)


def runtime_for_axis(axis):
    if type(axis) is not int or axis != 7:
        raise ValueError('夹爪入口只能控制明确的 J7')
    return dict(RUNTIME)


def proposal(limits, *, axis=7):
    runtime_for_axis(axis)
    if len(limits) not in (6, 7) or len(limits) == 7 and tuple(limits[6]) != (0., 5.):
        raise ValueError('需要原六轴限位及独立 RS 夹爪软件边界')
    raw, public = arm_proposal(limits[:6], axis=6)
    raw['version'] = VERSION
    raw['axes'].append(asdict(GRIPPER_LIMIT))
    GripperEnvelope.parse(raw)
    public.update(title='J7 夹爪电机轴 CSP +0.5° 小幅试验', axes=[7],
        summary='只控制 J7 电机轴增加 0.5°，最高 0.02 rad/s，首次使能起最多15秒；不是夹爪开口角或毫米距离。',
        runtime_registers={hex(k): v for k, v in RUNTIME.items()},
        gripper_range_source='锁定 Seeed ROS2 pick_place_rs.yaml：电机位置0..5 rad；不是本机夹爪端点/开口标定',
        gripper_range_calibrated=False,
        arm_model_scope='仅检查六轴支撑姿态；本试验不推断夹爪开口或夹持力')
    return raw, public


class GripperPermit(OnsitePermit):
    def allowed_axes(self, config):
        if type(config) is not GripperEnvelope or config.version != VERSION:
            return ()
        return (7,)


class GripperIntent:
    def __init__(self, waypoints, source, request_id, expected_start):
        if source != 'native-csp-j7' or not request_id:
            raise ValueError('J7 需要明确的一次性单夹爪意图')
        if len(expected_start) != 7 or not all(map(finite, expected_start)):
            raise ValueError('J7 必须绑定七轴新鲜原始起点')
        if len(waypoints) != 1 or len(waypoints[0]) != 7 or not all(map(finite, waypoints[0])):
            raise ValueError('J7 仅支持一次七轴反馈向量中的单夹爪目标')
        self.waypoints = (tuple(waypoints[0]),)
        self.expected_start = tuple(expected_start)
        self.source, self.request_id = source, request_id


class GripperTrajectory:
    """Reuse the existing septic evaluator on an explicit scalar gripper axis.

    The scalar evaluator uses an isolated configuration, never maps a J7 command
    to a physical arm ID. Returned state vectors retain actual IDs 1..7.
    """
    def __init__(self, start, scalar):
        self.start, self.scalar = tuple(start), scalar
        self.duration = scalar.duration

    @classmethod
    def create(cls, start, waypoints, config, *, axes):
        if axes != (7,) or len(start) != 7 or len(waypoints) != 1 or len(waypoints[0]) != 7:
            raise ValueError('夹爪轨迹只允许七轴状态中的 J7')
        target = tuple(waypoints[0])
        if not all(map(finite, (*start, *target))) or tuple(start[:6]) != target[:6]:
            raise ValueError('夹爪轨迹不允许修改六轴目标')
        # This is a pure mathematical scalar evaluator; it cannot access CAN.
        scalar_config = Envelope(config.version, (config.axes[6],) * 6, config.policy)
        scalar = Trajectory.create([start[6]] * 6, [[target[6]] * 6], scalar_config)
        return cls(start, scalar)

    def sample(self, elapsed):
        state = self.scalar.sample(elapsed)
        return tuple((self.start[:6] if order == 0 else (0.,) * 6) + (values[0],)
                     for order, values in enumerate(state))


class GripperTransport(CspTransport):
    feedback_axes = AXES

    def __init__(self, **kwargs):
        super().__init__(axis=7, runtime=RUNTIME, include_gripper=True, **kwargs)


class GripperTrial(CspAxisTrial):
    feedback_axes = AXES
    runtime_for_axis = staticmethod(runtime_for_axis)
    proposal = staticmethod(proposal)
    parse_envelope = staticmethod(GripperEnvelope.parse)
    trajectory_type = GripperTrajectory

    def __init__(self, config, transport, **kwargs):
        super().__init__(config, transport, axis=7, **kwargs)
