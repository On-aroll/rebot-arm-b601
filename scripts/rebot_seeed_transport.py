"""Reuse Seeed RebotArm registration/JointGroup behind the existing P4 guards.

Construction is inert. No upstream end-pose node, mode write, gravity feedforward,
group enable/disable, homing, context-manager exit or disconnect is called.
Actual hardware still needs the existing one-use onsite permit.
"""
import importlib.metadata
from dataclasses import replace
import importlib.util
from pathlib import Path
import subprocess
import sys

from rebot_mit_hardware import MODELS, SdkTransport
from rebot_mit_mock import finite

ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT / 'external/reBotArm_control_py'
COMMIT = '6415d43130d1e143c70dc106096a857ac5556f81'


def official_module():
    head = subprocess.check_output(['git', '-C', str(OFFICIAL), 'rev-parse', 'HEAD'], text=True).strip()
    changes = subprocess.check_output(['git', '-C', str(OFFICIAL), 'status', '--porcelain', '--untracked-files=no'], text=True)
    if head != COMMIT or changes.strip():
        raise RuntimeError('Seeed SDK 版本或源码变化；请先审核，不连接 CAN')
    source = OFFICIAL / 'reBotArm_control_py/actuator/rebotarm.py'
    name = 'rebot_seeed_sdk_actuator'
    # Import the actual actuator module without the package's eager Pinocchio /
    # dynamics imports. ROS Python and the simulation use different ABIs.
    rebotarm = sys.modules.get(name)
    if rebotarm is None:
        spec = importlib.util.spec_from_file_location(name, source)
        rebotarm = importlib.util.module_from_spec(spec)
        sys.modules[name] = rebotarm
        try:
            spec.loader.exec_module(rebotarm)
        except Exception:
            sys.modules.pop(name, None)
            raise
    if Path(rebotarm.__file__).resolve() != source:
        raise RuntimeError('加载的 Seeed SDK 来源不符；拒绝连接')
    return rebotarm


class SeeedTransport(SdkTransport):
    """The P4 executor supplies permission, timing and feedback checks unchanged."""
    def __init__(self, *, position_window=False, diagnostic_axes=(), **kwargs):
        super().__init__(**kwargs)
        if any(type(i) is not int or i not in MODELS for i in diagnostic_axes) or len(set(diagnostic_axes))!=len(diagnostic_axes):
            raise ValueError('诊断轴必须是明确且不重复的 J1–J6')
        self.diagnostic_axes=tuple(diagnostic_axes)
        self.arm = None
        self.trial_group = None
        self.trial_groups = {}
        self.position_window = position_window
        self.previous_samples = {}

    def open_readonly(self):
        if self.controller:
            raise RuntimeError('连接已打开')
        upstream = official_module()
        if self.factory is None:
            if importlib.metadata.version('motorbridge') != '0.5.6':
                raise RuntimeError('真实路径只审核了 motorbridge 0.5.6')
            from motorbridge import Controller
            factory = Controller
        else:
            factory = self.factory  # Injected fake SDK, only used by offline tests.
        transport = self

        class Arm(upstream.RebotArm):
            def _make_controller(self, vendor):
                if vendor != 'robstride' or transport.controller is not None:
                    raise RuntimeError('只允许一个 RobStride CAN 控制器')
                transport.controller = factory(channel='can0')
                return transport.controller

        # The official RS YAML is read, never modified. Its gain defaults are
        # never sent: trial frames below always carry explicit approved gains.
        self.arm = Arm(hw_yaml=str(OFFICIAL / 'config/rebotarm_rs.yaml'))
        # Register J1-J6 only; no gripper/group-wide lifecycle in this trial.
        self.arm._all_joints = [j for j in self.arm._all_joints if j.name in {f'joint{i}' for i in MODELS}]
        actual = [(j.name, j.motor_id, j.feedback_id, j.model, j.vendor) for j in self.arm._all_joints]
        expected = [(f'joint{i}', i, 0xFD, model, 'robstride') for i, model in MODELS.items()]
        if actual != expected:
            raise RuntimeError('Seeed RS 电机映射不符；未连接')
        self.ownership.acquire()
        try:
            self.arm.connect()  # Upstream connect only registers motors.
            self.motors = {i: self.arm._motor_map[f'joint{i}'] for i in MODELS}
            self.ownership.claim()
        except Exception:
            # Registration can fail partway; close every created handle without
            # invoking upstream disconnect(), which would disable all motors.
            self.motors = dict(enumerate(self.arm._motor_map.values(), 1))
            self.close()
            raise

    def read(self, axes, policy, check=lambda:None):
        axes=tuple(axes)
        samples=super().read(axes,policy,check)
        if self.position_window and any(i not in self.previous_samples for i in axes):
            self.previous_samples.update(samples)
            samples=super().read(axes,policy,check)
        for i in self.diagnostic_axes:
            if i not in samples: continue
            # Raw drive reports for identification, not a calibrated torque
            # sensor. Keep their timestamps separate from mechPos/velocity.
            requested=self.clock(); values=[]
            for index in (0x701A,0x302C):
                check();self.ownership.check()
                value=self.motors[i].robstride_get_param_f32_host_id(index,0xFD,max(1,int(policy.io_timeout*1000)))
                check()
                if not finite(value): raise ValueError(f'J{i} 诊断寄存器 {index:#x} 返回无效数值')
                values.append(value)
            received=self.clock()
            if received-requested>policy.feedback_age: raise TimeoutError(f'J{i} 诊断反馈过期')
            samples[i]=replace(samples[i],iq_raw_a=values[0],torque_raw_nm=values[1],
                               diagnostics_requested_at=requested,diagnostics_received_at=received)
        if not self.position_window:
            return samples
        result={}
        for i,s in samples.items():
            previous=self.previous_samples[i]
            dt=(s.requested_at+s.received_at-previous.requested_at-previous.received_at)/2
            if not 0<dt<=policy.feedback_age:
                raise TimeoutError(f'J{i} 位置差分窗口无效或过期')
            result[i]=replace(s,qd=(s.q-previous.q)/dt,qd_raw=s.qd)
        self.previous_samples.update(samples)
        return result

    def authorize(self, config, permit):
        if len(permit.axes) != 1 and not (permit.axes==tuple(MODELS) and permit.coordinated_start_checked is True):
            raise PermissionError('Seeed 多轴试验需要明确六轴许可；默认仅允许单轴')
        upstream = official_module()
        transport = self

        class CheckedMotor:
            def __init__(self, axis): self.axis=axis
            def send_mit(self, *values):
                try:
                    # Keep the already-tested permit, model and gain checks;
                    # convert SDK CallError so JointGroup cannot swallow it.
                    SdkTransport.frame(transport, self.axis, values)
                except Exception as error:
                    raise RuntimeError(f'Seeed J{self.axis} 发送失败：{error}') from error

        class NoGroupLifecycle:
            def __getattr__(self, name):
                raise PermissionError(f'禁止组级操作：{name}')

        super().authorize(config, permit)
        for axis in permit.axes:
            limits = config.axes[axis-1]
            name = f'joint{axis}'
            joint = upstream.JointCfg(name, axis, 0xFD, MODELS[axis], 'robstride', kp=limits.kp, kd=limits.kd)
            self.trial_groups[axis] = upstream.JointGroup('onsite-approved', [name], [joint], {name: CheckedMotor(axis)}, {'robstride': NoGroupLifecycle()})
        self.trial_group = self.trial_groups[permit.axes[0]]

    def frame(self, axis, values):
        if axis not in self.trial_groups or not self.permit or axis not in self.permit.axes:
            raise PermissionError('不是当次获准轴')
        if len(values) != 5 or not all(finite(v) for v in values):
            raise ValueError('需要五个明确 MIT 参数')
        self.trial_groups[axis].send_mit(*([v] for v in values))

    def close(self):
        super().close()
        self.trial_group = None
        self.trial_groups.clear()
        self.previous_samples.clear()
        if self.arm:
            self.arm._connected = False
            self.arm._motor_map.clear()
        self.arm = None
