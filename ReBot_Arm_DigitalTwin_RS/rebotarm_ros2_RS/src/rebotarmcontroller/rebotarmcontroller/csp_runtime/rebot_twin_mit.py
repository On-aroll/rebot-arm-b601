"""MIT position PD through the existing owner, permit and register restoration.

No upstream connect/stop lifecycle, group thread, PID writes or gravity model.
The current position is cached before enable; the first MIT frame is sent
immediately after enable. A disabled motor's ignored frame is never called a
successful drive preload. Physical startup/retained-target hold need acceptance.
"""
import math

from .rebot_native_csp import CspTransport
from .rebot_mit_hardware import SdkTransport
from ..twin_drive import mit_parameters


class MitDrive(CspTransport):
    def __init__(self, *, parameters, **kwargs):
        super().__init__(**kwargs)
        self.parameters = mit_parameters(parameters)
        self.runtime[0x7005] = 0
        self.mit_last = None

    def write_register(self, index, value, check, *, phase, verify=True):
        if index != 0x7016 or phase in ('restore', 'disabled-current'):
            return super().write_register(index, value, check, phase=phase, verify=verify)
        # MIT has no CSP loc_ref acknowledgement. Validate the same permit and
        # monotone boundary envelope explicitly; keep register readbacks intact.
        a = self.envelope.axes[self.axis-1]
        start = self.boundary_start.get(self.axis)
        low, high = a.q_min, a.q_max
        if self.axis not in self.boundary_recovered and start is not None:
            if start < a.q_min:low=max(a.q_min-self.envelope.start_boundary_tolerance,start-self.envelope.policy.arrival_q)
            if start > a.q_max:high=min(a.q_max+self.envelope.start_boundary_tolerance,start+self.envelope.policy.arrival_q)
        if (not self.permit or self.axis not in self.active or self.clock() >= self.permit.expires_at or
                not math.isfinite(value) or not low <= value <= high):
            raise PermissionError('MIT目标缺少本次许可或超出起点/关节范围')
        if not a.q_min <= value <= a.q_max and phase == 'trajectory':
            previous = self.last_boundary_target if self.last_boundary_target is not None else start
            if (value < a.q_min and value < previous-1e-12) or (value > a.q_max and value > previous+1e-12):
                raise PermissionError('MIT边界恢复目标不得继续远离名义范围')
        if phase == 'preload-current':
            if value != self.latest_samples[self.axis].q:
                raise PermissionError('MIT首目标必须来自当前轴新鲜实测位置')
            check(); self.ownership.check()
            self.preloaded_at, self.last_boundary_target = self.clock(), value
            return  # Host cache only. Do not send to a disabled drive.
        if phase not in ('hold', 'trajectory'):
            raise PermissionError('未知MIT发送阶段')
        check(); self.ownership.check()
        sample = self.latest_samples[self.axis]
        if not 0 <= self.clock()-sample.requested_at <= self.envelope.policy.feedback_age:
            raise TimeoutError('MIT位置反馈过期，未更新目标')
        kp, kd = self.parameters['kp'][self.axis-1], self.parameters['kd'][self.axis-1]
        demand = kp*(value-sample.q)-kd*sample.qd
        if abs(demand) > a.torque_max:
            raise ValueError('MIT估算PD力矩超过原力矩包络，未发送该目标')
        entry = dict(axis=self.axis, index='mit', value=value, values=[value, 0., kp, kd, 0.],
                     phase=phase, requested_at=self.clock(), status='attempted')
        self.register_log.append(entry)
        self.mit_last = entry
        try:
            self.motors[self.axis].send_mit(*entry['values'])
            entry.update(status='host-send-complete', completed_at=self.clock())
            self.last_boundary_target = value
            if a.q_min <= value <= a.q_max:
                self.boundary_recovered.add(self.axis)
            check()
        except Exception as exc:
            entry.update(status='unknown-may-have-reached-drive', error=str(exc))
            raise

    def enable(self, axis):
        if axis != self.axis or self.prepared != set(self.runtime) or self.preloaded_at is None:
            raise PermissionError('MIT须先完成运行参数读回与新鲜当前位置缓存')
        if self.clock()-self.preloaded_at > self.envelope.policy.command_age:
            raise TimeoutError('MIT首目标缓存过期，该轴未使能')
        SdkTransport.enable(self, axis)
        # No position query or another axis's enable is inserted between these.
        self.write_register(0x7016, self.last_boundary_target, lambda: None, phase='hold', verify=False)
