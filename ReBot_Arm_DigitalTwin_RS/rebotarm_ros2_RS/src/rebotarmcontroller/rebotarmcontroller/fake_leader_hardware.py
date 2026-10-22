"""The same leader lifecycle/watchdog against FakeRsDriver, without CAN."""
from __future__ import annotations

import math
import time

from .leader_policy import DEFAULT_LIMITS, TeleopLease, finite_vector


class FakeLeaderHardware:
    teleop_speed_max = 1.0

    def __init__(self, node):
        self.node = node
        self._cmd_lock = node._leader_lock
        self._teleop_lease = TeleopLease()
        self.teleop_limits = DEFAULT_LIMITS
        configured_speed = float(getattr(node, "max_joint_speed", 1.0))
        self.teleop_speed_max = max(0.05, min(1.0, configured_speed))
        self.gripper_close_position = 0.0
        self.gripper_open_position = node.gripper_open_position
        self.has_gripper = True
        self.gripper_authorized = False
        self._grasp_monitor = None

    def probe_gripper(self):
        pass  # Fake feedback is always local and has no CAN ownership.

    def gripper_state(self):
        # This Fake has no object/contact physics. Zero current can NEVER
        # establish holding; closing either reaches the empty endpoint or times out.
        stamp = getattr(self.node, 'last_time', None)
        sample_at = stamp.nanoseconds / 1e9 if stamp is not None else None
        if self._grasp_monitor:
            now = time.monotonic()
            self._grasp_monitor.update(position=self.node.gripper_position,
                velocity=self.node.gripper_velocity, current=0., stamp=now, now=now,
                reference=self.node.gripper_target)
            if self._grasp_monitor.status == 'failed':self.node.gripper_target = self.node.gripper_position
        return dict(available=True, fresh=True, authorized=self.gripper_authorized,
                    position=self.node.gripper_position, velocity=self.node.gripper_velocity,
                    target=self.node.gripper_target, active=self.gripper_authorized,
                    arrived=abs(self.node.gripper_position-self.node.gripper_target) <= .002,
                    error='', speed_max=5., range_rad=[0., 5.], current_a=0., grasp_supported=True,
                    sample_at=sample_at,
                    grasp=self._grasp_monitor.state() if self._grasp_monitor else {},
                    source='Fake driver, no CAN; no object contact physics')

    def authorize_gripper(self, session, enabled):
        self._teleop_lease.require(session)
        self.gripper_authorized = bool(enabled)
        if not enabled:self.node.gripper_target = self.node.gripper_position

    def gripper_target(self, session, position, speed):
        from .twin_program import number
        self._teleop_lease.require(session)
        if not self.gripper_authorized:raise ValueError('请先授权J7')
        self._grasp_monitor = None
        self.node.gripper_target = number(position, '夹爪位置', 0, 5)
        self.node.max_gripper_speed = number(speed, '夹爪速度', .005, 5)

    def gripper_grasp(self, session, profile, speed):
        import secrets
        from .twin_grasp import GraspMonitor, MAX_GRASP_SPEED
        from .twin_program import number
        self._teleop_lease.require(session)
        if not self.gripper_authorized:raise ValueError('请先授权J7')
        monitor = GraspMonitor(profile, self.node.gripper_position, time.monotonic(), secrets.token_hex(12))
        speed = number(speed, '接触检测闭合速度', .005, MAX_GRASP_SPEED)
        if speed <= monitor.profile['stall_velocity']*2:raise ValueError('闭合速度须大于停滞阈值的两倍')
        self._grasp_monitor = monitor
        self.node.gripper_target = monitor.profile['empty_position']
        self.node.max_gripper_speed = speed
        return monitor.id

    def gripper_jog(self, session, velocity):
        self._teleop_lease.require(session)
        if not math.isfinite(velocity) or abs(velocity) > 5:raise ValueError('J7速度超限')
        self.gripper_authorized = True
        if velocity:self._grasp_monitor = None
        # The twin supplies fresh 20 Hz samples; its watchdog freezes stale input.
        if velocity:
            self.node.gripper_target = max(0.,min(5.,self.node.gripper_target+velocity*.05))
            self.node.max_gripper_speed = abs(velocity)
        else:self.node.gripper_target = self.node.gripper_position

    @property
    def teleop_owned(self):
        return bool(self._teleop_lease.session_id)

    def get_joint_state(self, request_feedback=False):
        return list(self.node.positions), list(self.node.velocities), [0] * 6

    def get_gripper_state(self, request_feedback=False):
        return self.node.gripper_position, self.node.gripper_velocity, 0, 0

    def teleop_acquire(self):
        with self._cmd_lock:
            if not self.node.enabled or self.node.state_machine != "IDLE":
                raise RuntimeError("fake follower is disabled or busy")
            self._hold()
            self.gripper_authorized = False
            return self._teleop_lease.acquire()

    def teleop_heartbeat(self, session):
        with self._cmd_lock:
            self.teleop_watchdog()
            self._teleop_lease.heartbeat(session)

    def teleop_send(self, session, seq, sampled_at, targets, gripper, speed):
        with self._cmd_lock:
            self.teleop_watchdog()
            targets = finite_vector(targets, 6)
            if any(not lo <= value <= hi for value, (lo, hi) in zip(targets, self.teleop_limits)):
                raise ValueError("target out of limits")
            if not math.isfinite(speed) or not 0.05 <= speed <= self.teleop_speed_max:
                raise ValueError(
                    f"invalid leader speed: 0.05..{self.teleop_speed_max:g} rad/s"
                )
            if gripper is not None and (not math.isfinite(gripper) or not 0 <= gripper <= self.gripper_open_position):
                raise ValueError("invalid gripper target")
            self._teleop_lease.sample(session, seq, sampled_at)
            self.node.targets = list(targets)
            self.node.max_joint_speed = speed
            if gripper is not None:
                self.node.gripper_target = gripper
                self.node.max_gripper_speed = min(speed * 6, 3.0)
            self.node.state_machine = "LOWLEVEL_STREAMING"

    def _hold(self):
        if self._grasp_monitor and self._grasp_monitor.status not in ('holding', 'failed', 'cancelled'):
            self._grasp_monitor.cancel()
        self.node.targets = list(self.node.positions)
        self.node.gripper_target = self.node.gripper_position
        self.node.state_machine = "IDLE"

    def teleop_pause(self, session):
        with self._cmd_lock:
            self.teleop_watchdog()
            self._teleop_lease.require(session)
            self._hold()
            self._teleop_lease.paused = True

    def teleop_resume(self, session):
        with self._cmd_lock:
            self.teleop_watchdog()
            self._teleop_lease.require(session)
            self._teleop_lease.paused = False
            self._teleop_lease.last_sample = time.monotonic()
            self._teleop_lease.last_seq = -1

    def teleop_stop(self, session, reason="stopped"):
        with self._cmd_lock:
            self.gripper_authorized = False
            self._teleop_lease.require(session)
            self._hold()
            self._teleop_lease.release(reason)

    def teleop_watchdog(self):
        with self._cmd_lock:
            reason = self._teleop_lease.expired()
            if reason:
                self._hold()
                self._teleop_lease.release(reason)
            return reason
