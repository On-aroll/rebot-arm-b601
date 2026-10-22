"""Local Ruckig references. No ROS, actuator API, wall clock or CAN access."""
from functools import lru_cache

import numpy as np
from ruckig import ControlInterface, InputParameter, Result, Ruckig, Synchronization, Trajectory

PROFILES = ('ruckig', 'quintic')


def _calculate(inp):
    curve = Trajectory(inp.degrees_of_freedom)
    try:
        result = Ruckig(inp.degrees_of_freedom).calculate(inp, curve)
    except Exception as exc:
        raise RuntimeError(f'Ruckig轨迹计算失败：{exc}') from exc
    if result not in (Result.Working, Result.Finished):
        raise RuntimeError(f'Ruckig轨迹计算失败：{result}')
    return curve


@lru_cache(maxsize=512)
def _progress(delta, speed, acceleration, jerk, duration):
    # One scalar progress keeps the EXACT checked straight joint-space path.
    # Independently synchronizing six joint curves could change that path.
    inp = InputParameter(1)
    inp.current_position = inp.current_velocity = inp.current_acceleration = [0.]
    inp.target_position = [1.]
    inp.target_velocity = inp.target_acceleration = [0.]
    inp.max_velocity = [speed/delta]
    inp.max_acceleration = [acceleration/delta]
    inp.max_jerk = [jerk/delta]
    inp.minimum_duration = duration
    return _calculate(inp)


def point_duration(start, goal, speed, acceleration, jerk, duration):
    delta = float(np.max(np.abs(np.asarray(goal)-start)))
    return float(duration if delta < 1e-12 else _progress(delta, speed, acceleration, jerk, duration).duration)


def point_sample(profile, start, goal, speed, acceleration, jerk, duration, elapsed):
    start, goal = np.asarray(start), np.asarray(goal)
    delta = float(np.max(np.abs(goal-start)))
    if delta < 1e-12:
        return goal.copy()
    if profile == 'ruckig':
        curve = _progress(delta, speed, acceleration, jerk, duration)
        blend = curve.at_time(max(0., min(elapsed, curve.duration)))[0][0]
    else:
        t = np.clip(elapsed/duration, 0., 1.)
        blend = t**3*(10-15*t+6*t*t)
    return start+(goal-start)*blend


class RuckigVelocity:
    """Jerk-limited velocity transitions with a checked stopping envelope."""
    def __init__(self, axes=6):
        self.axes = axes
        self.reset()

    def reset(self):
        self.velocity = np.zeros(self.axes)
        self.acceleration = np.zeros(self.axes)
        self.boundary_braking = False

    def _curve(self, position, velocity, acceleration, target, maximum_a, jerk):
        inp = InputParameter(self.axes)
        inp.control_interface = ControlInterface.Velocity
        inp.synchronization = Synchronization.No
        inp.current_position = list(position)
        inp.current_velocity = list(velocity)
        inp.current_acceleration = list(acceleration)
        inp.target_velocity = list(target)
        inp.target_acceleration = [0.]*self.axes
        inp.max_acceleration = [maximum_a]*self.axes
        inp.max_jerk = [jerk]*self.axes
        return _calculate(inp)

    def step(self, position, desired, dt, speed, acceleration, lower, upper, *, jerk=30., speed_fraction=1.):
        position = np.asarray(position, dtype=float)
        dt = min(.05, max(0., dt))
        if dt <= 0:
            return position.copy()
        maximum_v = speed_fraction*speed
        # A newly tightened hard limit takes precedence over a smooth ramp.
        if np.max(np.abs(self.velocity)) > maximum_v+1e-9:
            self.velocity = np.clip(self.velocity, -maximum_v, maximum_v)
            self.acceleration[:] = 0.
        desired = np.clip(desired, -maximum_v, maximum_v)
        lo, hi = np.minimum(lower, position), np.maximum(upper, position)
        curve = self._curve(position, self.velocity, self.acceleration, desired, acceleration, jerk)
        q, v, a = map(np.asarray, curve.at_time(dt))
        # Candidate next state must still be able to stop before every limit,
        # including nonzero acceleration. This is stricter than sqrt(2*a*d).
        stop = self._curve(q, v, a, np.zeros(self.axes), acceleration, jerk)
        self.boundary_braking = any(e.min < low-1e-10 or e.max > high+1e-10
                                   for e, low, high in zip(stop.position_extrema, lo, hi))
        if self.boundary_braking:
            curve = self._curve(position, self.velocity, self.acceleration, np.zeros(self.axes), acceleration, jerk)
            q, v, a = map(np.asarray, curve.at_time(dt))
        if np.any(q < lo-1e-9) or np.any(q > hi+1e-9) or np.max(np.abs(v)) > maximum_v+1e-8:
            raise RuntimeError('Ruckig参考超出停止包络，暂停并保持')
        self.velocity, self.acceleration = v, a
        return np.clip(q, lo, hi)
