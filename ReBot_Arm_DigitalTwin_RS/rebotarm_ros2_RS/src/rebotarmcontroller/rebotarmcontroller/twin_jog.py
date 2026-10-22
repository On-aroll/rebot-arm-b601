"""Bounded online joint-velocity ramp. No ROS, CAN or actuator construction."""
import numpy as np


class JogRamp:
    def __init__(self):
        self.reset()

    def reset(self):
        self.velocity = np.zeros(6)

    def step(self, position, desired, dt, speed, acceleration, lower, upper, *, speed_fraction=.8):
        position, desired = np.asarray(position), np.asarray(desired)
        # Do not replay elapsed time after a delayed input packet.
        dt = min(.05, max(0., dt))
        if dt <= 0:
            return position.copy()
        lower, upper = np.minimum(lower, position), np.maximum(upper, position)
        desired = np.clip(desired, -speed_fraction*speed, speed_fraction*speed)
        # Reserve braking distance before a joint boundary, including this step.
        def braking(distance):
            return np.sqrt((acceleration*dt)**2 + 2*acceleration*np.maximum(0.,distance))-acceleration*dt
        lo, hi = -braking(position-lower), braking(upper-position)
        desired = np.clip(desired, lo, hi)
        self.velocity += np.clip(desired-self.velocity, -acceleration*dt, acceleration*dt)
        # A tighter speed/position limit takes precedence over smoothness.
        self.velocity = np.clip(self.velocity, np.maximum(lo,-speed_fraction*speed), np.minimum(hi,speed_fraction*speed))
        return position + self.velocity*dt
