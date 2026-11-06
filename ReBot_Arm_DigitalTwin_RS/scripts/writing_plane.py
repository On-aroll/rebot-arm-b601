"""Frozen writing frame through the model TCP, normal to gripper's local X.

The gripper_end frame is the existing control TCP, not an uncalibrated pen tip.
Local -X is the retract direction, -Y is text-right and +Z is text-up.
"""
import math


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def tool_plane(pose):
    if len(pose) != 6 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in pose):
        raise ValueError("需要六个有限 TCP 位姿数值")
    roll, pitch, yaw = pose[3:]
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    # R_base_tcp = Rz(yaw) Ry(pitch) Rx(roll), matching Pinocchio's RPY.
    x = [cy * cp, sy * cp, -sp]
    y = [cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr]
    z = [cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr]
    return dict(kind="tool_normal", frame_id="base_link", tcp_frame="gripper_end",
                origin=list(pose[:3]), orientation_rpy=list(pose[3:]),
                normal=[-v for v in x], u=[-v for v in y], v=z,
                gripper_longitudinal_axis="local X", retract_axis="local -X",
                frozen=True, pen_tip_calibrated=False)


def plane_point(plane, horizontal, vertical, retract=0.):
    return [plane["origin"][i] + horizontal * plane["u"][i] + vertical * plane["v"][i] + retract * plane["normal"][i]
            for i in range(3)]


def project(plane, point):
    delta = [a - b for a, b in zip(point, plane["origin"])]
    return [dot(delta, plane[key]) for key in ("u", "v", "normal")]
