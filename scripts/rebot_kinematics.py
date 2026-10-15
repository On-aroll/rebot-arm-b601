"""Read-only model coordinates; never initialize a pose or create a CAN client."""
import numpy as np


def coordinate_preview(robot, q_deg):
    if not isinstance(q_deg, list) or len(q_deg) != 6 or any(
            isinstance(v, bool) or not isinstance(v, (int, float)) for v in q_deg):
        raise ValueError('需要六个有限关节角，单位为度')
    q = np.asarray(q_deg, dtype=float)
    if not np.all(np.isfinite(q)):
        raise ValueError('需要六个有限关节角，单位为度')
    # Keep measured boundary readings; FK is not motion approval or zeroing.
    pose = robot.fk(np.deg2rad(q))
    return dict(ok=True, xyz_mm=(pose.translation * 1000).tolist(),
                frame='base_link', control_point='gripper_end',
                source='model-fk', q_deg=q_deg, hardware_command=False)
