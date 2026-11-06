"""One explicit two-degree software margin for the six arm joints."""
import math

JOINT_MARGIN_DEG = 2.0
JOINT_MARGIN_RAD = math.radians(JOINT_MARGIN_DEG)


def expanded_joint_limits(nominal):
    if len(nominal) != 6:
        raise ValueError('需要完整的六轴名义限位')
    result = []
    for lower, upper in nominal:
        if not all(math.isfinite(v) for v in (lower, upper)) or lower >= upper:
            raise ValueError('名义限位必须是有限且有序的范围')
        result.append((lower - JOINT_MARGIN_RAD, upper + JOINT_MARGIN_RAD))
    return result
