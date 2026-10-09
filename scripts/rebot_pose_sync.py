"""Initialize simulation from explicitly supplied read-only feedback. No driver imports.

Browser telemetry is adequate to set a simulator, never authority to start hardware.
The established B601 control-URDF mapping is identity; MuJoCo conversion lives in Robot.
"""
import math
import time
import uuid
import numpy as np


def synchronize_pose(robot, motors, mapping, *, now=None, max_age_s=3.):
    now = time.time() if now is None else now
    if not isinstance(mapping, dict) or mapping.get('version') != 1 or not isinstance(mapping.get('axes'), list) or len(mapping['axes']) != 6:
        raise ValueError('关节映射格式不完整；需要已有的六轴映射')
    q, measured = [], []
    for i, axis in enumerate(mapping['axes'], 1):
        # Reuse the established official mapping, not an arbitrary new display transform.
        if not isinstance(axis, dict) or type(axis.get('motorId')) is not int or axis.get('motorId') != i or axis.get('direction') != 1 or axis.get('offsetDeg') != 0:
            raise ValueError(f'J{i} 映射与已确认的官方 q_model=q_motor 不一致；没有修改电机零位')
        reading = motors.get(str(i), motors.get(i)) if isinstance(motors, dict) else None
        if not isinstance(reading, dict) or reading.get('id') != i or reading.get('model') != ('rs-06' if i <= 3 else 'rs-00'):
            raise ValueError(f'J{i} ID/型号与既有轴映射不一致')
        value, stamp = reading.get('mechPosRad'), reading.get('positionUpdatedAt')
        if type(value) not in (int, float) or not math.isfinite(value) or reading.get('error'):
            raise ValueError(f'J{i} 位置读取失败，请刷新状态')
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or not 0 <= now-stamp/1000 <= max_age_s:
            raise ValueError(f'J{i} 位置反馈超过 {max_age_s:g} 秒或时间异常，请重新读取')
        if not robot.lower[i-1] <= value <= robot.upper[i-1]:
            raise ValueError(f'J{i} 原始位置 {math.degrees(value):.2f}° 超出模型范围 '
                             f'{(math.degrees(robot.lower[i-1])+0.):.2f}°…{math.degrees(robot.upper[i-1]):.2f}°；未裁剪或改零位')
        q.append(value); measured.append(stamp/1000)
    q = np.asarray(q)
    robot.initialize_pose(q)
    errors = (robot.q-q).tolist()
    return dict(ok=True, synchronization_id=uuid.uuid4().hex, source='browser-readonly-feedback',
                hardware_motion=False, hardware_start_authority=False,
                initialized_at=now, measured_at=measured, q_rad=robot.q.tolist(), target_rad=robot.target.tolist(),
                errors_rad=errors, max_error_rad=max(map(abs, errors)), xyz_mm=(robot.fk(robot.q).translation*1000).tolist(),
                note='已写入 MuJoCo 关节状态和控制目标；实机未运动。原有静态映射继续使用，动态验收状态未改变。')
