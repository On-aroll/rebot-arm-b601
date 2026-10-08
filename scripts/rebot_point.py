"""Validated, expiring Cartesian point plans. Does not open CAN or drive hardware."""
import time
import uuid

import numpy as np


HARDWARE_REASON = '实机同步尚未开放：需要逐轴确认方向和零位，并完成MIT 独立执行器与停止/失联验收。'


def point_capabilities():
    return dict(simulation=True, hardware=False, hardware_reason=HARDWARE_REASON,
                frame='base_link', tcp='gripper_end', units='mm')


class PointPlanner:
    def __init__(self, robot, clock=time.monotonic, configuration=lambda: "simulation-v1"):
        self.configuration = configuration
        self.robot = robot
        self.clock = clock
        self.plans = {}

    def preview(self, body):
        if body.get('target', 'simulation') != 'simulation':
            raise ValueError(HARDWARE_REASON)
        try:
            xyz = np.asarray(body['xyz_mm'], dtype=float)
            speed = float(body.get('speed_rad_s', .15))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('请输入 X、Y、Z 三个坐标和有效速度') from exc
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError('X、Y、Z 必须为三个有限数值，单位 mm')
        if not np.isfinite(speed) or not .05 <= speed <= .3:
            raise ValueError('关节速度上限必须在 0.05–0.30 rad/s 之间')
        mode = body.get('orientation_mode', 'position')
        if mode not in ('position', 'hold'):
            raise ValueError('请选择自动调整朝向或保持当前朝向')
        r = self.robot
        configuration = self.configuration()
        start = r.motion_start()
        target = r.ik(xyz / 1000, seed=start, position_only=mode == 'position')
        if np.any(target < r.lower) or np.any(target > r.upper):
            raise ValueError('计算结果超出关节限位')
        r.check_path(start, target)
        seconds = max(1., 1.5 * float(np.max(np.abs(target - start))) / speed)
        if seconds > 60:
            raise ValueError('预计运动超过 60 秒，请选择更近的目标点')
        now = self.clock()
        self.plans = {key: val for key, val in self.plans.items() if val['expires'] > now}
        while len(self.plans) >= 8:
            del self.plans[next(iter(self.plans))]
        plan_id = uuid.uuid4().hex
        if configuration != self.configuration():
            raise ValueError('配置版本已改变，请重新计算目标点')
        self.plans[plan_id] = dict(start=start.copy(), target=target.copy(),
                                  configuration=configuration, task_version=1,
                                  finger=float(r.finger_target), expires=now + 60, seconds=seconds)
        actual_xyz = r.fk(target).translation.copy()
        path = [r.fk(start * (1-t) + target*t).translation.tolist() for t in np.linspace(0, 1, 61)]
        return dict(ok=True, plan_id=plan_id, expires_in_s=60, target='simulation',
                    xyz_mm=xyz.tolist(), q_deg=np.rad2deg(target).tolist(),
                    start_deg=np.rad2deg(start).tolist(), seconds=seconds,
                    speed_rad_s=speed, orientation_mode=mode, path_xyz=path,
                    position_error_mm=float(np.linalg.norm(actual_xyz * 1000 - xyz)),
                    **point_capabilities())

    def take(self, body):
        # The client supplies a token, never executable joint angles or a new duration.
        if body.get('target', 'simulation') != 'simulation':
            raise ValueError(HARDWARE_REASON)
        key = body.get('plan_id')
        if not isinstance(key, str) or key not in self.plans:
            raise ValueError('预览不存在，请重新计算目标点')
        plan = self.plans.pop(key)
        if plan['configuration'] != self.configuration() or plan['task_version'] != 1:
            raise ValueError('配置或任务版本已改变，请重新计算目标点')
        if self.clock() >= plan['expires']:
            raise ValueError('预览已过期，请重新计算目标点')
        r = self.robot
        if np.max(np.abs(r.motion_start() - plan['start'])) > np.deg2rad(.5):
            raise ValueError('机械臂姿态已改变，请重新计算目标点')
        if abs(r.finger_target - plan['finger']) > .0001:
            raise ValueError('夹爪状态已改变，请重新计算目标点')
        r.check_path(r.motion_start(), plan['target'])
        return dict(version=1, name='三维目标点 · 仿真到点', steps=[dict(
            op='move_joints', name='移动到指定三维坐标', q_deg=np.rad2deg(plan['target']).tolist(),
            seconds=plan['seconds'])])
