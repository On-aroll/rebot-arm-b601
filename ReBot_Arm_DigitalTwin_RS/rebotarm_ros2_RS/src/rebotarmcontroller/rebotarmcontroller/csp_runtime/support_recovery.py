"""One-shot, inward-only route from the known J2/J3 overrun to a support pose."""
import math

import numpy as np

from ..csp_official_limits import RATED_TORQUE


MAX_START_OVERRUN_RAD = math.radians(30.)
START_MATCH_RAD = math.radians(.1)
LOAD_RELIEF_J3_RAD = 1.5
RECOVERY_TORQUE_HARD_LIMIT_NM = 15.
RECOVERY_STAGE0_TORQUE_LIMITS_NM = np.array([RATED_TORQUE[0], 15., 15., *RATED_TORQUE[3:]], dtype=float)


def build_support_recovery(start, target, limits):
    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    limits = np.asarray(limits, dtype=float)
    if start.shape != (6,) or target.shape != (6,) or limits.shape != (6, 2):
        raise ValueError('支撑恢复需要六轴起点、目标和限位')
    if not np.all(np.isfinite(np.r_[start, target, limits.ravel()])):
        raise ValueError('支撑恢复输入含无效数值')
    if abs(target[0] - start[0]) > 1e-9:
        raise ValueError('支撑恢复必须保留当前 J1')
    if np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
        raise ValueError('支撑目标必须在全部名义关节范围内')

    overruns = np.flatnonzero((start < limits[:, 0]) | (start > limits[:, 1]))
    if tuple(overruns) != (1, 2):
        raise ValueError('当前仅定义 J2、J3 同时超上限时的支撑恢复路线')
    if any(start[i] <= limits[i, 1] or
           start[i] - limits[i, 1] > MAX_START_OVERRUN_RAD for i in overruns):
        raise ValueError('J2/J3 超出名义上限超过支撑恢复范围 30°')
    if any(not limits[i, 0] <= start[i] <= limits[i, 1] for i in (0, 3, 4, 5)):
        raise ValueError('J1、J4–J6 起点必须在名义关节范围内')
    if target[1] > math.radians(3.) or target[2] > math.radians(3.):
        raise ValueError('此恢复路线仅适用于 J2/J3 回到 0° 附近的已验证支撑姿态')
    if start[2] <= LOAD_RELIEF_J3_RAD:
        raise ValueError('J3 起点不适用于受控折叠卸载路线')

    fold_j3 = start.copy()
    fold_j3[2] = LOAD_RELIEF_J3_RAD
    lower_j2 = fold_j3.copy()
    lower_j2[1] = target[1]
    waypoints = [fold_j3, lower_j2, target.copy()]
    return dict(start=start.tolist(), target=target.tolist(),
                waypoints=[q.tolist() for q in waypoints],
                strategy='fold-j3-then-lower-j2-then-support',
                max_start_overrun_rad=MAX_START_OVERRUN_RAD,
                recovery_torque_hard_limit_nm=RECOVERY_TORQUE_HARD_LIMIT_NM,
                recovery_stage0_torque_limits_nm=RECOVERY_STAGE0_TORQUE_LIMITS_NM.tolist(),
                recovery_torque_exception='stage 0 of this fixed support route only; drive limits are unchanged')


def gravity_limits_for_recovery_stage(stage, nominal_limits):
    limits = np.asarray(nominal_limits, dtype=float).copy()
    if limits.shape != (6,):
        raise ValueError('支撑恢复力矩包络需要六轴额定限制')
    if stage == 0:
        limits[1:3] = RECOVERY_TORQUE_HARD_LIMIT_NM
    return limits


class SupportRecovery:
    """Runtime capability limited to the route produced by build_support_recovery."""

    def __init__(self, plan, limits):
        self.start = np.asarray(plan['start'], dtype=float)
        self.target = np.asarray(plan['target'], dtype=float)
        self.waypoints = [np.asarray(q, dtype=float) for q in plan['waypoints']]
        self.limits = np.asarray(limits, dtype=float)
        if (self.start.shape != (6,) or self.target.shape != (6,) or self.limits.shape != (6, 2) or
                len(self.waypoints) != 3 or any(q.shape != (6,) for q in self.waypoints) or
                not np.all(np.isfinite(np.r_[self.start, self.target, self.limits.ravel(),
                                               *[q for q in self.waypoints]]))):
            raise ValueError('支撑恢复许可数据无效')
        canonical = build_support_recovery(self.start, self.target, self.limits)
        if any(np.max(np.abs(actual - expected)) > 1e-9
               for actual, expected in zip(self.waypoints, canonical['waypoints'])):
            raise ValueError('支撑恢复许可不是固定的已审核路线')
        self.stage = 0
        self.segment_start = self.start.copy()
        self.segment_target = None
        self.trajectory_start = None
        self.last_reference = None
        self.active = False

    def payload(self):
        return dict(start=self.start.tolist(), target=self.target.tolist(),
                    waypoints=[q.tolist() for q in self.waypoints],
                    strategy='fold-j3-then-lower-j2-then-support',
                    max_start_overrun_rad=MAX_START_OVERRUN_RAD,
                    recovery_torque_hard_limit_nm=RECOVERY_TORQUE_HARD_LIMIT_NM,
                    recovery_stage0_torque_limits_nm=RECOVERY_STAGE0_TORQUE_LIMITS_NM.tolist(),
                    recovery_torque_exception='stage 0 of this fixed support route only; drive limits are unchanged')

    def preview_segment(self, stage, start, target):
        if type(stage) is not int or not 0 <= stage < len(self.waypoints):
            raise ValueError('支撑恢复阶段无效')
        start, target = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
        expected_start = self.segment_start if stage == self.stage else (
            self.start if stage == 0 else self.waypoints[stage - 1])
        if (start.shape != (6,) or target.shape != (6,) or
                not np.all(np.isfinite(np.r_[start, target])) or
                np.max(np.abs(target - self.waypoints[stage])) > 1e-9):
            raise ValueError('支撑恢复路径与本次只读预览不一致')
        if self.active and stage == self.stage:
            self._check_resume_start(start, target)
        else:
            if np.max(np.abs(start - expected_start)) > START_MATCH_RAD:
                raise ValueError('支撑恢复路径与本次只读预览不一致')
            self._check_corridor(start, target, stage)

    def begin_segment(self, start, target):
        start, target = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
        if (start.shape != (6,) or target.shape != (6,) or
                not np.all(np.isfinite(np.r_[start, target])) or self.stage >= len(self.waypoints) or
                np.max(np.abs(target - self.waypoints[self.stage])) > 1e-9):
            raise ValueError('恢复目标不匹配已预览的下一段支撑路线')
        if self.active:
            if (self.trajectory_start is not None and
                    np.max(np.abs(start - self.trajectory_start)) <= 1e-9 and
                    np.max(np.abs(target - self.segment_target)) <= 1e-9):
                return
            self._check_resume_start(start, target)
            self.segment_start = start.copy()
            self.trajectory_start = start.copy()
            self.last_reference = start.copy()
            return
        if np.max(np.abs(start - self.segment_start)) > START_MATCH_RAD:
            raise ValueError('支撑恢复分段起点改变，请重新预览')
        self._check_corridor(start, target, self.stage)
        self.segment_start = start.copy()
        self.segment_target = target.copy()
        self.trajectory_start = start.copy()
        self.last_reference = start.copy()
        self.active = True

    def complete_segment(self, actual, arrival_tolerance_rad=START_MATCH_RAD):
        actual = np.asarray(actual, dtype=float)
        if (not self.active or actual.shape != (6,) or not np.all(np.isfinite(actual)) or
                np.max(np.abs(actual[1:] - self.segment_target[1:])) > arrival_tolerance_rad or
                abs(actual[0] - self.segment_target[0]) > START_MATCH_RAD):
            raise ValueError('支撑恢复分段未确认到位，不能进入下一段')
        self.stage += 1
        self.segment_start = actual.copy()
        self.segment_target = None
        self.trajectory_start = None
        self.last_reference = None
        self.active = False
        return self.stage >= len(self.waypoints)

    def allows_feedback(self, axis, value):
        return self._allows_axis_value(axis, value, feedback=True)

    def allows_command(self, axis, value, phase):
        if self.active:
            return self._allows_axis_value(axis, value, feedback=False)
        if self.limits[axis - 1, 0] <= value <= self.limits[axis - 1, 1]:
            return True
        if phase == 'preload-current':
            return abs(value - self.start[axis - 1]) <= START_MATCH_RAD
        if phase == 'hold' and not self.active:
            return abs(value - self.start[axis - 1]) <= START_MATCH_RAD
        return self._allows_axis_value(axis, value, feedback=False)

    def allows_reference(self, target):
        target = np.asarray(target, dtype=float)
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            return False
        if not self.active:
            return np.max(np.abs(target - self.segment_start)) <= START_MATCH_RAD
        if abs(target[0]-self.segment_target[0])>START_MATCH_RAD:
            return False
        for i, value in enumerate(target):
            begin, end = self.segment_start[i], self.segment_target[i]
            if end >= begin and not begin-START_MATCH_RAD <= value <= end+START_MATCH_RAD:
                return False
            if end < begin and not end-START_MATCH_RAD <= value <= begin+START_MATCH_RAD:
                return False
            previous = self.last_reference[i]
            if end >= begin and value < previous-1e-9:
                return False
            if end < begin and value > previous+1e-9:
                return False
        return True

    def advance_reference(self, target):
        target = np.asarray(target, dtype=float)
        if not self.allows_reference(target):
            raise ValueError('支撑恢复参考偏离已确认的单调回位走廊')
        self.last_reference = target.copy()

    def _check_resume_start(self, start, target):
        if (self.segment_target is None or
                np.max(np.abs(target - self.segment_target)) > 1e-9 or
                self.last_reference is None):
            raise ValueError('支撑恢复分段目标已改变，请重新预览')
        for index, (value, previous, end) in enumerate(zip(start, self.last_reference, target)):
            lower, upper = sorted((self.segment_start[index], end))
            if not lower-START_MATCH_RAD <= value <= upper+START_MATCH_RAD:
                raise ValueError(f'J{index+1} 恢复续跑起点偏离既定路段')
            if end >= previous and value < previous-START_MATCH_RAD:
                raise ValueError(f'J{index+1} 恢复续跑起点向外偏离')
            if end < previous and value > previous+START_MATCH_RAD:
                raise ValueError(f'J{index+1} 恢复续跑起点向外偏离')

    def _allows_axis_value(self, axis, value, *, feedback):
        index = axis - 1
        low, high = self.limits[index]
        if self.active:
            start = self.segment_start[index]
            target = self.segment_target[index]
            lower, upper = sorted((start, target))
            if not lower-START_MATCH_RAD <= value <= upper+START_MATCH_RAD:
                return False
            if target > start and value < start-START_MATCH_RAD:
                return False
            if target < start and value > start+START_MATCH_RAD:
                return False
            if feedback:
                return True
            previous = self.last_reference[index]
            return value >= previous-1e-9 if target >= start else value <= previous+1e-9
        if low <= value <= high:
            return True
        if index not in (1, 2):
            return False
        baseline = self.start[index]
        if baseline > high:
            if value < high or value > baseline + START_MATCH_RAD:
                return False
        elif baseline < low:
            if value > low or value < baseline - START_MATCH_RAD:
                return False
        else:
            return False
        return feedback and abs(value - baseline) <= START_MATCH_RAD

    def _check_corridor(self, start, target, stage):
        start, target = np.asarray(start, dtype=float), np.asarray(target, dtype=float)
        if (start.shape != (6,) or target.shape != (6,) or
                not np.all(np.isfinite(np.r_[start, target]))):
            raise ValueError('支撑恢复目标维度无效')
        expected_start = self.start if stage == 0 else self.waypoints[stage - 1]
        expected_target = self.waypoints[stage]
        if np.max(np.abs(start - expected_start)) > START_MATCH_RAD or np.max(np.abs(target - expected_target)) > 1e-9:
            raise ValueError('只允许沿已预览的单段支撑恢复目标移动')
        for i, (value, (low, high)) in enumerate(zip(target, self.limits)):
            if low <= value <= high:
                continue
            start_value = start[i]
            if i not in (1, 2) or not (
                    start_value > high and high <= value <= start_value + START_MATCH_RAD or
                    start_value < low and start_value - START_MATCH_RAD <= value <= low):
                raise ValueError(f'J{i+1} 支撑恢复目标继续越过名义限位')
        return True
