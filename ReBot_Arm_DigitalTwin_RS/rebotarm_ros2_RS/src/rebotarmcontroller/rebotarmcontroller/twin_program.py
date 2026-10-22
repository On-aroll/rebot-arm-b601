"""Versioned, inert motion data. Importing a program never executes code."""
import math
import time
from numbers import Real
from .twin_grasp import validate_profile, MAX_GRASP_SPEED

VERSION = '1.0'
CALIBRATION = 'rs-identity-v1'
MAX_STEPS = 250


def number(value, label, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f'{label} 必须是有限数值')
    if low is not None and value < low or high is not None and value > high:
        raise ValueError(f'{label} 超出范围 {low}–{high}')
    return float(value)


def vector(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{label} 需要 {length} 个数值')
    return [number(v, label) for v in value]


def validate_program(raw, *, calibration=CALIBRATION):
    if not isinstance(raw, dict) or raw.get('schema_version') != VERSION:
        raise ValueError('程序 schema_version 必须为 1.0')
    if raw.get('robot_model') != 'b601_rs' or raw.get('base_frame') != 'base_link':
        raise ValueError('程序必须使用 b601_rs / base_link')
    if raw.get('calibration_id') != calibration:
        raise ValueError('程序标定版本与当前机器人不一致')
    if raw.get('units') != {'position': 'm', 'angle': 'rad', 'time': 's'}:
        raise ValueError('程序单位必须为 m / rad / s')
    steps = raw.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= MAX_STEPS:
        raise ValueError(f'程序需要 1–{MAX_STEPS} 步；长录制请抽取关键帧')
    result = dict(schema_version=VERSION, robot_model='b601_rs', base_frame='base_link',
                  calibration_id=calibration, units=raw['units'], name=str(raw.get('name', '未命名程序'))[:100], steps=[])
    for i, step in enumerate(steps):
        if not isinstance(step, dict):
            raise ValueError(f'第 {i+1} 步不是对象')
        kind = step.get('type')
        fields = {'joint': {'joints', 'speed', 'duration'}, 'pose': {'pose', 'orientation', 'speed', 'duration'},
                  'gripper': {'position', 'speed'}, 'grasp': {'profile', 'speed'}, 'wait': {'seconds'}}
        if kind not in fields or set(step) - fields[kind] - {'type', 'label'}:
            raise ValueError(f'第 {i+1} 步包含不支持的动作或字段')
        s = dict(type=kind, label=str(step.get('label', f'步骤 {i+1}'))[:100])
        if kind in ('joint', 'pose'):
            key = 'joints' if kind == 'joint' else 'pose'
            s[key] = vector(step.get(key), 6, f'第 {i+1} 步 {key}')
            s['speed'] = number(step.get('speed', .1), '关节速度', .005, 1.5)
            if 'duration' in step:
                s['duration'] = number(step['duration'], '时长', .05, 3600)
            if kind == 'pose':
                if type(step.get('orientation', False)) is not bool:
                    raise ValueError('orientation 必须是布尔值')
                s['orientation'] = step.get('orientation', False)
        elif kind == 'gripper':
            s.update(position=number(step.get('position'), 'J7 电机角度', 0, 5),
                     speed=number(step.get('speed', .1), 'J7 速度', .005, 5))
        elif kind == 'grasp':
            s.update(profile=validate_profile(step.get('profile')),
                     speed=number(step.get('speed', .1), '接触检测闭合速度', .005, MAX_GRASP_SPEED))
            if s['speed'] <= s['profile']['stall_velocity']*2:
                raise ValueError('闭合速度须大于停滞阈值的两倍')
        else:
            s['seconds'] = number(step.get('seconds'), '等待秒数', 0, 3600)
        result['steps'].append(s)
    return result


def compile_program(raw, start, kin, hardware, duration_for, *, calibration=CALIBRATION,
                    support_recovery_plan=None, support_recovery_stage_offset=0):
    program = validate_program(raw, calibration=calibration)
    recovery = support_recovery_plan is not None
    if recovery:
        if not callable(getattr(hardware, 'validate_support_recovery_preview', None)):
            raise ValueError('驱动没有支撑恢复专用路径校验')
        if (type(support_recovery_stage_offset) is not int or not 0 <= support_recovery_stage_offset < 3 or
                len(program['steps']) != 3-support_recovery_stage_offset or
                any(s['type'] != 'joint' for s in program['steps'])):
            raise ValueError('支撑恢复程序必须包含固定路线的全部剩余关节步骤')
    seed = vector(list(start), 6, '程序起点')
    compiled = []
    gripper_seed = None
    gripper_uncertain = False
    deadline = time.monotonic() + 25
    for index, step in enumerate(program['steps']):
        if time.monotonic() > deadline:
            raise ValueError('整套预览超过25秒，请拆分程序或减少采样点')
        item = dict(step, index=index)
        try:
            if step['type'] in ('joint', 'pose'):
                speed = step['speed']
                if not getattr(hardware, 'min_twin_speed', .05) <= speed <= getattr(hardware, 'max_twin_speed', .3):
                    raise ValueError('速度超过当前控制范围；请调整程序速度或选择完整控制')
                if step['type'] == 'pose':
                    joints, _ = kin.solve(step['pose'], seed, step['orientation'],
                                          path_check=getattr(hardware, 'validate_preview', None))
                else:
                    # Floating point FK/physics noise at a nominal endpoint
                    # is not an intended outside-limit command. Project only
                    # numerical epsilon, never a physical boundary offset.
                    joints = [min(hi,max(lo,q)) if lo-1e-9 <= q <= hi+1e-9 else q
                              for q,(lo,hi) in zip(step['joints'],hardware.teleop_limits)]
                    if recovery:
                        hardware.validate_support_recovery_preview(seed,joints,support_recovery_plan,
                                                                  support_recovery_stage_offset+index)
                    else:
                        if any(not lo <= q <= hi for q, (lo, hi) in zip(joints, hardware.teleop_limits)):
                            raise ValueError('关节目标越界')
                        if hasattr(hardware, 'validate_preview'):
                            hardware.validate_preview(seed, joints)
                minimum = duration_for(seed, joints, speed)
                duration = max(minimum, step.get('duration', 0))
                item.update(start=seed[:], joints=list(joints), pose=kin.pose(joints), orientation=True,
                            support_recovery=bool(support_recovery_plan),
                            duration=duration, retimed=step.get('duration', minimum) < minimum)
                seed = list(joints)
            elif step['type'] in ('gripper', 'grasp'):
                caps = hardware.gripper_state() if hasattr(hardware, 'gripper_state') else {}
                if not caps.get('available') or not caps.get('fresh') or caps.get('recovery_required'):
                    raise ValueError('请先连接并检查 J7 反馈，再预览含夹爪的程序')
                if step['speed'] > caps.get('speed_max', 0):
                    raise ValueError('夹爪速度超过设备与官方上限的较小值')
                if gripper_seed is None:
                    gripper_seed = number(caps.get('position'), 'J7起点', 0, 5)
                    if abs(caps.get('velocity') or 0) > .01:raise ValueError('J7仍在运动，请先暂停后预览程序')
                item['start_position'] = gripper_seed
                if step['type'] == 'grasp':
                    if not caps.get('grasp_supported') or not hasattr(hardware, 'gripper_grasp'):
                        raise ValueError('驱动尚不支持带反馈的夹持判定，请更新驱动')
                    p = step['profile']
                    if gripper_uncertain or gripper_seed < p['empty_position']+p['empty_tolerance']+p['min_gap']+p['min_travel']:
                        raise ValueError('夹持前须有已知张开步骤，开度不能不足')
                    item['duration'] = p['timeout']
                    gripper_uncertain = True
                else:
                    delta = 5. if gripper_uncertain else abs(step['position']-gripper_seed)
                    item['duration'] = max(.5, 1.875*delta/step['speed'], math.sqrt(5.78*delta/4), (60*delta/30)**(1/3))
                    gripper_seed = step['position']
                    gripper_uncertain = False
            else:
                item['duration'] = step['seconds']
        except (ValueError, RuntimeError) as exc:
            raise ValueError(f'第 {index+1} 步「{step["label"]}」：{exc}') from exc
        compiled.append(item)
    return program, compiled
