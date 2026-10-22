"""Sample current joint-space Ruckig references or a saved six-axis segment.

FK is model based. This does not prove physical TCP accuracy or collision safety.
"""
import hashlib
from pathlib import Path

import numpy as np
import pinocchio as pin
from rebotarmcontroller.twin_kinematics import TwinKinematics
from rebotarmcontroller.twin_motion import point_duration, point_sample

from .common import number, require_schema, review

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / 'rebotarm_ros2_RS/src/rebotarm_bringup/description/urdf/ReBot_Arm_RS.urdf'


def vector(value, name, size=6):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f'{name}: expected {size} numeric values')
    return np.array([number(v, name) for v in value])


def segment_report(samples, kin, offset, limits, name):
    if not 3 <= len(samples) <= 50000:raise ValueError('segment needs 3–50000 samples')
    times = np.array([number(v['t_s'], 't_s', 0) for v in samples])
    joints = np.array([vector(v['q_rad'], 'q_rad') for v in samples])
    gaps = np.diff(times)
    if np.any(gaps <= 0):raise ValueError('sample timestamps duplicated or reversed')
    if np.any(joints < kin.lower - 1e-9) or np.any(joints > kin.upper + 1e-9):
        raise ValueError('joint sample outside URDF limits')
    transforms = [kin.placement(q) for q in joints]
    xyz = np.array([t.translation + t.rotation @ offset for t in transforms])
    chord = xyz[-1] - xyz[0];length2 = float(chord @ chord)
    u = np.clip((xyz - xyz[0]) @ chord / length2, 0, 1) if length2 > 1e-16 else np.zeros(len(xyz))
    line_errors = np.linalg.norm(xyz - (xyz[0] + u[:, None] * chord), axis=1) * 1000
    delta = joints[-1] - joints[0];joint_length2 = float(delta @ delta)
    progress = np.clip((joints - joints[0]) @ delta / joint_length2, 0, 1) if joint_length2 > 1e-16 else np.linspace(0, 1, len(joints))
    rotation0 = transforms[0].rotation;rotation_delta = pin.log3(rotation0.T @ transforms[-1].rotation)
    angles = [float(np.rad2deg(np.linalg.norm(pin.log3((rotation0 @ pin.exp3(rotation_delta * blend)).T @ t.rotation))))
              for blend, t in zip(progress, transforms)]
    joint_steps = np.max(np.abs(np.diff(joints, axis=0)), axis=1)
    tcp_speed = np.linalg.norm(np.diff(xyz, axis=0), axis=1) / gaps * 1000
    failed = []
    if float(max(line_errors)) > limits['max_chord_error_mm']:failed.append('TCP chord deviation')
    if float(max(angles)) > limits['max_orientation_deviation_deg']:failed.append('orientation interpolation deviation')
    if float(max(joint_steps)) > limits['max_joint_step_rad']:failed.append('joint sample step')
    if float(max(gaps)) > limits['max_sample_gap_s']:failed.append('sample interval')
    stride = max(1, int(np.ceil((len(samples) - 1) / 1000)))
    selected = sorted(set([*range(0, len(samples), stride), len(samples) - 1, int(np.argmax(line_errors))]))
    return dict(id=name, sample_count=len(samples), duration_s=float(times[-1] - times[0]),
        chord_length_mm=float(np.sqrt(length2) * 1000), max_chord_error_mm=float(max(line_errors)),
        max_orientation_deviation_deg=float(max(angles)), max_joint_step_rad=float(max(joint_steps)),
        timing_s=dict(median=float(np.median(gaps)), p95=float(np.quantile(gaps, .95)), maximum=float(max(gaps))),
        tcp_speed_mm_s=dict(median=float(np.median(tcp_speed)), maximum=float(max(tcp_speed))),
        validation_passed=not failed, exceeded=failed,
        samples=[dict(t_s=float(times[i]), q_rad=joints[i].tolist(), tcp_m=xyz[i].tolist(),
                      chord_error_mm=float(line_errors[i])) for i in selected])


def analyze(data, kin=None):
    require_schema(data, 'b601-path-review-input/1.0')
    result = review('b601-path-review/1.0', data)
    if data.get('units') != dict(position='rad', translation='m', time='s'):
        raise ValueError('explicit rad/m/s unit declaration required')
    model = dict(kind='injected-model') if kin is not None else dict(
        kind='project-urdf', path=URDF.relative_to(ROOT).as_posix(), sha256=hashlib.sha256(URDF.read_bytes()).hexdigest())
    kin = kin or TwinKinematics(urdf=URDF)
    offset = vector(data['tcp_offset_m'], 'tcp_offset_m', 3)
    if np.linalg.norm(offset) > .5:raise ValueError('TCP offset exceeds 0.5m review range')
    names = ('max_chord_error_mm', 'max_orientation_deviation_deg', 'max_joint_step_rad', 'max_sample_gap_s')
    limits = {k:number(data['limits'][k], k, 1e-6) for k in names}
    reports = []
    if data['mode'] == 'reference_segments':
        period = number(data['period_s'], 'period_s', .001, .05)
        segments = data['segments']
        if not isinstance(segments, list) or not 1 <= len(segments) <= 50:
            raise ValueError('need 1–50 reference segments')
        previous = None
        for i, row in enumerate(segments):
            start, goal = vector(row['start_rad'], 'start_rad'), vector(row['goal_rad'], 'goal_rad')
            if any(np.any(q < kin.lower) or np.any(q > kin.upper) for q in (start, goal)):
                raise ValueError('joint endpoint outside URDF limits')
            if previous is not None and np.max(np.abs(start - previous)) > 1e-8:
                raise ValueError('adjacent reference segments have disconnected joint endpoints')
            previous = goal
            speed = number(row['speed_rad_s'], 'speed_rad_s', .0001, 1.5)
            acceleration = number(row['acceleration_rad_s2'], 'acceleration_rad_s2', .0001, 20)
            jerk = number(row['jerk_rad_s3'], 'jerk_rad_s3', .0001, 200)
            duration = number(row['minimum_duration_s'], 'minimum_duration_s', .05, 120)
            total = point_duration(start, goal, speed, acceleration, jerk, duration)
            count = max(51, int(np.ceil(total / period)) + 1)
            if count > 50000:raise ValueError('trajectory exceeds 50000-sample review budget')
            samples = [dict(t_s=float(t), q_rad=point_sample('ruckig', start, goal, speed, acceleration,
                jerk, duration, float(t)).tolist()) for t in np.linspace(0, total, count)]
            reports.append(segment_report(samples, kin, offset, limits, f'segment-{i + 1}'))
    elif data['mode'] == 'recorded_segment':
        reports.append(segment_report(data['samples'], kin, offset, limits, 'recorded-segment'))
    else:raise ValueError('mode must be reference_segments or recorded_segment')
    result.update(mode=data['mode'], model=model, tcp_frame='gripper_end + supplied rigid offset', tcp_offset_m=offset.tolist(),
        limits=limits, segments=reports, validation_passed=all(r['validation_passed'] for r in reports),
        limitations=['sampled model FK only; not an independent physical TCP measurement',
                     'no collision, load, gravity, continuous-curve bound or welding process validation',
                     'sample gaps identify timing symptoms; no automatic PID/current/mode changes'])
    return result
