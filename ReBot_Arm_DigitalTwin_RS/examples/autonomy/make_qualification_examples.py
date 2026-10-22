#!/usr/bin/env python3
"""Generate explicitly synthetic examples; never read or command a robot."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def pose(rotation, translation):
    value = np.eye(4);value[:3, :3] = cv2.Rodrigues(np.array(rotation, dtype=float))[0]
    value[:3, 3] = translation
    return value


def handeye(mode):
    rng = np.random.default_rng(41)
    x = pose([.18, -.12, .25], [.04, -.03, .08] if mode == 'eye_in_hand' else [.3, -.4, .5])
    constant = pose([-.1, .25, .2], [.3, .1, .4] if mode == 'eye_in_hand' else [.02, .01, .05])
    samples = []
    for i in range(12):
        b = pose(rng.uniform(-.6, .6, 3), [.2 + .04 * np.sin(i), .1 + .04 * np.cos(i), .3 + i * .005])
        c = np.linalg.inv(x) @ np.linalg.inv(b) @ constant if mode == 'eye_in_hand' else np.linalg.inv(x) @ b @ constant
        samples.append(dict(id=f'synthetic-{i}', split='train' if i < 8 else 'validation',
            stationary=True, robot_stamp_s=100 + i, camera_stamp_s=100 + i + .001,
            reprojection_rms_px=.1, T_base_tool=b.tolist(), T_camera_target=c.tolist()))
    return dict(schema_version='b601-handeye-data/1.0', source=dict(kind='synthetic'),
        mode=mode, calibration_id='synthetic-handeye-only', camera_id='synthetic-camera', robot_id='synthetic-robot',
        target_id='synthetic-board', units='m', time_basis='same_clock_seconds',
        frames=dict(base='base_link', tool='gripper_end', camera='camera_optical_frame', target='calibration_board'),
        limits=dict(max_pair_dt_s=.02, max_reprojection_rms_px=1,
                    max_translation_error_mm=2, max_rotation_error_deg=.5),
        samples=samples, synthetic_ground_truth_transform=x.tolist())


def jaw_width():
    def row(q, offset):return dict(position_rad=q, width_mm=12 * q + .2 * q * q + offset)
    return dict(schema_version='b601-j7-width-data/1.0', source=dict(kind='synthetic'),
        calibration_id='synthetic-jaw-only', device_id='synthetic-robot', gripper_id='synthetic-j7',
        max_validation_error_mm=.3,
        measurements={d:[row(q, offset) for q in [.2, 1.4, 2.8, 4.8]] for d, offset in [('opening', .4), ('closing', 0)]},
        validation={d:[row(q, offset) for q in [.8, 2., 3.8]] for d, offset in [('opening', .4), ('closing', 0)]})


def path():
    return dict(schema_version='b601-path-review-input/1.0', source=dict(kind='synthetic'),
        units=dict(position='rad', translation='m', time='s'), mode='reference_segments',
        tcp_offset_m=[0, 0, 0], period_s=.02,
        limits=dict(max_chord_error_mm=.1, max_orientation_deviation_deg=1,
                    max_joint_step_rad=.02, max_sample_gap_s=.05),
        segments=[dict(start_rad=[.7, .3, .4, -.1, .2, .2], goal_rad=[.9, .3, .4, -.1, .2, .2],
                       speed_rad_s=.1, acceleration_rad_s2=.3, jerk_rad_s3=2, minimum_duration_s=.5)])


def grasp():
    profile = dict(empty_position=.2, empty_tolerance=.02, min_gap=.05, contact_current_a=.4,
        abort_current_a=2., stall_velocity=.01, min_travel=.05, dwell=.3, hold_time=.3, timeout=12.)
    trials = [];truth = {}
    for label in ['object', 'fixture', 'empty', 'slip']:
        samples = []
        count = 105 if label == 'empty' else 76
        for i in range(count):
            stamp = 100 + i * .1
            if label == 'empty':
                q = max(.2, 1.2 - i * .01);velocity = -.1 if q > .2 else 0.;current = .1
            else:
                q = max(.6, 1.2 - i * .01);velocity = -.1 if i < 60 else 0.;current = .1 if i < 60 else .6
                if label == 'slip' and i >= 70:current = 0.
            samples.append(dict(sample_at=stamp, received_ms=100000 + i * 100, position=q,
                velocity=velocity, current_a=current, reference=max(0, q - .02), grasp_status=''))
        trials.append(dict(id=label, label='empty' if label == 'empty' else 'object', source='fake', samples=samples))
        truth[label] = dict(label=label, **(dict(event_at=107.) if label == 'slip' else {}))
    return dict(schema_version='b601-grasp-evaluation/1.0', source=dict(kind='synthetic'), profile=profile,
        recording=dict(schema_version='b601-j7-recording/1.0', trials=trials), truth=truth)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args();args.output_dir.mkdir(parents=True, exist_ok=True)
    examples = {'handeye-wrist.synthetic.json':handeye('eye_in_hand'),
                'handeye-fixed.synthetic.json':handeye('eye_to_hand'),
                'j7-width.synthetic.json':jaw_width(), 'path.synthetic.json':path(), 'grasp.synthetic.json':grasp()}
    for name, data in examples.items():
        with (args.output_dir / name).open('x') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False);stream.write('\n')


if __name__ == '__main__':main()
