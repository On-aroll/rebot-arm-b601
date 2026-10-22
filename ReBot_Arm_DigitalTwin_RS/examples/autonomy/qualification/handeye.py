"""Manual stationary pose data -> OpenCV hand-eye solution + held-out residuals.

eye_in_hand: B_i X C_i = Y, X=T_tool_camera, Y=T_base_target.
eye_to_hand: inv(B_i) X C_i = Y, X=T_base_camera, Y=T_tool_target.
B=T_base_tool, C=T_camera_target. Neither robot nor camera is opened.
"""
import cv2
import numpy as np

from .common import identifier, number, require_schema, review


def transform(value, name):
    raw = np.asarray(value)
    if raw.dtype.kind not in 'iuf' or raw.shape != (4, 4):
        raise ValueError(f'{name}: expected numeric 4x4 transform')
    matrix = raw.astype(float)
    if (not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5, rtol=0)
            or abs(np.linalg.det(matrix[:3, :3]) - 1) > 1e-5
            or np.linalg.norm(matrix[:3, 3]) > 5):
        raise ValueError(f'{name}: invalid rigid transform or translation exceeds 5m; check units')
    return matrix


def rotation_vector(rotation):
    return cv2.Rodrigues(rotation)[0].reshape(3)


def excitation(poses):
    vectors = np.array([rotation_vector(poses[0][:3, :3].T @ p[:3, :3]) for p in poses[1:]])
    singular = np.linalg.svd(vectors, compute_uv=False)
    if singular[1] < np.deg2rad(2) or singular[1] / max(singular[0], 1e-12) < .05:
        raise ValueError('rotation excitation insufficient: need appreciable nonparallel rotations')
    return singular.tolist()


def mean_transform(poses):
    u, _, vt = np.linalg.svd(sum(p[:3, :3] for p in poses))
    mean = np.eye(4)
    mean[:3, :3] = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    mean[:3, 3] = np.mean([p[:3, 3] for p in poses], axis=0)
    return mean


def analyze(data):
    require_schema(data, 'b601-handeye-data/1.0')
    result = review('b601-handeye-review/1.0', data)
    mode = data['mode']
    if mode not in ('eye_in_hand', 'eye_to_hand') or data.get('units') != 'm' or data.get('time_basis') != 'same_clock_seconds':
        raise ValueError('explicit mode, meter units and same_clock_seconds timestamps are required')
    result.update({k:identifier(data.get(k), k) for k in ('calibration_id', 'camera_id', 'robot_id', 'target_id')})
    frames = {k:identifier(data['frames'][k], k) for k in ('base', 'tool', 'camera', 'target')}
    if len(set(frames.values())) != 4:raise ValueError('base/tool/camera/target frames must differ')
    limits = data['limits']
    pair_limit = number(limits['max_pair_dt_s'], 'max_pair_dt_s', .0001, .1)
    pixel_limit = number(limits['max_reprojection_rms_px'], 'max_reprojection_rms_px', .01, 5)
    translation_limit = number(limits['max_translation_error_mm'], 'max_translation_error_mm', .01, 20)
    rotation_limit = number(limits['max_rotation_error_deg'], 'max_rotation_error_deg', .001, 5)
    samples = data['samples']
    if not isinstance(samples, list) or not 9 <= len(samples) <= 100:
        raise ValueError('need 6+ training and 3+ held-out poses, at most 100 total')
    rows = [];ids = set();last_robot = last_camera = -1.
    for sample in samples:
        key = identifier(sample['id'], 'sample id')
        if key in ids:raise ValueError('sample ids must be unique')
        ids.add(key)
        robot_stamp = number(sample['robot_stamp_s'], 'robot_stamp_s', 0)
        camera_stamp = number(sample['camera_stamp_s'], 'camera_stamp_s', 0)
        if robot_stamp <= last_robot or camera_stamp <= last_camera or abs(robot_stamp - camera_stamp) > pair_limit:
            raise ValueError('timestamps duplicated, reversed or mismatched')
        last_robot, last_camera = robot_stamp, camera_stamp
        if sample.get('stationary') is not True or sample['split'] not in ('train', 'validation'):
            raise ValueError('only explicitly stationary train/validation samples are supported')
        if number(sample['reprojection_rms_px'], 'reprojection_rms_px', 0) > pixel_limit:
            raise ValueError('target reprojection error exceeds selected limit')
        b = transform(sample['T_base_tool'], 'T_base_tool')
        c = transform(sample['T_camera_target'], 'T_camera_target')
        if any(np.allclose(b, row['B'], atol=1e-7, rtol=0) for row in rows):
            raise ValueError('duplicate robot pose; held-out poses must not reuse training poses')
        rows.append(dict(id=key, split=sample['split'], B=b, C=c,
                         A=b if mode == 'eye_in_hand' else np.linalg.inv(b)))
    train = [r for r in rows if r['split'] == 'train']
    heldout = [r for r in rows if r['split'] == 'validation']
    if len(train) < 6 or len(heldout) < 3:raise ValueError('need 6+ train and 3+ validation poses')
    robot_excitation = excitation([r['A'] for r in train])
    camera_excitation = excitation([r['C'] for r in train])
    try:
        r, t = cv2.calibrateHandEye([v['A'][:3, :3] for v in train], [v['A'][:3, 3] for v in train],
            [v['C'][:3, :3] for v in train], [v['C'][:3, 3] for v in train], method=cv2.CALIB_HAND_EYE_PARK)
    except cv2.error as error:
        raise ValueError(f'hand-eye solver rejected these poses: {error}') from error
    x = np.eye(4);x[:3, :3] = r;x[:3, 3] = t.reshape(3)
    x = transform(x, 'solved hand-eye')
    reference = mean_transform([v['A'] @ x @ v['C'] for v in train])
    residuals = []
    for v in rows:
        actual = v['A'] @ x @ v['C']
        translation = float(np.linalg.norm(actual[:3, 3] - reference[:3, 3]) * 1000)
        angle = float(np.rad2deg(np.linalg.norm(rotation_vector(reference[:3, :3].T @ actual[:3, :3]))))
        residuals.append(dict(id=v['id'], split=v['split'], translation_error_mm=translation,
                              rotation_error_deg=angle, passed=translation <= translation_limit and angle <= rotation_limit))
    result.update(mode=mode, frames=frames, method='OpenCV CALIB_HAND_EYE_PARK', opencv_version=cv2.__version__,
        transform_name='T_tool_camera' if mode == 'eye_in_hand' else 'T_base_camera', transform=x.tolist(),
        constant_frame_transform=reference.tolist(), residuals=residuals,
        excitation_singular_values_rad=dict(robot=robot_excitation, camera=camera_excitation),
        validation_passed=all(r['passed'] for r in residuals), calibration_ready=False, limits=limits,
        limitations=['stationary pose replay; no camera, robot motion, board detection or intrinsic calibration',
                     'held-out closure consistency is not independent physical TCP accuracy',
                     'no automatic installation into runtime calibration'])
    return result
