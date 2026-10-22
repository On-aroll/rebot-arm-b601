"""Run pinned Seeed OBB geometry on saved detections/depth; never import its robot SDK.

Input: b601-rgbd-replay/1.0 with detection.obb_px (four cyclic pixel corners).
The shared JS reviewer checks time, calibration, alignment and depth first.
This is a geometry replay, not a YOLO inference or executable robot program.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys
from types import ModuleType, SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
SHA = '547faa08e5161af996892497c0aaa788401454fc'
EXPECTED = {
    'common_utils.py': 'cd99399b3919af5a5f4297305765ae4b268ab6d95c0be1524ebed9263383c9ca',
    'transforms.py': '4549a022952bd506bb247a66dffdc4a9ef25342436d97b93a322c255c8f0762b',
    'ordinary_grasp.py': 'd63fd0b04df374dfa23d043e0886f71396b5849698171c3db5997a31bbcc6e78',
}
DEFAULT_UPSTREAM = ROOT / 'outputs/autonomy-research-20261018/upstream/Seeed-Projects__reBot-DevArm-Grasp'
DEFAULT_NODE = shutil.which('node') or str(Path.home() / '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node')


def load_geometry(upstream: Path):
    """Hash all executed files, then load only three pure perception modules."""
    paths = {name: upstream / 'utils' / name for name in EXPECTED}
    for name, path in paths.items():
        if hashlib.sha256(path.read_bytes()).hexdigest() != EXPECTED[name]:
            raise ValueError(f'官方源码校验失败: {path}')
    package = '_b601_pinned_vision'
    module = ModuleType(package)
    module.__path__ = [str(upstream / 'utils')]
    sys.modules[package] = module
    loaded = {}
    for name, path in paths.items():
        qualified = f'{package}.{path.stem}'
        spec = importlib.util.spec_from_file_location(qualified, path)
        item = importlib.util.module_from_spec(spec)
        sys.modules[qualified] = item
        spec.loader.exec_module(item)
        loaded[path.stem] = item
    return loaded['ordinary_grasp'], loaded['transforms']


def replay(input_path: Path, upstream: Path = DEFAULT_UPSTREAM, node: str = DEFAULT_NODE):
    import numpy as np

    raw = input_path.read_bytes()
    if len(raw) > 8 * 1024 * 1024:
        raise ValueError('回放文件须小于8MB')
    value = json.loads(raw)
    checked = subprocess.run([node, str(ROOT / 'reBotArm_simulator-RS/scripts/offline-analysis.js'), 'vision', str(input_path)],
                             check=False, capture_output=True, text=True, timeout=10)
    if checked.returncode:
        raise ValueError(checked.stderr.strip())
    review = json.loads(checked.stdout)
    camera, detection, patch = value['camera'], value['detection'], value['depth']['roi']
    corners = np.asarray(detection.get('obb_px'), dtype=float)
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ValueError('官方OBB回放需要4×2有限像素角点，按边缘顺序排列')
    if (corners[:, 0] < 2).any() or (corners[:, 0] >= camera['width']).any() or (corners[:, 1] < 2).any() or (corners[:, 1] >= camera['height']).any():
        raise ValueError('OBB角点须在图像内；本适配器使用绝对像素，排除上游归一化歧义')
    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    if (lengths < 2).any() or abs(np.dot(edges[0], edges[1])) > 1e-3 * lengths[0] * lengths[1] or not np.allclose(edges[:2], -edges[2:], atol=1e-3):
        raise ValueError('OBB必须是非退化矩形，角点按边缘顺序排列')
    if np.linalg.norm(corners.mean(axis=0) - detection['center_px']) > 1:
        raise ValueError('OBB中心与已复核检测中心不一致')
    if (corners[:, 0] < patch['x']).any() or (corners[:, 0] > patch['x'] + len(patch['values'][0]) - 1).any() or (corners[:, 1] < patch['y']).any() or (corners[:, 1] > patch['y'] + len(patch['values']) - 1).any():
        raise ValueError('官方OBB深度分位数需要ROI覆盖完整OBB')
    # Bound allocation independently of browser limits.
    if camera['width'] * camera['height'] > 4_000_000:
        raise ValueError('离线OBB图像最多400万像素')
    depth = np.zeros((camera['height'], camera['width']), dtype=np.float32)
    values = np.asarray(patch['values'], dtype=np.float32) * value['depth']['scale_m'] * 1000
    depth[patch['y']:patch['y'] + values.shape[0], patch['x']:patch['x'] + values.shape[1]] = values
    # Use the same reviewed 0.05–1m depth range throughout the full OBB mask.
    depth[(depth < 50) | (depth > 1000)] = 0
    result = SimpleNamespace(names={0: detection['label']}, orig_shape=depth.shape, masks=None,
                             obb=SimpleNamespace(xyxyxyxy=np.array([corners]), cls=np.array([0]), conf=np.array([detection['confidence']])))
    estimate, transforms = load_geometry(upstream)
    grasp = estimate.estimate_grasp(result, 0, depth, np.array(camera['K']).reshape(3, 3), depth_quantile=.5)
    if not grasp.is_valid:
        raise ValueError(f'官方OBB位姿估计拒绝: {grasp.rejected_reason}')
    pose, pregrasp = transforms.transform_grasp_pose_to_base(
        grasp.position, grasp.tcp_rotation, np.array(review['geometry']['T_base_camera']), .03, insertion_depth_m=0)
    if not all(math.isfinite(x) for x in (*pose, *pregrasp)) or any(abs(x) > .8 for x in (*pose[:3], *pregrasp[:3])):
        raise ValueError('官方输出无效或超过参考工作范围')
    center_review = review.pop('geometry')
    return {**review, 'schema_version': 'b601-official-obb-review/1.0',
            'target': {**review['target'], 'position': list(pose[:3])},
            'center_depth_review': center_review,
            'upstream': {'repo': 'Seeed-Projects/reBot-DevArm-Grasp', 'commit': SHA, 'sha256': EXPECTED},
            'input_sha256': hashlib.sha256(raw).hexdigest(),
            'official_geometry': {'pose_base_m_rad': list(pose), 'pregrasp_base_m_rad': list(pregrasp),
                                  'center_px': list(grasp.center_px), 'jaw_width_m': grasp.jaw_width_m,
                                  'valid_depth_pixels': grasp.valid_depth_pixels, 'depth_quantile': .5,
                                  'pregrasp_offset_m': .03, 'insertion_depth_m': 0},
            'pending': ['未运行YOLO推理', '未验证相机/手眼/TCP/开口标定', '未做IK/碰撞/真实夹取验证']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--upstream', type=Path, default=DEFAULT_UPSTREAM)
    parser.add_argument('--node', default=DEFAULT_NODE)
    args = parser.parse_args()
    try:
        print(json.dumps(replay(args.input, args.upstream, args.node), ensure_ascii=False, indent=2, allow_nan=False))
    except (ValueError, OSError, subprocess.TimeoutExpired) as error:
        parser.exit(1, str(error) + '\n')


if __name__ == '__main__':
    main()
