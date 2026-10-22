"""Independent offline venv: python -m unittest discover -s examples/autonomy -p test_replay_official_vision.py."""
import copy
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

from replay_official_vision import DEFAULT_NODE, DEFAULT_UPSTREAM, EXPECTED, ROOT, load_geometry, replay


class OfficialGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.example = json.loads(subprocess.check_output([DEFAULT_NODE, str(ROOT / 'reBotArm_simulator-RS/scripts/offline-analysis.js'), 'example']))

    def run_replay(self, value):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'frame.json'
            path.write_text(json.dumps(value))
            return replay(path)

    def test_official_estimator_runs_with_real_pinned_modules_but_synthetic_detections(self):
        result = self.run_replay(self.example)
        self.assertFalse(result['hardware_execution_allowed'])
        self.assertEqual(result['source']['kind'], 'synthetic')
        self.assertEqual(result['stamp_ms'], 1000)
        pose = result['official_geometry']['pose_base_m_rad']
        for actual, expected in zip(pose[:3], [.108, 0, .2]):
            self.assertAlmostEqual(actual, expected, places=5)
        self.assertGreater(result['official_geometry']['jaw_width_m'], 0)
        self.assertFalse(any('reBotArm_control' in name or 'can.interface' in name for name in sys.modules))

    def test_image_time_wrist_rotation_changes_official_base_position(self):
        value = copy.deepcopy(self.example)
        value['robot_pose']['T_base_tool'] = [[0, -1, 0, .1], [1, 0, 0, 0], [0, 0, 1, -.2], [0, 0, 0, 1]]
        pose = self.run_replay(value)['official_geometry']['pose_base_m_rad']
        for actual, expected in zip(pose[:3], [.1, .008, .2]):
            self.assertAlmostEqual(actual, expected, places=5)

    def test_stale_pose_bad_depth_and_incomplete_obb_are_rejected(self):
        values = []
        value = copy.deepcopy(self.example); value['robot_pose']['stamp_ms'] += 51; values.append(value)
        value = copy.deepcopy(self.example); value['depth']['roi']['values'] = [[0] * 9] * 5; values.append(value)
        value = copy.deepcopy(self.example); value['detection']['obb_px'][0] = [0, 0]; values.append(value)
        value = copy.deepcopy(self.example); value['depth']['roi']['values'] = [[400] * 5] * 5; values.append(value)
        for value in values:
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.run_replay(value)

    def test_changed_upstream_source_is_rejected_before_import(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp); (root / 'utils').mkdir()
            for name in EXPECTED:
                shutil.copyfile(DEFAULT_UPSTREAM / 'utils' / name, root / 'utils' / name)
            with (root / 'utils/ordinary_grasp.py').open('a') as stream:
                stream.write('\n# changed\n')
            with self.assertRaisesRegex(ValueError, '源码校验失败'):
                load_geometry(root)


if __name__ == '__main__':
    unittest.main()
