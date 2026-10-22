"""Recover both transform directions from synthetic truth and reject bad data."""
from copy import deepcopy
from pathlib import Path
import unittest

import numpy as np
from qualification.common import read_json
from qualification.handeye import analyze

FIXTURES = Path(__file__).parent / 'fixtures/qualification'


class HandeyeTests(unittest.TestCase):
    def data(self, mount='wrist'):return read_json(FIXTURES / f'handeye-{mount}.synthetic.json')

    def test_known_wrist_and_fixed_transforms_recovered_without_using_ground_truth(self):
        for mount, name in [('wrist', 'T_tool_camera'), ('fixed', 'T_base_camera')]:
            with self.subTest(mount=mount):
                data = self.data(mount);expected = np.array(data.pop('synthetic_ground_truth_transform'))
                result = analyze(data)
                np.testing.assert_allclose(result['transform'], expected, atol=1e-8)
                self.assertEqual(result['transform_name'], name);self.assertTrue(result['validation_passed'])
                self.assertFalse(result['calibration_ready']);self.assertFalse(result['hardware_execution_allowed'])

    def test_heldout_error_does_not_change_solution_and_fails_validation(self):
        data = self.data();baseline = analyze(data)
        data['samples'][-1]['T_camera_target'][0][3] += .02
        result = analyze(data)
        np.testing.assert_allclose(result['transform'], baseline['transform'], atol=1e-10)
        self.assertFalse(result['validation_passed'])
        self.assertGreater(result['residuals'][-1]['translation_error_mm'], 19)

    def test_bad_time_units_rotation_stationarity_frame_and_holdout_reuse_rejected(self):
        mutations = [lambda d:d['samples'][0].update(camera_stamp_s=101),
            lambda d:d.update(units='mm'), lambda d:d['samples'][0]['T_camera_target'][0].__setitem__(0, 2),
            lambda d:d['samples'][0].update(stationary=False), lambda d:d['frames'].update(camera='base_link'),
            lambda d:d['samples'][-1].update(T_base_tool=deepcopy(d['samples'][0]['T_base_tool'])),
            lambda d:d['samples'][1].update(robot_stamp_s=d['samples'][0]['robot_stamp_s'])]
        for change in mutations:
            bad = self.data();change(bad)
            with self.assertRaises(ValueError):analyze(bad)

    def test_parallel_rotation_excitation_is_rejected(self):
        from qualification.handeye import excitation
        from make_qualification_examples import pose
        with self.assertRaisesRegex(ValueError, 'excitation'):
            excitation([pose([0, 0, i * .1], [0, 0, .2]) for i in range(8)])

    def test_wrong_frame_direction_fails_closure(self):
        data = self.data()
        for s in data['samples']:s['T_camera_target'] = np.linalg.inv(s['T_camera_target']).tolist()
        try:result = analyze(data)
        except ValueError:return
        self.assertFalse(result['validation_passed'])


if __name__ == '__main__':unittest.main()
