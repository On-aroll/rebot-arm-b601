"""Known circular path geometry and actual B601 FK, without ROS nodes or SDK."""
from copy import deepcopy
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'rebotarm_ros2_RS/src/rebotarmcontroller'))
import numpy as np
import pinocchio as pin
from qualification.common import read_json
from qualification.path_review import analyze, segment_report


class Circle:
    lower = np.full(6, -10.)
    upper = np.full(6, 10.)
    def placement(self, q):
        return SimpleNamespace(translation=np.array([math.cos(q[0]), math.sin(q[0]), 0]),
                               rotation=pin.exp3(np.array([0., 0., q[0]])))


class PathTests(unittest.TestCase):
    def data(self):return read_json(Path(__file__).parent / 'fixtures/qualification/path.synthetic.json')

    def test_circle_sagitta_matches_analytic_result_and_tcp_offset_is_applied(self):
        samples = [dict(t_s=i * .02, q_rad=[i * .002, 0, 0, 0, 0, 0]) for i in range(101)]
        limits = self.data()['limits'];limits['max_chord_error_mm'] = 1
        report = segment_report(samples, Circle(), np.zeros(3), limits, 'circle')
        expected = (1 - math.cos(.1)) * 1000
        self.assertAlmostEqual(report['max_chord_error_mm'], expected, places=7)
        self.assertFalse(report['validation_passed'])
        offset = segment_report(samples, Circle(), np.array([1, 0, 0]), limits, 'offset')
        self.assertAlmostEqual(offset['max_chord_error_mm'], expected * 2, places=7)

    def test_recorded_gap_and_joint_jump_exceed_selected_budgets(self):
        data = self.data();data['mode'] = 'recorded_segment'
        data['samples'] = [dict(t_s=0., q_rad=[0]*6), dict(t_s=.02, q_rad=[.05, 0, 0, 0, 0, 0]),
                           dict(t_s=.5, q_rad=[.1, 0, 0, 0, 0, 0])]
        result = analyze(data, kin=Circle())
        self.assertIn('sample interval', result['segments'][0]['exceeded'])
        self.assertIn('joint sample step', result['segments'][0]['exceeded'])
        self.assertAlmostEqual(result['segments'][0]['timing_s']['maximum'], .48)

    def test_actual_model_reference_exposes_tcp_arc_without_authorizing_execution(self):
        result = analyze(self.data())
        self.assertGreater(result['segments'][0]['max_chord_error_mm'], .1)
        self.assertFalse(result['hardware_execution_allowed'])
        self.assertFalse(result['validation_passed'])
        self.assertNotIn('motorbridge', sys.modules);self.assertNotIn('rclpy', sys.modules)

    def test_units_bad_times_disconnected_segments_and_joint_limits_rejected(self):
        bad = self.data();bad['units']['translation'] = 'mm'
        with self.assertRaises(ValueError):analyze(bad, kin=Circle())
        bad = self.data();extra = deepcopy(bad['segments'][0]);extra['start_rad'][0] += .01;bad['segments'].append(extra)
        with self.assertRaisesRegex(ValueError, 'disconnected'):analyze(bad, kin=Circle())
        bad = self.data();bad['segments'][0]['goal_rad'][0] = 12
        with self.assertRaises(ValueError):analyze(bad, kin=Circle())
        bad = self.data();bad['mode'] = 'recorded_segment';bad['samples'] = [dict(t_s=0, q_rad=[0]*6)] * 3
        with self.assertRaisesRegex(ValueError, 'timestamps'):analyze(bad, kin=Circle())


if __name__ == '__main__':unittest.main()
