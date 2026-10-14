"""Offline FK display checks: no gateway, no CAN and no pose initialization."""
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rebot_core import Robot
from grasp_demo import build_scene
from rebot_kinematics import coordinate_preview


class CoordinatePreviewTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Robot(build_scene())

    def test_feedback_at_boundary_keeps_actual_pose_target_and_simulation_state(self):
        r = self.robot
        qpos, qvel, target, finger = r.d.qpos.copy(), r.d.qvel.copy(), r.target.copy(), r.finger_target
        result = coordinate_preview(r, [62, -.39, .1, .6, -.9, 4.5])
        self.assertEqual(result['frame'], 'base_link')
        self.assertEqual(result['control_point'], 'gripper_end')
        self.assertFalse(result['hardware_command'])
        self.assertEqual(result['q_deg'][1], -.39)
        self.assertTrue(np.isfinite(result['xyz_mm']).all())
        self.assertTrue(np.array_equal(qpos, r.d.qpos))
        self.assertTrue(np.array_equal(qvel, r.d.qvel))
        self.assertTrue(np.array_equal(target, r.target))
        self.assertEqual(finger, r.finger_target)

    def test_invalid_coordinates_are_rejected(self):
        for q in (None, [], [0]*7, [True, 0, 0, 0, 0, 0], ['0']*6, [float('nan')]*6, [float('inf')]*6):
            with self.subTest(q=q), self.assertRaises(ValueError):
                coordinate_preview(self.robot, q)


if __name__ == '__main__':
    unittest.main()
