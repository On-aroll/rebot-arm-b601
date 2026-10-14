"""Point previews from measured virtual joints: offline only, no ROS or CAN."""
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rebot_core import Robot
from rebot_point import PointPlanner
from grasp_demo import build_scene


class FeedbackPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Robot(build_scene())

    def setUp(self):
        self.robot.reset()
        self.now = 0.
        self.planner = PointPlanner(self.robot, clock=lambda: self.now)
        self.start = np.zeros(6)
        self.body = dict(xyz_mm=[200,150,200], orientation_mode='position', execution_source='ros2-fake')

    def test_preview_uses_feedback_without_changing_mujoco_pose_or_target(self):
        r = self.robot
        before = [r.d.qpos.copy(), r.d.qvel.copy(), r.target.copy()]
        plan = self.planner.preview(self.body, start=self.start)
        np.testing.assert_array_equal(plan['start_deg'], self.start)
        self.assertEqual(plan['execution_source'], 'ros2-fake')
        for old, new in zip(before, [r.d.qpos,r.d.qvel,r.target]):
            np.testing.assert_array_equal(old,new)
        doc = self.planner.take({'plan_id':plan['plan_id']}, start=self.start, allowed_sources=('ros2-fake',))
        self.assertEqual(len(doc['steps']), 1)

    def test_virtual_token_cannot_be_executed_by_mujoco(self):
        plan = self.planner.preview(self.body, start=self.start)
        with self.assertRaisesRegex(ValueError, '来源与执行器不同'):
            self.planner.take({'plan_id':plan['plan_id']})
        self.assertIn(plan['plan_id'], self.planner.plans)

    def test_hardware_preview_cannot_fall_back_to_virtual_execution(self):
        plan = self.planner.preview({**self.body,'execution_source':'hardware-preview'}, start=self.start)
        with self.assertRaisesRegex(ValueError, '来源与执行器不同'):
            self.planner.take({'plan_id':plan['plan_id']},start=self.start,allowed_sources=('mujoco','ros2-fake'))

    def test_current_feedback_drift_and_expiry_reject_execution(self):
        plan = self.planner.preview(self.body,start=self.start)
        with self.assertRaisesRegex(ValueError,'姿态已改变'):
            self.planner.take({'plan_id':plan['plan_id']},start=self.start + .02,allowed_sources=('ros2-fake',))
        plan = self.planner.preview(self.body,start=self.start)
        self.now = 61.
        with self.assertRaisesRegex(ValueError,'过期'):
            self.planner.take({'plan_id':plan['plan_id']},start=self.start,allowed_sources=('ros2-fake',))

    def test_invalid_feedback_rejects_preview(self):
        for start in ([0]*5,[float('nan')]*6,[99]*6):
            with self.subTest(start=start), self.assertRaisesRegex(ValueError,'起点'):
                self.planner.preview(self.body,start=start)


if __name__ == '__main__':
    unittest.main()
