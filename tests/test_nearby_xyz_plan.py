"""Model-only planning and rejection before opening a CAN transport."""
import json
from pathlib import Path
import sys
import unittest
from dataclasses import replace
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_core import Robot
from rebot_mit_hardware import Envelope
from rebot_mit_mock import Trajectory
from rebot_xyz_trial import plan_nearby,execute


class NearbyPlan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot=Robot()
        path=Path(__file__).resolve().parents[1]/'configs/trials/seeed-nearby-xyz-pending.json'
        cls.config=Envelope.parse(json.loads(path.read_text())['envelope'])

    def test_current_seed_selects_feasible_five_mm_point_with_small_angles(self):
        start=np.deg2rad([61.96,1.5,.1,.56,-.91,4.46])
        plan=plan_nearby(self.robot,start,self.config)
        self.assertAlmostEqual(np.linalg.norm(np.array(plan['goal_xyz_m'])-plan['start_xyz_m']),.005,places=12)
        self.assertLess(plan['position_error_mm'],.002)
        self.assertLess(max(abs(d) for d in plan['delta_deg']),2.)
        self.assertEqual(plan['controlled_axes'],[1,2,3,4,5,6])
        self.assertFalse(plan['gripper_commanded'])
        tr=Trajectory.create(start.tolist(),[plan['target']],self.config)
        for t in np.linspace(0,tr.duration,50):
            q,v,a,j=tr.sample(t)
            self.assertFalse(self.robot.contacts(np.asarray(q)))
            for i,axis in enumerate(self.config.axes):
                self.assertTrue(axis.q_min<=q[i]<=axis.q_max)
                self.assertLessEqual(abs(v[i]),axis.v_max+1e-12)
                self.assertLessEqual(abs(a[i]),axis.a_max+1e-12)
                self.assertLessEqual(abs(j[i]),axis.jerk_max+1e-12)

    def test_raw_negative_j2_is_rejected_without_clipping(self):
        start=np.deg2rad([61.96,-.39,.1,.56,-.91,4.46])
        before=start.copy()
        with self.assertRaisesRegex(ValueError,'实测起点越限'):
            plan_nearby(self.robot,start,replace(self.config,start_boundary_tolerance=0.))
        np.testing.assert_array_equal(start,before)

    def test_explicit_tolerance_preserves_raw_start_and_identifies_load_deficit(self):
        start=np.deg2rad([61.96,-.39,.1,.56,-.91,4.46]);before=start.copy()
        plan=plan_nearby(self.robot,start,self.config)
        np.testing.assert_array_equal(start,before)
        np.testing.assert_array_equal(plan['expected_start'],start)
        self.assertEqual(plan['boundary_recovery_axes'],[2])
        self.assertEqual(plan['start_boundary_tolerance_deg'],1.)
        self.assertTrue(all(a.q_min<=q<=a.q_max for a,q in zip(self.config.axes,plan['target'])))
        self.assertEqual(plan['gravity_check']['insufficient_axes'],[2,3,4])
        self.assertEqual(plan['gravity_check']['sample_count'],51)
        self.assertTrue(np.all(np.asarray(plan['gravity_check']['sampled_peak_Nm'])>=np.abs(plan['gravity_check']['estimated_Nm'])))
        self.assertFalse(plan['execution_ready'])
        with patch('rebot_seeed_transport.SeeedTransport') as bus:
            with self.assertRaisesRegex(ValueError,'承重控制尚未验证'):
                execute(self.config,plan,'synthetic approval')
            bus.assert_not_called()


if __name__=='__main__': unittest.main()
