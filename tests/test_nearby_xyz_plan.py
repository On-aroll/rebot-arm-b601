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
        self.assertEqual(plan['gravity_check']['insufficient_axes'],[])
        self.assertEqual(plan['gravity_check']['sample_count'],51)
        self.assertTrue(np.all(np.asarray(plan['gravity_check']['sampled_peak_Nm'])>=np.abs(plan['gravity_check']['estimated_Nm'])))
        self.assertFalse(plan['execution_ready'])
        self.assertEqual(plan['static_balance_check']['insufficient_axes'],[2,3,4])
        with patch('rebot_seeed_transport.SeeedTransport') as bus:
            with self.assertRaisesRegex(ValueError,'承重控制尚未验证'):
                execute(self.config,plan,'synthetic approval')
            bus.assert_not_called()

    def test_raising_effort_ceiling_alone_does_not_authorize_unbalanced_motion(self):
        start=np.deg2rad([61.96,-.39,.1,.56,-.91,4.46])
        raised=replace(self.config,axes=tuple(replace(a,torque_max=10.) for a in self.config.axes))
        plan=plan_nearby(self.robot,start,raised)
        self.assertEqual(plan['gravity_check']['insufficient_axes'],[])
        self.assertEqual(plan['static_balance_check']['insufficient_axes'],[2,3,4])
        self.assertFalse(plan['execution_ready'])
        self.assertGreater(plan['static_balance_check']['required_offset_deg'][2],100.)
        with patch('rebot_seeed_transport.SeeedTransport') as bus:
            with self.assertRaisesRegex(ValueError,'PD/前馈'):
                execute(raised,plan,'synthetic onsite evidence')
            bus.assert_not_called()

    def test_pending_gravity_profile_has_capacity_in_model_but_is_not_the_active_config(self):
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        start=np.deg2rad([61.96,-.39,.1,.56,-.91,4.46])
        proposal=json.loads((Path(__file__).resolve().parents[1]/'configs/trials/seeed-nearby-xyz-gravity-proposal.json').read_text())
        self.assertEqual(proposal['approval'],'pending-explicit-MIT-gain-and-feedforward-change')
        self.assertTrue(all(p['proposed_kp']==3. for p in proposal['axes']))
        gravity=compute_generalized_gravity(self.robot.pm,start)[:6]
        # Dataclass replacement is for pure offline comparison only. The real
        # parser still rejects these unapproved gains/feedforward values.
        preview=replace(self.config,axes=tuple(replace(a,kp=p['proposed_kp'],kd=p['proposed_kd'],
            kp_max=p['proposed_kp'],kd_max=p['proposed_kd'],tau_ff=float(g))
            for a,p,g in zip(self.config.axes,proposal['axes'],gravity)))
        plan=plan_nearby(self.robot,start,preview)
        self.assertEqual(plan['gravity_check']['insufficient_axes'],[])
        self.assertEqual(plan['static_balance_check']['insufficient_axes'],[])
        self.assertLess(max(plan['static_balance_check']['required_offset_deg']),1.)
        # A hypothetical 0.5 Nm unmodelled load/friction is only a scenario,
        # not calibration of this arm or proof of hardware tracking.
        self.assertTrue(all(.5/a.kp>a.error_max for a in preview.axes))
        self.assertTrue(all(a.torque_max<rated for a,rated in zip(preview.axes,[11,11,11,5,5,5])))
        self.assertTrue(all((a.kp,a.kd,a.tau_ff)==(3.,1.,0.) for a in self.config.axes))
        from dataclasses import asdict
        with self.assertRaisesRegex(ValueError,'3/1/0'):
            Envelope.parse(dict(asdict(preview),source='explicit-onsite-trial-proposal'))


if __name__=='__main__': unittest.main()
