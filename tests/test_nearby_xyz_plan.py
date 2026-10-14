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
from rebot_xyz_trial import plan_nearby,execute,assess_motion,configure_motion,hardware_motion_options


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

    def test_far_xyz_preview_reports_trial_time_instead_of_generic_exception(self):
        start=np.deg2rad([-1.15,-.37,.02,.98,2.41,2.45])
        target=self.robot.ik(np.array([.30071,.200,.22487]),seed=start,position_only=True)
        before=self.config.fingerprint
        with patch('rebot_seeed_transport.SeeedTransport') as bus:
            result=assess_motion(self.robot,start,target,self.config)
            bus.assert_not_called()
        self.assertFalse(result['execution_ready'])
        self.assertGreater(result['hardware_seconds'],5.)
        self.assertEqual(result['task_timeout_s'],5.)
        self.assertIn('0.02 rad/s',result['execution_reasons'][0])
        self.assertIn('时限只有 5 秒',result['execution_reasons'][0])
        self.assertIn('未启动电机',result['execution_reasons'][0])
        self.assertEqual(result['gravity_check']['status'],'not-evaluated')
        self.assertEqual(self.config.fingerprint,before)

    def test_web_timing_is_bound_to_task_without_changing_gains_or_stop_deadlines(self):
        import copy
        from rebot_point import PointPlanner
        raw=json.loads((Path(__file__).resolve().parents[1]/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        saved=copy.deepcopy(raw)
        start=np.deg2rad([-1.15,-.37,.02,.98,2.41,2.45])
        planner=PointPlanner(self.robot)
        plan=planner.preview(dict(execution_source='hardware-preview',xyz_mm=[300.71,200,224.87],speed_rad_s=.02),start=start)
        target=np.deg2rad(plan['q_deg'])
        task,config=configure_motion(raw,start,target,dict(speed_rad_s=.02,timeout_mode='auto'))
        self.assertEqual(raw,saved)
        trajectory=Trajectory.create(start.tolist(),[target.tolist()],config,start_tolerance=config.start_boundary_tolerance)
        self.assertGreater(trajectory.duration,70.)
        self.assertEqual(config.policy.task_timeout,np.ceil(trajectory.duration+5.))
        for field in ('feedback_age','command_age','lease_timeout','io_timeout','period'):
            self.assertEqual(getattr(config.policy,field),getattr(self.config.policy,field))
        self.assertEqual([a.kp for a in config.axes],[3.]*6)
        self.assertEqual([a.tau_ff for a in config.axes],[0.]*6)
        # Correct timing does not override a separate load or tracking check.
        assessment=assess_motion(self.robot,start,target,config)
        self.assertFalse(any('时限' in reason for reason in assessment['execution_reasons']))
        self.assertFalse(assessment['execution_ready'])
        self.assertIn(3,assessment['static_balance_check']['insufficient_axes'])
        _,faster=configure_motion(raw,start,target,dict(speed_rad_s=.05,timeout_mode='manual',timeout_s=60.))
        self.assertEqual([a.v_max for a in faster.axes],[.05]*6)
        self.assertEqual(faster.policy.task_timeout,60.)
        self.assertLess(Trajectory.create(start.tolist(),[target.tolist()],faster,start_tolerance=faster.start_boundary_tolerance).duration,trajectory.duration)

    def test_web_timing_rejects_invalid_values_and_keeps_maximum_bounded(self):
        raw=json.loads((Path(__file__).resolve().parents[1]/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        start=np.deg2rad([-1.15,-.37,.02,.98,2.41,2.45])
        target=self.robot.ik(np.array([.30071,.200,.22487]),seed=start,position_only=True)
        for settings in (None,{},dict(speed_rad_s=.02,timeout_mode='unknown'),
                         dict(speed_rad_s=True,timeout_mode='auto'),dict(speed_rad_s=.2,timeout_mode='auto'),
                         dict(speed_rad_s=.02,timeout_mode='manual',timeout_s=121.),
                         dict(speed_rad_s=.02,timeout_mode='manual',timeout_s=0.),
                         dict(speed_rad_s=.02,timeout_mode='manual',timeout_s=float('nan')),
                         dict(speed_rad_s=.02,timeout_mode='auto',kp=100.)):
            with self.subTest(settings=settings),self.assertRaises(ValueError):
                configure_motion(raw,start,target,settings)
        _,config=configure_motion(raw,start,target,dict(speed_rad_s=.005,timeout_mode='auto'))
        self.assertEqual(config.policy.task_timeout,120.)
        self.assertFalse(assess_motion(self.robot,start,target,config)['execution_ready'])
        self.assertEqual(hardware_motion_options(raw)['default_speed_rad_s'],.02)

    def test_automatic_duration_does_not_silently_trim_five_second_margin(self):
        from types import SimpleNamespace
        raw=json.loads((Path(__file__).resolve().parents[1]/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        with patch('rebot_xyz_trial.Trajectory.create',return_value=SimpleNamespace(duration=117.)):
            with self.assertRaisesRegex(ValueError,'需 122 秒'):
                configure_motion(raw,[0.]*6,[0.]*6,dict(speed_rad_s=.02,timeout_mode='auto'))

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
