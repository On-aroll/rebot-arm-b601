"""Reviewed extended XYZ profile; virtual motors only, no live CAN."""
from dataclasses import replace
import math
import unittest

import test_native_xyz_offline as legacy
from rebot_mit_hardware import Envelope,MotionIntent
from rebot_mit_mock import Trajectory
from rebot_native_csp import proposal as single_proposal
from rebot_native_xyz import MODE,PATH_MODE,PROFILES,NativeXYZExecutor,proposal,validate_target
from rebot_xyz_trial import configure_motion


class NativeXYZPathOffline(unittest.TestCase):
    def setUp(self):
        self.case=legacy.NativeXYZOffline();self.case.setUp()
        self.case.config=Envelope.parse(proposal([(-1.,1.)]*6,mode=PATH_MODE)[0])
        self.case.executor=NativeXYZExecutor(self.case.config,self.case.bus,
            clock=self.case.clock,wait=self.case.wait,fk=lambda q:[v*1000 for v in q[:3]])
        self.case.permit.config_hash=self.case.config.fingerprint
        self.case.intent=MotionIntent([[math.radians(3.)]*6],'native-csp-xyz','virtual-path',expected_start=[0.]*6)
        self.case.beat()

    def test_old_profile_fingerprint_and_all_fast_safety_deadlines_remain_unchanged(self):
        old,_=single_proposal([(-1.,1.)]*6);old['version']=MODE
        current,_=proposal([(-1.,1.)]*6)
        self.assertEqual(Envelope.parse(old).fingerprint,Envelope.parse(current).fingerprint)
        extended,_=proposal([(-1.,1.)]*6,mode=PATH_MODE)
        self.assertEqual(extended['axes'],current['axes'])
        self.assertEqual(extended['policy'],dict(current['policy'],task_timeout=30.))
        self.assertNotEqual(Envelope.parse(extended).fingerprint,Envelope.parse(current).fingerprint)

    def test_three_degree_motion_arrives_then_disables_and_restores_without_pid_or_j7(self):
        c=self.case;c.run_trial()
        self.assertEqual(c.executor.state,'ARRIVED',c.executor.reason)
        self.assertEqual(c.executor.status()['control']['mode'],PATH_MODE)
        self.assertEqual(c.executor.status()['control']['task_timeout_s'],30.)
        self.assertTrue(c.executor.restoration['ok']);self.assertTrue(c.executor.stop_result['stationary_sample'])
        self.assertEqual([call[1] for call in c.calls('enable')],list(legacy.AXES))
        self.assertTrue(all(call[1] in legacy.AXES for call in c.sdk.calls if call[0] in ('write','enable','disable')))
        self.assertFalse(c.calls('mit'))
        self.assertFalse(any(call[2] in legacy.PROTECTED for call in c.calls('write')))
        self.assertLess(c.executor.timing['stop_requested_at'],c.executor.deadline)

    def test_old_half_degree_permit_cannot_authorize_extended_profile(self):
        c=self.case;c.permit.config_hash=Envelope.parse(proposal([(-1.,1.)]*6)[0]).fingerprint
        c.run_trial();self.assertEqual(c.executor.state,'FAULT');self.assertEqual(c.sdk.calls,[])

    def test_old_profile_rejects_three_degrees_and_extended_rejects_over_five_before_can(self):
        small=Envelope.parse(proposal([(-1.,1.)]*6)[0])
        with self.assertRaisesRegex(ValueError,'0.5°'):validate_target([0.]*6,[math.radians(3.)]*6,small)
        self.case.intent.waypoints=([math.radians(5.001)]*6,)
        self.case.run_trial();self.assertEqual(self.case.executor.state,'FAULT');self.assertEqual(self.case.sdk.calls,[])

    def test_unknown_profile_or_modified_timeout_is_rejected(self):
        with self.assertRaises(ValueError):proposal([(-1.,1.)]*6,mode='unreviewed')
        with self.assertRaises(ValueError):
            NativeXYZExecutor(replace(self.case.config,policy=replace(self.case.config.policy,task_timeout=60.)),self.case.bus,fk=lambda q:q[:3])

    def test_large_stall_still_stops_on_original_tracking_error_boundary(self):
        c=self.case;c.sdk.stalled=True;c.run_trial()
        self.assertEqual(c.executor.state,'FAULT');self.assertIn('跟踪误差超限',c.executor.reason)
        self.assertLess(c.executor.timing['stop_requested_at'],c.executor.deadline)
        self.assertFalse(c.executor.motion_arrived);self.assertTrue(c.executor.restoration['ok'])
        self.assertTrue(c.executor.stop_result['stationary_sample'])

    def test_small_stall_expires_at_thirty_seconds_without_later_motion_targets(self):
        self.case.intent.waypoints=([math.radians(.4)]*6,)
        c=self.case;c.sdk.stalled=True;c.run_trial()
        self.assertEqual(c.executor.state,'FAULT');self.assertIn('30 秒',c.executor.reason)
        self.assertFalse(c.executor.motion_arrived);self.assertTrue(c.executor.restoration['ok'])
        self.assertTrue(c.executor.stop_result['stationary_sample'])
        self.assertTrue(all(r['requested_at']<c.executor.deadline for r in c.bus.register_log if r['phase']=='trajectory'))

    def test_lost_lease_still_stops_before_extended_deadline(self):
        c=self.case;c.renew=False;c.run_trial()
        self.assertEqual(c.executor.state,'FAULT');self.assertFalse(c.executor.motion_arrived)
        self.assertTrue(c.executor.stop_result['sdk_status_ack_axes'])
        self.assertLess(c.executor.timing['stop_requested_at'],30.)

    def test_five_degree_curve_retains_velocity_acceleration_jerk_and_continuous_endpoints(self):
        config=self.case.config;target=[math.radians(5.)]*6
        curve=Trajectory.create([0.]*6,[target],config)
        self.assertLess(curve.duration+2.,config.policy.task_timeout)
        previous=[0.]*6
        for n in range(1001):
            q,v,a,j=curve.sample(curve.duration*n/1000)
            self.assertTrue(all(x>=p-1e-12 for x,p in zip(q,previous)));previous=q
            for values,name in ((v,'v_max'),(a,'a_max'),(j,'jerk_max')):
                self.assertTrue(all(abs(x)<=getattr(limit,name)+1e-9 for x,limit in zip(values,config.axes)))
        self.assertEqual(curve.sample(0.)[0],tuple([0.]*6))
        self.assertTrue(all(abs(a-b)<1e-12 for a,b in zip(curve.sample(curve.duration)[0],target)))

    def test_visible_profile_selection_has_fixed_speed_and_duration(self):
        base,_=proposal([(-1.,1.)]*6)
        _,config=configure_motion(base,[0.]*6,[math.radians(3.)]*6,
            dict(speed_rad_s=.02,timeout_mode='auto',feedforward_mode=PATH_MODE))
        self.assertEqual(config.version,PATH_MODE);self.assertEqual(config.policy.task_timeout,30.)
        for settings in (dict(speed_rad_s=.03,timeout_mode='auto'),dict(speed_rad_s=.02,timeout_mode='manual',timeout_s=15.)):
            with self.assertRaises(ValueError):configure_motion(base,[0.]*6,[math.radians(3.)]*6,dict(settings,feedforward_mode=PATH_MODE))


if __name__=='__main__':unittest.main()
