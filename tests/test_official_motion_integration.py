"""Real pinned upstream MIT callback + virtual SDK only. Never opens CAN."""
from dataclasses import replace
import copy
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from test_mit_hardware_offline import Clock,Controller,Owner
from rebot_mit_hardware import Envelope,HardwareExecutor,MotionIntent,OnsitePermit,Sample
from rebot_seeed_transport import SeeedTransport
from rebot_official_motion import OfficialMIT,policy,apply_slow_gains,gain_policy


class OfficialMotion(unittest.TestCase):
    def setUp(self):
        raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        raw['feedforward']=policy();raw['policy']['task_timeout']=15.
        self.config=Envelope.parse(raw)
        self.start=np.deg2rad([0,1,.02,.98,2.41,2.45]).tolist()
        self.clock=Clock();self.sdk=Controller(self.clock);self.renew=True;self.seq=0
        original=self.sdk.add_robstride_motor
        def register(i,host,model):
            motor=original(i,host,model);motor.q=self.start[i-1];return motor
        self.sdk.add_robstride_motor=register
        self.bus=SeeedTransport(factory=lambda **_:self.sdk,ownership=Owner(),clock=self.clock)
        self.executor=HardwareExecutor(self.config,self.bus,clock=self.clock,wait=self.wait)
        self.beat()
        axes=tuple(range(1,7))
        self.permit=OnsitePermit(self.config.fingerprint,axes,25.,'OFFLINE synthetic confirmation',axes,True,True,coordinated_start_checked=True)
        self.target=list(self.start);self.target[1]+=.005;self.target[2]+=.005
        self.intent=MotionIntent([self.target],'xyz','offline-gravity',expected_start=self.start)

    def beat(self):
        self.seq+=1;self.executor.heartbeat(self.executor.session,self.seq,True)

    def wait(self,seconds):
        self.clock.wait(seconds)
        if self.renew:self.beat()

    def calls(self,name):return [c for c in self.sdk.calls if c[0]==name]

    def test_upstream_callback_matches_official_gravity_and_changes_with_feedback(self):
        binding=OfficialMIT(self.config)
        from reBotArm_control_py.dynamics import compute_generalized_gravity
        expected=compute_generalized_gravity(binding.ctrl._model,np.array(self.start))[:6]
        values=np.asarray(binding.commands(self.target,[.001]*6,self.start))
        np.testing.assert_allclose(values[:,4],expected,atol=1e-12)
        np.testing.assert_allclose(values[:,0],self.target)
        np.testing.assert_allclose(values[:,2:4],[[3.,1.]]*6)
        moved=list(self.start);moved[1]+=.2
        changed=np.asarray(binding.commands(self.target,[.001]*6,moved))
        self.assertGreater(float(np.max(np.abs(changed[:,4]-values[:,4]))),.01)
        self.assertEqual(self.sdk.calls,[])

    def test_supported_current_pose_hold_first_frames_and_stop(self):
        self.intent=MotionIntent([self.start],'xyz','offline-hold',expected_start=self.start)
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        for axis in range(1,7):
            first=next(c for c in self.calls('mit') if c[1]==axis)
            self.assertAlmostEqual(first[2],self.start[axis-1]);self.assertEqual(first[3],0.)
            self.assertEqual(first[4:6],(3.,1.))
        self.assertGreater(next(c for c in self.calls('mit') if c[1]==3)[6],6.)
        self.assertEqual({c[1] for c in self.calls('disable')},set(range(1,7)))

    def test_six_axis_small_motion_uses_upstream_frames_without_parameter_writes(self):
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        np.testing.assert_allclose([self.sdk.motors[i].q for i in range(1,7)],self.target,atol=1e-8)
        self.assertTrue(all(c[4:6]==(3.,1.) for c in self.calls('mit')))
        self.assertFalse(any('param_write' in c[0] or 'home' in c[0] for c in self.sdk.calls))
        self.assertGreater(len(self.executor.frames),1)

    def test_raw_negative_start_recovers_inward_without_clipping_or_zero_writes(self):
        # A supported J2 can rest just below nominal zero. Its first hold must
        # retain that raw angle; only the final target returns inside limits.
        self.start[1]=float(np.deg2rad(-.37))
        self.target=list(self.start);self.target[1]=float(np.deg2rad(.1))
        self.intent=MotionIntent([self.target],'xyz','offline-boundary',expected_start=self.start)
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        first=next(c for c in self.calls('mit') if c[1]==2)
        self.assertAlmostEqual(first[2],self.start[1]);self.assertLess(first[2],0.)
        self.assertAlmostEqual(self.sdk.motors[2].q,self.target[1])
        self.assertTrue(all(c[3]>=0. for c in self.calls('mit') if c[1]==2 and c[2]<0.))
        self.assertFalse(any('param_write' in c[0] or 'zero' in c[0] or 'home' in c[0] for c in self.sdk.calls))

    def test_invalid_official_outputs_and_excess_effort_reject_before_enable(self):
        for torque in (float('nan'),float('inf'),100.):
            self.setUp()
            def bad_gravity(model,q,data):return np.full(model.nv,torque)
            with patch('reBotArm_control_py.controllers.rebotarm_endpose_controller.compute_generalized_gravity',bad_gravity):
                self.executor.run(self.intent,self.permit)
            self.assertEqual(self.executor.state,'FAULT')
            self.assertFalse(self.calls('enable'));self.assertFalse(self.calls('mit'))

    def test_stale_coupled_feedback_prevents_even_one_axis_hold_frame(self):
        self.executor.official_motion=OfficialMIT(self.config)
        samples={i:Sample(i,self.start[i-1],0.,0,self.clock.t,self.clock.t) for i in range(1,7)}
        samples[6]=replace(samples[6],requested_at=self.clock.t-self.config.policy.feedback_age-.01)
        with self.assertRaisesRegex(TimeoutError,'J6 反馈过期'):
            self.executor._send(self.start,[0.]*6,samples,replace(self.permit,axes=(1,)))
        self.assertFalse(self.calls('mit'))

    def test_slow_startup_gravity_computation_cannot_send_an_expired_first_frame(self):
        original=OfficialMIT.commands;calls=0
        def slow(binding,*args):
            nonlocal calls
            calls+=1
            if calls==2:self.clock.wait(self.config.policy.command_age+.01)
            return original(binding,*args)
        with patch.object(OfficialMIT,'commands',slow):
            self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'FAULT')
        self.assertFalse(self.calls('mit'))
        self.assertEqual(self.calls('disable'),[('disable',1)])

    def test_cancel_at_first_frame_stops_without_homing_or_followup_targets(self):
        self.sdk.after_frame=self.executor.cancel
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'CANCELLED',self.executor.reason)
        self.assertEqual(len(self.calls('mit')),1)
        self.assertEqual(self.calls('disable'),[('disable',1)])

    def test_gravity_error_and_model_change_reject_before_enable(self):
        for failure in ('model','compute'):
            self.setUp()
            if failure=='model':
                self.config=replace(self.config,feedforward=replace(self.config.feedforward,model_sha256='0'*64))
                self.executor.config=self.config;self.permit.config_hash=self.config.fingerprint
                self.executor.run(self.intent,self.permit)
            else:
                with patch('rebot_official_motion.OfficialMIT.commands',side_effect=ValueError('synthetic gravity error')):
                    self.executor.run(self.intent,self.permit)
            self.assertEqual(self.executor.state,'FAULT')
            self.assertFalse(self.calls('enable'));self.assertFalse(self.calls('mit'))

    def test_feedback_timeout_after_first_frame_stops_and_never_resumes(self):
        self.sdk.after_frame=lambda:setattr(self.sdk,'fault','timeout')
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'FAULT')
        # The started axis may receive its fresh startup hold before the next
        # sweep detects the timeout; no trajectory or post-stop frame is allowed.
        self.assertTrue(all(abs(c[2]-self.start[c[1]-1])<1e-12 for c in self.calls('mit')))
        self.assertEqual({c[1] for c in self.calls('disable')},{c[1] for c in self.calls('enable')})
        stop=next(i for i,c in enumerate(self.sdk.calls) if c[0]=='disable')
        self.assertFalse(any(c[0]=='mit' for c in self.sdk.calls[stop+1:]))

    def test_no_heartbeat_keeps_old_short_lease(self):
        self.renew=False
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('失联',self.executor.reason)
        self.assertLess(self.clock.t,2.)

    def test_startup_overspeed_preserves_failure_sample_and_control_source(self):
        original=self.sdk.add_robstride_motor
        def register(i,host,model):
            motor=original(i,host,model)
            if i==2:
                enable=motor.enable
                def moving_enable():
                    enable();motor.qd=self.config.axes[1].v_max*2
                motor.enable=moving_enable
            return motor
        self.sdk.add_robstride_motor=register
        self.executor.run(self.intent,self.permit)
        result=self.executor.status()
        self.assertEqual(result['state'],'FAULT')
        self.assertIn('J2 实测速度超过试验边界：0.040000 rad/s，允许 ±0.02 rad/s',result['reason'])
        self.assertEqual(result['failure_feedback']['2']['qd'],.04)
        self.assertEqual(result['feedback']['2']['qd'],0.)
        self.assertEqual(result['feedback']['2'],result['stop']['observations'][-1]['feedback']['2'])
        self.assertEqual(result['control']['feedforward'],policy())
        self.assertEqual(result['control']['kp'],[3.]*6)
        self.assertEqual(result['control']['config_fingerprint'],self.config.fingerprint)
        self.assertEqual(self.calls('disable'),[('disable',1),('disable',2)])
        self.assertFalse(any(c[1]==2 for c in self.calls('mit')))

    def test_feedforward_policy_changes_fingerprint_and_preserves_old_profiles(self):
        raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        saved=copy.deepcopy(raw);before=Envelope.parse(raw).fingerprint
        raw['feedforward']=policy()
        self.assertNotEqual(before,Envelope.parse(raw).fingerprint)
        raw.pop('feedforward');self.assertEqual(raw,saved)
        self.assertEqual(Envelope.parse(raw).fingerprint,before)

    def test_reviewed_official_gains_move_six_virtual_motors_and_preserve_source(self):
        raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        apply_slow_gains(raw);raw['feedforward']=policy();raw['policy']['task_timeout']=15.
        self.config=Envelope.parse(raw);self.executor.config=self.config
        self.permit.config_hash=self.config.fingerprint
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        for c in self.calls('mit'):
            self.assertEqual(c[4:6],(self.config.axes[c[1]-1].kp,self.config.axes[c[1]-1].kd))
        self.assertEqual(self.executor.status()['control']['gain_review'],gain_policy())
        j3=[c for c in self.calls('mit') if c[1]==3]
        self.assertEqual(j3[0][6],0.)
        self.assertGreater(j3[-1][6],6.)
        ramp=[c[6] for c in j3[:-1]]
        self.assertTrue(all(a<=b+1e-8 for a,b in zip(ramp,ramp[1:])))
        self.assertEqual(self.config.policy.start_speed,.01)

    def test_gravity_fade_is_cancelled_without_any_post_stop_target(self):
        raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        apply_slow_gains(raw);raw['feedforward']=policy();raw['policy']['task_timeout']=15.
        self.executor.config=Envelope.parse(raw);self.permit.config_hash=self.executor.config.fingerprint
        def cancel_during_fade():
            if any(c[0]=='mit' and c[1]==3 and c[6]>0 for c in self.sdk.calls): self.executor.cancel()
        self.sdk.after_frame=cancel_during_fade
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'CANCELLED',self.executor.reason)
        first_stop=next(i for i,c in enumerate(self.sdk.calls) if c[0]=='disable')
        self.assertFalse(any(c[0]=='mit' for c in self.sdk.calls[first_stop:]))
        self.assertFalse(self.executor.motion_arrived)

    def test_reviewed_profile_installs_fresh_hold_before_the_post_enable_sweep(self):
        raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        apply_slow_gains(raw);raw['feedforward']=policy();raw['policy']['task_timeout']=15.
        self.executor.config=Envelope.parse(raw);self.permit.config_hash=self.executor.config.fingerprint
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        for axis in range(1,7):
            index=next(i for i,c in enumerate(self.sdk.calls) if c==('enable',axis))
            first=self.sdk.calls[index+1]
            self.assertEqual(first[0:2],('mit',axis))
            self.assertAlmostEqual(first[2],self.start[axis-1])
            self.assertEqual(first[3],0.)
            self.assertEqual(first[6],0.)

    def test_reviewed_startup_waits_for_sub_limit_speed_before_enabling_next_axis(self):
        raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        apply_slow_gains(raw);raw['feedforward']=policy();raw['policy']['task_timeout']=15.
        self.executor.config=Envelope.parse(raw);self.permit.config_hash=self.executor.config.fingerprint
        original=self.bus.read; injected=False
        def read(*args,**kwargs):
            nonlocal injected
            samples=original(*args,**kwargs)
            if self.bus.active=={1,2,3} and not injected:
                samples[3]=replace(samples[3],qd=.015);injected=True
            return samples
        self.bus.read=read
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        self.assertTrue(injected)
        self.assertEqual(self.executor.config.axes[2].v_max,.02)

    def test_gains_need_explicit_source_and_cannot_exceed_official_limits(self):
        raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        original=copy.deepcopy(raw);old_hash=Envelope.parse(raw).fingerprint
        raw['axes'][0]['kp']=raw['axes'][0]['kp_max']=50.
        with self.assertRaisesRegex(ValueError,'3/1/0'):Envelope.parse(raw)
        raw['gain_review']=gain_policy();Envelope.parse(raw)
        for field,value in (('kp',81.),('kd',6.),('tau_ff',.1)):
            changed=copy.deepcopy(raw);changed['axes'][0][field]=value
            if field=='kp':changed['axes'][0]['kp_max']=value
            if field=='kd':changed['axes'][0]['kd_max']=value
            with self.assertRaisesRegex(ValueError,'官方到点增益'):Envelope.parse(changed)
        raw['gain_review']['source_commit']='wrong'
        with self.assertRaisesRegex(ValueError,'配置来源'):Envelope.parse(raw)
        self.assertEqual(Envelope.parse(original).fingerprint,old_hash)


if __name__=='__main__':unittest.main()
