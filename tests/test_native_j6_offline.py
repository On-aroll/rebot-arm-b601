"""Fixed wrist trial, virtual drives only; no real CAN or motor settings."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_native_csp_offline import NativeController
from test_mit_hardware_offline import Clock,Owner
from rebot_mit_hardware import Envelope,MotionIntent,OnsitePermit
from rebot_native_csp import CspTransport,CspJ3Trial,CspJ6Trial,DELTA,PROTECTED,proposal,probe,runtime_for_axis


class NativeJ6Offline(unittest.TestCase):
    def setUp(self):
        self.clock=Clock(); self.sdk=NativeController(self.clock); self.owner=Owner()
        raw,_=proposal([(-1.,1.)]*6,axis=6); self.config=Envelope.parse(raw)
        self.bus=CspTransport(axis=6,runtime=runtime_for_axis(6),factory=lambda **_:self.sdk,ownership=self.owner,clock=self.clock)
        self.executor=CspJ6Trial(self.config,self.bus,clock=self.clock,wait=self.wait)
        self.seq=0; self.renew=True; self.beat()
        self.permit=OnsitePermit(self.config.fingerprint,(6,),100.,'synthetic J6 approval',(6,),True,True)
        self.intent=MotionIntent([[0.,0.,0.,0.,0.,DELTA]],'native-csp-j6','virtual-j6',expected_start=[0.]*6)

    def beat(self):
        self.seq+=1; self.executor.heartbeat(self.executor.session,self.seq,True)

    def wait(self,seconds):
        self.clock.wait(seconds)
        if self.renew:self.beat()

    def run_trial(self):self.executor.run(self.intent,self.permit)
    def writes(self):return [c for c in self.sdk.calls if c[0]=='write']

    def assert_cleanup(self):
        self.assertTrue(self.executor.restore_result['ok'],self.executor.reason)
        self.assertEqual(self.executor.stop_result['sdk_status_ack_axes'],[6])
        self.assertTrue(self.executor.stop_result['stationary_sample'])
        self.assertTrue(all(c[1]==6 for c in self.sdk.calls if c[0] in ('enable','disable','write')))
        self.assertFalse(any(c[2] in PROTECTED for c in self.writes()))
        self.assertFalse(any(c[0]=='mit' for c in self.sdk.calls))
        self.assertNotIn(7,self.sdk.motors)

    def test_readonly_probe_never_writes_or_enables(self):
        result=probe(self.config,self.bus)
        self.assertTrue(result['ok'],result);self.assertEqual(result['hardware_writes'],0)
        self.assertFalse(any(c[0] in ('enable','disable','write') for c in self.sdk.calls))

    def test_current_preload_and_one_nm_cap_then_arrival_disable_restore(self):
        self.run_trial();self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        enable=self.sdk.calls.index(('enable',6));before=[c for c in self.sdk.calls[:enable] if c[0]=='write']
        self.assertEqual([(c[2],c[3]) for c in before],[(0x7005,5),(0x700B,1.),(0x7018,10.),(0x7017,.02),(0x7016,0.)])
        self.assertAlmostEqual(self.executor.arrival_feedback[6].q,DELTA)
        self.assertEqual(self.executor.status()['control']['axis'],6)
        self.assert_cleanup()

    def test_j3_plan_and_permit_cannot_be_reused_for_j6(self):
        with self.assertRaises(ValueError):CspJ3Trial(self.config,self.bus)
        self.permit.axes=(3,)
        self.run_trial();self.assertEqual(self.executor.state,'FAULT');self.assertEqual(self.sdk.calls,[])

    def test_cannot_change_another_axis_or_double_delta(self):
        for target in ([.001,0.,0.,0.,0.,DELTA],[0.,0.,0.,0.,0.,2*DELTA]):
            self.setUp();self.intent.waypoints=(tuple(target),)
            self.run_trial();self.assertEqual(self.executor.state,'FAULT');self.assertEqual(self.sdk.calls,[])

    def test_stalled_wrist_expires_without_false_arrival(self):
        self.sdk.stalled_axes={6};self.run_trial()
        self.assertEqual(self.executor.state,'FAULT');self.assertIn('15 秒',self.executor.reason)
        self.assertFalse(self.executor.motion_arrived);self.assert_cleanup()

    def test_feedback_timeout_after_enable_preserves_lower_limits(self):
        self.sdk.after_enable=lambda:setattr(self.sdk,'fault','timeout')
        self.run_trial();self.assertEqual(self.executor.state,'FAULT')
        self.assertFalse(self.executor.restore_result['ok'])
        self.assertEqual(self.sdk.motors[6].regs[0x700B],1.)
        self.assertTrue(any(c==('disable',6) for c in self.sdk.calls))

    def test_cancel_after_partial_mode_write_restores_without_enable(self):
        self.sdk.after_write=lambda index:self.executor.cancel() if index==0x7005 else None
        self.run_trial();self.assertEqual(self.executor.state,'CANCELLED')
        self.assertFalse(any(c[0]=='enable' for c in self.sdk.calls));self.assert_cleanup()

    def test_foreign_axis_motion_stops_wrist(self):
        self.sdk.after_enable=lambda:setattr(self.sdk.motors[1],'q',math.radians(.11))
        self.run_trial();self.assertEqual(self.executor.state,'FAULT')
        self.assertRegex(self.executor.reason,'未控制轴|J1 实测速度');self.assert_cleanup()

    def test_lower_existing_torque_cap_is_not_raised(self):
        register=self.sdk.add_robstride_motor
        def limited(*args):
            motor=register(*args)
            if motor.i==6:motor.regs[0x700B]=.5
            return motor
        self.sdk.add_robstride_motor=limited;self.run_trial()
        self.assertEqual(self.executor.state,'FAULT');self.assertIn('不能自动提高',self.executor.reason)
        self.assertFalse(self.writes());self.assertFalse(any(c[0]=='enable' for c in self.sdk.calls))


if __name__=='__main__':unittest.main()
