"""Virtual SDK only: native six-axis XYZ lifecycle and feedback acceptance."""
import math
import sys
from dataclasses import replace
from pathlib import Path
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_native_csp_offline import NativeController
from test_mit_hardware_offline import Clock, Owner
from rebot_mit_hardware import Envelope, MotionIntent, OnsitePermit
from rebot_native_csp import PROTECTED
from rebot_native_xyz import AXES, MODE, RUNTIME, NativeXYZExecutor, NativeXYZTransport, proposal, validate_target


class NativeXYZOffline(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.sdk=NativeController(self.clock);self.owner=Owner();self.renew=True;self.sequence=0
        self.config=Envelope.parse(proposal([(-1.,1.)]*6)[0])
        self.bus=NativeXYZTransport(factory=lambda **_:self.sdk,ownership=self.owner,clock=self.clock)
        self.executor=NativeXYZExecutor(self.config,self.bus,clock=self.clock,wait=self.wait,fk=lambda q:[v*1000 for v in q[:3]])
        self.permit=OnsitePermit(self.config.fingerprint,AXES,100.,'synthetic six-axis permission',AXES,True,True,coordinated_start_checked=True)
        self.intent=MotionIntent([[math.radians(.4)]*6],'native-csp-xyz','virtual-native-xyz',expected_start=[0.]*6)
        self.beat()

    def beat(self):
        self.sequence+=1;self.executor.heartbeat(self.executor.session,self.sequence,True)

    def wait(self,seconds):
        self.clock.wait(seconds)
        if self.renew:self.beat()

    def run_trial(self):self.executor.run(self.intent,self.permit)
    def calls(self,kind):return [c for c in self.sdk.calls if c[0]==kind]

    def test_full_xyz_uses_single_controller_all_axes_preloaded_and_restores_without_pid_writes(self):
        self.assertEqual(self.sdk.calls,[])
        self.run_trial();self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        self.assertEqual([c[1] for c in self.calls('enable')],list(AXES))
        self.assertTrue(self.executor.restoration['ok'])
        self.assertEqual(len(self.calls('register')),6)
        for i in AXES:
            enable=self.sdk.calls.index(('enable',i))
            preloads=[c for c in self.sdk.calls[:enable] if c[0]=='write' and c[1]==i and c[2]==0x7016]
            self.assertEqual(preloads[-1][3],0.)
            self.assertEqual([self.sdk.motors[i].regs[k] for k in RUNTIME[i]],[0,1.,57.,36.])
        self.assertFalse(any(c[2] in PROTECTED for c in self.calls('write')))
        self.assertFalse(self.calls('mit'))
        self.assertTrue(all(c[1] in AXES for c in self.calls('write')+self.calls('enable')+self.calls('disable')))
        self.assertEqual(self.executor.status()['control']['mode'],MODE)
        self.assertLessEqual(self.executor.tcp_error_mm,1.)

    def test_joint_arrival_alone_cannot_pass_wrong_tcp(self):
        self.executor.fk=lambda q:[0.,0.,0.] if self.executor.target is None else ([0.,0.,0.] if self.executor.deadline is None else [2.,0.,0.])
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT',self.executor.reason)
        self.assertIn('15 秒',self.executor.reason)
        self.assertFalse(self.executor.motion_arrived)
        self.assertTrue(self.executor.restoration['ok'])

    def test_no_six_axis_permission_and_larger_delta_rejected_before_open(self):
        self.permit.coordinated_start_checked=False
        self.run_trial();self.assertEqual(self.sdk.calls,[])
        self.setUp();self.intent.waypoints=([.02]*6,)
        self.run_trial();self.assertEqual(self.sdk.calls,[])

    def test_runtime_readback_failure_never_enables_and_restores_every_changed_axis(self):
        self.sdk.bad_readback=0x7017
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertFalse(self.calls('enable'))
        self.assertEqual(len(self.calls('disable')),12)
        self.assertTrue(self.executor.restoration['ok'])

    def test_slow_disabled_configuration_requires_new_position_pair_before_first_enable(self):
        authorize=self.bus.authorize
        def instrument(*args):
            authorize(*args)
            for drive in self.bus.drives.values():
                read=drive.read_register
                def delayed(index,read=read):
                    value=read(index);self.clock.t+=.02;self.beat();return value
                drive.read_register=delayed
        self.bus.authorize=instrument
        self.run_trial();self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)

    def test_cancel_during_partial_prepare_restores_without_continuing_enable(self):
        self.sdk.after_write=lambda index:self.executor.cancel() if index==0x7005 else None
        self.run_trial()
        self.assertEqual(self.executor.state,'CANCELLED',self.executor.reason)
        self.assertFalse(self.calls('enable'))
        self.assertTrue(self.executor.restoration['ok'])

    def test_uncertain_disable_never_restores_higher_limits(self):
        self.sdk.after_enable=lambda:setattr(self.sdk,'bad_stop',True)
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIsNone(self.executor.restoration)
        for i in AXES:self.assertEqual(self.sdk.motors[i].regs[0x7017],.02)

    def test_stalled_drives_and_lost_lease_stop_without_retry(self):
        for lease in (False,True):
            self.setUp();self.sdk.stalled=True;self.renew=not lease
            self.run_trial()
            self.assertEqual(self.executor.state,'FAULT',self.executor.reason)
            self.assertFalse(self.executor.motion_arrived)
            self.assertTrue(self.executor.restoration['ok'])
            self.assertEqual(len(self.calls('enable')),6)
            if not lease:
                self.assertGreaterEqual(self.executor.timing['stop_requested_at'],self.executor.deadline)
                self.assertTrue(all(r['requested_at']<self.executor.deadline for r in self.bus.register_log if r['phase']=='trajectory'))

    def test_current_guard_stops_startup_and_attempts_disable_for_all_owned_axes(self):
        self.sdk.after_enable=lambda:setattr(self.sdk,'current',10.1)
        self.run_trial();self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('电流反馈',self.executor.reason)
        self.assertFalse(self.executor.motion_arrived)
        self.assertEqual(self.executor.stop_result['attempted_axes'],list(AXES))

    def test_approved_start_tolerance_preserves_raw_negative_j2_and_ends_nominal(self):
        limits=[(-1.,1.)]*6;limits[1]=(0.,1.)
        self.config=Envelope.parse(proposal(limits)[0])
        self.executor=NativeXYZExecutor(self.config,self.bus,clock=self.clock,wait=self.wait,fk=lambda q:[v*1000 for v in q[:3]])
        self.permit.config_hash=self.config.fingerprint
        register=self.sdk.add_robstride_motor
        def negative(*args):
            motor=register(*args)
            if motor.i==2:motor.q=-.004
            return motor
        self.sdk.add_robstride_motor=negative
        self.intent.expected_start=(0.,-.004,0.,0.,0.,0.)
        self.intent.waypoints=([0.,0.,.004,0.,0.,0.],)
        self.beat();self.run_trial();self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        targets=[r for r in self.bus.register_log if r['axis']==2 and r['index']=='0x7016']
        self.assertEqual(targets[0]['value'],-.004)
        trajectory=[r['value'] for r in targets if r['phase']=='trajectory']
        self.assertTrue(all(b>=a-1e-12 for a,b in zip(trajectory,trajectory[1:])))
        self.assertGreaterEqual(trajectory[-1],0.)

    def test_slow_cycle_is_not_hidden_by_longer_trial(self):
        read=self.bus.read
        def slow(*args,**kwargs):
            result=read(*args,**kwargs)
            if any(m.enabled for m in self.sdk.motors.values()):self.clock.t+=.31
            return result
        self.bus.read=slow;self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertFalse(self.executor.motion_arrived)
        self.assertTrue(self.calls('disable'))

    def test_fixed_native_profile_rejects_other_gains_limits_or_duration(self):
        with self.assertRaises(ValueError):
            NativeXYZExecutor(replace(self.config,policy=replace(self.config.policy,task_timeout=60.)),self.bus,fk=lambda q:q[:3])

    def test_invalid_numeric_inputs_and_fk_fail_before_can_open(self):
        for value in (True,'0',float('nan'),float('inf')):
            with self.assertRaises(ValueError): validate_target([0.]*6,[value,0.,0.,0.,0.,0.],self.config)
        for bad in ([0.,0.],[0.,0.,float('nan')]):
            self.setUp();self.executor.fk=lambda q,bad=bad:bad
            self.run_trial();self.assertEqual(self.sdk.calls,[])
            self.assertEqual(self.executor.state,'FAULT')


if __name__=='__main__':unittest.main()
