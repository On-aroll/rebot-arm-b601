"""Six-axis trial contracts: actual Seeed classes, timed virtual motors only."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import math
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_mit_hardware_offline import Clock,Controller,Motor,Owner,config
from rebot_mit_hardware import HardwareExecutor,MotionIntent,OnsitePermit,HoldIntent
from rebot_seeed_transport import SeeedTransport
from rebot_xyz_trial import read_start


class TimedMotor(Motor):
    def robstride_get_param_f32_host_id(self,*args):
        self.c.clock.t+=.008
        return super().robstride_get_param_f32_host_id(*args)
    def robstride_get_param_i8(self,*args):
        self.c.clock.t+=.008
        return super().robstride_get_param_i8(*args)
    def enable(self):
        super().enable(); self.c.clock.t+=self.c.enable_delay


class TimedController(Controller):
    def __init__(self,clock): super().__init__(clock); self.enable_delay=.04
    def add_robstride_motor(self,i,host,model):
        self.calls.append(('register',i,host,model))
        self.motors[i]=TimedMotor(i,self)
        self.motors[i].qd=.07 if i>=4 else 0.
        return self.motors[i]


class CoordinatedTrial(unittest.TestCase):
    def setUp(self):
        self.clock=Clock(); self.sdk=TimedController(self.clock)
        base=config()
        self.config=replace(base,axes=tuple(replace(a,v_max=.02,a_max=.04,jerk_max=.2,error_max=.017453292519943295,torque_max=.2) for a in base.axes),
            policy=replace(base.policy,period=.1,feedback_age=.6,command_age=.3,jitter_max=.15,lease_timeout=.75,settle_time=.3,task_timeout=5.))
        self.bus=SeeedTransport(position_window=True,factory=lambda **_:self.sdk,ownership=Owner(),clock=self.clock)
        self.executor=HardwareExecutor(self.config,self.bus,clock=self.clock,wait=self.wait)
        self.seq=0; self.beat()
        self.permit=OnsitePermit(self.config.fingerprint,tuple(range(1,7)),15,'synthetic six-axis approval',tuple(range(1,7)),True,True,coordinated_start_checked=True)
        self.target=[.004,.003,.002,-.004,-.003,-.002]
        self.intent=MotionIntent([self.target],'xyz','virtual-six',expected_start=[0.]*6)
        # Foreground heartbeat also runs during SDK startup.
        self.sdk.callback=self.beat
    def beat(self):
        self.seq+=1; self.executor.heartbeat(self.executor.session,self.seq,True)
    def wait(self,t): self.clock.wait(t); self.beat()
    def calls(self,op): return [c for c in self.sdk.calls if c[0]==op]

    def test_tolerance_is_explicit_bounded_and_old_configuration_hash_stays_valid(self):
        from dataclasses import asdict
        data=asdict(config());data.pop('start_boundary_tolerance')
        expected=hashlib.sha256(json.dumps(data,sort_keys=True,allow_nan=False).encode()).hexdigest()
        self.assertEqual(config().fingerprint,expected)
        data['source']='explicit-onsite-trial-proposal'
        from rebot_mit_hardware import Envelope
        self.assertEqual(Envelope.parse(data).start_boundary_tolerance,0.)
        for tolerance in (-.01,math.radians(1.01),float('nan'),True):
            with self.subTest(tolerance=tolerance),self.assertRaises(ValueError):
                Envelope.parse(dict(data,start_boundary_tolerance=tolerance))

    def boundary_fixture(self, start=-.39):
        limits=list(self.config.axes); limits[1]=replace(limits[1],q_min=0.)
        self.config=replace(self.config,axes=tuple(limits),start_boundary_tolerance=math.radians(1))
        self.executor.config=self.config; self.permit.config_hash=self.config.fingerprint
        register=self.sdk.add_robstride_motor
        def with_boundary(i,host,model):
            motor=register(i,host,model)
            if i==2: motor.q=math.radians(start)
            return motor
        self.sdk.add_robstride_motor=with_boundary
        start_q=[0.]*6; start_q[1]=math.radians(start)
        target=[0.]*6; target[1]=math.radians(.2)
        self.intent=MotionIntent([target],'xyz','boundary-inward',expected_start=start_q)

    def test_explicit_sub_degree_start_recovers_inward_without_clipping(self):
        self.boundary_fixture()
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        frames=[c for c in self.calls('mit') if c[1]==2]
        self.assertAlmostEqual(frames[0][2],math.radians(-.39))
        self.assertAlmostEqual(frames[-1][2],math.radians(.2))
        self.assertTrue(all(c[3]>=0 for c in frames if c[2]<0))
        self.assertFalse(any(c[0] in ('zero','write_param','ensure_mode') for c in self.sdk.calls))

    def test_more_than_one_degree_or_outside_destination_never_enable(self):
        self.boundary_fixture(-1.01)
        self.executor.run(self.intent,self.permit)
        self.assertFalse(self.calls('enable'))
        self.setUp(); self.boundary_fixture()
        target=[0.]*6; target[1]=math.radians(-.1)
        self.intent=MotionIntent([target],'xyz','invalid-boundary-end',expected_start=self.intent.expected_start)
        self.executor.run(self.intent,self.permit)
        self.assertFalse(self.calls('enable'))

    def test_transport_refuses_outward_frame_and_reentry_after_recovery(self):
        self.boundary_fixture()
        self.bus.open_readonly(); self.bus.read(range(1,7),self.config.policy)
        self.bus.authorize(self.config,self.permit); self.bus.enable(2)
        try:
            with self.assertRaises(RuntimeError): self.bus.frame(2,(math.radians(-.4),-.001,3.,1.,0.))
            with self.assertRaises(RuntimeError): self.bus.frame(2,(math.radians(-.9),0.,3.,1.,0.))
            self.bus.frame(2,(math.radians(-.39),0.,3.,1.,0.))
            self.bus.frame(2,(0.,0.,3.,1.,0.))
            with self.assertRaises(RuntimeError): self.bus.frame(2,(math.radians(-.1),0.,3.,1.,0.))
        finally:
            self.bus.stop_supported(); self.bus.close()

    def test_six_axes_timed_startup_refreshes_current_targets_and_finishes(self):
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        for axis in range(1,7):
            frames=[c for c in self.calls('mit') if c[1]==axis]
            self.assertEqual(frames[0][2:],(0.,0.,3.,1.,0.))
            self.assertAlmostEqual(frames[-1][2],self.target[axis-1])
            self.assertTrue(all(c[4:]==(3.,1.,0.) for c in frames))
        self.assertEqual([c[1] for c in self.calls('disable')],list(range(1,7)))
        self.assertFalse(self.executor.stop_result['physical_stopped'])
        self.assertTrue(self.executor.stop_result['stationary_sample'])
        self.assertGreater(self.clock.t,.3) # Startup takes longer than one command lease.
        self.assertTrue(all(op[0] in {'register','read','enable','mit','disable','close_motor','close_bus','close'} for op in self.sdk.calls))

    def test_readonly_derivative_keeps_raw_velocity_and_never_writes(self):
        result=read_start(self.config,self.bus)
        self.assertTrue(result['ok'],result)
        for reading in result['readings']:
            self.assertEqual(reading['4']['qd_raw'],.07)
            self.assertEqual(reading['4']['qd'],0.)
        self.assertFalse(self.calls('enable') or self.calls('mit') or self.calls('disable'))

    def test_missing_six_axis_permission_rejects_before_enable(self):
        self.permit.coordinated_start_checked=False
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'FAULT')
        self.assertFalse(self.calls('enable'))

    def test_slow_enable_and_cancel_do_not_finish_starting_other_axes(self):
        self.sdk.enable_delay=.31
        self.executor.run(self.intent,self.permit)
        self.assertFalse(self.calls('mit'))
        self.assertEqual(self.calls('disable'),[('disable',1)])
        self.setUp()
        self.sdk.after_frame=self.executor.cancel
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'CANCELLED')
        self.assertEqual(self.calls('enable'),[('enable',1)])
        self.assertEqual(self.calls('disable'),[('disable',1)])

    def test_expired_position_window_stops_all_attempted_axes(self):
        self.sdk.enable_delay=.08
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('位置差分窗口',self.executor.reason)
        self.assertEqual(self.calls('disable'),[('disable',i) for i in range(1,7)])

    def test_communication_failure_mid_start_stops_only_attempted_axes(self):
        def fail_after_third(axis_values=None):
            if self.sdk.calls[-1][1]==3: self.sdk.fault='send'
        self.sdk.after_frame=fail_after_third
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'FAULT')
        self.assertEqual(self.calls('disable'),[('disable',i) for i in (1,2,3)])
        stop=next(n for n,c in enumerate(self.sdk.calls) if c[0]=='disable')
        self.assertFalse(any(c[0]=='mit' for c in self.sdk.calls[stop+1:]))

    def test_all_axes_are_resampled_after_enable_before_trajectory(self):
        axis_six_frames=0
        def drift_after_last_start_hold():
            nonlocal axis_six_frames
            if self.sdk.calls[-1][1]==6:
                axis_six_frames+=1
                if axis_six_frames==2: self.sdk.motors[1].q=.003
        self.sdk.after_frame=drift_after_last_start_hold
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('起点发生变化',self.executor.reason)
        self.assertTrue(all(c[2]==0. for c in self.calls('mit')))
        self.assertEqual(self.calls('disable'),[('disable',i) for i in range(1,7)])

    def test_raw_velocity_still_rejects_excessive_damping_effort(self):
        def raw_spike():
            self.beat()
            self.sdk.motors[4].qd=.3
        self.sdk.callback=raw_spike
        self.executor.run(self.intent,self.permit)
        self.assertFalse(self.calls('enable'))
        self.assertIn('原始速度',self.executor.reason)

    def test_mode_and_position_change_block_before_enable(self):
        def wrong_mode(): self.beat(); self.sdk.motors[6].mode=1
        self.sdk.callback=wrong_mode
        self.executor.run(self.intent,self.permit)
        self.assertFalse(self.calls('enable'))
        self.assertIn('MIT',self.executor.reason)

    def test_passive_measurement_noise_is_rebased_but_foreign_targets_reject(self):
        self.permit.axes=(1,); self.permit.mechanically_supported_axes=(1,)
        def noise(): self.beat(); self.sdk.motors[2].q=.0001
        self.sdk.callback=noise
        target=[.004,0,0,0,0,0]
        self.intent=MotionIntent([target],'onsite-single-axis','passive-noise',expected_start=[0.]*6)
        self.executor.run(self.intent,self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        self.assertEqual({c[1] for c in self.calls('mit')},{1})
        self.setUp(); self.permit.axes=(1,); self.permit.mechanically_supported_axes=(1,)
        target[1]=.000001
        self.executor.run(MotionIntent([target],'onsite-single-axis','foreign',expected_start=[0.]*6),self.permit)
        self.assertFalse(self.calls('enable'))
        self.assertIn('未获准',self.executor.reason)
        self.setUp(); self.permit.axes=(1,); self.permit.mechanically_supported_axes=(1,)
        def passive_noise(): self.beat(); self.sdk.motors[2].q=.0001
        self.sdk.callback=passive_noise
        self.executor.run(HoldIntent('fresh-hold',[0.]*6,3),self.permit)
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        self.assertEqual({c[1] for c in self.calls('mit')},{1})


if __name__=='__main__': unittest.main()
