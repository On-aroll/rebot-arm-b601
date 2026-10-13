"""Six-axis trial contracts: actual Seeed classes, timed virtual motors only."""
from dataclasses import replace
from pathlib import Path
import sys
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
