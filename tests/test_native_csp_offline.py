"""Native CSP adapter exercised with virtual drives; never opens real CAN."""
from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from test_mit_hardware_offline import Clock,Owner,Controller,Motor
from rebot_mit_hardware import Envelope,MotionIntent,OnsitePermit
from rebot_native_csp import CspTransport,CspJ3Trial,DELTA,PROTECTED,RUNTIME,TASK_TIMEOUT,proposal,probe


class NativeMotor(Motor):
    def __init__(self,*args):
        super().__init__(*args)
        self.regs={0x7005:0,0x7017:1.,0x7018:57.,0x700B:36.,0x7016:.7,
                   0x701E:30.,0x701F:6.,0x7020:.02,0x702B:0.}
        self.enabled=False

    def robstride_get_param_f32_host_id(self,param,host,timeout):
        self.c.clock.t+=.001; self.c.calls.append(('read',self.i,param))
        if self.c.fault=='timeout': raise TimeoutError('synthetic read timeout')
        if self.c.callback: self.c.callback()
        if param==0x7019: return self.q
        if param==0x701B: return self.qd
        if param in (0x701A,0x302C): return self.c.current if param==0x701A else 0.
        value=self.regs[param]
        if self.c.bad_readback==param and self.regs[0x7005]==5: return value+1
        return value

    def robstride_get_param_i8(self,index,timeout):
        self.c.clock.t+=.001
        return self.regs[index]

    def robstride_write_param_f32(self,index,value):
        self.c.calls.append(('write',self.i,index,value,self.c.clock()))
        self.regs[index]=value
        if index==0x7016 and self.enabled and not self.c.stalled: self.q=value
        if self.c.after_write: self.c.after_write(index)

    def robstride_write_param_u8(self,index,value):
        raise TypeError('SDK run_mode is Int8, not UInt8')

    def robstride_write_param_i8(self,index,value):
        self.robstride_write_param_f32(index,value)
        self.mode=value; self.enabled=False

    def enable(self):
        super().enable(); self.enabled=True
        if self.c.after_enable: self.c.after_enable()

    def disable(self):
        super().disable(); self.enabled=False

    def get_state(self):
        mode=2 if self.c.bad_stop else 0
        return SimpleNamespace(arbitration_id=(2<<24)|(mode<<22)|(self.i<<8)|0xFD)


class NativeController(Controller):
    def __init__(self,clock):
        super().__init__(clock)
        self.bad_readback=None; self.bad_stop=False; self.current=0.; self.stalled=False
        self.after_write=None; self.after_enable=None

    def add_robstride_motor(self,i,host,model):
        self.calls.append(('register',i,host,model))
        self.motors[i]=NativeMotor(i,self)
        return self.motors[i]


class NativeCspOffline(unittest.TestCase):
    def setUp(self):
        self.clock=Clock(); self.sdk=NativeController(self.clock); self.owner=Owner()
        raw,_=proposal([(-1.,1.)]*6); self.config=Envelope.parse(raw)
        self.bus=CspTransport(factory=lambda **_:self.sdk,ownership=self.owner,clock=self.clock)
        self.seq=0; self.renew=True
        self.executor=CspJ3Trial(self.config,self.bus,clock=self.clock,wait=self.wait)
        self.beat()
        self.permit=OnsitePermit(self.config.fingerprint,(3,),100.,'synthetic approval',(3,),True,True)
        target=[0.]*6; target[2]=DELTA
        self.intent=MotionIntent([target],'native-csp-j3','virtual-only',expected_start=[0.]*6)

    def beat(self):
        self.seq+=1; self.executor.heartbeat(self.executor.session,self.seq,True)

    def wait(self,seconds):
        self.clock.wait(seconds)
        if self.renew: self.beat()

    def run_trial(self): self.executor.run(self.intent,self.permit)
    def calls(self,name): return [c for c in self.sdk.calls if c[0]==name]

    def test_import_construction_and_readonly_probe_never_write(self):
        self.assertEqual(self.sdk.calls,[])
        with self.assertRaises(PermissionError): self.bus.write_register(0x7005,5,lambda:None,phase='prepare')
        with self.assertRaises(PermissionError): self.bus.enable(3)
        with self.assertRaises(PermissionError): self.bus.quiesce()
        result=probe(self.config,self.bus)
        self.assertTrue(result['ok'],result)
        self.assertEqual(result['hardware_writes'],0)
        self.assertFalse(self.calls('write') or self.calls('enable') or self.calls('disable'))

    def test_preload_limits_before_enable_arrive_then_disable_and_restore(self):
        self.run_trial()
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        enable=self.sdk.calls.index(('enable',3))
        writes=[c for c in self.sdk.calls[:enable] if c[0]=='write']
        self.assertEqual([(c[2],c[3]) for c in writes],[(0x7005,5),(0x700B,9.),(0x7018,10.),(0x7017,.02),(0x7016,0.)])
        self.assertTrue(all(c[1]==3 for c in self.calls('write')+self.calls('enable')+self.calls('disable')))
        stop=max(i for i,c in enumerate(self.sdk.calls) if c==('disable',3))
        restored=[c[2] for c in self.sdk.calls[stop:] if c[0]=='write']
        self.assertEqual(restored,[0x7016,0x7005,0x7017,0x7018,0x700B])
        motor=self.sdk.motors[3]
        self.assertEqual([motor.regs[i] for i in RUNTIME],[0,1.,57.,36.])
        self.assertAlmostEqual(motor.regs[0x7016],DELTA)
        self.assertFalse(any(c[2] in PROTECTED for c in self.calls('write')))
        self.assertEqual(self.calls('mit'),[])
        status=self.executor.status()
        self.assertTrue(status['restoration']['ok'])
        self.assertTrue(status['stop']['stationary_sample'])
        self.assertFalse(status['physical_acceptance'])
        self.assertAlmostEqual(status['arrival_feedback']['3']['q'],DELTA)
        timing=status['timing']
        self.assertLess(timing['enable_attempted_at'],timing['stop_requested_at'])
        self.assertLessEqual(timing['stop_requested_at'],timing['stop_call_completed_at'])
        self.assertEqual(timing['power_deadline']-timing['enable_attempted_at'],15.)

    def test_readback_mismatch_never_enables(self):
        self.sdk.bad_readback=0x7017
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('读回不一致',self.executor.reason)
        self.assertFalse(self.calls('enable'))
        self.assertTrue(self.calls('disable'))

    def test_cancel_after_partial_runtime_write_disables_before_restore(self):
        self.sdk.after_write=lambda index:self.executor.cancel() if index==0x7005 else None
        self.run_trial()
        self.assertEqual(self.executor.state,'CANCELLED',self.executor.reason)
        self.assertFalse(self.calls('enable'))
        self.assertTrue(self.executor.restore_result['ok'])
        self.assertEqual(self.sdk.motors[3].regs[0x7005],0)

    def test_timeout_after_enable_stops_and_does_not_send_target(self):
        self.sdk.after_enable=lambda:setattr(self.sdk,'fault','timeout')
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        enable=self.sdk.calls.index(('enable',3))
        self.assertFalse(any(c[0]=='write' for c in self.sdk.calls[enable:]))
        self.assertFalse(self.executor.restore_result['ok'])

    def test_enable_uncertainty_still_attempts_disable(self):
        self.sdk.fault='enable-timeout'; self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertEqual(len(self.calls('disable')),2)
        self.assertTrue(self.executor.restore_result['ok'])

    def test_stop_mode_not_disabled_leaves_lower_limits(self):
        self.sdk.after_enable=lambda:setattr(self.sdk,'bad_stop',True)
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertFalse(self.executor.restore_result['ok'])
        self.assertEqual(self.sdk.motors[3].regs[0x7018],10.)
        self.assertEqual(self.sdk.motors[3].regs[0x700B],9.)

    def test_drive_already_lower_limited_is_not_raised(self):
        original=self.sdk.add_robstride_motor
        def register(*args):
            motor=original(*args);motor.regs[0x7018]=5.;return motor
        self.sdk.add_robstride_motor=register
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('不能自动提高',self.executor.reason)
        self.assertFalse(self.calls('write') or self.calls('enable'))

    def test_firmware_position_readback_resolution_uses_small_explicit_tolerance(self):
        read=CspTransport.read_register
        def quantized(bus,index):
            value=read(bus,index)
            return value-.000128 if index==0x7016 else value
        self.bus.read_register=lambda index:quantized(self.bus,index)
        self.run_trial()
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        self.assertTrue(self.executor.restore_result['ok'])

    def test_feedback_sweep_time_is_included_in_cycle_without_widening_deadline(self):
        read=self.bus.read
        def timed(*args,**kwargs):
            result=read(*args,**kwargs)
            if self.sdk.motors[3].enabled: self.clock.t+=.14
            return result
        self.bus.read=timed
        self.run_trial()
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        self.assertEqual(self.config.policy.period,.1)
        self.assertEqual(self.config.policy.command_age,.3)
        gaps=[b['time']-a['time'] for a,b in zip(self.executor.frames,self.executor.frames[1:])]
        self.assertTrue(gaps and max(gaps)<.25)

    def test_fixed_profile_cannot_be_widened(self):
        with self.assertRaises(ValueError):
            CspJ3Trial(replace(self.config,policy=replace(self.config.policy,task_timeout=50.)),self.bus)

    def test_target_cannot_command_another_axis_or_increase_delta(self):
        for target in ([.01,0.,DELTA,0.,0.,0.],[0.,0.,DELTA*2,0.,0.,0.]):
            self.setUp()
            self.intent=MotionIntent([target],'native-csp-j3','bad-target',expected_start=[0.]*6)
            self.run_trial();self.assertEqual(self.executor.state,'FAULT');self.assertEqual(self.sdk.calls,[])

    def test_stalled_position_stops_at_approved_fifteen_second_deadline(self):
        self.sdk.stalled=True; self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('15 秒',self.executor.reason)
        self.assertEqual(TASK_TIMEOUT,15.)
        self.assertGreaterEqual(self.executor.stop_requested_at,self.executor.power_deadline)
        targets=[r for r in self.bus.register_log if r['phase']=='trajectory']
        self.assertTrue(targets)
        self.assertLess(targets[-1]['requested_at'],self.executor.power_deadline)
        self.assertFalse(self.executor.motion_arrived)

    def test_lost_lease_cancels_without_automatic_retry(self):
        self.renew=False; self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('失联',self.executor.reason)
        self.assertEqual(len(self.calls('enable')),1)

    def test_foreign_can_owner_change_preempts_enable(self):
        self.sdk.after_write=lambda _:setattr(self.owner,'changed',True)
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertFalse(self.calls('enable'))
        self.assertTrue(self.calls('disable'))

    def test_current_limit_exceeded_aborts_motion(self):
        self.sdk.after_enable=lambda:setattr(self.sdk,'current',10.5)
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertIn('电流反馈',self.executor.reason)
        self.assertFalse(self.executor.motion_arrived)

    def test_changed_start_and_out_of_limit_goal_never_write(self):
        self.intent.expected_start=(0.,0.,.2,0.,0.,0.)
        self.intent.waypoints=((0.,0.,.2+DELTA,0.,0.,0.),)
        self.run_trial()
        self.assertEqual(self.executor.state,'FAULT')
        self.assertFalse(self.calls('write') or self.calls('enable'))

    def test_nonzero_first_target_is_measured_and_unsafe_old_target_never_restored(self):
        register=self.sdk.add_robstride_motor
        def nonzero(*args):
            motor=register(*args)
            if motor.i==3: motor.q=.12
            return motor
        self.sdk.add_robstride_motor=nonzero
        self.intent.expected_start=(0.,0.,.12,0.,0.,0.)
        self.intent.waypoints=((0.,0.,.12+DELTA,0.,0.,0.),)
        self.run_trial()
        self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        targets=[r for r in self.bus.register_log if r['index']=='0x7016']
        self.assertEqual(targets[0]['phase'],'preload-current')
        self.assertEqual(targets[0]['value'],.12)
        self.assertFalse(any(r['value']==.7 for r in targets))

    def test_slow_sdk_and_cancel_preempt_next_target(self):
        for delay in (False,True):
            self.setUp()
            def after(index):
                if index==0x7016 and self.sdk.motors[3].enabled:
                    if delay: self.clock.t+=.31
                    else: self.executor.cancel()
            self.sdk.after_write=after
            self.run_trial()
            self.assertEqual(self.executor.state,'FAULT' if delay else 'CANCELLED',self.executor.reason)
            self.assertEqual(len([r for r in self.bus.register_log if r['phase'] in ('hold','trajectory')]),1)
            self.assertTrue(self.calls('disable'))


if __name__=='__main__': unittest.main()
