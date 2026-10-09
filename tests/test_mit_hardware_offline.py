"""Real-adapter/worker contracts exercised with a fake SDK and virtual clock ONLY."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_mit_hardware import Envelope,OnsitePermit,SdkTransport,HardwareExecutor,MotionIntent,CanOwnership

class Clock:
    def __init__(self): self.t=0.
    def __call__(self): return self.t
    def wait(self,t): self.t+=t

class Owner:
    def __init__(self): self.busy=False; self.changed=False
    def acquire(self):
        if self.busy: raise RuntimeError('busy CAN')
        self.busy=True
    def claim(self): pass
    def check(self):
        if self.changed: raise RuntimeError('ownership changed')
    def close(self): self.busy=False

class Motor:
    def __init__(self,i,controller): self.i=i; self.c=controller; self.q=0.; self.qd=0.; self.mode=0
    def robstride_get_param_f32_host_id(self,param,host,timeout):
        self.c.calls.append(('read',self.i,param))
        if self.c.fault=='timeout': raise TimeoutError('synthetic timeout')
        if self.c.fault=='slow': self.c.clock.t+=1.
        if self.c.callback: self.c.callback()
        return self.q if param==0x7019 else self.qd
    def robstride_get_param_i8(self,*args): return self.mode
    def enable(self):
        self.c.calls.append(('enable',self.i))
        if self.c.fault=='enable-timeout': raise TimeoutError('enable status unknown')
    def send_mit(self,*values):
        self.c.calls.append(('mit',self.i,*values))
        if self.c.fault=='send': raise IOError('synthetic send failure')
        if self.c.fault!='tracking': self.q=values[0]; self.qd=values[1]
        if self.c.after_frame: self.c.after_frame()
    def disable(self):
        self.c.calls.append(('disable',self.i)); self.qd=0.
        if self.c.fault=='disable': raise TimeoutError('stop status unknown')
    def close(self): self.c.calls.append(('close_motor',self.i))

class Controller:
    def __init__(self,clock):
        self.clock=clock; self.calls=[]; self.motors={}; self.fault=''; self.callback=None; self.after_frame=None
    def add_robstride_motor(self,i,host,model):
        self.calls.append(('register',i,host,model)); self.motors[i]=Motor(i,self); return self.motors[i]
    def close_bus(self): self.calls.append(('close_bus',))
    def close(self): self.calls.append(('close',))

# Deliberately synthetic. This fixture is created inside tests and never published
# as a recommended/approved hardware configuration.
def config():
    axis=dict(q_min=-1,q_max=1,v_max=.5,a_max=1,jerk_max=4,kp=3,kd=1,tau_ff=0,kp_max=3,kd_max=1,torque_max=1,error_max=.15)
    return Envelope.parse(dict(source='explicit-onsite-trial-proposal',version='OFFLINE-SYNTHETIC-NOT-FOR-HARDWARE',axes=[copy.copy(axis) for _ in range(6)],policy=dict(period=.02,io_timeout=.01,feedback_age=.1,command_age=.08,jitter_max=.08,lease_timeout=.1,start_speed=.01,arrival_q=.001,arrival_qd=.01,settle_time=.06,task_timeout=15)))

class HardwareOffline(unittest.TestCase):
    def setUp(self):
        self.clock=Clock(); self.config=config(); self.sdk=Controller(self.clock); self.owner=Owner()
        self.bus=SdkTransport(factory=lambda **kw:self.sdk,ownership=self.owner,clock=self.clock)
        self.executor=HardwareExecutor(self.config,self.bus,clock=self.clock,wait=self.wait)
        self.seq=0; self.renew=True; self.beat()
        self.permit=OnsitePermit(self.config.fingerprint,(6,),100,'synthetic explicit approval',(6,),True,True)
        self.target=[0.]*6; self.target[5]=.1
        self.intent=MotionIntent([self.target],'onsite-single-axis','test-1')
    def beat(self):
        self.seq+=1; self.executor.heartbeat(self.executor.session,self.seq,True)
    def wait(self,t):
        self.clock.wait(t)
        if self.renew: self.beat()
    def run_trial(self): self.executor.run(self.intent,self.permit)
    def commands(self,name): return [x for x in self.sdk.calls if x[0]==name]

    def test_constructor_import_does_not_open_or_write(self):
        self.assertEqual(self.sdk.calls,[])
        with self.assertRaises(PermissionError): self.bus.enable(6)
        self.assertEqual(self.sdk.calls,[])

    def test_readonly_lifecycle_no_shutdown_no_defaults(self):
        self.bus.open_readonly(); self.bus.read(range(1,7),self.config.policy); self.bus.close()
        self.assertFalse(self.commands('enable')); self.assertFalse(self.commands('mit')); self.assertFalse(self.commands('disable'))
        self.assertTrue(self.commands('close_bus'))

    def test_single_axis_success_first_frame_feedback_and_stop_evidence(self):
        self.run_trial(); self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        frames=self.commands('mit'); self.assertEqual(frames[0],('mit',6,0.,0.,3,1,0))
        self.assertTrue(all(x[1]==6 for x in frames)); self.assertEqual(self.commands('enable'),[('enable',6)])
        self.assertEqual(self.commands('disable'),[('disable',6)])
        self.assertEqual(self.executor.stop_result['sdk_status_ack_axes'],[6])
        self.assertFalse(self.executor.stop_result['physical_stopped'])
        self.assertTrue(self.executor.stop_result['stationary_sample'])
        self.assertAlmostEqual(frames[-1][2],.1)

    def test_six_axis_common_trajectory(self):
        self.permit.axes=tuple(range(1,7)); self.permit.mechanically_supported_axes=self.permit.axes
        self.intent=MotionIntent([[.1,.08,-.06,.04,-.02,.1]],'xyz','six-mock',expected_start=[0.]*6)
        self.run_trial(); self.assertEqual(self.executor.state,'ARRIVED',self.executor.reason)
        self.assertEqual({x[1] for x in self.commands('mit')},set(range(1,7)))

    def test_authorization_missing_expired_wrong_config_or_support(self):
        for field,value in [('expires_at',0),('config_hash','wrong'),('operator_present',False),('independent_power_cut_ready',False),('mechanically_supported_axes',()),('used',True),('evidence','')]:
            with self.subTest(field=field):
                self.setUp()
                permit=copy.copy(self.permit); setattr(permit,field,value)
                self.executor.run(self.intent,permit)
                self.assertFalse(self.commands('enable')); self.assertFalse(self.commands('mit'))
                self.executor.used.clear()

    def test_cancel_preempts_feedback_and_targets(self):
        self.sdk.after_frame=self.executor.cancel
        self.run_trial(); self.assertEqual(self.executor.state,'CANCELLED')
        self.assertEqual(len(self.commands('mit')),1); self.assertEqual(self.commands('disable'),[('disable',6)])

    def test_cancel_during_query_stops_before_next_target(self):
        def after(): self.sdk.callback=self.executor.cancel
        self.sdk.after_frame=after
        self.run_trial(); self.assertEqual(len(self.commands('mit')),1)
        self.assertEqual(self.executor.state,'CANCELLED')

    def test_lease_and_blur_do_not_resume(self):
        self.renew=False; self.run_trial()
        self.assertIn('失联',self.executor.reason); count=len(self.commands('mit'))
        self.executor.heartbeat(self.executor.session,999,False)
        with self.assertRaises(RuntimeError): self.run_trial()
        self.assertEqual(len(self.commands('mit')),count)

    def test_heartbeat_session_sequence(self):
        with self.assertRaises(PermissionError): self.executor.heartbeat('old',100,True)
        with self.assertRaises(PermissionError): self.executor.heartbeat(self.executor.session,self.seq,True)

    def test_faults_after_enable_always_attempt_stop_never_continue_target(self):
        for failure in ('timeout','slow','send','tracking','disable','enable-timeout'):
            with self.subTest(failure=failure):
                self.setUp()
                if failure=='enable-timeout': self.sdk.fault=failure
                else: self.sdk.after_frame=lambda f=failure:setattr(self.sdk,'fault',f)
                if failure=='tracking':
                    axes=list(self.config.axes)
                    from dataclasses import replace
                    axes[5]=replace(axes[5],error_max=.00001)
                    self.config=replace(self.config,axes=tuple(axes)); self.executor.config=self.config; self.permit.config_hash=self.config.fingerprint
                self.run_trial()
                self.assertTrue(self.commands('disable'))
                if failure=='disable':
                    self.assertIn('6',self.executor.stop_result['errors'])
                    self.assertEqual(self.executor.stop_result['send_status']['6'],'unknown-may-have-reached-drive')
                else: self.assertEqual(self.executor.state,'FAULT')
                stop_index=next(i for i,x in enumerate(self.sdk.calls) if x[0]=='disable')
                self.assertFalse(any(x[0]=='mit' for x in self.sdk.calls[stop_index+1:]))

    def test_can_busy_and_ownership_change(self):
        self.owner.busy=True; self.run_trial(); self.assertFalse(self.commands('enable'))
        self.setUp(); self.sdk.after_frame=lambda:setattr(self.owner,'changed',True)
        self.run_trial(); self.assertEqual(self.commands('disable'),[('disable',6)])

    def test_foreign_axis_and_mode_rejected(self):
        self.target[0]=.1; self.intent=MotionIntent([self.target],'onsite-single-axis','foreign')
        self.run_trial(); self.assertFalse(self.commands('enable'))
        self.setUp()
        def callback(): self.sdk.motors[6].mode=1
        self.sdk.callback=callback
        self.run_trial(); self.assertFalse(self.commands('enable')); self.assertIn('MIT',self.executor.reason)

    def test_replay_rejected_and_parameters_never_written(self):
        self.run_trial(); before=len(self.sdk.calls)
        with self.assertRaises(RuntimeError): self.run_trial()
        self.assertEqual(len(self.sdk.calls),before)
        allowed={'register','read','mit','enable','disable','close_bus','close_motor','close'}
        self.assertTrue(all(x[0] in allowed for x in self.sdk.calls))

    def test_reviewed_start_moved_is_rejected_before_enable(self):
        self.intent=MotionIntent([self.target],'xyz','changed',expected_start=[.01]*6)
        self.run_trial(); self.assertFalse(self.commands('enable')); self.assertIn('预演',self.executor.reason)

    def test_first_frame_after_enable_uses_new_feedback(self):
        original=Motor.enable
        from unittest.mock import patch
        def enabled(m): original(m); m.q=.0002
        with patch.object(Motor,'enable',enabled): self.run_trial()
        self.assertAlmostEqual(self.commands('mit')[0][2],.0002)
        self.assertAlmostEqual(self.commands('mit')[1][2],.0002)

    def test_trajectory_timeouts_and_torque_envelope(self):
        from dataclasses import replace
        self.config=replace(self.config,policy=replace(self.config.policy,task_timeout=.1))
        self.executor.config=self.config; self.permit.config_hash=self.config.fingerprint
        self.run_trial(); self.assertFalse(self.commands('enable'))
        self.setUp(); self.sdk.after_frame=lambda:setattr(self.sdk.motors[6],'qd',.4)
        self.config=replace(self.config,axes=tuple(replace(a,torque_max=.01) for a in self.config.axes))
        self.executor.config=self.config; self.permit.config_hash=self.config.fingerprint
        self.run_trial(); self.assertTrue('转矩' in self.executor.reason or '起点' in self.executor.reason); self.assertEqual(len(self.commands('mit')),1)

    def test_permit_cannot_replay_in_a_new_executor(self):
        self.run_trial(); previous=len(self.sdk.calls)
        self.executor=HardwareExecutor(self.config,self.bus,clock=self.clock,wait=self.wait)
        self.beat(); self.executor.run(MotionIntent([self.target],'onsite-single-axis','new-request'),self.permit)
        self.assertEqual(len(self.sdk.calls),previous)
        self.assertIn('已使用',self.executor.reason)

    def test_slow_enable_and_startup_queries_do_not_send_expired_hold(self):
        from unittest.mock import patch
        original=Motor.enable
        def slow(m): original(m); self.clock.t+=.09
        with patch.object(Motor,'enable',slow): self.run_trial()
        self.assertIn('命令有效期',self.executor.reason)
        self.assertFalse(self.commands('mit')); self.assertEqual(self.commands('disable'),[('disable',6)])

    def test_foreign_subscriber_during_claim_rejected(self):
        owner=CanOwnership()
        owner.entries=lambda:[('all','can0','first'),('all','can0','second')]
        with self.assertRaises(RuntimeError): owner.claim()

    def test_real_adapter_rejects_multiaxis_before_authorizing(self):
        self.bus.factory=None
        self.permit.axes=(1,6); self.permit.mechanically_supported_axes=(1,6)
        with self.assertRaisesRegex(PermissionError,'单轴'): self.bus.authorize(self.config,self.permit)
        self.assertFalse(self.commands('enable'))

    def test_adapter_rejects_gain_and_range_mutation(self):
        self.bus.open_readonly(); self.bus.authorize(self.config,self.permit); self.bus.enable(6)
        try:
            for frame in ((0,0,9,1,0),(2,0,3,1,0),(0,1,3,1,0),(float('nan'),0,3,1,0)):
                with self.assertRaises(ValueError): self.bus.frame(6,frame)
            self.assertFalse(self.commands('mit'))
        finally: self.bus.stop_supported(); self.bus.close()

    def test_failed_post_stop_verification_is_not_arrival_success(self):
        from unittest.mock import patch
        original=Motor.disable
        for fault in ('moving','timeout'):
            with self.subTest(fault=fault):
                self.setUp()
                def stop(m):
                    original(m)
                    if fault=='moving': m.qd=.2
                    else: m.c.fault='timeout'
                with patch.object(Motor,'disable',stop): self.run_trial()
                self.assertEqual(self.executor.state,'FAULT')
                self.assertTrue(self.executor.motion_arrived)
                self.assertTrue(self.executor.reason)
                self.assertFalse(self.executor.stop_result['physical_stopped'])

    def test_stop_phase_is_visible_before_success(self):
        from unittest.mock import patch
        original=Motor.disable; observed=[]
        def stop(m): observed.append(self.executor.status()['state']); original(m)
        with patch.object(Motor,'disable',stop): self.run_trial()
        self.assertEqual(observed,['STOPPING']); self.assertEqual(self.executor.state,'ARRIVED')

    def test_process_lock(self):
        with tempfile.TemporaryDirectory() as temp:
            a=CanOwnership(directory=temp); b=CanOwnership(directory=temp)
            a.entries=lambda:[]; b.entries=lambda:[]
            a.acquire()
            try:
                with self.assertRaises(BlockingIOError): b.acquire()
            finally: a.close(); b.close()

if __name__=='__main__': unittest.main(verbosity=2)
