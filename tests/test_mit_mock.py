"""B1 tests: virtual clock/transport exclusively, no sockets, CAN or live profiles."""
import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_mit_mock import (HardwareDisabledTransport, MockConfig, MockExecutor,
                           MockMotorTransport, Trajectory, VirtualClock, shape)
FIXTURE = Path(__file__).parent/'fixtures/mit_mock_only.json'
START = (6.4, -3.8, .5, -.8, 2.1, -7.2)  # deliberately raw multi-turn
TARGET = (6.7, -3.6, .1, -.4, 1.8, -7.1)
WAYPOINTS = [TARGET, (6.5, -3.9, .2, -.7, 2.0, -7.3)]


class MitMockTests(unittest.TestCase):
    def setUp(self):
        self.raw = json.loads(FIXTURE.read_text())
        self.build()

    def build(self):
        self.config = MockConfig.parse(self.raw)
        self.clock = VirtualClock()
        self.bus = MockMotorTransport(self.clock, self.config.device, START)
        self.executor = MockExecutor(self.config, self.bus, self.clock)

    def start(self, points=None):
        plan = self.executor.prepare(points or WAYPOINTS, 'test-v1')
        self.executor.start(plan, 'test-v1')
        return plan

    def tick(self, count=1, dt=None):
        for _ in range(count):
            self.clock.advance(self.config.policy.period if dt is None else dt)
            self.executor.tick()

    def run_task(self):
        for _ in range(math.ceil(self.config.policy.task_timeout/self.config.policy.period)+10):
            if self.executor.state != 'RUNNING_MOCK':
                return
            self.tick()
        self.fail('Unbounded executor')

    def fault(self, fragment):
        self.assertEqual(self.executor.state, 'FAULT', self.executor.status())
        self.assertIn(fragment, self.executor.reason)
        count = len(self.bus.frames)
        self.tick(5)
        self.assertEqual(len(self.bus.frames), count, 'No auto-resume/retry after fault')
        self.assertFalse(self.executor.status()['hardware'])
        self.assertFalse(self.executor.status()['physical_stop_verified'])

    def test_first_target_raw_feedback_and_complete_six_axes(self):
        plan = self.start()
        first = self.bus.frames[0]
        self.assertEqual(first.q_des, START)
        self.assertEqual(first.qd_des, (0.,)*6)
        self.assertEqual(first.sequence, 1)
        self.assertEqual(first.config_hash, self.config.fingerprint)
        self.run_task()
        self.assertEqual(self.executor.state, 'COMPLETED', self.executor.reason)
        self.assertEqual(self.bus.frames[-1].q_des, plan.trajectory.points[-1])
        self.assertEqual(self.bus.frames[-1].qd_des, (0.,)*6)
        self.assertEqual(self.executor.events[-1]['kind'], 'arrival_confirmed')
        for i, frame in enumerate(self.bus.frames):
            self.assertEqual(frame.sequence, i+1)
            self.assertEqual(len(frame.q_des), 6)
            self.assertEqual(frame.kp, tuple(a.kp for a in self.config.axes))
            self.assertGreater(frame.expires_at, frame.issued_at)

    def test_continuity_and_bounds_independent_numerical_derivatives(self):
        trajectory = Trajectory.create(START, WAYPOINTS, self.config)
        # Differentiate position samples independently, rather than trusting sample's derivatives.
        for index, duration in enumerate(trajectory.durations):
            segment = Trajectory(trajectory.points[index:index+2], (duration,))
            count = 1000
            dt = duration/count
            qs = [segment.sample(i*dt)[0] for i in range(count+1)]
            for axis, limit in enumerate(self.config.axes):
                values = [q[axis] for q in qs]
                for order, bound in enumerate((limit.v_max, limit.a_max, limit.jerk_max), 1):
                    values = [(b-a)/dt for a, b in zip(values, values[1:])]
                    self.assertLessEqual(max(map(abs, values)), bound*1.0001, (axis, order))
            for i in range(count+1):
                _, v, a, j = segment.sample(i*dt)
                for axis, limit in enumerate(self.config.axes):
                    for value, bound in zip((v[axis], a[axis], j[axis]), (limit.v_max, limit.a_max, limit.jerk_max)):
                        self.assertLessEqual(abs(value), bound+1e-9)
        boundary = trajectory.durations[0]
        q, v, a, j = trajectory.sample(boundary)
        self.assertEqual(q, TARGET)
        self.assertEqual((v, a, j), ((0.,)*6,)*3)
        for derivative in range(4):
            left = trajectory.sample(boundary-1e-7)[derivative]
            right = trajectory.sample(boundary+1e-7)[derivative]
            self.assertLess(max(abs(x-y) for x,y in zip(left,right)), 1e-4)
        self.assertEqual(shape(0), (0,0,0,0)); self.assertEqual(shape(1), (1,0,0,0))

    def test_configuration_is_explicit_mock_only_and_immutable(self):
        variations = []
        for key, value in [('source','human-approved'),('approval_scope','physical'),('device','real-device'),('units',{}),('axes',self.raw['axes'][:5]),('axes',self.raw['axes']+[self.raw['axes'][0]])]:
            data=copy.deepcopy(self.raw); data[key]=value; variations.append(data)
        for key, value in [('kp',None),('tau_ff',float('nan')),('v_max',float('inf')),('a_max',0),('jerk_max',-1),('kp',8),('kd',2),('tau_ff',3),('q_min',99),('kp',True)]:
            data=copy.deepcopy(self.raw); data['axes'][0][key]=value; variations.append(data)
        data=copy.deepcopy(self.raw); del data['axes'][0]['kp']; variations.append(data)
        data=copy.deepcopy(self.raw); del data['policy']['period']; variations.append(data)
        for data in variations:
            with self.subTest(data=data), self.assertRaises(ValueError): MockConfig.parse(data)
        with self.assertRaises(AttributeError): self.config.axes[0].kp=1
        # Even bypassing parse does not bypass constructor validation.
        bad=replace(self.config, axes=(replace(self.config.axes[0],kp=9),)+self.config.axes[1:])
        with self.assertRaises(ValueError): MockExecutor(bad,self.bus,self.clock)

    def test_no_hardware_transport_or_subclass_adapter(self):
        with self.assertRaises(PermissionError): HardwareDisabledTransport()
        class ExternalAdapter(MockMotorTransport): pass
        with self.assertRaises(PermissionError): MockExecutor(self.config,ExternalAdapter(self.clock,self.config.device,START),self.clock)
        with self.assertRaises(PermissionError): MockExecutor(self.config,object(),self.clock)
        self.assertFalse(self.executor.status()['hardware'])
        self.assertEqual(self.executor.status()['hardware_state'],'LOCKED')

    def test_bad_waypoint_limits_nonfinite_and_seventh_axis(self):
        for target in ([99]*6,[float('nan')]*6,[float('inf')]*6,[1]*7,[1]*5,[True]*6):
            with self.subTest(target=target), self.assertRaises(ValueError): self.executor.prepare([target],'v1')
        self.assertEqual(self.bus.frames,[])

    def test_expired_replayed_changed_task_and_copied_plan(self):
        p=self.executor.prepare(WAYPOINTS,'v1'); self.clock.advance(3)
        with self.assertRaisesRegex(ValueError,'expired'): self.executor.start(p,'v1')
        self.executor.reset(); p=self.executor.prepare(WAYPOINTS,'v1')
        with self.assertRaisesRegex(ValueError,'unknown'): self.executor.start(replace(p),'v1')
        p=self.executor.prepare(WAYPOINTS,'v1')
        with self.assertRaisesRegex(ValueError,'version'): self.executor.start(p,'v2')
        p=self.start()
        with self.assertRaises(ValueError): self.executor.start(p,'test-v1')
        with self.assertRaises(ValueError): self.executor.prepare(WAYPOINTS,'test-v2')
        with self.assertRaises(ValueError): self.executor.reset()
        self.assertEqual(len(self.bus.frames),1)

    def test_invalid_replacement_preview_invalidates_previous_token(self):
        previous=self.executor.prepare(WAYPOINTS,'v1')
        with self.assertRaises(ValueError): self.executor.prepare([[99]*6],'v2')
        with self.assertRaises(ValueError): self.executor.start(previous,'v1')
        self.assertEqual(self.bus.frames,[])

    def test_extreme_finite_duration_is_rejected_without_frames(self):
        for axis in self.raw['axes']: axis['v_max']=1e-310
        self.build()
        with self.assertRaises(ValueError): self.executor.prepare(WAYPOINTS,'v1')
        self.assertEqual(self.bus.frames,[])
        self.raw=json.loads(FIXTURE.read_text())
        self.raw['policy'].update(period=1e106,command_age=1e108,ack_timeout=1e107,task_timeout=1e110)
        self.build()
        with self.assertRaises(ValueError): self.executor.prepare(WAYPOINTS,'v1')
        self.assertEqual(self.bus.frames,[])

    def test_first_pose_drift_and_moving_start_rejected(self):
        p=self.executor.prepare(WAYPOINTS,'v1'); self.bus.q=tuple(x+.001 for x in START)
        with self.assertRaisesRegex(ValueError,'Start moved'): self.executor.start(p,'v1')
        self.bus.qd=(.1,)*6
        with self.assertRaisesRegex(ValueError,'stationary'): self.executor.prepare(WAYPOINTS,'v1')
        self.assertEqual(self.bus.frames,[])

    def test_config_changes_invalidate_preview_and_stop_active_task(self):
        p=self.executor.prepare(WAYPOINTS,'v1')
        self.executor.replace_config(replace(self.config,version='v2'))
        with self.assertRaises(ValueError): self.executor.start(p,'v1')
        self.start()
        with self.assertRaises(ValueError): self.executor.replace_config(self.config)
        self.fault('configuration changed')

    def test_feedback_timeout_each_axis(self):
        self.raw['policy']['feedback_skew']=.1  # isolate the per-axis age guard
        for axis in range(6):
            with self.subTest(axis=axis):
                self.build(); self.start(); self.bus.frozen_at[axis]=self.clock.now
                self.tick(5); self.fault('feedback stale')

    def test_six_axis_feedback_skew_is_not_hidden_by_freshest_axis(self):
        self.start(); self.bus.frozen_at[4]=self.clock.now
        self.tick(3); self.fault('sampling skew')

    def test_feedback_nonfinite_mode_identity_validity_speed(self):
        cases=[('q',(float('nan'),)*6,'finite'),('qd',(float('inf'),)*6,'finite'),
               ('modes',(1,)*6,'mode'),('device','virtual:other','identity'),
               ('valid',(False,)*6,'validity'),('q',(99,)*6,'envelope'),('qd',(1.,)*6,'envelope')]
        for name,value,reason in cases:
            with self.subTest(name=name):
                self.build(); self.start(); setattr(self.bus,name,value); self.tick(); self.fault(reason)

    def test_feedback_timestamp_and_sequence_order(self):
        self.start(); self.bus.frozen_at[3]=10.; self.tick(); self.fault('future')
        self.build(); self.clock.advance(.1); self.start(); self.tick()
        self.bus.frozen_at[2]=.105; self.tick(); self.fault('reversed')
        self.build(); self.start(); self.bus.freeze_sequence=True; self.tick(); self.fault('sequence')

    def test_connection_read_write_and_negative_ack_errors(self):
        for field,reason in [('connected','communication'),('fail_read','communication'),('fail_write','communication'),('ack_ok','negative acknowledgement')]:
            with self.subTest(field=field):
                self.build()
                if field=='ack_ok': self.bus.ack_ok=False
                self.start()
                if field!='ack_ok': setattr(self.bus,field,field!='connected')
                self.tick(); self.fault(reason)
                if field=='connected': self.assertFalse(self.executor.status()['stop_sent'])

    def test_missing_ack_no_extra_motion_and_fault(self):
        self.bus.drop_ack=True; self.start(); self.tick(4)
        self.fault('acknowledgement timeout')
        self.assertEqual(len(self.bus.frames),1)
        self.assertTrue(self.executor.status()['stop_acknowledged'])

    def test_cancel_preempts_missing_feedback_and_motion_ack(self):
        self.bus.drop_ack=True; self.start(); self.bus.fail_read=True
        self.executor.request_cancel(); self.tick()
        self.assertEqual(self.executor.state,'STOPPED')
        self.assertEqual(len(self.bus.frames),1)
        kinds=[e['kind'] for e in self.executor.events]
        self.assertEqual(kinds[-3:],['stop_requested','stop_sent','stop_acknowledged'])
        self.assertFalse(self.executor.status()['physical_stop_verified'])
        self.tick(3); self.assertEqual(len(self.bus.stops),1)

    def test_cancel_during_read_prevents_next_frame(self):
        self.start(); self.bus.on_read=self.executor.request_cancel
        self.tick(); self.assertEqual(self.executor.state,'STOPPED')
        self.assertEqual(len(self.bus.frames),1)

    def test_stop_ack_delayed_missing_negative_and_send_failure(self):
        self.bus.stop_ack_delay=.02; self.start(); self.executor.request_cancel(); self.tick()
        self.assertEqual(self.executor.state,'STOP_REQUESTED')
        self.assertFalse(self.executor.status()['stop_acknowledged'])
        self.tick(2); self.assertEqual(self.executor.state,'STOPPED')
        for flag,reason in [('drop_stop_ack','stop acknowledgement timeout'),('stop_ack_ok','negative'),('fail_stop','stop send failed')]:
            with self.subTest(flag=flag):
                self.build(); self.start(); setattr(self.bus,flag,flag!='stop_ack_ok')
                self.executor.request_cancel(); self.tick(6); self.fault(reason)
                self.assertEqual(len(self.bus.frames),1)
                self.assertFalse(self.executor.status()['stop_acknowledged'])

    def test_clock_reverse_early_tick_and_cycle_stall(self):
        for dt,reason in [(-.01,'clock'),(.001,'too early'),(.02,'overrun')]:
            with self.subTest(dt=dt):
                self.build(); self.start(); self.tick(dt=dt); self.fault(reason)
                self.assertEqual(len(self.bus.frames),1)

    def test_expired_command_is_not_replayed(self):
        self.start()
        self.executor._last_frame=replace(self.executor._last_frame,expires_at=0.)
        self.tick(); self.fault('command expired')

    def test_delayed_ack_does_not_create_catchup_burst(self):
        self.bus.ack_delay=.02; self.start(); self.tick(2)
        self.fault('cadence overrun'); self.assertEqual(len(self.bus.frames),1)

    def test_tracking_error_and_estimated_torque_rejection(self):
        self.start(); self.bus.q=tuple(x+.2 for x in START)
        self.tick(); self.fault('tracking error')
        self.raw['axes'][0]['torque_max']=.000001
        self.build(); self.bus.follow=False; self.start(); self.run_task()
        self.fault('estimated MIT effort')

    def test_ack_alone_does_not_confirm_arrival_and_task_timeout(self):
        for axis in self.raw['axes']: axis['error_max']=2
        self.raw['policy']['task_timeout']=8
        self.build(); self.bus.follow=False; self.start([TARGET]); self.run_task()
        self.fault('task timeout')
        self.assertFalse(any(e['kind']=='arrival_confirmed' for e in self.executor.events))

    def test_reset_and_process_restart_never_replay_frames(self):
        self.start(); self.executor.request_cancel(); self.tick(); before=len(self.bus.frames)
        self.executor.reset(); self.tick(5)
        self.assertEqual(len(self.bus.frames),before)
        self.assertIsNone(self.executor._last_frame)
        self.assertFalse(self.executor.status()['stop_acknowledged'])
        restarted=MockExecutor(self.config,self.bus,self.clock); restarted.tick()
        self.assertEqual(restarted.state,'READY_FOR_MOCK'); self.assertEqual(len(self.bus.frames),before)
        self.assertFalse(restarted.status()['hardware'])


if __name__=='__main__': unittest.main(verbosity=2)
