"""Actual pinned Seeed classes + fake MotorBridge. No CAN, no device profiles."""
from pathlib import Path
from dataclasses import replace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from test_mit_hardware_offline import Clock, Controller, Owner, config
from rebot_mit_hardware import HardwareExecutor, MotionIntent, OnsitePermit
from rebot_seeed_transport import SeeedTransport
from rebot_seeed_trial import prepare


class SeeedOffline(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.sdk = Controller(self.clock)
        self.bus = SeeedTransport(factory=lambda **_: self.sdk, ownership=Owner(), clock=self.clock)
        self.config = config()
        self.executor = HardwareExecutor(self.config, self.bus, clock=self.clock, wait=self.wait)
        self.renew = True
        self.seq = 0
        self.beat()
        self.permit = OnsitePermit(self.config.fingerprint, (1,), 100, 'offline synthetic permit', (1,), True, True)
        self.intent = MotionIntent([[.00872664626, 0, 0, 0, 0, 0]], 'onsite-single-axis', 'offline-only')

    def beat(self):
        self.seq += 1
        self.executor.heartbeat(self.executor.session, self.seq, True)

    def wait(self, seconds):
        self.clock.wait(seconds)
        if self.renew:
            self.beat()

    def calls(self, op):
        return [call for call in self.sdk.calls if call[0] == op]

    def test_connect_and_close_are_readonly_using_official_registration(self):
        self.assertEqual(self.sdk.calls, [])
        self.bus.open_readonly()
        self.assertEqual(self.bus.arm.__class__.__mro__[1].__name__, 'RebotArm')
        self.bus.read(range(1, 7), self.config.policy)
        self.bus.close()
        self.assertEqual([c[1] for c in self.calls('register')], list(range(1, 7)))
        self.assertFalse(any(self.calls(op) for op in ('enable', 'disable', 'mit')))
        self.assertEqual(len(self.calls('close_motor')), 6)

    def test_rejected_preparation_keeps_raw_feedback_and_never_enables(self):
        register = self.sdk.add_robstride_motor
        def with_out_of_range_joint(i, host, model):
            motor = register(i, host, model)
            if i == 1:
                motor.q = -1.01
            return motor
        self.sdk.add_robstride_motor = with_out_of_range_joint
        result = prepare(self.config, self.bus)
        self.assertFalse(result['ok'])
        self.assertIn('J1 原始位置越限', result['reason'])
        self.assertEqual(result['readings'][0]['1']['q'], -1.01)
        self.assertEqual(result['hardware_writes'], 0)
        self.assertFalse(any(self.calls(op) for op in ('enable', 'disable', 'mit')))

    def test_supported_j1_trial_preserves_uncommanded_out_of_model_raw_pose(self):
        register = self.sdk.add_robstride_motor
        def with_passive_joint(i, host, model):
            motor = register(i, host, model)
            if i == 2: motor.q = -1.01
            if i == 4: motor.qd = .09
            return motor
        self.sdk.add_robstride_motor = with_passive_joint
        prepared = prepare(self.config, self.bus)
        self.assertTrue(prepared['ok'], prepared)
        self.assertEqual(prepared['controlled_axes'], [1])
        self.assertEqual(prepared['expected_start'][1], -1.01)
        self.assertTrue(prepared['uncommanded_model_warnings'])
        target = list(prepared['expected_start']); target[0] += .00872664626
        intent = MotionIntent([target], 'onsite-single-axis', 'scoped-raw-pose', expected_start=prepared['expected_start'])
        self.executor.run(intent, self.permit)
        self.assertEqual(self.executor.state, 'ARRIVED', self.executor.reason)
        self.assertEqual(self.calls('enable'), [('enable', 1)])
        self.assertEqual(self.calls('disable'), [('disable', 1)])
        self.assertTrue(all(frame[1] == 1 for frame in self.calls('mit')))
        self.assertEqual(self.sdk.motors[2].q, -1.01)

    def test_xyz_still_rejects_an_out_of_model_passive_axis(self):
        register = self.sdk.add_robstride_motor
        def with_passive_joint(i, host, model):
            motor = register(i, host, model)
            if i == 2: motor.q = -1.01
            return motor
        self.sdk.add_robstride_motor = with_passive_joint
        start = [0, -1.01, 0, 0, 0, 0]
        target = list(start); target[0] = .00872664626
        self.executor.run(MotionIntent([target], 'xyz', 'xyz-must-check-six', expected_start=start), self.permit)
        self.assertEqual(self.executor.state, 'FAULT')
        self.assertIn('J2 原始位置越限', self.executor.reason)
        self.assertFalse(self.calls('enable'))

    def test_single_axis_uses_official_group_explicit_gains_and_fresh_first_frame(self):
        self.executor.run(self.intent, self.permit)
        self.assertEqual(self.executor.state, 'ARRIVED', self.executor.reason)
        self.assertEqual(self.calls('enable'), [('enable', 1)])
        self.assertEqual(self.calls('disable'), [('disable', 1)])
        frames = self.calls('mit')
        self.assertEqual(frames[0], ('mit', 1, 0., 0., 3., 1., 0.))
        self.assertTrue(all(c[1] == 1 and c[4:] == (3., 1., 0.) for c in frames))
        self.assertAlmostEqual(frames[-1][2], self.intent.waypoints[0][0])
        self.assertFalse(self.executor.stop_result['physical_stopped'])

    def test_no_permission_or_multiple_axes_never_enable(self):
        for multi in (False, True):
            self.setUp()
            if multi:
                self.permit.axes = (1, 2)
                self.permit.mechanically_supported_axes = (1, 2)
            else:
                self.permit.operator_present = False
            self.executor.run(self.intent, self.permit)
            self.assertFalse(self.calls('enable'))
            self.assertFalse(self.calls('mit'))

    def test_sdk_failure_not_swallowed_by_upstream_group(self):
        self.sdk.fault = 'send'
        self.executor.run(self.intent, self.permit)
        self.assertEqual(self.executor.state, 'FAULT')
        self.assertIn('synthetic send failure', self.executor.reason)
        self.assertEqual(self.calls('disable'), [('disable', 1)])

    def test_cancel_preempts_old_targets_and_does_not_home(self):
        self.sdk.after_frame = self.executor.cancel
        self.executor.run(self.intent, self.permit)
        self.assertEqual(self.executor.state, 'CANCELLED', self.executor.reason)
        self.assertEqual(len(self.calls('mit')), 1)
        self.assertEqual(self.calls('disable'), [('disable', 1)])

    def test_feedback_timeout_and_mode_change_never_enable(self):
        for fault in ('timeout', 'slow', 'mode'):
            self.setUp()
            if fault == 'mode':
                self.sdk.callback = lambda: setattr(self.sdk.motors[1], 'mode', 1)
            else:
                self.sdk.fault = fault
            self.executor.run(self.intent, self.permit)
            self.assertEqual(self.executor.state, 'FAULT')
            self.assertFalse(self.calls('enable'))

    def test_expired_lease_stops_selected_axis(self):
        self.renew = False
        self.executor.run(self.intent, self.permit)
        self.assertEqual(self.executor.state, 'FAULT')
        self.assertEqual(self.calls('disable'), [('disable', 1)])

    def test_group_lifecycle_blocked_and_bad_frames_rejected(self):
        self.bus.open_readonly()
        self.bus.authorize(self.config, self.permit)
        self.bus.enable(1)
        self.assertEqual(type(self.bus.trial_group).__name__, 'JointGroup')
        for op in ('enable', 'disable'):
            with self.assertRaises(PermissionError):
                getattr(self.bus.trial_group, op)()
        for axis, values in [(2, (0., 0., 3., 1., 0.)), (1, (0., 0., 50., 3., 0.)),
                             (1, (0., 0., 3., 1., .1)), (1, (True, 0., 3., 1., 0.))]:
            with self.assertRaises((PermissionError, ValueError, RuntimeError)):
                self.bus.frame(axis, values)
        self.assertFalse(self.calls('mit'))
        self.bus.stop_supported()
        self.bus.close()

    def test_proposed_half_degree_envelope_and_readonly_preparation(self):
        self.config = replace(self.config, axes=tuple(replace(a, v_max=.02, a_max=.04, jerk_max=.2,
                              error_max=.017453292519943295, torque_max=.2) for a in self.config.axes))
        prepared = prepare(self.config, self.bus)
        self.assertTrue(prepared['ok'], prepared)
        self.assertEqual(len(prepared['readings']), 3)
        self.assertFalse(self.calls('enable'))
        self.executor = HardwareExecutor(self.config, self.bus, clock=self.clock, wait=self.wait)
        self.beat()
        self.permit.config_hash = self.config.fingerprint
        self.executor.run(self.intent, self.permit)
        self.assertEqual(self.executor.state, 'ARRIVED', self.executor.reason)
        previous = None
        for sample in self.executor.frames:
            command = sample['commands'][0]
            self.assertLessEqual(abs(command['values'][1]), .02+1e-12)
            self.assertLessEqual(command['estimated_effort'], .2+1e-12)
            if previous:
                delta_t = sample['time']-previous['time']
                self.assertLessEqual(abs(command['values'][0]-previous['commands'][0]['values'][0]), .02*delta_t+1e-12)
            previous = sample


if __name__ == '__main__':
    unittest.main()
