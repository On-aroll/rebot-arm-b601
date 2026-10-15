"""J1 power-trial contracts, using virtual motors and fake child processes only.

This module never imports the installed motorbridge driver, opens CAN, or starts
a real process.  It does not create an approved hardware configuration.
"""
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from test_mit_hardware_offline import Clock, Controller, Motor, Owner, config
from rebot_mit_hardware import HardwareExecutor, HoldIntent, MotionIntent, OnsitePermit, SdkTransport
from rebot_motor_power import CONFIRMATIONS, MotorPowerAPI, trial_proposal
from rebot_power_worker import probe, run_trial


class FakeStore:
    def __init__(self):
        self.state = dict(revision=25, approved={}, drafts={
            str(i): {'fields': {k: {'value': v} for k, v in [('kp', 3), ('kd', 1), ('tau_ff', 0)]}}
            for i in range(1, 7)})

    def status(self):
        return {'state': copy.deepcopy(self.state)}


class FakeProcess:
    def __init__(self):
        self.stdin = io.StringIO()
        self.stdout = io.StringIO()
        self.returncode = None

    def poll(self):
        return self.returncode

    def wait(self):
        self.returncode = 0
        return 0

    def messages(self):
        return [json.loads(line) for line in self.stdin.getvalue().splitlines()]


class PowerAPI(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        python = self.root / '.venv/bin/python'
        python.parent.mkdir(parents=True)
        python.touch()  # Existence guard only; no execution permission or content.
        self.store = FakeStore()
        self.clock = Clock()
        self.process = FakeProcess()
        self.popen = Mock(return_value=self.process)
        self.pose = Mock()
        self.api = MotorPowerAPI(self.store, 8765, self.root, [(-1., 1.)] * 6,
                                 self.pose, popen=self.popen, clock=self.clock)
        # A regression that bypasses the injected child factory fails immediately.
        self.real_spawn = patch('subprocess.Popen', side_effect=AssertionError('real child process forbidden'))
        self.real_spawn.start()
        self.addCleanup(self.real_spawn.stop)
        self.threads = patch('rebot_motor_power.threading.Thread')
        self.thread = self.threads.start()
        self.addCleanup(self.threads.stop)
        self.api._readonly_probe = Mock(return_value=dict(ok=True, expected_start=[0.] * 6,
            feedback={str(i): dict(q=0., qd=0., mode=0) for i in range(1, 7)}, hardware_writes=0))
        self.base_headers = {'Host': '127.0.0.1:8765', 'Origin': 'http://127.0.0.1:8765',
                             'Sec-Fetch-Site': 'same-origin'}
        self.token = self.session()

    def session(self):
        return self.api.request('/motor-power/session', self.base_headers, '127.0.0.1', {})['token']

    def action(self, action, token=None, **body):
        headers = dict(self.base_headers, **{'X-Motor-Session': self.token if token is None else token})
        return self.api.request('/motor-power/action', headers, '127.0.0.1', dict(action=action, **body))

    def prepare(self):
        return self.action('prepare')['plan_id']

    def start(self, plan=None, **extra):
        return self.action('start', plan_id=plan or self.prepare(),
                           confirmed={k: True for k in CONFIRMATIONS}, **extra)

    def test_construction_status_and_session_do_not_start_hardware(self):
        self.assertEqual(self.api.status()['state'], 'IDLE')
        self.assertFalse(self.api.status()['busy'])
        self.assertFalse(self.api.status()['enabled_feedback_available'])
        self.popen.assert_not_called()
        self.api._readonly_probe.assert_not_called()

    def test_prepare_is_readonly_checks_pose_and_preserves_config(self):
        before = copy.deepcopy(self.store.state)
        result = self.action('prepare')
        self.assertEqual(result['status']['state'], 'READY')
        self.assertEqual(result['proposal']['seconds'], 3)
        self.pose.assert_called_once_with([0.] * 6)
        self.api._readonly_probe.assert_called_once()
        self.popen.assert_not_called()
        self.assertEqual(self.store.state, before)

    def test_start_without_plan_cannot_spawn(self):
        with self.assertRaises(ValueError):
            self.action('start', plan_id='invented', confirmed={k: True for k in CONFIRMATIONS})
        self.popen.assert_not_called()

    def test_xyz_namespace_uses_only_server_plan_and_preserves_parameters(self):
        raw,_=trial_proposal(self.store.state,self.api.limits)
        trusted=dict(id='xyz-token',expires=60.,config=raw,revision=self.store.state['revision'],
                     expected_start=[0.]*6,target=[.001]*6)
        self.api.point_plan=Mock(return_value=trusted)
        headers=dict(self.base_headers,**{'X-Motor-Session':self.token})
        before=copy.deepcopy(self.store.state)
        result=self.api.request('/hardware-point/action',headers,'127.0.0.1',dict(action='execute',plan_id='xyz-token',
            confirmed={k:True for k in CONFIRMATIONS},target=[99.]*6,kp=500.))
        self.api.point_plan.assert_called_once_with('xyz-token')
        self.assertTrue(result['status']['busy'])
        message=self.process.messages()[0]
        self.assertEqual(message['operation'],'move-xyz')
        self.assertEqual(message['target'],trusted['target'])
        self.assertEqual(message['config'],raw)
        self.assertEqual(self.store.state,before)

    def test_xyz_unconfirmed_or_failed_preflight_never_starts_a_worker(self):
        self.api.point_plan=Mock(side_effect=ValueError('PD/前馈不足以承重'))
        headers=dict(self.base_headers,**{'X-Motor-Session':self.token})
        with self.assertRaises(ValueError): self.api.request('/hardware-point/action',headers,'127.0.0.1',dict(action='execute',plan_id='xyz-token'))
        self.api.point_plan.assert_not_called()
        with self.assertRaisesRegex(ValueError,'承重'):
            self.api.request('/hardware-point/action',headers,'127.0.0.1',dict(action='execute',plan_id='xyz-token',confirmed={k:True for k in CONFIRMATIONS}))
        self.popen.assert_not_called()

    def test_startup_observation_requires_the_reviewed_plan_and_keeps_the_fixed_duration(self):
        from rebot_official_motion import apply_slow_gains,policy
        raw,_=trial_proposal(self.store.state,self.api.limits)
        trusted=dict(id='diagnostic-token',expires=60.,config=raw,revision=self.store.state['revision'],
                     expected_start=[0.]*6,target=[.001]*6)
        self.api.point_plan=Mock(return_value=trusted)
        headers=dict(self.base_headers,**{'X-Motor-Session':self.token})
        body=dict(action='probe-startup',plan_id=trusted['id'],confirmed={k:True for k in CONFIRMATIONS},seconds=1000)
        with self.assertRaisesRegex(ValueError,'官方范围增益'):
            self.api.request('/hardware-point/action',headers,'127.0.0.1',body)
        self.popen.assert_not_called()
        apply_slow_gains(raw);raw['feedforward']=policy()
        # The real pinned-source check is covered by OfficialMotion. This unit
        # forbids every child process, including git version queries.
        with patch('rebot_official_motion.official_gain_limits',return_value=[(80,5),(75,6),(150,10),(50,5),(40,4),(40,4)]):
            self.api.request('/hardware-point/action',headers,'127.0.0.1',body)
        message=self.process.messages()[0]
        self.assertEqual(message['operation'],'probe-startup')
        self.assertEqual(message['seconds'],3)
        self.assertEqual(message['target'],trusted['target'])

    def test_native_csp_prepares_and_runs_only_fixed_j3_plan(self):
        import math
        headers=dict(self.base_headers,**{'X-Motor-Session':self.token})
        before=copy.deepcopy(self.store.state)
        result=self.api.request('/hardware-point/action',headers,'127.0.0.1',dict(action='prepare-csp-j3'))
        self.api._readonly_probe.assert_called_once_with(self.api.plan['config'],native=True)
        self.assertEqual(self.pose.call_count,102)
        self.assertFalse(self.popen.called)
        self.assertAlmostEqual(self.api.plan['target'][2],math.radians(.5))
        plan_id=result['plan_id']
        with self.assertRaises(ValueError): self.start(plan_id)
        self.api.request('/hardware-point/action',headers,'127.0.0.1',dict(action='csp-j3-trial',
            plan_id=plan_id,confirmed={k:True for k in CONFIRMATIONS},target=[99]*6,seconds=500))
        message=self.process.messages()[0]
        self.assertEqual(message['operation'],'csp-j3-trial')
        self.assertEqual(message['seconds'],5)
        self.assertEqual(message['target'],[0,0,math.radians(.5),0,0,0])
        self.assertEqual(self.store.state,before)
        with self.assertRaises(ValueError):
            self.api.request('/hardware-point/action',headers,'127.0.0.1',dict(action='csp-j3-trial',plan_id=plan_id,confirmed={k:True for k in CONFIRMATIONS}))

    def test_native_csp_model_collision_and_cancel_never_produce_a_plan(self):
        headers=dict(self.base_headers,**{'X-Motor-Session':self.token})
        self.pose.side_effect=ValueError('synthetic collision')
        result=self.api.request('/hardware-point/action',headers,'127.0.0.1',dict(action='prepare-csp-j3'))
        self.assertNotIn('plan_id',result)
        self.assertEqual(result['status']['state'],'FAULT')
        self.pose.side_effect=None
        probe=self.api._readonly_probe.return_value
        def cancel(*args,**kwargs): self.api.stop(); return probe
        self.api._readonly_probe.side_effect=cancel
        result=self.api.request('/hardware-point/action',headers,'127.0.0.1',dict(action='prepare-csp-j3'))
        self.assertNotIn('plan_id',result)
        self.popen.assert_not_called()

    def test_failed_trial_trace_is_saved_once_and_excluded_from_status_polling(self):
        self.start()
        trace=[dict(time=1.,commands=[dict(axis=3,values=[.01,0.,150.,10.,1.])])]
        terminal=dict(kind='result',state='FAULT',reason='J3 tracking boundary',frames=1,
                      command_log=trace,expected_start=[0.]*6,target=[.001]*6,
                      physical_acceptance=False)
        self.process.stdout=io.StringIO('REBOT_POWER_JSON '+json.dumps(terminal)+'\n')
        self.api._read_worker(self.process)
        status=self.api.status()
        self.assertFalse(status['busy'])
        self.assertEqual(status['reason'],'J3 tracking boundary')
        self.assertNotIn('command_log',status)
        self.assertNotIn('command_log',status['result'])
        files=list((self.root/'outputs/mit-p5/runtime').glob('*.json'))
        self.assertEqual(len(files),1)
        report=json.loads(files[0].read_text())
        self.assertEqual(report['command_log'],trace)
        self.assertEqual(report['expected_start'],[0.]*6)
        self.assertEqual(report['target'],[.001]*6)

    def test_xyz_worker_reuses_executor_with_six_axis_permit_and_never_j7(self):
        raw,_=trial_proposal(self.store.state,self.api.limits)
        raw['policy']['task_timeout']=83.
        message=dict(operation='move-xyz',config=raw,expected_start=[0.]*6,target=[.001]*6,
                     request_id='virtual-point-test',evidence='synthetic onsite confirmation')
        with patch('rebot_power_worker.HardwareExecutor') as factory, patch('rebot_power_worker.threading.Thread'), \
             patch('rebot_power_worker.signal.signal'),patch('rebot_power_worker.emit') as emit, \
             patch('rebot_power_worker.time.monotonic',return_value=10.):
            executor=factory.return_value
            executor.worker.is_alive.return_value=False
            executor.status.return_value={'state':'FAULT','frames':1}
            executor.frames=[dict(commands=[dict(axis=3,values=(.01,0.,150.,10.,1.))])]
            run_trial(message,bus=Mock())
            intent,permit=executor.start.call_args.args
            self.assertEqual(list(intent.expected_start),message['expected_start'])
            self.assertEqual([list(q) for q in intent.waypoints],[message['target']])
            self.assertEqual(permit.axes,(1,2,3,4,5,6))
            self.assertTrue(permit.coordinated_start_checked)
            self.assertTrue(all((a.kp,a.kd,a.tau_ff)==(3.,1.,0.) for a in factory.call_args.args[0].axes))
            self.assertEqual(permit.expires_at,103.)
            final=emit.call_args.args[0]
            self.assertEqual(final['kind'],'result')
            self.assertEqual(final['command_log'],executor.frames)
            self.assertEqual(final['expected_start'],message['expected_start'])
            self.assertEqual(final['target'],message['target'])
            self.assertIn('no per-frame drive ACK',final['command_evidence'])

    def test_each_confirmation_requires_literal_true(self):
        plan = self.prepare()
        for key in CONFIRMATIONS:
            for invalid in (None, False, 1, 'true'):
                with self.subTest(key=key, invalid=invalid):
                    confirmations = {k: True for k in CONFIRMATIONS}
                    confirmations[key] = invalid
                    with self.assertRaises(ValueError):
                        self.action('start', plan_id=plan, confirmed=confirmations)
        self.popen.assert_not_called()

    def test_plan_belongs_to_page_session(self):
        plan = self.prepare()
        with self.assertRaises(ValueError):
            self.action('start', token=self.session(), plan_id=plan,
                        confirmed={k: True for k in CONFIRMATIONS})
        self.popen.assert_not_called()

    def test_expired_plan_is_not_started(self):
        plan = self.prepare()
        self.clock.wait(60.)
        with self.assertRaises(ValueError):
            self.start(plan)
        self.popen.assert_not_called()

    def test_revision_change_requires_new_prepare(self):
        plan = self.prepare()
        self.store.state['revision'] += 1
        with self.assertRaises(ValueError):
            self.start(plan)
        self.assertIsNone(self.api.plan)
        self.popen.assert_not_called()

    def test_start_sends_exact_fixed_hold_and_consumes_plan_once(self):
        before = copy.deepcopy(self.store.state)
        plan = self.prepare()
        result = self.start(plan)
        self.assertEqual(result['status']['state'], 'PREPARING')
        self.assertTrue(result['status']['busy'])
        self.popen.assert_called_once()
        self.thread.return_value.start.assert_called_once()
        sent = self.process.messages()
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0]['operation'], 'hold-j1')
        self.assertEqual(sent[0]['seconds'], 3)
        self.assertEqual(sent[0]['expected_start'], [0.] * 6)
        self.assertNotIn('target', sent[0])
        self.assertIsNone(self.api.plan)
        with self.assertRaises(ValueError):
            self.start(plan)
        self.api.snapshot['busy'] = False
        with self.assertRaises(ValueError):
            self.start(plan)
        self.assertEqual(self.popen.call_count, 1)
        self.assertEqual(self.store.state, before)

    def test_stop_can_come_from_new_local_session_for_owned_trial(self):
        self.start()
        result = self.action('stop', token=self.session())
        self.assertEqual(result['status']['state'], 'STOPPING')
        self.assertTrue(self.api.stop_requested)
        self.assertEqual(self.process.messages()[-1], {'action': 'stop'})

    def test_idle_stop_never_spawns_or_disables_unknown_motors(self):
        result = self.action('stop')
        self.assertIn('未对未知状态设备', result['note'])
        self.popen.assert_not_called()
        self.assertEqual(self.process.messages(), [])

    def test_stop_during_readonly_prepare_cannot_restore_a_ready_plan(self):
        def cancelled_probe(_):
            self.action('stop', token=self.session())
            return dict(ok=True, expected_start=[0.] * 6, feedback={})
        self.api._readonly_probe.side_effect = cancelled_probe
        result = self.action('prepare')
        self.assertEqual(result['status']['state'], 'IDLE')
        self.assertFalse(result['status']['busy'])
        self.assertNotIn('plan_id', result)
        self.assertIsNone(self.api.plan)
        self.popen.assert_not_called()

    def test_heartbeat_owner_sequence_and_focus(self):
        self.start()
        for body in (dict(token=self.session(), sequence=1, focused=True),
                     dict(sequence=True, focused=True), dict(sequence=0, focused=True)):
            with self.assertRaises(ValueError):
                self.action('heartbeat', **body)
        self.action('heartbeat', sequence=1, focused=True)
        with self.assertRaises(ValueError):
            self.action('heartbeat', sequence=1, focused=True)
        self.action('heartbeat', sequence=2, focused=False)
        self.assertEqual(self.process.messages()[-1], {'action': 'heartbeat', 'sequence': 2, 'focused': False})

    def test_remote_cross_origin_and_unknown_session_are_rejected(self):
        for peer, change in [('192.0.2.1', {}), ('127.0.0.1', {'Host': 'attacker.example:8765'}),
                             ('127.0.0.1', {'Origin': 'https://attacker.example'}),
                             ('127.0.0.1', {'Origin': ''}),
                             ('127.0.0.1', {'Sec-Fetch-Site': 'cross-site'})]:
            with self.subTest(peer=peer, headers=change):
                with self.assertRaises(ValueError):
                    self.api.request('/motor-power/session', dict(self.base_headers, **change), peer, {})
        with self.assertRaises(ValueError):
            self.action('prepare', token='unknown')
        self.clock.wait(1800.)
        with self.assertRaises(ValueError):
            self.action('prepare')
        self.popen.assert_not_called()

    def test_arbitrary_control_operations_and_paths_are_rejected(self):
        for action in ('enable', 'disable', 'mit', 'ensure_mode', 'set_zero', 'write_param', 'hold-j2'):
            with self.subTest(action=action), self.assertRaises(ValueError):
                self.action(action)
        with self.assertRaises(ValueError):
            self.api.request('/motor-power/mit', dict(self.base_headers, **{'X-Motor-Session': self.token}),
                             '127.0.0.1', {})
        self.popen.assert_not_called()

    def test_probe_failure_collision_or_revision_change_produces_no_plan(self):
        scenarios = ('probe', 'collision', 'revision')
        for scenario in scenarios:
            with self.subTest(scenario=scenario):
                self.api._readonly_probe.side_effect = None
                self.api._readonly_probe.return_value['ok'] = True
                self.pose.side_effect = None
                if scenario == 'probe':
                    self.api._readonly_probe.return_value['ok'] = False
                elif scenario == 'collision':
                    self.pose.side_effect = ValueError('synthetic collision')
                else:
                    def changed(_):
                        self.store.state['revision'] += 1
                        return dict(ok=True, expected_start=[0.] * 6, feedback={})
                    self.api._readonly_probe.side_effect = changed
                result = self.action('prepare')
                self.assertEqual(result['status']['state'], 'FAULT')
                self.assertIsNone(self.api.plan)
        self.popen.assert_not_called()

    def test_existing_gain_conflict_is_never_replaced(self):
        self.store.state['drafts']['3']['fields']['kp']['value'] = 7
        before = copy.deepcopy(self.store.state)
        result = self.action('prepare')
        self.assertEqual(result['status']['state'], 'FAULT')
        self.api._readonly_probe.assert_not_called()
        self.assertEqual(self.store.state, before)


class HoldTrial(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.config = config()
        self.sdk = Controller(self.clock)
        self.bus = SdkTransport(factory=lambda **_: self.sdk, ownership=Owner(), clock=self.clock)
        self.executor = HardwareExecutor(self.config, self.bus, clock=self.clock, wait=self.wait)
        self.sequence = 0
        self.executor.heartbeat(self.executor.session, self.sequence, True)
        self.permit = OnsitePermit(self.config.fingerprint, (1,), 100., 'OFFLINE synthetic confirmation', (1,), True, True)
        self.intent = HoldIntent('OFFLINE-hold-test', [0.] * 6, 3)

    def wait(self, seconds):
        self.clock.wait(seconds)
        if self.executor.state != 'STOPPING':
            self.sequence += 1
            self.executor.heartbeat(self.executor.session, self.sequence, True)

    def commands(self, operation):
        return [c for c in self.sdk.calls if c[0] == operation]

    def test_only_j1_holds_new_post_enable_feedback_for_three_seconds_then_releases(self):
        original = Motor.enable
        def enable(motor):
            original(motor)
            motor.q = .0002
        disabled_at = []
        original_disable = Motor.disable
        def disable(motor):
            disabled_at.append(self.clock())
            original_disable(motor)
        with patch.object(Motor, 'enable', enable), patch.object(Motor, 'disable', disable):
            self.executor.run(self.intent, self.permit)
        self.assertEqual(self.executor.state, 'ARRIVED', self.executor.reason)
        self.assertEqual(self.commands('enable'), [('enable', 1)])
        self.assertEqual(self.commands('disable'), [('disable', 1)])
        frames = self.commands('mit')
        self.assertGreater(len(frames), 2)
        for frame in frames:
            self.assertEqual(frame[1], 1)
            self.assertAlmostEqual(frame[2], .0002)
            self.assertEqual(frame[3:], (0., 3, 1, 0))
        self.assertGreaterEqual(disabled_at[0], 3.)
        self.assertLess(disabled_at[0], 3. + 2 * self.config.policy.period)
        self.assertFalse(self.executor.status()['physical_acceptance'])
        self.assertFalse(self.executor.stop_result['physical_stopped'])

    def test_blur_after_first_frame_cancels_and_releases_without_later_targets(self):
        self.sdk.after_frame = lambda: self.executor.heartbeat(self.executor.session, 1, False)
        self.executor.run(self.intent, self.permit)
        self.assertEqual(self.executor.state, 'CANCELLED')
        self.assertEqual(len(self.commands('mit')), 1)
        self.assertEqual(self.commands('disable'), [('disable', 1)])
        self.assertLess(self.clock(), 3.)

    def test_nonsettling_hold_cuts_off_at_three_seconds_before_or_after_read(self):
        for crosses_during_read in (False, True):
            with self.subTest(crosses_during_read=crosses_during_read):
                self.setUp()
                def residual_error():
                    # Preserve the first post-enable check; introduce a small
                    # non-settling error only after the running loop starts.
                    if len(self.commands('mit')) >= 2:
                        self.sdk.motors[1].q = .005
                self.sdk.after_frame = residual_error
                if crosses_during_read:
                    def read_delay():
                        if self.clock() >= 2.98:
                            self.clock.wait(.015)
                    self.sdk.callback = read_delay
                sent_at = []
                disabled_at = []
                original = Motor.send_mit
                original_disable = Motor.disable
                def send(motor, *values):
                    sent_at.append(self.clock())
                    original(motor, *values)
                def disable(motor):
                    disabled_at.append(self.clock())
                    original_disable(motor)
                with patch.object(Motor, 'send_mit', send), patch.object(Motor, 'disable', disable):
                    self.executor.run(self.intent, self.permit)
                self.assertEqual(self.executor.state, 'FAULT')
                self.assertIn('3 秒保持已结束', self.executor.reason)
                self.assertEqual(self.commands('disable'), [('disable', 1)])
                self.assertGreaterEqual(self.clock(), 3.)
                self.assertLess(disabled_at[0], 3.1)
                self.assertTrue(all(t < 3. for t in sent_at))
                self.assertFalse(self.executor.motion_arrived)

    def test_hold_does_not_accept_other_axes_or_forged_motion_intent(self):
        self.permit.axes = (2,)
        self.permit.mechanically_supported_axes = (2,)
        self.executor.run(self.intent, self.permit)
        self.assertEqual(self.executor.state, 'FAULT')
        self.assertFalse(self.commands('enable'))
        self.setUp()
        forged = MotionIntent([[.1] + [0.] * 5], 'onsite-hold', 'forged', [0.] * 6)
        self.executor.run(forged, self.permit)
        self.assertEqual(self.executor.state, 'FAULT')
        self.assertFalse(self.commands('enable'))

    def test_hold_duration_is_fixed_and_prepared_start_still_matters(self):
        for seconds in (0, 1, 4, '3', None):
            with self.assertRaises(ValueError):
                HoldIntent('bad', [0.] * 6, seconds)
        moved = HoldIntent('moved', [.01] + [0.] * 5, 3)
        self.executor.run(moved, self.permit)
        self.assertEqual(self.executor.state, 'FAULT')
        self.assertFalse(self.commands('enable'))

    def test_worker_probe_uses_three_readonly_sweeps_and_no_power_commands(self):
        result = probe(self.config, self.bus)
        self.assertTrue(result['ok'], result['reason'])
        self.assertEqual(len(result['readings']), 3)
        self.assertEqual(result['hardware_writes'], 0)
        self.assertEqual(result['expected_start'], [0.] * 6)
        for operation in ('enable', 'mit', 'disable'):
            self.assertFalse(self.commands(operation))

    def test_worker_probe_retains_boundary_and_nonstationary_readings(self):
        def readings():
            self.sdk.motors[2].q = -1.1
            self.sdk.motors[6].qd = .02
        self.sdk.callback = readings
        result = probe(self.config, self.bus)
        self.assertFalse(result['ok'])
        self.assertEqual(result['feedback']['2']['q'], -1.1)
        self.assertIn('J2', result['reason'])
        self.assertIn('J6', result['reason'])
        self.assertFalse(self.commands('enable'))

    def test_sdk_version_is_checked_without_importing_real_driver(self):
        factory = Mock(return_value=self.sdk)
        fake_module = types.ModuleType('motorbridge')
        fake_module.Controller = factory
        with patch.dict(sys.modules, {'motorbridge': fake_module}):
            for version in ('0.5.5', '0.5.7', '1.0.0'):
                with self.subTest(version=version), patch('rebot_mit_hardware.importlib.metadata.version', return_value=version):
                    bus = SdkTransport(ownership=Owner(), clock=self.clock)
                    with self.assertRaisesRegex(RuntimeError, '0.5.6'):
                        bus.open_readonly()
                    self.assertIsNone(bus.controller)
            factory.assert_not_called()
            with patch('rebot_mit_hardware.importlib.metadata.version', return_value='0.5.6'):
                bus = SdkTransport(ownership=Owner(), clock=self.clock)
                bus.open_readonly()
                bus.close()
            factory.assert_called_once_with(channel='can0')
        self.assertFalse(self.commands('enable'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
