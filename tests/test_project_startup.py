"""Offline launcher checks: no real CAN and no project service lifecycle changes."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import start_project as launcher


class StartupTests(unittest.TestCase):
    def service(self, root):
        return launcher.Service('test', 8765, [sys.executable, 'test.py'],
            Path(root) / 'test.py', Path(root) / 'test.log', Path(root) / 'test.pid', .01)

    def test_existing_service_reused_without_spawn(self):
        service = self.service('/tmp')
        with patch.object(launcher, 'listener', return_value=123), patch.object(launcher, 'owns', return_value=True), patch.object(launcher, 'healthy'), patch.object(launcher.subprocess, 'Popen') as spawn:
            result = launcher.ensure_service(service)
        self.assertTrue(result['reused'])
        spawn.assert_not_called()

    def test_foreign_listener_never_spawned_or_stopped(self):
        with patch.object(launcher, 'listener', return_value=123), patch.object(launcher, 'owns', return_value=False), patch.object(launcher.subprocess, 'Popen') as spawn:
            with self.assertRaisesRegex(RuntimeError, '其他进程'):
                launcher.ensure_service(self.service('/tmp'))
        spawn.assert_not_called()

    def test_status_missing_service_has_no_writes(self):
        with tempfile.TemporaryDirectory() as root, patch.object(launcher, 'listener', return_value=None), patch.object(launcher.subprocess, 'Popen') as spawn:
            result = launcher.ensure_service(self.service(root), status_only=True)
            self.assertEqual(list(Path(root).iterdir()), [])
        self.assertEqual(result['status'], 'not-running')
        spawn.assert_not_called()

    def test_detached_start_records_verified_pid(self):
        with tempfile.TemporaryDirectory() as root:
            service = self.service(root)
            service.identity.touch()
            child = Mock(pid=55)
            child.poll.return_value = None
            with patch.object(launcher, 'listener', side_effect=[None, 56]), patch.object(launcher, 'owns', return_value=True), patch.object(launcher, 'healthy'), patch.object(launcher.subprocess, 'Popen', return_value=child) as spawn:
                result = launcher.ensure_service(service, Path(root))
            self.assertFalse(result['reused'])
            self.assertEqual(service.pidfile.read_text(), '56\n')
            self.assertTrue(spawn.call_args.kwargs['start_new_session'])
            self.assertEqual(spawn.call_args.kwargs['stdin'], launcher.subprocess.DEVNULL)

    def test_failed_start_is_not_retried(self):
        with tempfile.TemporaryDirectory() as root:
            service = self.service(root)
            service.identity.touch()
            child = Mock(pid=55)
            child.poll.return_value = 2
            with patch.object(launcher, 'listener', return_value=None), patch.object(launcher.subprocess, 'Popen', return_value=child) as spawn:
                with self.assertRaisesRegex(RuntimeError, '退出'):
                    launcher.ensure_service(service, Path(root))
            self.assertEqual(spawn.call_count, 1)

    def test_can_readiness_checks_operating_modes(self):
        for flags, state, bitrate, modes, expected in [
            (['UP'], 'ERROR-ACTIVE', 1000000, [], True),
            ([], 'STOPPED', 1000000, [], False),
            (['UP'], 'BUS-OFF', 1000000, [], False),
            (['UP'], 'ERROR-ACTIVE', 500000, [], False),
            (['UP'], 'ERROR-ACTIVE', 1000000, ['LISTEN-ONLY'], False),
            (['UP'], 'ERROR-ACTIVE', 1000000, ['LOOPBACK'], False),
        ]:
            with self.subTest(state=state, modes=modes):
                data = [{'flags': flags, 'linkinfo': {'info_data': {'state': state, 'bittiming': {'bitrate': bitrate}, 'ctrlmode': modes}}}]
                with patch.object(launcher.subprocess, 'run', return_value=Mock(returncode=0, stdout=json.dumps(data))):
                    self.assertEqual(launcher.can_state()['ready'], expected)

    def test_can_already_up_is_not_reconfigured(self):
        with patch.object(launcher, 'can_state', return_value={'ready': False, 'up': True}), patch.object(launcher.subprocess, 'run') as run:
            with self.assertRaisesRegex(RuntimeError, '未中断'):
                launcher.prepare_can()
        run.assert_not_called()

    def test_can_down_explicit_configuration_does_not_force_down(self):
        with patch.object(launcher, 'can_state', return_value={'ready': False, 'up': False}), patch.object(launcher.os, 'geteuid', return_value=1000), patch.object(launcher.subprocess, 'run') as run:
            launcher.prepare_can()
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(len(commands), 2)
        self.assertEqual(commands[1], ['sudo', 'ip', 'link', 'set', 'can0', 'up'])
        self.assertTrue(all('down' not in command for command in commands))

    def test_stalled_simulation_rejected(self):
        state = {'backend': 'mujoco', 'error': None, 'stream_id': 'a', 'sequence': 10}
        with patch.object(launcher, 'get_state', return_value=state), patch.object(launcher.time, 'sleep'):
            with self.assertRaisesRegex(RuntimeError, '未持续更新'):
                launcher.healthy(self.service('/tmp'))

    def test_ros_real_or_stale_backend_rejected(self):
        service = self.service('/tmp')
        service.port = 8767
        state = {'backend': 'ros2-fake', 'namespace': 'rebotarm_validation', 'hardware': False, 'domain': 43, 'fresh': True}
        with patch.object(launcher, 'get_state', return_value=state):
            launcher.healthy(service)
        for change in [{'hardware': True}, {'fresh': False}, {'domain': 42}, {'namespace': 'real'}]:
            with patch.object(launcher, 'get_state', return_value=dict(state, **change)):
                with self.assertRaises(RuntimeError):
                    launcher.healthy(service)

    def test_status_main_never_prepares_can_spawns_or_writes(self):
        with tempfile.TemporaryDirectory() as root, patch.object(launcher, 'ROOT', Path(root)), patch.object(launcher.shutil, 'which', return_value='/bin/tool'), patch.object(launcher, 'ensure_service', return_value={'status': 'not-running', 'pid': None}), patch.object(launcher, 'can_state', return_value={'ready': False, 'message': 'missing'}), patch.object(launcher, 'prepare_can') as prepare, patch.object(launcher.subprocess, 'Popen') as spawn, contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(launcher.main(['--status']), 1)
            self.assertEqual(list(Path(root).iterdir()), [])
        prepare.assert_not_called()
        spawn.assert_not_called()

    def test_sim_only_never_includes_gateway_or_ros(self):
        result = launcher.services(sim_only=True)
        self.assertEqual([service.port for service in result], [8765])


if __name__ == '__main__':
    unittest.main()
