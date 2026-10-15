"""Real planner and production hardware-plan resolver, without serve() or CAN.

The resolver is nested inside serve(). Compile its original AST, together with
hardware_configuration(), to exercise that code without starting the HTTP server,
render thread, gateway, or motor worker. This covers the server-side resolution
boundary; the HTTP transport and actual motor execution are outside this test.
"""
import ast
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from rebot_config import ConfigStore
from rebot_core import Robot
from rebot_mit_hardware import Envelope, OnsitePermit
from rebot_native_xyz import AXES, MODE, PATH_MODE, proposal
from rebot_point import PointPlanner
from rebot_xyz_trial import assess_motion, configure_motion
from test_mit_hardware_offline import Clock


class NativeXYZPlanResolverOffline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = Robot()
        source = ROOT / 'scripts/rebot_sim.py'
        tree = ast.parse(source.read_text(), filename=str(source))
        serve = next(node for node in tree.body
                     if isinstance(node, ast.FunctionDef) and node.name == 'serve')
        names = ('hardware_configuration', 'resolve_hardware_point')
        functions = [node for node in serve.body
                     if isinstance(node, ast.FunctionDef) and node.name in names]
        if tuple(node.name for node in functions) != names:
            raise AssertionError('Production resolver structure changed; review the test seam')
        # No copied check logic and no AST rewrites: run the production bodies.
        cls.resolver_code = compile(ast.Module(body=functions, type_ignores=[]),
                                    str(source), 'exec')
        record = json.loads((ROOT / 'docs/validation/official-mit-2026-10-15/'
                             'native-xyz-web-preflight.json').read_text())
        cls.start = np.deg2rad(record['preview']['start_deg'])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        # Only private, temporary configuration state is used by this fixture.
        self.store = ConfigStore(Path(self.temp.name) / 'private_config')
        self.clock = Clock()
        self.planner = PointPlanner(self.robot, clock=self.clock,
                                    configuration=self.store.fingerprint)
        scope = dict(ROOT=ROOT, json=json, r=self.robot, config_store=self.store,
                     point_planner=self.planner, lock=threading.RLock())
        exec(self.resolver_code, scope)
        self.resolve = scope['resolve_hardware_point']
        self.hardware_configuration = scope['hardware_configuration']
        for target in ('motorbridge.Controller',
                       'rebot_seeed_transport.SeeedTransport.open_readonly',
                       'subprocess.Popen', 'socket.socket'):
            guard = patch(target, side_effect=AssertionError('No CAN, process, or network allowed'))
            guard.start()
            self.addCleanup(guard.stop)
        base_raw, base = self.hardware_configuration(native=True)
        goal = self.robot.fk(self.start).translation + np.array([0., 0., .005])
        result = self.planner.preview(dict(execution_source='hardware-preview',
            xyz_mm=(goal * 1000).tolist(), orientation_mode='hold', speed_rad_s=.02),
            start=self.start)
        self.plan_id = result['plan_id']
        self.plan = self.planner.plans[self.plan_id]
        raw, config = configure_motion(base_raw, self.plan['start'], self.plan['target'],
            dict(speed_rad_s=.02, timeout_mode='auto', feedforward_mode=PATH_MODE))
        assessment = assess_motion(self.robot, self.plan['start'], self.plan['target'], config)
        self.assertTrue(assessment['execution_ready'], assessment)
        self.assertGreater(assessment['max_axis_delta_deg'], .5)
        self.plan.update(hardware_config=raw, hardware_config_hash=config.fingerprint,
                         hardware_base_hash=base.fingerprint,
                         config_revision=self.store.status()['state']['revision'])

    def test_extended_plan_resolves_once_with_real_model_and_preserves_saved_state(self):
        before = copy.deepcopy(self.store.status()['state'])
        result = self.resolve(self.plan_id)
        config = Envelope.parse(result['config'])
        self.assertEqual(config.version, PATH_MODE)
        self.assertEqual(config.policy.task_timeout, 30.)
        self.assertEqual(config.fingerprint, self.plan['hardware_config_hash'])
        np.testing.assert_array_equal(result['expected_start'], self.start)
        np.testing.assert_array_equal(result['target'], self.plan['target'])
        self.assertNotIn(self.plan_id, self.planner.plans)
        with self.assertRaisesRegex(ValueError, '已消费'):
            self.resolve(self.plan_id)
        self.assertEqual(self.store.status()['state'], before)

    def test_changed_base_fingerprint_rejects_before_consuming_plan(self):
        self.plan['hardware_base_hash'] = '0' * 64
        with self.assertRaisesRegex(ValueError, '实机试验参数已变化'):
            self.resolve(self.plan_id)
        self.assertIn(self.plan_id, self.planner.plans)

    def test_changed_task_fingerprint_rejects_before_consuming_plan(self):
        self.plan['hardware_config']['policy']['task_timeout'] = 29.
        with self.assertRaisesRegex(ValueError, '本次任务设置已改变'):
            self.resolve(self.plan_id)
        self.assertIn(self.plan_id, self.planner.plans)

    def test_expired_plan_rejects_at_exact_deadline_without_consuming_it(self):
        self.clock.t = self.plan['expires']
        with self.assertRaisesRegex(ValueError, '过期'):
            self.resolve(self.plan_id)
        self.assertIn(self.plan_id, self.planner.plans)

    def test_saved_revision_change_and_wrong_source_each_reject_before_consumption(self):
        self.plan['config_revision'] += 1
        with self.assertRaisesRegex(ValueError, '软件草稿已改变'):
            self.resolve(self.plan_id)
        self.plan['config_revision'] -= 1
        self.plan['source'] = 'ros2-fake'
        with self.assertRaisesRegex(ValueError, '预演不存在'):
            self.resolve(self.plan_id)
        self.assertIn(self.plan_id, self.planner.plans)

    def test_old_profile_hash_and_permit_cannot_authorize_extended_plan(self):
        limits = list(zip(self.robot.lower, self.robot.upper))
        old_config = Envelope.parse(proposal(limits, mode=MODE)[0])
        self.plan['hardware_config_hash'] = old_config.fingerprint
        with self.assertRaisesRegex(ValueError, '本次任务设置已改变'):
            self.resolve(self.plan_id)
        config = Envelope.parse(self.plan['hardware_config'])
        self.plan['hardware_config_hash'] = config.fingerprint
        resolved = self.resolve(self.plan_id)
        permit = OnsitePermit(old_config.fingerprint, AXES, self.clock() + 60.,
            'synthetic old-profile permit', AXES, True, True,
            coordinated_start_checked=True)
        with self.assertRaisesRegex(PermissionError, '配置已变化'):
            permit.validate(Envelope.parse(resolved['config']), self.clock())
        self.assertFalse(permit.used)


if __name__ == '__main__':
    unittest.main()
