"""Offline width and frozen grasp-profile tests; no NumPy, ROS or SDK required."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'rebotarm_ros2_RS/src/rebotarmcontroller'))
from qualification.common import number, read_json
from qualification.j7_width import analyze as width_review, query
from qualification.grasp_review import analyze as grasp_review

FIXTURES = Path(__file__).parent / 'fixtures/qualification'


class WidthTests(unittest.TestCase):
    def setUp(self):self.data = read_json(FIXTURES / 'j7-width.synthetic.json')

    def test_independent_validation_and_hysteresis_inverse_queries(self):
        report = width_review(self.data)
        self.assertTrue(report['validation_passed']);self.assertFalse(report['calibration_ready'])
        closing = query(report, 'closing', position_rad=2.)['width_mm']
        opening = query(report, 'opening', position_rad=2.)['width_mm']
        self.assertAlmostEqual(opening - closing, .4)
        self.assertAlmostEqual(query(report, 'closing', width_mm=closing)['position_rad'], 2.)
        self.assertFalse(report['hardware_execution_allowed'])

    def test_nonmonotonic_duplicate_reused_validation_and_extrapolation_are_rejected(self):
        bad = deepcopy(self.data);bad['measurements']['opening'][1]['width_mm'] = 0
        with self.assertRaises(ValueError):width_review(bad)
        bad = deepcopy(self.data);bad['measurements']['closing'][1]['position_rad'] = .2
        with self.assertRaises(ValueError):width_review(bad)
        bad = deepcopy(self.data);bad['validation']['opening'][0]['position_rad'] = .2
        with self.assertRaises(ValueError):width_review(bad)
        for value in [-1., 1000.]:
            with self.assertRaisesRegex(ValueError, 'extrapolation'):query(width_review(self.data), 'opening', width_mm=value)

    def test_failed_heldout_error_blocks_queries(self):
        self.data['validation']['opening'][0]['width_mm'] += 2
        report = width_review(self.data);self.assertFalse(report['validation_passed'])
        with self.assertRaises(ValueError):query(report, 'opening', position_rad=2.)

    def test_json_and_numbers_fail_cleanly_for_duplicates_bool_nan_and_huge_integer(self):
        for value in [True, float('nan'), float('inf'), 10**1000]:
            with self.assertRaises(ValueError):number(value, 'test')
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory) / 'bad.json'
            for text in ['{"q":1,"q":2}', '{"q":NaN}']:
                p.write_text(text)
                with self.assertRaises(ValueError):read_json(p)

    def test_cli_does_not_overwrite_input_or_report_and_requires_explicit_direction(self):
        cli = Path(__file__).with_name('qualify.py')
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'review.json'
            command = [sys.executable, str(cli), 'j7-width', str(FIXTURES / 'j7-width.synthetic.json'), '--output', str(output)]
            subprocess.run(command, check=True, capture_output=True)
            before = output.read_bytes()
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)
            self.assertEqual(output.read_bytes(), before)
            self.assertNotEqual(subprocess.run(command[:4] + ['--width-mm', '20'], capture_output=True).returncode, 0)


class GraspTests(unittest.TestCase):
    def setUp(self):self.data = read_json(FIXTURES / 'grasp.synthetic.json')

    def test_fixture_false_acceptance_and_slip_are_reported_separately_from_hardware(self):
        result = grasp_review(self.data)
        by_id = {r['id']:r for r in result['trials']}
        self.assertEqual(by_id['object']['final_status'], 'holding')
        self.assertTrue(by_id['fixture']['false_object_acceptance'])
        self.assertFalse(by_id['empty']['false_object_acceptance'])
        self.assertTrue(by_id['slip']['slip_detected'])
        self.assertLessEqual(by_id['slip']['slip_detection_delay_s'], .31)
        self.assertEqual(result['hardware_metrics']['trials'], 0)
        self.assertEqual(result['nonhardware_metrics']['false_object_acceptances'], 1)
        self.assertFalse(result['object_verified'])

    def test_duplicate_gap_missing_reference_and_annotation_are_rejected(self):
        mutations = [lambda d:d['recording']['trials'][0]['samples'][1].update(sample_at=100),
                     lambda d:d['recording']['trials'][0]['samples'][1].update(sample_at=101),
                     lambda d:d['recording']['trials'][0]['samples'][1].update(reference=None),
                     lambda d:d['truth'].pop('fixture')]
        for change in mutations:
            bad = deepcopy(self.data);change(bad)
            with self.assertRaises((ValueError, KeyError)):grasp_review(bad)


class NetworkTests(unittest.TestCase):
    def test_ipv4_ipv6_wildcard_and_missing_listeners(self):
        spec = importlib.util.spec_from_file_location('audit_network', ROOT / 'scripts/audit_local_network.py')
        audit = importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
        report = audit.analyze('LISTEN 0 511 127.0.0.1:3002 0.0.0.0:*\nLISTEN 0 100 [::1]:9090 [::]:*')
        self.assertTrue(report['observed_listeners_loopback_only']);self.assertEqual(report['actions_performed'], [])
        for host in ['0.0.0.0', '[::]', '*', '192.168.1.2']:
            self.assertEqual(len(audit.analyze(f'LISTEN 0 511 {host}:9090 *:*')['exposed']), 1)
        self.assertFalse(audit.analyze('')['observed_listeners_loopback_only'])


if __name__ == '__main__':unittest.main()
