"""Offline request boundaries and required task verification, using stdlib only."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import unittest

from fixed_pick import build_proposal, PROPOSAL_SCHEMA

ROOT = Path(__file__).resolve().parents[2]
EXAMPLE = json.loads(Path(__file__).with_name('fixed-pick.example.json').read_text())


class FixedPickTest(unittest.TestCase):
    def test_required_barriers_precede_lifting_and_retreat(self):
        result = build_proposal(EXAMPLE)
        names = [s['name'] for s in result['stages']]
        self.assertLess(names.index('close_at_pick'), names.index('confirm_grasp_and_attach_payload'))
        self.assertLess(names.index('confirm_grasp_and_attach_payload'), names.index('lift_from_pick'))
        self.assertLess(names.index('confirm_release_and_detach_payload'), names.index('retreat_from_place'))
        self.assertEqual(result['stages'][0]['type'], 'verification_required')
        self.assertFalse(result['hardware_execution_allowed'])

    def test_geometry_preserves_input_and_spacing(self):
        request = copy.deepcopy(EXAMPLE)
        result = build_proposal(request)
        self.assertEqual(request, EXAMPLE)
        for stage in result['stages']:
            if stage['type'] != 'move_candidate':
                continue
            points = stage['tcp_waypoints']
            for point in points:
                self.assertEqual(point[3:], request['pick_pose'][3:])
                self.assertTrue(all(lo <= x <= hi for lo, x, hi in
                                    zip(request['workspace']['min'], point[:3], request['workspace']['max'])))
            for first, second in zip(points, points[1:]):
                self.assertLessEqual(math.dist(first[:3], second[:3]), request['sample_step_m']+1e-12)

    def test_pose_change_needs_separate_path_planning(self):
        request = copy.deepcopy(EXAMPLE)
        request['place_pose'][5] += .1
        result = build_proposal(request)
        stage = next(s for s in result['stages'] if s['name'] == 'reorient_above_place')
        self.assertEqual(stage['path_kind'], 'endpoint_requires_path_planning')
        self.assertEqual(stage['tcp_waypoints'][-1][3:], request['place_pose'][3:])

    def test_existing_executor_rejects_proposal_schema(self):
        path = ROOT/'rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/twin_program.py'
        spec = importlib.util.spec_from_file_location('existing_twin_program', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        proposal = build_proposal(EXAMPLE)
        self.assertEqual(proposal['schema_version'], PROPOSAL_SCHEMA)
        with self.assertRaisesRegex(ValueError, 'schema_version'):
            module.validate_program(proposal)

    def test_rejects_invalid_or_ambiguous_requests(self):
        mutations = [
            lambda r: r.update(base_frame='camera_optical_frame'),
            lambda r: r.update(tcp_frame='torch_tip'),
            lambda r: r['units'].update(position='mm'),
            lambda r: r.update(calibration_id=''),
            lambda r: r['pick_pose'].__setitem__(0, float('nan')),
            lambda r: r['pick_pose'].__setitem__(0, 10**400),
            lambda r: r['pick_pose'].__setitem__(3, True),
            lambda r: r['pick_pose'].pop(),
            lambda r: r.update(sample_step_m=0),
            lambda r: r.update(approach_m=-.01),
            lambda r: r.update(clearance_z_m=.1),
            lambda r: r['workspace']['min'].__setitem__(0, .5),
            lambda r: r['workspace']['max'].__setitem__(2, .315),
            lambda r: r['place_pose'].__setitem__(0, .8),
            lambda r: r['gripper'].update(close_rad=5),
            lambda r: r['gripper'].update(speed_rad_s=float('inf')),
            lambda r: r.update(joint_speed_rad_s=True),
            lambda r: r.update(hardware_execution_allowed=True),
            lambda r: r['workspace'].update(ignore_collision=True),
        ]
        for mutate in mutations:
            with self.subTest(mutation=mutations.index(mutate)):
                request = copy.deepcopy(EXAMPLE)
                mutate(request)
                with self.assertRaises(ValueError):
                    build_proposal(request)

    def test_sample_budget_is_bounded_before_large_allocation(self):
        request = copy.deepcopy(EXAMPLE)
        request['workspace']['max'][2] = 1
        request['clearance_z_m'] = .9
        request['sample_step_m'] = .001
        with self.assertRaisesRegex(ValueError, '240'):
            build_proposal(request)


if __name__ == '__main__':
    unittest.main()
