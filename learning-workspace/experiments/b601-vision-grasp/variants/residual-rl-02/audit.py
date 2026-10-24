"""Independent final score recomputation, paired resets, splits and actor outputs."""
import collections
import argparse
import json
from pathlib import Path
from curriculum import HERE, PREVIOUS, base, write, V3
from runtime import digest, verify_v3
from task import GOALS
import numpy as np
import torch
from infer_numpy import predict as numpy_predict


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--suite', choices=['broad', 'boundary'], default='broad')
    args = parser.parse_args()
    boundary = args.suite == 'boundary'
    prefix, seed_start, cases = ('boundary-r2', 54000000, 30) if boundary else ('formal-r2', 53000000, 36)
    kinds = ['bias_positive', 'bias_negative'] if boundary else ['nominal', 'force', 'bias_positive', 'bias_negative']
    verify_v3()
    checkpoint = digest(HERE / 'delivery/policy.zip')
    actor = torch.jit.load(str(HERE / 'delivery/actor.torchscript')).eval()
    with np.load(HERE / 'delivery/numpy_actor.npz') as archive:
        numpy_weights = {name: archive[name] for name in archive.files}
    paired, records = collections.defaultdict(list), []
    score_rows, actor_checks, numpy_max_error = 0, 0, 0.
    roots = [HERE / f'outputs/{prefix}-base-b{i}' for i in range(3)] + [HERE / f'outputs/{prefix}-sac-b{i}' for i in range(3)]
    for root in roots:
        config = json.loads((root / 'config.json').read_text())
        if 'sac' in config['controllers']:
            assert config['checkpoint_sha256'] == checkpoint
        for path in sorted(root.glob('episode-*/result.json')):
            controller = path.parent.name.rsplit('-', 1)[1]
            result = json.loads(path.read_text())
            scene = result.get('scene')
            seed = result['reset']['seed']
            goal = scene['goal'] if scene else result['task']['destination']
            paired[seed].append(result['reset'])
            successes = []
            for line in (path.parent / 'final_score_trace.jsonl').read_text().splitlines():
                row = json.loads(line)
                extent = np.abs(np.asarray(row['object_rotation_base'])) @ np.asarray(row['object_half_size_m'])
                inside = np.all(np.abs(np.asarray(row['object_xyz_m'])[:2] - GOALS[goal]) + extent[:2] <= .06)
                support = abs(row['object_xyz_m'][2] - extent[2] - .1) < .006
                slow = np.linalg.norm(row['object_linear_velocity_m_s']) < .02
                released = not any(row['finger_contacts'].values())
                success = bool(inside and support and slow and released)
                assert success == row['success']
                successes.append(success)
                score_rows += 1
            assert len(successes) == 500
            physical = all(successes) and result['physics_tick_audit']['forbidden_steps'] == 0
            assert physical == result['physical_success']
            assert bool(physical and result['controller_success']) == result['success']
            longest, held_time = 0., 0.
            for line in (path.parent / 'trajectory.jsonl').read_text().splitlines():
                row = json.loads(line)['truth_for_scoring_only']
                held = row['contact']['left'] and row['contact']['right'] and row['object_xyz_m'][2] > result['reset']['initial_xyz'][2] + .03
                held_time = held_time + .05 if held else 0.
                longest = max(longest, held_time)
            assert not result['success'] or longest >= .4
            if controller == 'sac':
                for attempt in result['attempts']:
                    for step in attempt.get('skill_actions', []):
                        observation = np.asarray(step['observation'], dtype='float32')
                        assert observation.shape == (39,) and np.isfinite(observation).all()
                        with torch.inference_mode():
                            action = actor(torch.from_numpy(observation[None]))[0].numpy()
                        assert np.max(np.abs(action - step['action'])) < 1e-6
                        numpy_error = float(np.abs(numpy_predict(observation, numpy_weights) - action).max())
                        numpy_max_error = max(numpy_max_error, numpy_error)
                        assert numpy_error < 1e-6
                        previous = observation[32:36] * base.BOUNDS
                        delta = previous + np.clip(np.asarray(step['action']) * base.BOUNDS - previous,
                                                   [-.002] * 3 + [-.0015], [.002] * 3 + [.0015])
                        assert np.max(np.abs(delta - step['applied_residual_m'])) < 1e-8
                        actor_checks += 1
            records.append({'controller': controller, 'seed': seed, 'goal': goal,
                'kind': kinds[(seed - seed_start) % len(kinds)],
                'success': result['success'], 'retries': result['retries'],
                'first_outcome': result['attempts'][0].get('outcome'),
                'false_success': result['false_success'], 'forbidden_steps': result['physics_tick_audit']['forbidden_steps'],
                'sim_seconds': result['sim_seconds'], 'bilateral_lift_seconds_20hz': longest, 'result_sha256': digest(path)})
    assert len(records) == cases * 4 and set(paired) == set(range(seed_start, seed_start + cases))
    assert all(len(group) == 4 and all(reset == group[0] for reset in group) for group in paired.values())
    training = json.loads((HERE / 'outputs/long-r2-hour/training.json').read_text())
    config = json.loads((HERE / 'outputs/long-r2-hour/config.json').read_text())
    train, validation, test = map(set, [config['training_seeds'], config['validation_seeds'], paired])
    assert train.isdisjoint(validation) and train.isdisjoint(test) and validation.isdisjoint(test)
    actual_train = {json.loads(p.read_text())['reset']['seed'] for p in (HERE / 'outputs/long-r2-hour').glob('env-*/episode-*/reset.json')}
    assert actual_train <= train
    complete = sum(json.loads(p.read_text())['decisions'] for p in (HERE / 'outputs/long-r2-hour').glob('env-*/episode-*/result.json'))
    partial = sum(json.loads(p.read_text())['decisions'] for p in (HERE / 'outputs/long-r2-hour').glob('env-*/episode-*/partial_result.json'))
    assert complete + partial == training['timesteps']
    summary = {}
    for controller in ['v3', 'zero', 'previous', 'sac']:
        rows = [r for r in records if r['controller'] == controller]
        assert len(rows) == cases and collections.Counter(r['goal'] for r in rows) == dict.fromkeys('ABC', cases // 3)
        assert all(n == cases // len(kinds) for n in collections.Counter(r['kind'] for r in rows).values())
        summary[controller] = {'successes': sum(r['success'] for r in rows), 'trials': cases,
            'first_successes': sum(r['success'] and r['retries'] == 0 for r in rows),
            'recovered_successes': sum(r['success'] and r['retries'] > 0 for r in rows),
            'empty_grasp_then_recovered': sum(r['first_outcome'] == 'empty_grasp_or_slip' and r['success'] for r in rows),
            'false_successes': sum(r['false_success'] for r in rows), 'forbidden_steps': sum(r['forbidden_steps'] for r in rows),
            'mean_sim_seconds': float(np.mean([r['sim_seconds'] for r in rows])),
            'by_kind': {kind: {'successes': sum(r['success'] for r in rows if r['kind'] == kind),
                              'first_successes': sum(r['success'] and r['retries'] == 0 for r in rows if r['kind'] == kind)}
                        for kind in kinds}}
    summary_name, audit_name = ('BOUNDARY_SUMMARY.json', 'BOUNDARY_AUDIT.json') if boundary else ('FORMAL_SUMMARY.json', 'AUDIT.json')
    write(HERE / 'delivery' / summary_name, {'checkpoint_sha256': checkpoint, 'summary': summary, 'scenes': records})
    write(HERE / 'delivery' / audit_name, {'passed': True, 'score_rows_recomputed': score_rows,
        'actor_proposals_checked': actor_checks, 'paired_resets_equal': True, 'source_sha256': digest(Path(__file__)),
        'numpy_proposals_checked': actor_checks, 'numpy_max_abs_error': numpy_max_error,
        'complete_episode_decisions': complete, 'partial_episode_decisions': partial,
        'train_validation_test_seeds_disjoint': True, 'actual_distinct_training_seeds': len(actual_train)})
    print(json.dumps({'summary': summary, 'score_rows_recomputed': score_rows, 'actor_proposals_checked': actor_checks}, indent=2))
