"""Recompute task outcomes, pair conditions and actor proposals from saved evidence."""
import collections
import json
from pathlib import Path

from environment import HERE, BOUNDS, GOALS
from runtime import write, digest, verify_v3
import numpy as np
import torch


if __name__ == '__main__':
    verify_v3()
    checkpoint = digest(HERE / 'delivery/policy.zip')
    actor = torch.jit.load(str(HERE / 'delivery/actor.torchscript')).eval()
    records, scoring_rows, actor_checks = [], 0, 0
    paired = collections.defaultdict(list)
    for batch in range(3):
        root = HERE / f'outputs/formal-r1-{batch}'
        config = json.loads((root / 'config.json').read_text())
        assert config['checkpoint_sha256'] == checkpoint
        for controller in ['v3', 'zero', 'sac']:
            for path in sorted(root.glob(f'episode-*-{controller}/result.json')):
                result = json.loads(path.read_text())
                scene = result.get('scene')
                seed = result['reset']['seed']
                goal = scene['goal'] if scene else result['task']['destination']
                kind = ['static', 'force', 'offset'][(seed - 49000000) % 3]
                paired[seed].append(result['reset'])
                final = []
                for line in (path.parent / 'final_score_trace.jsonl').read_text().splitlines():
                    r = json.loads(line)
                    extent = np.abs(np.asarray(r['object_rotation_base'])) @ np.asarray(r['object_half_size_m'])
                    inside = bool(np.all(np.abs(np.asarray(r['object_xyz_m'])[:2] - GOALS[goal]) + extent[:2] <= .06))
                    support = abs(r['object_xyz_m'][2] - extent[2] - .1) < .006
                    slow = np.linalg.norm(r['object_linear_velocity_m_s']) < .02
                    released = not any(r['finger_contacts'].values())
                    success = bool(inside and support and slow and released)
                    assert success == r['success']
                    final.append(success)
                    scoring_rows += 1
                assert len(final) == 500
                physical = all(final) and result['physics_tick_audit']['forbidden_steps'] == 0
                assert physical == result['physical_success']
                assert bool(physical and result['controller_success']) == result['success']
                trajectories = [json.loads(line) for line in (path.parent / 'trajectory.jsonl').read_text().splitlines()]
                initial_z = result['reset']['initial_xyz'][2]
                held_time, longest = 0., 0.
                for row in trajectories:
                    truth = row['truth_for_scoring_only']
                    held = truth['contact']['left'] and truth['contact']['right'] and truth['object_xyz_m'][2] > initial_z + .03
                    held_time = held_time + .05 if held else 0.
                    longest = max(longest, held_time)
                assert not result['success'] or longest >= .1
                if controller == 'sac':
                    for attempt in result['attempts']:
                        for step in attempt['skill_actions']:
                            observation = np.asarray(step['observation'], dtype=np.float32)
                            assert observation.shape == (36,) and np.isfinite(observation).all()
                            with torch.inference_mode():
                                action = actor(torch.from_numpy(observation[None]))[0].numpy()
                            assert np.max(np.abs(action - step['action'])) < 1e-6
                            previous = observation[-4:] * BOUNDS
                            delta = previous + np.clip(np.asarray(step['action']) * BOUNDS - previous,
                                                       [-.002] * 3 + [-.0015], [.002] * 3 + [.0015])
                            assert np.max(np.abs(delta - step['applied_residual_m'])) < 1e-8
                            actor_checks += 1
                records.append({'controller': controller, 'seed': seed, 'goal': goal, 'kind': kind,
                    'success': result['success'], 'retries': result['retries'],
                    'first_outcome': result['attempts'][0]['outcome'],
                    'false_success': result['false_success'], 'forbidden_steps': result['physics_tick_audit']['forbidden_steps'],
                    'sim_seconds': result['sim_seconds'], 'bilateral_lift_seconds_20hz': longest,
                    'result_sha256': digest(path)})
    assert len(records) == 90 and set(paired) == set(range(49000000, 49000030))
    assert all(len(group) == 3 and group[0] == group[1] == group[2] for group in paired.values())
    training = json.loads((HERE / 'outputs/sac-r1/training.json').read_text())
    train_seeds, validation_seeds, test_seeds = map(set, [training['training_seeds'], training['validation_seeds'], paired])
    assert train_seeds.isdisjoint(validation_seeds) and train_seeds.isdisjoint(test_seeds) and validation_seeds.isdisjoint(test_seeds)
    actual_train = {json.loads(p.read_text())['reset']['seed'] for p in (HERE / 'outputs/sac-r1').glob('env-*/episode-*/reset.json')}
    actual_validation = {json.loads(p.read_text())['reset']['seed'] for p in (HERE / 'outputs/sac-r1/validation').glob('episode-*/reset.json')}
    assert actual_train == train_seeds and actual_validation == validation_seeds
    original = json.loads((HERE / 'outputs/sac-r1/config.json').read_text())
    assert original['source_sha256']['environment.py'] == digest(HERE / 'outputs/sac-r1/source/environment.py')
    for batch in range(3):
        formal = json.loads((HERE / f'outputs/formal-r1-{batch}/config.json').read_text())
        assert formal['source_sha256']['environment.py'] == original['source_sha256']['environment.py']
    completed_decisions = sum(json.loads(p.read_text())['decisions'] for p in (HERE / 'outputs/sac-r1').glob('env-*/episode-*/result.json'))
    partial_decisions = sum(len(json.loads(p.read_text())['steps']) for p in (HERE / 'outputs/sac-r1').glob('env-*/episode-*/partial_replay_recovered.json'))
    assert completed_decisions + partial_decisions == training['timesteps'] == 8192
    assert checkpoint == digest(HERE / 'outputs/sac-r1/best/best_model.zip')
    summary = {}
    for controller in ['v3', 'zero', 'sac']:
        rows = [r for r in records if r['controller'] == controller]
        assert len(rows) == 30 and collections.Counter(r['goal'] for r in rows) == {'A': 10, 'B': 10, 'C': 10}
        assert collections.Counter(r['kind'] for r in rows) == {'static': 10, 'force': 10, 'offset': 10}
        summary[controller] = {'successes': sum(r['success'] for r in rows), 'trials': len(rows),
            'first_successes': sum(r['success'] and r['retries'] == 0 for r in rows),
            'recovered_successes': sum(r['success'] and r['retries'] > 0 for r in rows),
            'actual_offset_empty_grasp_recovered': sum(r['kind'] == 'offset' and r['first_outcome'] == 'empty_grasp_or_slip' and r['success'] for r in rows),
            'false_successes': sum(r['false_success'] for r in rows), 'forbidden_steps': sum(r['forbidden_steps'] for r in rows),
            'mean_sim_seconds': float(np.mean([r['sim_seconds'] for r in rows]))}
    write(HERE / 'delivery/FORMAL_SUMMARY.json', {'checkpoint_sha256': checkpoint, 'summary': summary, 'scenes': records})
    write(HERE / 'delivery/AUDIT.json', {'passed': True, 'score_rows_recomputed': scoring_rows,
        'actor_proposals_checked': actor_checks, 'paired_resets_equal': True,
        'complete_episode_decisions': completed_decisions, 'partial_decisions_recovered': partial_decisions,
        'train_validation_test_seeds_disjoint': True, 'source_sha256': digest(Path(__file__))})
    print(json.dumps({'summary': summary, 'score_rows_recomputed': scoring_rows, 'actor_proposals_checked': actor_checks}, indent=2))
