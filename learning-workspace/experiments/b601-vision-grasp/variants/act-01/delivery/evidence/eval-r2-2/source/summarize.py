"""Recompute scores from saved evidence and combine 30 paired held-out scenes."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from runtime import ROOT, digest, verify_v3, write


def summarize(round_name):
    verify_v3()
    outputs = ROOT / 'outputs'
    runs = [outputs / f'eval-{round_name}-{i}' for i in range(3)]
    rows, weights, seeds = [], set(), set()
    checked_scores = 0
    for run in runs:
        weights.add(digest(run / 'policy.pt'))
        provenance = json.loads((run / 'provenance.json').read_text())
        for name, sha in provenance['source_sha256'].items():
            assert digest(run / 'source' / name) == sha
        batch = json.loads((run / 'results.json').read_text())
        assert len(batch) == 20
        scenes = json.loads((run / 'frozen_scenes.json').read_text())
        for i, scene in enumerate(scenes):
            assert scene['seed'] not in seeds
            seeds.add(scene['seed'])
            for mode in ('v3', 'act'):
                episode = run / f'{mode}-{i:03d}'
                result = json.loads((episode / 'result.json').read_text())
                assert result['retries'] <= 2 and len(result['attempts']) <= 3
                assert result['hardware_executable'] is False
                trace = [json.loads(s) for s in (episode / 'final_score_trace.jsonl').read_text().splitlines()]
                for score in trace:
                    pos = np.asarray(score['object_xyz_m'])
                    extent = np.abs(score['object_rotation_base']) @ np.asarray(score['object_half_size_m'])
                    goal = [.42, {'A': -.16, 'B': 0., 'C': .16}[scene['goal']]]
                    expected = (np.all(np.abs(pos[:2] - goal) + extent[:2] <= .06)
                        and abs(pos[2] - extent[2] - .1) < .006
                        and np.linalg.norm(score['object_linear_velocity_m_s']) < .02
                        and not any(score['finger_contacts'].values()))
                    assert bool(expected) == score['success']
                    checked_scores += 1
                expected_physical = bool(trace) and all(s['success'] for s in trace) and not result['physics_tick_audit']['forbidden_steps']
                assert expected_physical == result['physical_success']
                assert result['success'] == bool(expected_physical and result['controller_success'])
        rows += batch
    assert len(seeds) == 30 and len(weights) == 1
    training_seeds = set()
    for folder in ('demos-r1', 'demos-r2-recovery', 'validation-r1'):
        for path in (outputs / folder).glob('episode-*/result.json'):
            training_seeds.add(json.loads(path.read_text())['reset']['seed'])
    assert not seeds & training_seeds
    groups = {}
    for mode in ('v3', 'act'):
        subset = [r for r in rows if r['mode'] == mode]
        groups[mode] = {'trials': len(subset), 'successes': sum(r['success'] for r in subset),
            'first_attempt_successes': sum(r['success'] and not r['retries'] for r in subset),
            'recovered_successes': sum(r['success'] and bool(r['retries']) for r in subset),
            'false_successes': sum(r['false_success'] for r in subset),
            'total_retries': sum(r['retries'] for r in subset),
            'forbidden_contact_steps': sum(r['forbidden_steps'] for r in subset),
            'mean_sim_seconds': float(np.mean([r['sim_seconds'] for r in subset])),
            'mean_wall_seconds_before_saving': float(np.mean([r['wall_seconds'] for r in subset])),
            'failures': dict(Counter(r['reason'] for r in subset if not r['success'])),
            'by_goal': {g: sum(r['success'] for r in subset if r['scene']['goal'] == g) for g in 'ABC'},
            'by_challenge': {k: {'n': sum(r['scene']['kind'] == k for r in subset),
                'successes': sum(r['success'] and r['scene']['kind'] == k for r in subset)} for k in ('static', 'force', 'offset')}}
    result = {'round': round_name, 'paired_scenes': 30, 'policy_sha256': next(iter(weights)),
        'audit_passed': True, 'independent_score_rows_recomputed': checked_scores,
        'goal_counts': dict(Counter(r['scene']['goal'] for r in rows if r['mode'] == 'act')),
        'groups': groups}
    write(outputs / f'eval-{round_name}-summary.json', result)
    write(outputs / f'eval-{round_name}-combined.json', rows)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--round', required=True)
    summarize(p.parse_args().round)
