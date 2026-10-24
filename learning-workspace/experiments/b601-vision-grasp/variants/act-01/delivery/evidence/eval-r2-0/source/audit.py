"""Independent evidence checks: causality, splits, frozen weights and finite retries."""
import argparse
import json
from pathlib import Path

import numpy as np
from runtime import verify_v3, write, digest


def audit(directories, output):
    verify_v3()
    sets, counts = {}, {}
    for name, directory in directories.items():
        seeds = set()
        n, frames, eligible, failures = 0, 0, 0, 0
        for path in sorted(Path(directory).glob('episode-*/result.json')):
            result = json.loads(path.read_text())
            seed = result['reset']['seed']
            assert seed not in seeds
            seeds.add(seed)
            assert result['max_retries'] == 2 and len(result['attempts']) <= 3
            failures += not result['success']
            npz = path.parent / 'imitation.npz'
            n += 1
            if not npz.exists():
                assert not result['success']
                continue
            rows = [json.loads(x) for x in (path.parent / 'trajectory.jsonl').read_text().splitlines()]
            indices = [i for i, r in enumerate(rows) if r['observation_index'] is not None]
            with np.load(npz, allow_pickle=False) as d:
                assert set(d.files) == {'rgb', 'depth', 'state', 'action', 'goal', 'eligible', 'timestamp_s'}
                assert d['state'].shape == (len(indices), 20)
                assert d['action'].shape == (len(indices), 7)
                assert np.isfinite(d['state']).all() and np.isfinite(d['action']).all()
                assert np.all(np.diff(d['timestamp_s']) > 0)
                for j, i in enumerate(indices):
                    row = rows[min(i + 4, len(rows) - 1)]
                    expected = row['q_command_rad'] + [row['gripper_command_m']]
                    np.testing.assert_allclose(d['action'][j], expected, atol=2e-7)
                frames += len(indices)
                eligible += int(d['eligible'].sum())
                if not result['success']:
                    assert not d['eligible'].any()
        sets[name] = seeds
        counts[name] = {'episodes': n, 'frames': frames, 'eligible_frames': eligible, 'failed_episodes_preserved': failures}
    names = list(sets)
    for i, name in enumerate(names):
        for other in names[i + 1:]:
            assert not sets[name] & sets[other], (name, other)
    result = {'passed': True, 'checks': ['V3 exact file hashes', 'causal future commands',
        'no scoring fields in learning features', 'disjoint episode seeds', 'at most two retries',
        'failures retained and excluded from positive imitation'], 'datasets': counts}
    write(output, result)
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--training', nargs='+', required=True)
    p.add_argument('--validation', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    audit({**{f'train_{i}': d for i, d in enumerate(a.training)}, 'validation': a.validation}, a.output)
