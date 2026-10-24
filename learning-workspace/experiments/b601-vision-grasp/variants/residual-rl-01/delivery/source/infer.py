"""Standalone CPU residual proposal; never opens a simulator or robot connection."""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import torch


def proposal(model, observation, config):
    action = model(torch.from_numpy(observation[None]).float())[0].numpy()
    bounds = np.asarray(config['residual_bounds_m'])
    last = observation[-4:] * bounds
    requested = action * bounds
    change = np.asarray(config['max_residual_change_m_per_decision'])
    return {'normalized_action': action.tolist(), 'requested_residual_m': requested.tolist(),
        'next_residual_m': (last + np.clip(requested - last, -change, change)).tolist(),
        'hardware_executable': False}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--observation', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--benchmark', action='store_true')
    args = parser.parse_args()
    torch.set_num_threads(2)
    model = torch.jit.load(str(args.model), map_location='cpu').eval()
    config = json.loads(args.config.read_text())
    with np.load(args.observation, allow_pickle=False) as data:
        observation = data['observation'].astype('float32')
    assert observation.shape == (36,)
    with torch.inference_mode():
        result = proposal(model, observation, config)
        if args.benchmark:
            for _ in range(10):
                proposal(model, observation, config)
            times = []
            for _ in range(200):
                started = time.perf_counter()
                proposal(model, observation, config)
                times.append((time.perf_counter() - started) * 1000)
            sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'act-01'))
            from benchmark_inference import rss
            result['benchmark'] = {'device': 'host_x86_CPU_two_threads', 'board_measured': False,
                'samples': len(times), 'median_ms': float(np.median(times)),
                'p95_ms': float(np.quantile(times, .95)), 'p99_ms': float(np.quantile(times, .99)),
                'model_bytes': args.model.stat().st_size, 'process_rss_bytes': rss(),
                'includes': 'residual actor and residual bounding; 36-dimensional features already provided',
                'excludes': 'RGB-D acquisition, frozen V3 vision, IK, transport, physical board and power'}
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))
