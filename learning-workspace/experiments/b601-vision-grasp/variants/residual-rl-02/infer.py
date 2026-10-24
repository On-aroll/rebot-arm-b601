"""CPU-only residual proposal; does not connect to or move hardware."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=Path, default=Path(__file__).resolve().parent)
    parser.add_argument('--observation', type=Path)
    args = parser.parse_args()
    torch.set_num_threads(1)
    config = json.loads((args.directory / 'inference_config.json').read_text())
    example = args.observation or args.directory / 'example_observation.npz'
    observation = np.load(example)['observation'].astype('float32')
    if observation.shape != (39,) or not np.isfinite(observation).all():
        raise ValueError('expected 39 finite observation values in documented order')
    model = torch.jit.load(str(args.directory / 'actor.torchscript')).eval()
    with torch.inference_mode():
        action = model(torch.from_numpy(observation[None]))[0].numpy()
    bounds = np.asarray(config['residual_bounds_m'])
    previous = observation[32:36] * bounds
    limits = np.asarray(config['max_residual_change_m_per_decision'])
    delta = previous + np.clip(action * bounds - previous, -limits, limits)
    print(json.dumps({'normalized_proposal': action.tolist(), 'bounded_residual_m': delta.tolist(),
        'simulation_only': True, 'scope': 'residual proposal; requires frozen V3 vision, IK, timing and supervisor'}))
