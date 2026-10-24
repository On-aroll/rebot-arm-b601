"""Freeze the selected policy and measure only its deterministic residual actor."""
import json
import argparse
from pathlib import Path
import shutil
import time
from curriculum import HERE, base, write
from runtime import digest
from stable_baselines3 import SAC
import numpy as np
import torch
from infer_numpy import predict as numpy_predict


class Actor(torch.nn.Module):
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, observation):
        return self.actor(observation, deterministic=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=Path, default=HERE / 'outputs/long-r2-hour/best_model.zip')
    parser.add_argument('--directory', type=Path, default=HERE / 'delivery')
    args = parser.parse_args()
    delivery = args.directory
    delivery.mkdir(parents=True, exist_ok=True)
    shutil.copy2(args.checkpoint, delivery / 'policy.zip')
    model = SAC.load(delivery / 'policy.zip', device='cpu')
    actor = Actor(model.actor.cpu().eval())
    state = model.actor.state_dict()
    weights = {short: state[name].numpy() for short, name in {
        'w0': 'latent_pi.0.weight', 'b0': 'latent_pi.0.bias', 'w1': 'latent_pi.2.weight',
        'b1': 'latent_pi.2.bias', 'wm': 'mu.weight', 'bm': 'mu.bias'}.items()}
    np.savez_compressed(delivery / 'numpy_actor.npz', **weights)
    example = next((HERE / 'outputs/long-r2-hour').glob('env-*/episode-*/rl_steps.json'))
    examples = json.loads(example.read_text())
    observations = np.asarray([row['observation'] for row in examples], dtype='float32')
    torch.set_num_threads(1)
    with torch.inference_mode():
        x = torch.from_numpy(observations)
        exported = torch.jit.trace(actor, x[:1])
        exported.save(str(delivery / 'actor.torchscript'))
        error = float((exported(x) - actor(x)).abs().max())
        assert error < 1e-6
        numpy_error = float(np.abs(numpy_predict(observations, weights) - actor(x).numpy()).max())
        assert numpy_error < 1e-6
        for _ in range(100):
            exported(x[:1])
        timings = []
        for _ in range(2000):
            started = time.perf_counter_ns()
            exported(x[:1])
            timings.append((time.perf_counter_ns() - started) / 1e6)
    np.savez_compressed(delivery / 'example_observation.npz', observation=observations[0])
    write(delivery / 'inference_config.json', {'observation_dim': 39,
        'observation_order': ['visual_features_8', 'visible_1', 'own_state_20', 'goal_onehot_3',
                              'previous_residual_normalized_4', 'visual_grasp_minus_own_tcp_normalized_3'],
        'relative_error_scale_m': [.04, .04, .16], 'relative_error_clip': [-3., 3.],
        'previous_residual_slice': [32, 36], 'residual_bounds_m': base.BOUNDS.tolist(),
        'max_residual_change_m_per_decision': [.002, .002, .002, .0015],
        'open_alignment_decisions': 5, 'grasp_lift_decisions': 18,
        'checkpoint_sha256': digest(delivery / 'policy.zip'), 'actor_sha256': digest(delivery / 'actor.torchscript'),
        'actor_parameters': sum(p.numel() for p in actor.parameters()),
        'comparison_samples': len(observations), 'export_max_abs_error': error,
        'numpy_export_max_abs_error': numpy_error, 'numpy_actor_sha256': digest(delivery / 'numpy_actor.npz'),
        'numpy_deterministic_parameters': sum(w.size for w in weights.values()),
        'scope': 'bounded grasp/lift residual, frozen V3 system required', 'hardware_executable': False})
    write(delivery / 'cpu_benchmark.json', {'threads': 1, 'samples': len(timings),
        'median_ms': float(np.median(timings)), 'p95_ms': float(np.quantile(timings, .95)),
        'actor_bytes': (delivery / 'actor.torchscript').stat().st_size,
        'scope': 'actor only; excludes image inference, IK, motion and hardware', 'measured_on_board': False})
    shutil.copy2(HERE / 'infer.py', delivery / 'infer.py')
    shutil.copy2(HERE / 'infer_numpy.py', delivery / 'infer_numpy.py')
