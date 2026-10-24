"""Freeze the selected policy and measure only its deterministic residual actor."""
import json
import shutil
import time
from curriculum import HERE, base, write
from runtime import digest
from stable_baselines3 import SAC
import numpy as np
import torch


class Actor(torch.nn.Module):
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, observation):
        return self.actor(observation, deterministic=True)


if __name__ == '__main__':
    delivery = HERE / 'delivery'
    delivery.mkdir(exist_ok=True)
    shutil.copy2(HERE / 'outputs/long-r2-hour/best_model.zip', delivery / 'policy.zip')
    model = SAC.load(delivery / 'policy.zip', device='cpu')
    actor = Actor(model.actor.cpu().eval())
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
        'scope': 'bounded grasp/lift residual, frozen V3 system required', 'hardware_executable': False})
    write(delivery / 'cpu_benchmark.json', {'threads': 1, 'samples': len(timings),
        'median_ms': float(np.median(timings)), 'p95_ms': float(np.quantile(timings, .95)),
        'actor_bytes': (delivery / 'actor.torchscript').stat().st_size,
        'scope': 'actor only; excludes image inference, IK, motion and hardware', 'measured_on_board': False})
    shutil.copy2(HERE / 'infer.py', delivery / 'infer.py')
