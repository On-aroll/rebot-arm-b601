"""Export only the deterministic trained SAC actor and check recorded inputs."""
import json
from pathlib import Path
import shutil

from environment import HERE, BOUNDS
from runtime import write, digest
import numpy as np
import torch
from torch import nn
from stable_baselines3 import SAC


class Actor(nn.Module):
    def __init__(self, actor):
        super().__init__()
        self.actor = actor

    def forward(self, observation):
        return self.actor(observation, deterministic=True)


if __name__ == '__main__':
    directory = HERE / 'delivery'
    model = SAC.load(directory / 'policy.zip', device='cpu')
    actor = Actor(model.actor.cpu().eval())
    example_file = next((HERE / 'outputs/sac-r1').glob('env-*/episode-*/rl_steps.json'))
    examples = json.loads(example_file.read_text())
    observation = np.asarray(examples[0]['observation'], dtype=np.float32)
    with torch.inference_mode():
        exported = torch.jit.trace(actor, torch.from_numpy(observation[None]))
        exported.save(str(directory / 'actor.torchscript'))
        errors = []
        for row in examples[:10]:
            x = torch.tensor(row['observation'], dtype=torch.float32)[None]
            errors.append(float((actor(x) - exported(x)).abs().max()))
        assert max(errors) < 1e-6
    np.savez_compressed(directory / 'example_observation.npz', observation=observation)
    write(directory / 'inference_config.json', {'schema': 'b601-v3-sac-residual-skill-v1',
        'observation_dim': 36, 'observation_order': ['visual_features_8', 'visible_1', 'own_state_20', 'goal_onehot_3', 'previous_residual_normalized_4'],
        'residual_bounds_m': BOUNDS.tolist(), 'max_residual_change_m_per_decision': [.002, .002, .002, .0015],
        'scope': 'grasp/lift residual only; frozen V3 vision, base trajectory and supervisor required',
        'actor_parameters': sum(p.numel() for p in actor.parameters()),
        'checkpoint_sha256': digest(directory / 'policy.zip'), 'actor_sha256': digest(directory / 'actor.torchscript'),
        'comparison_samples': len(errors), 'maximum_export_error': max(errors), 'hardware_executable': False})
    shutil.copy2(HERE / 'infer.py', directory / 'infer.py')
