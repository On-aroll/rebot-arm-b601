"""Export the selected network and verify a portable CPU TorchScript artifact."""
import argparse
import json
from pathlib import Path

import torch
from torch import nn
from chunk_policy import Inference


class Network(nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, image, state, goal):
        return self.policy(image, state, goal)[0]


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--checkpoint', required=True)
    a = p.parse_args()
    checkpoint = Path(a.checkpoint)
    inference = Inference(checkpoint)
    torch.set_num_threads(2)
    torch.backends.mha.set_fastpath_enabled(False)
    network = Network(inference.model).eval()
    example = (torch.zeros(1, 4, 60, 80), torch.zeros(1, 20), torch.tensor([[0., 1., 0.]]))
    with torch.inference_mode():
        traced = torch.jit.trace(network, example, check_trace=True)
        path = checkpoint.parent / 'policy_cpu.ts'
        traced.save(str(path))
        loaded = torch.jit.load(str(path)).eval()
        maximum = 0.
        torch.manual_seed(73)
        for _ in range(10):
            test = (torch.rand(1, 4, 60, 80), torch.randn(1, 20), torch.eye(3)[torch.randint(3, (1,))])
            difference = float((network(*test) - loaded(*test)).abs().max())
            assert difference < 1e-5
            maximum = max(maximum, difference)
    (checkpoint.parent / 'config.json').write_text(json.dumps({
        'schema': 'b601-small-act-v1', 'network': inference.model.config,
        'normalization': inference.stats, 'hardware_executable': False,
        'torchscript_input': 'normalized image[1,4,60,80], normalized state[1,20], goal[1,3]',
        'torchscript_output': 'normalized action[1,10,7]',
        'export_max_absolute_error': maximum}, indent=2) + '\n')
    print(json.dumps({'path': str(path), 'max_absolute_error': maximum}), flush=True)
