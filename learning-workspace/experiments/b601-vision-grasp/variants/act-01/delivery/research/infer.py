"""Offline inference: NPZ RGB/depth/state -> action chunk JSON. No device writes."""
import argparse
import json
from pathlib import Path

import numpy as np
import torch
from chunk_policy import Inference


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', required=True)
    p.add_argument('--observation', required=True, help='NPZ keys rgb, depth_m, state (20,)')
    p.add_argument('--goal', choices=['A', 'B', 'C'], required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--elapsed-s', type=float, default=0., help='Seconds since goal/attempt began')
    a = p.parse_args()
    torch.set_num_threads(2)
    policy = Inference(a.checkpoint)
    with np.load(a.observation, allow_pickle=False) as d:
        rgb, depth, own = d['rgb'], d['depth_m'], d['state']
    if rgb.ndim != 3 or rgb.shape[2] != 3 or depth.shape != rgb.shape[:2] or own.shape != (20,):
        raise ValueError('Observation shape mismatch')
    if not np.isfinite(own).all() or not np.isfinite(depth).all():
        raise ValueError('Nonfinite observation')
    actions = policy.predict({'rgb': rgb, 'depth': depth}, own, a.goal, a.elapsed_s)
    Path(a.output).write_text(json.dumps({'schema': 'b601-small-act-proposal-v1',
        'goal': a.goal, 'joint_order': [f'joint{i}' for i in range(1, 7)],
        'joint_unit': 'rad', 'gripper_unit': 'single_finger_simulation_m',
        'step_seconds': .2, 'n_action_steps': 1, 'action_chunk': actions.tolist(),
        'hardware_executable': False}, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
