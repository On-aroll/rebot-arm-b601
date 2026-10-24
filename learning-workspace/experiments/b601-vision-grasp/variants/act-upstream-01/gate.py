"""Single-demonstration simulation gate using the reference project's pinned ACT."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
PREVIOUS = HERE.parent / 'act-01'
UPSTREAM = REPO / '.tools/lerobot-upstream'
REVISION = '10b7b3532543b4adfb65760f02a49b4c537afde7'
EPISODE = PREVIOUS / 'outputs/demos-r1/episode-000'
sys.path.insert(0, str(PREVIOUS))
from runtime import RealisticSimulation, state, verify_v3, write, digest
from task import GOALS, independent_score

import numpy as np
import torch


def demonstration():
    with np.load(EPISODE / 'imitation.npz', allow_pickle=False) as raw:
        data = {k: raw[k].copy() for k in raw.files}
    with np.load(EPISODE / 'observations.npz', allow_pickle=False) as raw:
        data['rgb'] = raw['rgb'].copy()
    return data


def command(sim, proposed):
    """Identical limited transport for recorded and model-produced commands."""
    target = proposed.copy()
    q = sim.target_q.copy()
    target[:6] = np.clip(target[:6], np.maximum(sim.lower, q - .08),
                         np.minimum(sim.upper, q + .08))
    target[6] = np.clip(target[6], max(0., sim.grip - .008), min(.05, sim.grip + .008))
    sim.target_q, sim.grip = target[:6], float(target[6])
    return not np.allclose(target, proposed)


def rollout(data, policy=None):
    result = json.loads((EPISODE / 'result.json').read_text())
    sim = RealisticSimulation()
    reset = sim.reset(result['reset']['seed'], force_pulse=None)
    assert reset == result['reset']
    if policy is not None:
        policy.reset()
    forbidden, clipped, rows = 0, 0, []
    started = time.monotonic()
    try:
        # This gate uses the demonstration's fixed duration, with no teacher stages
        # or elapsed-time input to the policy and no scripted recovery movement.
        for i in range(len(data['action'])):
            if policy is None:
                proposed = data['action'][i].astype(float)
            else:
                frame = sim.observe(labels=False)
                batch = {
                    'observation.image': torch.from_numpy(frame['rgb'].copy()).permute(2, 0, 1)[None].float().cuda() / 255,
                    'observation.state': torch.from_numpy(np.r_[state(sim), data['goal'][i]])[None].float().cuda(),
                }
                with torch.inference_mode():
                    proposed = policy.select_action(batch)[0].cpu().numpy().astype(float)
            clipped += command(sim, proposed)
            rows.append({'time_s': float(sim.data.time), 'state': state(sim).tolist(),
                         'proposal': proposed.tolist(), 'command': np.r_[sim.target_q, sim.grip].tolist()})
            for _ in range(round(.2 / sim.model.opt.timestep)):
                sim.step()
                forbidden += bool(sim.contacts()['forbidden'])
        scores = []
        for _ in range(round(1. / sim.model.opt.timestep)):
            sim.step()
            forbidden += bool(sim.contacts()['forbidden'])
            scores.append(independent_score(sim, GOALS[result['task']['destination']]))
        return {'success': all(s['success'] for s in scores) and not forbidden,
                'forbidden_steps': forbidden, 'clipped_commands': int(clipped),
                'final_score': scores[-1], 'sim_seconds': float(sim.data.time),
                'wall_seconds': time.monotonic() - started, 'trajectory': rows}
    finally:
        sim.close()


def upstream():
    actual = subprocess.check_output(['git', '-C', str(UPSTREAM), 'rev-parse', 'HEAD'], text=True).strip()
    assert actual == REVISION, actual
    assert not subprocess.check_output(['git', '-C', str(UPSTREAM), 'status', '--porcelain'], text=True).strip()
    sys.path.insert(0, str(UPSTREAM))
    from lerobot.common.policies.act.configuration_act import ACTConfig
    from lerobot.common.policies.act.modeling_act import ACTPolicy
    from lerobot.configs.types import FeatureType, PolicyFeature
    return ACTConfig, ACTPolicy, FeatureType, PolicyFeature


def train(data, output, steps):
    ACTConfig, ACTPolicy, FeatureType, PolicyFeature = upstream()
    torch.manual_seed(20261024)
    torch.set_num_threads(4)
    torch.backends.cuda.matmul.allow_tf32 = True
    config = ACTConfig(input_features={
        'observation.image': PolicyFeature(FeatureType.VISUAL, (3, 120, 160)),
        'observation.state': PolicyFeature(FeatureType.STATE, (23,)),
    }, output_features={'action': PolicyFeature(FeatureType.ACTION, (7,))},
        chunk_size=25, n_action_steps=5, device='cuda', optimizer_lr=1e-4,
        optimizer_lr_backbone=1e-5)
    own = np.concatenate([data['state'], data['goal']], -1)
    eligible = np.flatnonzero(data['eligible'])
    stats = {
        'observation.image': {'mean': torch.tensor([.485, .456, .406])[:, None, None],
                              'std': torch.tensor([.229, .224, .225])[:, None, None]},
        'observation.state': {'mean': torch.tensor(own[eligible].mean(0)),
                              'std': torch.tensor(np.maximum(own[eligible].std(0), .02))},
        'action': {'mean': torch.tensor(data['action'][eligible].mean(0)),
                   'std': torch.tensor(np.maximum(data['action'][eligible].std(0), [.02] * 6 + [.002]), dtype=torch.float32)},
    }
    policy = ACTPolicy(config, dataset_stats=stats).cuda()
    images = torch.from_numpy(data['rgb']).permute(0, 3, 1, 2).float().cuda() / 255
    states = torch.from_numpy(own).float().cuda()
    actions = torch.from_numpy(data['action']).float().cuda()
    optimizer = torch.optim.AdamW(policy.get_optim_params(), lr=config.optimizer_lr, weight_decay=1e-4)
    rng = np.random.default_rng(20261024)
    started = time.monotonic()
    history = []
    for step in range(1, steps + 1):
        ids = rng.choice(eligible, 16)
        future = ids[:, None] + np.arange(config.chunk_size)
        pad = future >= eligible[-1] + 1
        future = np.minimum(future, eligible[-1])
        batch = {'observation.image': images[ids], 'observation.state': states[ids],
                 'action': actions[future], 'action_is_pad': torch.tensor(pad, device='cuda')}
        policy.train()
        loss, metrics = policy(batch)
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(policy.parameters(), 10.)
        optimizer.step()
        if step == 1 or step % 100 == 0 or step == steps:
            torch.cuda.synchronize()
            row = {'step': step, 'loss': float(loss), **metrics,
                   'seconds': time.monotonic() - started}
            history.append(row)
            write(output / 'history.json', history)
            print(json.dumps(row), flush=True)
    policy.eval()
    policy.save_pretrained(output / 'checkpoint')
    write(output / 'training.json', {'upstream_revision': REVISION, 'steps': steps,
        'episode': str(EPISODE.relative_to(REPO)), 'frames': len(eligible),
        'parameters': sum(p.numel() for p in policy.parameters()),
        'peak_cuda_allocated_bytes': torch.cuda.max_memory_allocated(),
        'scope': 'one-episode overfit diagnostic, not held-out evaluation'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['teacher', 'train', 'evaluate'])
    parser.add_argument('--name', required=True)
    parser.add_argument('--steps', type=int, default=2000)
    parser.add_argument('--repeats', type=int, default=10)
    parser.add_argument('--checkpoint', type=Path)
    args = parser.parse_args()
    verify_v3()
    output = HERE / 'outputs' / args.name
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, output / 'gate.py')
    write(output / 'run.json', {**vars(args), 'checkpoint': str(args.checkpoint),
        'source_sha256': digest(Path(__file__)), 'upstream_revision': REVISION,
        'data_sha256': {name: digest(EPISODE / name) for name in ['imitation.npz', 'observations.npz', 'result.json']}})
    data = demonstration()
    if args.mode == 'train':
        train(data, output, args.steps)
        return
    policy = None
    if args.mode == 'evaluate':
        _, ACTPolicy, _, _ = upstream()
        torch.set_num_threads(4)
        policy = ACTPolicy.from_pretrained(args.checkpoint, local_files_only=True).cuda().eval()
    results = []
    for i in range(args.repeats):
        row = rollout(data, policy)
        write(output / f'episode-{i:03d}.json', row)
        del row['trajectory']
        results.append(row)
        write(output / 'summary.json', {'scope': 'same known initial scene repeated, not independent generalization',
            'controller': args.mode, 'successes': sum(r['success'] for r in results),
            'trials': len(results), 'results': results})
        print(json.dumps({'trial': i + 1, **row}), flush=True)


if __name__ == '__main__':
    main()
