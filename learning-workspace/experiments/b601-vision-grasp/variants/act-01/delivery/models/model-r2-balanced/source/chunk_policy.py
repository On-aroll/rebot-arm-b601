"""Small ACT-style CVAE, trained from episodes; no simulator imports."""
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np
import torch
from torch import nn

CONFIG = {'chunk_size': 10, 'hidden_dim': 96, 'latent_dim': 8,
          'image_shape': [60, 80], 'state_dim': 24, 'goal_dim': 3,
          'action_dim': 7, 'fps': 5, 'n_action_steps': 1,
          'visual_summary': True, 'relative_actions': True, 'proprio_noise_rad': .01,
          'latent_dropout': .5, 'kl_weight': .05}


class ChunkPolicy(nn.Module):
    def __init__(self, config=None):
        super().__init__()
        self.config = c = dict(CONFIG if config is None else config)
        h, z, k = c['hidden_dim'], c['latent_dim'], c['chunk_size']
        self.cnn = nn.Sequential(nn.Conv2d(4, 24, 5, 2, 2), nn.ReLU(),
            nn.Conv2d(24, 48, 3, 2, 1), nn.ReLU(),
            nn.Conv2d(48, h, 3, 2, 1), nn.ReLU(), nn.AdaptiveAvgPool2d((6, 8)))
        self.image_position = nn.Parameter(torch.randn(1, 48, h) * .02)
        self.proprio = nn.Linear(c['state_dim'] + c['goal_dim'], h)
        self.z_embed = nn.Linear(z, h)
        self.query = nn.Parameter(torch.randn(1, k, h) * .02)
        self.encoder = nn.TransformerEncoder(nn.TransformerEncoderLayer(h, 4, 256,
            dropout=0., batch_first=True, activation='gelu'), 2)
        self.decoder = nn.TransformerDecoder(nn.TransformerDecoderLayer(h, 4, 256,
            dropout=0., batch_first=True, activation='gelu'), 2)
        self.posterior = nn.Sequential(nn.Linear(k * 7 + c['state_dim'] + 3, h),
                                      nn.GELU(), nn.Linear(h, z * 2))
        self.head = nn.Linear(h, 7)

    def forward(self, image, state, goal, actions=None):
        batch = len(state)
        condition = torch.cat([state, goal], -1)
        kl = state.new_zeros(())
        z = state.new_zeros((batch, self.config['latent_dim']))
        if actions is not None:
            mu, logvar = self.posterior(torch.cat([condition, actions.flatten(1)], -1)).chunk(2, -1)
            logvar = logvar.clamp(-10, 6)
            z = mu + torch.exp(.5 * logvar) * torch.randn_like(mu)
            kl = -.5 * (1 + logvar - mu.square() - logvar.exp()).sum(-1).mean()
            if self.training and self.config.get('latent_dropout', 0.):
                z = z * (torch.rand(batch, 1, device=z.device) >= self.config['latent_dropout'])
        visual = self.cnn(image).flatten(2).transpose(1, 2) + self.image_position
        memory = torch.cat([self.proprio(condition)[:, None], self.z_embed(z)[:, None], visual], 1)
        memory = self.encoder(memory)
        prediction = self.head(self.decoder(self.query.expand(batch, -1, -1), memory))
        return prediction, kl


def image_input(rgb, depth):
    import cv2
    rgb = cv2.resize(rgb, (80, 60), interpolation=cv2.INTER_AREA).astype('float32') / 255
    depth = cv2.resize(depth, (80, 60), interpolation=cv2.INTER_NEAREST)
    return np.concatenate([rgb, np.clip(depth[..., None], 0, 2)], -1).transpose(2, 0, 1)


def visual_summary(rgb, depth):
    """Visible red/orange pixel centroid/depth only; no world/object labels."""
    r, g, b = rgb.astype('float32').transpose(2, 0, 1)
    mask = (r > 80) & (r > 1.3 * g) & (r > 1.3 * b) & (depth > .1) & (depth < 1.5)
    v, u = np.where(mask)
    if len(u) < 3:
        return np.zeros(4, 'float32')
    return np.array([2 * (np.median(u) / rgb.shape[1] - .5),
                     2 * (np.median(v) / rgb.shape[0] - .5),
                     (np.median(depth[mask]) - .65) / .3, 1.], 'float32')


class EpisodeDataset:
    def __init__(self, directories, stats=None):
        self.episodes, self.indices = [], []
        self.paths = []
        for directory in directories:
            for p in sorted(Path(directory).glob('episode-*/imitation.npz')):
                with np.load(p, allow_pickle=False) as d:
                    episode = {k: d[k].copy() for k in ('rgb', 'depth', 'state', 'action', 'goal', 'eligible')}
                episode['state'] = np.concatenate([episode['state'], np.stack([
                    visual_summary(rgb, depth) for rgb, depth in zip(episode['rgb'], episode['depth'])])], -1)
                self.paths.append(str(p.resolve()))
                eid = len(self.episodes)
                self.episodes.append(episode)
                self.indices.extend((eid, int(i)) for i in np.flatnonzero(episode['eligible']))
        if not self.indices:
            raise ValueError('No eligible demonstration frames')
        if stats is None:
            states = np.concatenate([e['state'][e['eligible']] for e in self.episodes])
            actions = np.concatenate([e['action'][e['eligible']] for e in self.episodes])
            stats = {'state_mean': states.mean(0).tolist(), 'state_std': np.maximum(states.std(0), .02).tolist(),
                     'action_mean': [0.] * 7, 'action_std': [.08] * 6 + [.008]}
        self.stats = stats

    def batch(self, ids, device, augment=False):
        images, states, goals, actions, masks = [], [], [], [], []
        for sample in ids:
            eid, i = self.indices[int(sample)]
            e = self.episodes[eid]
            future = np.arange(i, i + CONFIG['chunk_size'])
            valid = future < len(e['state'])
            future = np.minimum(future, len(e['state']) - 1)
            valid &= e['eligible'][future]
            images.append(np.concatenate([e['rgb'][i].astype('float32') / 255,
                                          np.clip(e['depth'][i, ..., None], 0, 2)], -1).transpose(2, 0, 1))
            own = e['state'][i].copy()
            if augment:
                own[:6] += np.random.normal(0, CONFIG['proprio_noise_rad'], 6)
                own[6:12] += np.random.normal(0, .02, 6)
                own[13:19] += np.random.normal(0, CONFIG['proprio_noise_rad'], 6)
                own[[12, 19]] += np.random.normal(0, .02, 2)
            baseline = np.r_[own[13:19], own[19] * .05]
            states.append(own); goals.append(e['goal'][i]); actions.append(e['action'][future] - baseline); masks.append(valid)
        s = self.stats
        arrays = [np.stack(images), (np.stack(states) - s['state_mean']) / s['state_std'],
                  np.stack(goals), (np.stack(actions) - s['action_mean']) / s['action_std'], np.stack(masks)]
        return [torch.as_tensor(a, device=device, dtype=torch.float32) for a in arrays]


def train(directories, validation, output, steps=4000, seed=20261024):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    torch.manual_seed(seed)
    np.random.seed(seed)
    rng = np.random.default_rng(seed)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    torch.set_num_threads(4)
    if device == 'cuda':
        torch.backends.cudnn.benchmark = True
        torch.backends.cuda.matmul.allow_tf32 = True
    training = EpisodeDataset(directories)
    valid = EpisodeDataset([validation], training.stats)
    source = output / 'source'
    source.mkdir()
    for path in Path(__file__).parent.glob('*.py'):
        shutil.copy2(path, source / path.name)
    fingerprint = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
    (output / 'frozen_training.json').write_text(json.dumps({'seed': seed, 'steps': steps,
        'config': CONFIG, 'device': device, 'batch_size': 32, 'learning_rate': .0003,
        'kl_weight': CONFIG['kl_weight'], 'source_sha256': {p.name: fingerprint(p) for p in source.iterdir()},
        'train_data_sha256': {p: fingerprint(p) for p in training.paths},
        'validation_data_sha256': {p: fingerprint(p) for p in valid.paths}}, indent=2))
    model = ChunkPolicy().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=3e-4, weight_decay=1e-4)
    history, best, best_state, selected = [], float('inf'), None, 0
    validation_ids = np.random.default_rng(700).choice(len(valid.indices), min(512, len(valid.indices)), replace=False)
    started = time.monotonic()
    for step in range(1, steps + 1):
        model.train()
        image, state, goal, action, mask = training.batch(rng.integers(len(training.indices), size=32), device, augment=True)
        prediction, kl = model(image, state, goal, action * mask[..., None])
        l1 = ((prediction - action).abs() * mask[..., None]).sum() / (mask.sum() * 7)
        loss = l1 + CONFIG['kl_weight'] * kl
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), 1.)
        optimizer.step()
        if step == 1 or step % 200 == 0 or step == steps:
            model.eval()
            errors = []
            with torch.inference_mode():
                for ids in np.array_split(validation_ids, max(1, len(validation_ids)//32)):
                    vi, vs, vg, va, vm = valid.batch(ids, device)
                    vp, _ = model(vi, vs, vg)  # zero latent, identical to deployment
                    errors.append(float(((vp - va).abs() * vm[..., None]).sum() / (vm.sum() * 7)))
            error = float(np.mean(errors))
            row = {'step': step, 'train_l1': float(l1.detach()), 'kl': float(kl.detach()),
                   'validation_zero_latent_l1': error, 'wall_s': time.monotonic() - started}
            history.append(row)
            print(json.dumps(row), flush=True)
            if error < best:
                best, selected = error, step
                best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
                torch.save({'schema': 'b601-small-act-v1', 'state_dict': best_state,
                    'config': CONFIG, 'stats': training.stats, 'selected_step': selected,
                    'hardware_executable': False}, output / 'policy.pt')
            (output / 'training.json').write_text(json.dumps({'device': device,
                'parameters': sum(p.numel() for p in model.parameters()), 'steps': steps,
                'train_episodes': len(training.episodes), 'validation_episodes': len(valid.episodes),
                'train_frames': len(training.indices), 'validation_frames': len(valid.indices),
                'train_paths': training.paths, 'validation_paths': valid.paths,
                'selected_step': selected, 'history': history,
                'cuda_peak_allocated_bytes': torch.cuda.max_memory_allocated() if device == 'cuda' else 0}, indent=2))
    torch.save({'schema': 'b601-small-act-v1', 'state_dict': model.cpu().state_dict(),
                'config': CONFIG, 'stats': training.stats, 'selected_step': steps,
                'hardware_executable': False}, output / 'last.pt')
    return output / 'policy.pt'


class Inference:
    def __init__(self, path, device='cpu'):
        checkpoint = torch.load(path, map_location=device, weights_only=True)
        self.device, self.stats = device, checkpoint['stats']
        self.model = ChunkPolicy(checkpoint['config']).to(device).eval()
        self.model.load_state_dict(checkpoint['state_dict'])
        self.latencies = []

    def predict(self, frame, state, goal):
        start = time.perf_counter()
        s = self.stats
        image_array = image_input(frame['rgb'], frame['depth'])
        baseline = np.r_[state[13:19], state[19] * .05]
        if self.model.config.get('visual_summary'):
            small_rgb = (image_array[:3].transpose(1, 2, 0) * 255).astype('uint8')
            state = np.r_[state, visual_summary(small_rgb, image_array[3])]
        image = torch.as_tensor(image_array[None], device=self.device)
        state = torch.as_tensor(((state - s['state_mean']) / s['state_std'])[None], device=self.device, dtype=torch.float32)
        goal = torch.as_tensor(np.eye(3, dtype='float32')['ABC'.index(goal)][None], device=self.device)
        with torch.inference_mode():
            prediction = self.model(image, state, goal)[0][0].cpu().numpy()
        self.latencies.append((time.perf_counter() - start) * 1000)
        action = prediction * s['action_std'] + s['action_mean']
        return action + baseline if self.model.config.get('relative_actions') else action
