"""One-hour SAC continuation with curriculum, rehearsal and held-out selection."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import time

from curriculum import HERE, PREVIOUS, CurriculumEnv, base, write
from runtime import digest, verify_v3
import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv

TRAIN_SEEDS = list(range(50000000, 50001280))
VALIDATION_SEEDS = list(range(51000000, 51000036))


def factory(seeds, directory):
    def create():
        return Monitor(CurriculumEnv(seeds, directory), str(Path(directory) / 'monitor'))
    return create


def transfer(previous, model):
    """Insert three zero-weight observations; preserve the old actor exactly."""
    old, new = previous.policy.state_dict(), model.policy.state_dict()
    for name, value in new.items():
        if value.shape == old[name].shape:
            value.copy_(old[name])
        elif name == 'actor.latent_pi.0.weight':
            value.zero_()
            value[:, :36].copy_(old[name])
        else:
            assert name.endswith('.0.weight') and value.shape == (64, 43), name
            value.zero_()
            value[:, :36].copy_(old[name][:, :36])
            value[:, 39:].copy_(old[name][:, 36:])
    model.policy.load_state_dict(new)
    sample = np.random.default_rng(20261024).normal(size=(128, 39)).astype('float32')
    error = float(np.max(np.abs(previous.predict(sample[:, :36], deterministic=True)[0] -
                                model.predict(sample, deterministic=True)[0])))
    assert error == 0., error
    return error


def validate(env, model, output, label, zero=False):
    env.rewind_cases()
    rows = []
    for index in range(len(VALIDATION_SEEDS)):
        obs, _ = env.reset()
        done, total = False, 0.
        while not done:
            action = np.zeros(4) if zero else model.predict(obs, deterministic=True)[0]
            obs, reward, done, _, info = env.step(action)
            total += reward
        rows.append({'seed': VALIDATION_SEEDS[index], 'level': env.episode_level,
            'success': info['skill_success'], 'forbidden_steps': info['forbidden_steps'],
            'reward': total, 'mean_abs_residual_m': float(np.abs(env.delta).mean())})
    counts = {str(level): sum(r['success'] for r in rows if r['level'] == level) for level in range(3)}
    summary = {'label': label, 'successes_by_level_of_12': counts,
        'total_successes_of_36': sum(r['success'] for r in rows),
        'forbidden_steps': sum(r['forbidden_steps'] for r in rows),
        'mean_abs_residual_m': float(np.mean([r['mean_abs_residual_m'] for r in rows])),
        'mean_reward': float(np.mean([r['reward'] for r in rows])), 'episodes': rows}
    write(output / f'{label}.json', summary)
    print(json.dumps({k: v for k, v in summary.items() if k != 'episodes'}), flush=True)
    return summary


def selection(row):
    return (row['successes_by_level_of_12']['0'], row['total_successes_of_36'],
            -row['forbidden_steps'], -row['mean_abs_residual_m'])


class LongRun(BaseCallback):
    def __init__(self, output, validation, seconds, initial):
        super().__init__()
        self.output, self.validation, self.seconds = output, validation, seconds
        self.best, self.history = initial, []
        self.started = time.monotonic()
        self.last_print, self.next_eval, self.last_phase = 0., 300., -1

    def _on_step(self):
        elapsed = time.monotonic() - self.started
        phase = 0 if elapsed < self.seconds * .15 else 1 if elapsed < self.seconds * .4 else 2
        if phase != self.last_phase:
            self.training_env.env_method('set_phase', phase)
            lr = [3e-4, 1e-4, 3e-5][phase]
            self.model.lr_schedule = lambda _: lr
            self.last_phase = phase
            self.model.save(self.output / f'phase-{phase}-start')
        if elapsed >= self.next_eval:
            label = f'validation-{self.num_timesteps:07d}'
            row = validate(self.validation, self.model, self.output / 'validation_history', label)
            row.update(timesteps=self.num_timesteps, elapsed_seconds=time.monotonic() - self.started, phase=phase)
            self.history.append(row)
            if selection(row) > selection(self.best):
                self.best = row
                self.model.save(self.output / 'best_model')
                write(self.output / 'best_validation.json', row)
            self.model.save(self.output / 'last_model')
            self.model.save(self.output / f'checkpoint-{self.num_timesteps:07d}')
            self.model.save_replay_buffer(self.output / 'replay_buffer.pkl')
            write(self.output / 'validation_history.json', self.history)
            self.next_eval = time.monotonic() - self.started + 300.
        if elapsed - self.last_print >= 60.:
            progress = {'elapsed_seconds': time.monotonic() - self.started,
                'budget_seconds': self.seconds, 'timesteps': self.num_timesteps, 'phase': phase,
                'best_validation_successes_of_36': self.best['total_successes_of_36'],
                'best_nominal_successes_of_12': self.best['successes_by_level_of_12']['0']}
            write(self.output / 'progress.json', progress)
            print(json.dumps(progress), flush=True)
            self.last_print = elapsed
        return time.monotonic() - self.started < self.seconds


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=3600.)
    parser.add_argument('--output', default='long-r2')
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args()
    verify_v3()
    torch.set_num_threads(1)
    output = HERE / 'outputs' / args.output
    output.mkdir(parents=True, exist_ok=False)
    (output / 'validation_history').mkdir()
    snapshot = output / 'source'
    snapshot.mkdir()
    for path in HERE.glob('*.py'):
        shutil.copy2(path, snapshot / path.name)
    config = {'algorithm': 'Stable-Baselines3 SAC 2.7.1', 'seconds': args.seconds,
        'training_seeds': TRAIN_SEEDS, 'validation_seeds': VALIDATION_SEEDS,
        'workers': args.workers, 'buffer_size': 200000, 'batch_size': 256,
        'gamma': .98, 'gradient_steps': 1, 'warm_start': str(PREVIOUS / 'delivery/policy.zip'),
        'warm_start_sha256': digest(PREVIOUS / 'delivery/policy.zip'),
        'learning_rates': [3e-4, 1e-4, 3e-5], 'phase_budget_fractions': [.15, .4, 1.],
        'rehearsal_fraction': .25, 'potential': '-4*min(norm(visual_tcp_error_xy)/.035,2)',
        'shaping': 'gamma*Phi(next)-Phi(current), terminal Phi=0',
        'selection': 'lexicographic nominal successes, total successes, negative forbidden, negative correction',
        'source_commit': subprocess.check_output(['git', '-C', str(HERE), 'rev-parse', 'HEAD'], text=True).strip(),
        'optimizer_and_replay': 'new; old observation width was 36, now 39', 'simulation_only': True}
    env = SubprocVecEnv([factory(TRAIN_SEEDS[i::args.workers], output / f'env-{i}')
                        for i in range(args.workers)], start_method='spawn')
    validation = CurriculumEnv(VALIDATION_SEEDS, output / 'validation', validation=True)
    model = SAC('MlpPolicy', env, learning_rate=3e-4, buffer_size=200000,
        learning_starts=512, batch_size=256, train_freq=1, gradient_steps=1, gamma=.98,
        ent_coef='auto_0.05', policy_kwargs={'net_arch': [64, 64], 'log_std_init': -3.},
        seed=20261025, device='cpu', verbose=0)
    previous = SAC.load(PREVIOUS / 'delivery/policy.zip', device='cpu')
    config['warm_start_actor_max_abs_error_128_samples'] = transfer(previous, model)
    write(output / 'config.json', config)
    started = time.monotonic()
    try:
        validate(validation, None, output / 'validation_history', 'zero-initial', zero=True)
        initial = validate(validation, model, output / 'validation_history', 'warm-initial')
        model.save(output / 'warm_initial')
        model.save(output / 'best_model')
        write(output / 'best_validation.json', initial)
        callback = LongRun(output, validation, args.seconds, initial)
        model.learn(total_timesteps=1000000, callback=callback)
        model.save(output / 'last_model')
        model.save_replay_buffer(output / 'replay_buffer.pkl')
        final = validate(validation, model, output / 'validation_history', 'final')
        if selection(final) > selection(callback.best):
            callback.best = final
            model.save(output / 'best_model')
            write(output / 'best_validation.json', final)
        write(output / 'training.json', {'timesteps': model.num_timesteps,
            'learn_wall_seconds': time.monotonic() - callback.started,
            'including_initial_validation_wall_seconds': time.monotonic() - started,
            'best_validation': callback.best, 'final_validation': final,
            'actor_parameters': sum(p.numel() for p in model.actor.parameters())})
    finally:
        env.close()
        validation.close()


if __name__ == '__main__':
    main()
