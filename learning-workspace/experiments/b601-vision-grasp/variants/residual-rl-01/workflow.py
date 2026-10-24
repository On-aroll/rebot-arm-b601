"""Train SAC on actual grasp outcomes and evaluate the V3-plus-skill system."""
import argparse
import json
from pathlib import Path
import shutil
import time

from environment import HERE, V3, GraspEnv, LoggedGraspEnv, prepare, STEPS
from runtime import verify_v3, write, digest
from task import Recorder, independent_score, visual_target, run_task, GOALS
from sim import HOME
from episode import DOWN
import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import EvalCallback


def factory(seeds, directory):
    def create():
        return Monitor(LoggedGraspEnv(seeds, directory), str(Path(directory) / 'monitor'))
    return create


def train(args, output):
    torch.set_num_threads(1)
    seeds = list(range(48000000, 48000072))
    env = SubprocVecEnv([factory(seeds[i::args.workers], output / f'env-{i}')
                        for i in range(args.workers)], start_method='spawn')
    validation = Monitor(LoggedGraspEnv(range(48100000, 48100012), output / 'validation'))
    model = SAC('MlpPolicy', env, learning_rate=3e-4, buffer_size=30000,
        learning_starts=144, batch_size=128, train_freq=1, gradient_steps=1,
        gamma=.98, ent_coef='auto_0.1', policy_kwargs={'net_arch': [64, 64], 'log_std_init': -3.},
        seed=20261024, device='cpu', verbose=0)
    torch.nn.init.zeros_(model.actor.mu.weight)
    torch.nn.init.zeros_(model.actor.mu.bias)
    callback = EvalCallback(validation, best_model_save_path=str(output / 'best'),
        log_path=str(output / 'validation_history'), eval_freq=max(1, 2048 // args.workers),
        n_eval_episodes=12, deterministic=True, verbose=1)
    started = time.monotonic()
    try:
        model.learn(total_timesteps=args.steps, callback=callback)
        model.save(output / 'last_model')
        model.save_replay_buffer(output / 'replay_buffer.pkl')
        write(output / 'training.json', {'algorithm': 'Stable-Baselines3 SAC 2.7.1',
            'timesteps': model.num_timesteps, 'workers': args.workers,
            'training_seeds': seeds, 'validation_seeds': list(range(48100000, 48100012)),
            'selection': 'highest validation mean grasp/lift reward, not formal task test',
            'actor_parameters': sum(p.numel() for p in model.actor.parameters()),
            'wall_seconds': time.monotonic() - started, 'hardware_executable': False})
    finally:
        env.close()
        validation.close()


def hybrid_task(env, model, scene, directory, zero=False):
    sim, perception, residual = env.sim, env.perception, env.residual
    directory.mkdir(parents=True, exist_ok=False)
    reset = sim.reset(scene['seed'], force_pulse=scene['force_pulse'])
    recorder = Recorder(sim, directory, False)
    result = {'controller': 'zero_skill' if zero else 'sac_residual_skill', 'scene': scene,
        'reset': reset, 'success': False, 'controller_success': False,
        'physical_success': False, 'false_success': False, 'attempts': [],
        'hardware_executable': False, 'skill_decisions': 0}
    started = time.monotonic()
    try:
        for attempt in range(3):
            row = {'attempt': attempt + 1}
            result['attempts'].append(row)
            center, grasp = prepare(sim, perception, residual, recorder,
                scene['first_grasp_offset_m'] if attempt == 0 else 0.)
            env.current_observation = env.begin(recorder, scene['goal'], grasp)
            while True:
                action = np.zeros(4) if zero else model.predict(env.current_observation, deterministic=True)[0]
                _, _, done, _, info = env.step(action)
                result['skill_decisions'] += 1
                if done:
                    break
            row['skill_actions'] = env.steps
            row['scoring_only_skill_result'] = info
            # Deployment recovery uses jaw feedback, not the reward scorer's success.
            jaw = float(sim.data.qpos[sim.gq])
            row['jaw_feedback_after_lift_m'] = jaw
            if env.failed:
                raise ValueError(env.failed)
            if not .008 < jaw < .038:
                row['outcome'] = 'empty_grasp_or_slip'
                recorder.move(sim.target_q, .046, .8, 'reopen')
                recorder.move(HOME, .046, 2., 'return_for_reobserve')
                continue
            destination = grasp.copy()
            destination[:2] += np.asarray(GOALS[scene['goal']]) - center[:2]
            destination[2] += .002
            qt = sim.ik(destination + [0., 0., .11], DOWN, sim.target_q)
            qp = sim.ik(destination, DOWN, qt)
            recorder.move(qt, 0., 2., 'transport')
            recorder.move(qp, 0., 1.8, 'place')
            recorder.move(qp, .046, 1.2, 'release')
            recorder.move(qt, .046, 1.8, 'retreat')
            recorder.move(HOME, .046, 2., 'return_for_verification')
            recorder.move(HOME, .046, .5, 'settle')
            frame, _ = recorder.capture_decision('verify')
            prediction = visual_target(perception, frame)
            row['verification_prediction'] = prediction
            verified = prediction is not None and np.max(np.abs(np.asarray(prediction['top_xyz_m'])[:2] - GOALS[scene['goal']])) < .032
            verified = bool(verified and .13 <= prediction['top_xyz_m'][2] <= .155 and sim.data.qpos[sim.gq] > .04)
            row['outcome'] = 'visually_verified' if verified else 'placement_not_verified'
            if verified:
                result['controller_success'] = True
                break
    except (ValueError, RuntimeError) as exc:
        result['reason'] = str(exc)
    recorder.stage = 'independent_score'
    for _ in range(round(1. / sim.model.opt.timestep)):
        recorder.tick()
        recorder.final_scores.append(independent_score(sim, GOALS[scene['goal']]))
    result['physical_success'] = all(s['success'] for s in recorder.final_scores) and not recorder.physics_audit['forbidden_steps']
    result['success'] = result['controller_success'] and result['physical_success']
    result['false_success'] = result['controller_success'] and not result['physical_success']
    result.update(retries=max(0, len(result['attempts']) - 1), sim_seconds=float(sim.data.time),
        wall_seconds=time.monotonic() - started, physics_tick_audit=recorder.physics_audit,
        final_score=recorder.final_scores[-1])
    recorder.save()
    write(directory / 'result.json', result)
    return result


def evaluate(args, output):
    env = GraspEnv(range(args.seed, args.seed + args.episodes), output / 'unused_resets')
    model = SAC.load(args.checkpoint, device='cpu')
    results = []
    try:
        for i in range(args.episodes):
            scene = {'seed': args.seed + i, 'goal': 'ABC'[i % 3],
                'kind': ['static', 'force', 'offset'][i % 3],
                'force_pulse': {'start_s': .5, 'duration_s': .06, 'force_world_n': [0., .8, 0.]} if i % 3 == 1 else None,
                'first_grasp_offset_m': .065 if i % 3 == 2 else 0.}
            # Rotate goal assignments every group to balance each challenge across A/B/C.
            scene['goal'] = 'ABC'[(i // 3 + i % 3) % 3]
            for controller in ('v3', 'zero', 'sac'):
                directory = output / f'episode-{i:03d}-{controller}'
                if controller == 'v3':
                    result = run_task(env.sim, env.perception, env.residual, scene['seed'], scene['goal'], directory,
                        first_grasp_offset_m=scene['first_grasp_offset_m'], force_pulse=scene['force_pulse'],
                        max_retries=2, observation_mode='feedback',
                        feedback_config=json.loads((V3 / 'feedback_config.json').read_text()))
                else:
                    result = hybrid_task(env, model, scene, directory, zero=controller == 'zero')
                item = {'controller': controller, 'seed': scene['seed'], 'goal': scene['goal'],
                    'kind': scene['kind'], 'success': result['success'], 'retries': result['retries'],
                    'false_success': result['false_success'], 'sim_seconds': result['sim_seconds'],
                    'forbidden_steps': result['physics_tick_audit']['forbidden_steps']}
                results.append(item)
                write(output / 'summary.json', {'checkpoint_sha256': digest(args.checkpoint), 'results': results})
                print(json.dumps(item), flush=True)
    finally:
        env.close()


def skill_test(args, output):
    env = LoggedGraspEnv(range(args.seed, args.seed + args.episodes), output / 'episodes')
    model = SAC.load(args.checkpoint, device='cpu') if args.checkpoint else None
    results = []
    try:
        for i in range(args.episodes):
            observation, _ = env.reset()
            total = 0.
            while True:
                action = np.zeros(4) if model is None else model.predict(observation, deterministic=True)[0]
                observation, reward, done, _, info = env.step(action)
                total += reward
                if done:
                    break
            results.append({'seed': args.seed + i, 'reward': total, **info})
            write(output / 'summary.json', {'results': results})
            print(json.dumps(results[-1]), flush=True)
    finally:
        env.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['train', 'evaluate', 'skill-test'])
    parser.add_argument('--name', required=True)
    parser.add_argument('--steps', type=int, default=8192)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--seed', type=int, default=48200000)
    parser.add_argument('--episodes', type=int, default=12)
    parser.add_argument('--checkpoint', type=Path)
    args = parser.parse_args()
    verify_v3()
    output = HERE / 'outputs' / args.name
    output.mkdir(parents=True, exist_ok=False)
    (output / 'source').mkdir()
    for source in HERE.glob('*.py'):
        shutil.copy2(source, output / 'source' / source.name)
    write(output / 'config.json', {**vars(args), 'checkpoint': str(args.checkpoint),
        'source_sha256': {p.name: digest(p) for p in (output / 'source').glob('*.py')},
        'v3_manifest_sha256': digest(V3 / 'release_manifest.json'), 'hardware_executable': False})
    {'train': train, 'evaluate': evaluate, 'skill-test': skill_test}[args.mode](args, output)
