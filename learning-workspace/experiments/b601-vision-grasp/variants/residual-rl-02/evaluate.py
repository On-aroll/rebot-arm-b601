"""Predeclared full-task paired test; never used for checkpoint selection."""
import argparse
import json
import importlib.util
from pathlib import Path
import shutil

from curriculum import HERE, PREVIOUS, CurriculumEnv, STEPS, base, write, V3
from runtime import digest, verify_v3
spec = importlib.util.spec_from_file_location('residual_rl_01_workflow', PREVIOUS / 'workflow.py')
previous_runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(previous_runner)
from task import run_task
from stable_baselines3 import SAC


class PreviousActor:
    def __init__(self):
        self.model = SAC.load(PREVIOUS / 'delivery/policy.zip', device='cpu')

    def predict(self, observation, deterministic=True):
        return self.model.predict(observation[:36], deterministic=deterministic)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--checkpoint', type=Path, default=HERE / 'outputs/long-r2-hour/best_model.zip')
    parser.add_argument('--output', default='formal-r2')
    parser.add_argument('--controllers', nargs='+', default=['v3', 'zero', 'previous', 'sac'])
    parser.add_argument('--start', type=int, default=0)
    parser.add_argument('--count', type=int, default=36)
    parser.add_argument('--suite', choices=['broad', 'boundary'], default='broad')
    args = parser.parse_args()
    verify_v3()
    output = HERE / 'outputs' / args.output
    output.mkdir(parents=True, exist_ok=False)
    (output / 'source').mkdir()
    for source in [Path(__file__), HERE / 'curriculum.py', PREVIOUS / 'workflow.py', PREVIOUS / 'environment.py']:
        shutil.copy2(source, output / 'source' / (source.parent.name + '-' + source.name))
    seed_start = 53000000 if args.suite == 'broad' else 54000000
    config = {'suite': args.suite, 'test_seed_start': seed_start, 'indices': list(range(args.start, args.start + args.count)),
        'controllers': args.controllers, 'checkpoint_sha256': digest(args.checkpoint) if 'sac' in args.controllers else None,
        'source_sha256': {str(p.relative_to(HERE.parent)): digest(p) for p in
                          [Path(__file__), HERE / 'curriculum.py', PREVIOUS / 'workflow.py', PREVIOUS / 'environment.py']},
        'kinds': ['nominal', 'force', 'bias_positive', 'bias_negative'] if args.suite == 'broad' else ['bias_positive', 'bias_negative'],
        'bias_m': .035 if args.suite == 'broad' else None,
        'boundary_magnitudes_m': [.027, .028, .029] if args.suite == 'boundary' else None,
        'boundary_magnitude_index_rule': '(i//2+i//6)%3' if args.suite == 'boundary' else None,
        'retries_max': 2,
        'previous_controller_note': 'R1 actor with R2 five-step open alignment, not unchanged R1 system'}
    write(output / 'config.json', config)
    previous_runner.STEPS = STEPS
    env = CurriculumEnv([0], output / 'unused_resets', log_episodes=False)
    model = SAC.load(args.checkpoint, device='cpu') if 'sac' in args.controllers else None
    old = PreviousActor() if 'previous' in args.controllers else None
    results = []
    try:
        for index in range(args.start, args.start + args.count):
            kind = config['kinds'][index % len(config['kinds'])]
            level = 0 if kind == 'nominal' else 1 if kind == 'force' else 2
            scene = {'seed': 53000000 + index, 'goal': 'ABC'[index % 3], 'kind': kind,
                'force_pulse': {'start_s': .5, 'duration_s': .06, 'force_world_n': [0., .8, 0.]} if kind == 'force' else None,
                'first_grasp_offset_m': .035 if kind == 'bias_positive' else -.035 if kind == 'bias_negative' else 0.}
            if args.suite == 'boundary':
                kind = 'bias_positive' if index % 2 == 0 else 'bias_negative'
                magnitude = [.027, .028, .029][(index // 2 + index // 6) % 3]
                level = 0
                scene.update(seed=seed_start + index, kind=kind, force_pulse=None,
                             first_grasp_offset_m=magnitude * (1 if index % 2 == 0 else -1))
            for controller in args.controllers:
                env.sim.stress_level, env.sim.profile_salt = level, 930000000
                directory = output / f'episode-{index:03d}-{controller}'
                if controller == 'v3':
                    result = run_task(env.sim, env.perception, env.residual, scene['seed'], scene['goal'], directory,
                        first_grasp_offset_m=scene['first_grasp_offset_m'], force_pulse=scene['force_pulse'],
                        max_retries=2, observation_mode='feedback',
                        feedback_config=json.loads((V3 / 'feedback_config.json').read_text()))
                else:
                    result = previous_runner.hybrid_task(env, old if controller == 'previous' else model,
                        scene, directory, zero=controller == 'zero')
                row = {'controller': controller, 'seed': scene['seed'], 'goal': scene['goal'], 'kind': kind,
                    'success': result['success'], 'retries': result['retries'],
                    'false_success': result['false_success'], 'forbidden_steps': result['physics_tick_audit']['forbidden_steps'],
                    'sim_seconds': result['sim_seconds'], 'first_outcome': result['attempts'][0].get('outcome')}
                results.append(row)
                write(output / 'summary.json', results)
                print(json.dumps(row), flush=True)
    finally:
        env.close()
