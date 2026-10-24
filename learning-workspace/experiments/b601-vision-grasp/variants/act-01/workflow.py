"""Collect, train, evaluate and benchmark a simulation-only action policy."""
import argparse
import json
from pathlib import Path
import shutil
import time

from runtime import ROOT, V3, RealisticSimulation, models, state, verify_v3, write, digest, mj
import numpy as np
import torch
from PIL import Image
from task import Recorder, GOALS, run_task, independent_score, visual_target
from sim import HOME
from chunk_policy import Inference, train

OUTPUTS = ROOT / 'outputs'


def scenario(seed, index):
    kind = index % 3
    return {'seed': seed, 'goal': 'ABC'[(index // 3 + index % 3) % 3],
            'kind': ['static', 'force', 'offset'][kind],
            'first_grasp_offset_m': .065 if kind == 2 else 0.,
            'force_pulse': {'start_s': .5, 'duration_s': .06,
                            'force_world_n': [0., .8, 0.]} if kind == 1 else None}


def teacher(sim, perception, residual, scene, directory, video=False):
    return run_task(sim, perception, residual, scene['seed'], scene['goal'], directory,
        video=video, first_grasp_offset_m=scene['first_grasp_offset_m'],
        absent=scene.get('absent', False),
        force_pulse=scene['force_pulse'], max_retries=2, observation_mode='feedback',
        feedback_config=json.loads((V3 / 'feedback_config.json').read_text()))


def imitation(directory, scene, result):
    """Match current observation to NEXT command, never feedback-as-action."""
    import cv2
    directory = Path(directory)
    rows = [json.loads(line) for line in (directory / 'trajectory.jsonl').read_text().splitlines()]
    if not rows:
        return
    with np.load(directory / 'observations.npz', allow_pickle=False) as data:
        cameras = json.loads(str(data['camera_json']))
        rgb = np.stack([cv2.resize(x, (80, 60), interpolation=cv2.INTER_AREA) for x in data['rgb']])
        depth = np.stack([cv2.resize(x, (80, 60), interpolation=cv2.INTER_NEAREST) for x in data['depth_m']]).astype('float16')
    # Frames share a tick with rows; each label is the commanded target 0.2s later.
    indices = [i for i, r in enumerate(rows) if r['observation_index'] is not None]
    states, actions, eligible = [], [], []
    recovery_start = 0.
    if result['success'] and result['retries']:
        # Drop the deliberately wrong grasp, keep reopening and return/reobserve.
        recovery_start = next((r['t_sim_s'] for r in rows if r['stage'] == 'reopen'), float('inf'))
    for i in indices:
        r = rows[i]
        states.append(r['q_feedback_rad'] + r['qd_feedback_rad_s'] +
            [r['gripper_feedback_m'] / .05] + r['q_command_rad'] + [r['gripper_command_m'] / .05])
        future = rows[min(i + 4, len(rows) - 1)]
        actions.append(future['q_command_rad'] + [future['gripper_command_m']])
        eligible.append(result['success'] and r['t_sim_s'] >= recovery_start and
                        r['stage'] != 'independent_score' and future['stage'] != 'independent_score')
    assert len(indices) == len(rgb)
    np.savez_compressed(directory / 'imitation.npz', rgb=rgb, depth=depth,
        state=np.asarray(states, 'float32'), action=np.asarray(actions, 'float32'),
        goal=np.repeat(np.eye(3, dtype='float32')['ABC'.index(scene['goal'])][None], len(rgb), 0),
        eligible=np.asarray(eligible), timestamp_s=np.asarray([c['simulation_time_s'] for c in cameras]))
    write(directory / 'imitation_contract.json', {'fps': 5, 'action_delay_s': .2,
        'state': 'q6, qd6, jaw_feedback/0.05, last_command_q6, last_command_jaw/0.05',
        'action': 'next_0.2s_joint_target_rad6_and_single_finger_m',
        'goal': scene['goal'], 'seed': scene['seed'], 'eligible_frames': sum(eligible),
        'truth_in_policy_inputs': False, 'hardware_executable': False})


def freeze(directory, args):
    directory.mkdir(parents=True, exist_ok=False)
    source = directory / 'source'
    source.mkdir()
    for path in ROOT.glob('*.py'):
        shutil.copy2(path, source / path.name)
    write(directory / 'config.json', vars(args))
    write(directory / 'provenance.json', {'v3_manifest_sha256': digest(V3 / 'release_manifest.json'),
        'source_sha256': {p.name: digest(p) for p in source.iterdir()},
        'torch': torch.__version__, 'mujoco': mj.__version__,
        'render_backend': __import__('os').environ['MUJOCO_GL'], 'hardware_executable': False})


def collect(args):
    directory = OUTPUTS / args.name
    freeze(directory, args)
    warm = json.loads(Path(args.warm_start).read_text()) if args.warm_start else None
    sim = RealisticSimulation(realistic=not args.ideal, warm_starts=warm)
    perception, residual = models()
    results = []
    try:
        for i in range(args.episodes):
            scene = scenario(args.seed + i, i)
            result = teacher(sim, perception, residual, scene, directory / f'episode-{i:03d}', video=args.video and i == 0)
            imitation(directory / f'episode-{i:03d}', scene, result)
            results.append({'scene': scene, 'success': result['success'], 'retries': result['retries'],
                            'reason': result['reason'], 'wall_s': result['wall_seconds']})
            write(directory / 'summary.json', results)
            print(json.dumps(results[-1]), flush=True)
    finally:
        sim.close()


def run_act(sim, perception, policy, scene, directory, video=False):
    """ACT owns every normal control target; supervisor only verifies and retries."""
    directory.mkdir(parents=True, exist_ok=False)
    reset = sim.reset(scene['seed'], force_pulse=scene['force_pulse'], absent=scene.get('absent', False))
    recorder = Recorder(sim, directory, video)
    result = {'controller': 'small_act', 'hardware_executable': False, 'reset': reset,
              'scene': scene, 'success': False, 'controller_success': False, 'physical_success': False,
              'false_success': False, 'attempts': [], 'normal_action_calls': 0,
              'model_inference_calls': 0, 'chunk_execute_steps': getattr(policy, 'n_action_steps', 1),
              'safety_clipped_calls': 0, 'supervisor_recovery_moves': 0}
    started = time.monotonic()
    try:
        if visual_target(perception, sim.observe(labels=False)) is None:
            result['reason'] = 'no_confident_target'
            return result
        for attempt in range(3):
            attempt_start_s = float(sim.data.time)
            record = {'attempt': attempt + 1, 'outcome': 'timeout'}
            result['attempts'].append(record)
            closed_s = 0.
            injected = False
            pending_actions = []
            for step in range(450):  # 90 seconds maximum per attempt
                if not pending_actions:
                    frame = sim.observe(labels=False)
                    chunk = policy.predict(frame, state(sim), scene['goal'], float(sim.data.time) - attempt_start_s)
                    pending_actions = list(chunk[:getattr(policy, 'n_action_steps', 1)])
                    result['model_inference_calls'] += 1
                proposed = pending_actions.pop(0)
                target = proposed.copy()
                q0 = sim.target_q.copy()
                target[:6] = np.clip(target[:6], np.maximum(sim.lower, q0 - .08),
                                    np.minimum(sim.upper, q0 + .08))
                target[6] = np.clip(target[6], max(0., sim.grip - .008), min(.05, sim.grip + .008))
                # Same challenge: physically approach a biased grasp at the first close.
                # Uses proprioception and the ACT command, never object pose.
                if (attempt == 0 and not injected and scene['first_grasp_offset_m'] and
                        target[6] < .02 and sim.grip >= .02):
                    from episode import DOWN
                    xyz = sim.data.site_xpos[sim.tip].copy()
                    xyz[1] += scene['first_grasp_offset_m']
                    target[:6] = sim.ik(xyz, DOWN, q0)
                    # Enact the same 65mm error through normal limited interpolation.
                    recorder.move(target[:6], .046, 1., 'challenge_offset')
                    injected = True
                    pending_actions.clear()
                result['safety_clipped_calls'] += int(not np.allclose(target, proposed))
                result['normal_action_calls'] += 1
                recorder.stage = f'act_attempt_{attempt + 1}'
                sim.target_q, sim.grip = target[:6], float(target[6])
                for _ in range(round(.2 / sim.model.opt.timestep)):
                    recorder.tick()
                closed_s = closed_s + .2 if sim.grip < .006 else 0.
                # Empty jaws after a lifted closing attempt trigger bounded recovery.
                if closed_s >= 1. and sim.data.qpos[sim.gq] < .007 and sim.data.site_xpos[sim.tip, 2] > .20:
                    record['outcome'] = 'empty_grasp'
                    break
                if step % 5 == 0 and sim.grip > .04 and np.max(np.abs(sim.data.qpos[sim.qa] - HOME)) < .13:
                    prediction = visual_target(perception, sim.observe(labels=False))
                    if prediction is not None:
                        top = np.asarray(prediction['top_xyz_m'])
                        if np.max(np.abs(top[:2] - GOALS[scene['goal']])) < .032 and .13 <= top[2] <= .155:
                            result['controller_success'] = True
                            record['outcome'] = 'visually_verified'
                            break
            if result['controller_success']:
                break
            if attempt < 2:
                recorder.move(sim.target_q, .046, 1., 'supervisor_reopen')
                recorder.move(HOME, .046, 3., 'supervisor_return')
                result['supervisor_recovery_moves'] += 2
        recorder.stage = 'independent_score'
        for _ in range(round(1. / sim.model.opt.timestep)):
            recorder.tick()
            recorder.final_scores.append(independent_score(sim, GOALS[scene['goal']]))
        result['physical_success'] = all(s['success'] for s in recorder.final_scores) and not recorder.physics_audit['forbidden_steps']
        result['success'] = result['physical_success'] and result['controller_success']
        result['false_success'] = result['controller_success'] and not result['physical_success']
        result['reason'] = result['attempts'][-1]['outcome']
    except (ValueError, RuntimeError) as exc:
        result['reason'] = str(exc)
    finally:
        result.update(retries=max(0, len(result['attempts']) - 1), sim_seconds=float(sim.data.time),
            wall_seconds=time.monotonic() - started, physics_tick_audit=recorder.physics_audit)
        recorder.save()
        write(directory / 'result.json', result)
    return result


def evaluate(args):
    directory = OUTPUTS / args.name
    freeze(directory, args)
    policy = Inference(args.checkpoint, 'cuda' if torch.cuda.is_available() else 'cpu')
    policy.n_action_steps = args.n_action_steps
    shutil.copy2(args.checkpoint, directory / 'policy.pt')
    scenes = [scenario(args.seed + i, args.index_offset + i) for i in range(args.episodes)]
    if args.absent:
        for scene in scenes:
            scene.update(absent=True, kind='empty', first_grasp_offset_m=0., force_pulse=None)
    write(directory / 'frozen_scenes.json', scenes)
    sim = RealisticSimulation(realistic=not args.ideal)
    perception, residual = models()
    torch.set_num_threads(1)
    results = []
    try:
        for i, scene in enumerate(scenes):
            for mode in ('v3', 'act'):
                episode = directory / f'{mode}-{i:03d}'
                if mode == 'v3':
                    r = teacher(sim, perception, residual, scene, episode, args.video and i == 0)
                else:
                    r = run_act(sim, perception, policy, scene, episode, args.video and i == 0)
                row = {k: r.get(k) for k in ('success', 'controller_success', 'physical_success',
                    'false_success', 'retries', 'sim_seconds', 'wall_seconds', 'reason', 'normal_action_calls',
                    'supervisor_recovery_moves', 'safety_clipped_calls')}
                row.update(mode=mode, scene=scene, forbidden_steps=r['physics_tick_audit']['forbidden_steps'])
                results.append(row)
                write(directory / 'results.json', results)
                print(json.dumps(row), flush=True)
        write(directory / 'summary.json', {mode: {
            'trials': sum(r['mode'] == mode for r in results),
            'successes': sum(r['mode'] == mode and r['success'] for r in results),
            'first_attempt_successes': sum(r['mode'] == mode and r['success'] and not r['retries'] for r in results),
            'recovered_successes': sum(r['mode'] == mode and r['success'] and bool(r['retries']) for r in results),
            'false_successes': sum(r['mode'] == mode and r['false_success'] for r in results),
            'forbidden_steps': sum(r['forbidden_steps'] for r in results if r['mode'] == mode)} for mode in ('v3', 'act')})
    finally:
        sim.close()


def benchmark(args):
    torch.set_num_threads(2)
    policy = Inference(args.checkpoint, 'cpu')
    sim = RealisticSimulation()
    sim.reset(43000999)
    frame = sim.observe()
    own = state(sim)
    for _ in range(10):
        policy.predict(frame, own, 'B')
    policy.latencies.clear()
    for _ in range(200):
        policy.predict(frame, own, 'B')
    result = {'device': 'host_x86_CPU_2_threads', 'samples': 200,
        'median_ms': float(np.median(policy.latencies)), 'p95_ms': float(np.quantile(policy.latencies, .95)),
        'parameters': sum(p.numel() for p in policy.model.parameters()),
        'checkpoint_bytes': Path(args.checkpoint).stat().st_size,
        'board_measured': False, 'input_includes': 'resize_and_normalization',
        'excludes': 'camera_driver, transport, hardware_executor'}
    sim.close()
    write(Path(args.checkpoint).parent / 'cpu_benchmark.json', result)
    print(json.dumps(result), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['collect', 'train', 'evaluate', 'benchmark'])
    parser.add_argument('--name', default='smoke')
    parser.add_argument('--episodes', type=int, default=30)
    parser.add_argument('--seed', type=int, default=43000000)
    parser.add_argument('--index-offset', type=int, default=0)
    parser.add_argument('--ideal', action='store_true')
    parser.add_argument('--video', action='store_true')
    parser.add_argument('--data', nargs='+')
    parser.add_argument('--validation')
    parser.add_argument('--steps', type=int, default=4000)
    parser.add_argument('--checkpoint')
    parser.add_argument('--warm-start', help='Development-derived initial arm poses, collection only')
    parser.add_argument('--absent', action='store_true', help='Empty-scene evaluation control')
    parser.add_argument('--n-action-steps', type=int, choices=range(1, 11), default=1)
    args = parser.parse_args()
    verify_v3()
    if args.command == 'train':
        train(args.data, args.validation, OUTPUTS / args.name, args.steps)
    else:
        globals()[args.command](args)


if __name__ == '__main__':
    main()
