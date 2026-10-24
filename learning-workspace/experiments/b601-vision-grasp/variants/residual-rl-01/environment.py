"""A bounded grasp/lift skill: frozen V3 provides vision, IK and motion limits."""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'act-01'))
from runtime import V3, RealisticSimulation, models, state, write
from task import Recorder, collect_feedback, grasp_geometry, correct_prediction, GOALS
from policy import features, valid_prediction
from vision import estimate
from episode import DOWN
from sim import HOME

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import torch

BOUNDS = np.array([.012, .012, .008, .006])
STEPS = 18


class CompactRecorder(Recorder):
    """Training retains 20Hz commands/truth and decision RGB-D, not a full video."""
    def __init__(self, sim, directory, video=False):
        super().__init__(sim, directory, video)
        self.next_frame = float('inf')


def prepare(sim, perception, residual, recorder, first_offset=0.):
    frame, _ = recorder.capture_decision('initial')
    initial = correct_prediction(estimate(perception, frame), residual)
    if not valid_prediction(initial):
        raise ValueError('no_confident_initial_target')
    feedback = collect_feedback(sim, recorder, perception, residual, initial,
                               json.loads((V3 / 'feedback_config.json').read_text()))
    if not feedback['accepted']:
        raise ValueError('reobservation_rejected')
    top = np.asarray(feedback['selected_prediction']['top_xyz_m']).copy()
    top[1] += first_offset
    center, grasp = grasp_geometry(top)
    qa = sim.ik(grasp + [0., 0., .12], DOWN, HOME)
    qg = sim.ik(grasp, DOWN, qa)
    recorder.move(qa, .046, 2., 'approach')
    recorder.move(qg, .046, 1.8, 'descend')
    return center, grasp


class GraspEnv(gym.Env):
    """Actor sees image-derived features, own feedback and requested region only."""
    def __init__(self, seeds, directory):
        super().__init__()
        torch.set_num_threads(1)
        self.sim = RealisticSimulation()
        self.perception, self.residual = models()
        self.seeds = list(seeds)
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        self.counter = 0
        self.observation_space = spaces.Box(-np.inf, np.inf, (36,), dtype=np.float32)
        self.action_space = spaces.Box(-1., 1., (4,), dtype=np.float32)

    def begin(self, recorder, goal, grasp):
        self.recorder, self.goal, self.grasp = recorder, goal, np.asarray(grasp)
        self.delta = np.zeros(4)
        self.index, self.held_s = 0, 0.
        self.last_visual = np.zeros(8, dtype=np.float32)
        self.steps = []
        # These variables belong to the reward scorer and never enter observation().
        self.initial_object_z = float(self.sim.data.xpos[self.sim.obj, 2])
        self.failed = None
        return self.observation()

    def observation(self):
        frame = self.sim.observe(labels=False)
        predicted = correct_prediction(estimate(self.perception, frame), self.residual)
        visible = valid_prediction(predicted)
        if visible:
            self.last_visual = features(predicted)
        return np.r_[self.last_visual, float(visible), state(self.sim),
                     np.eye(3)['ABC'.index(self.goal)], self.delta / BOUNDS].astype(np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        scene_seed = self.seeds[self.counter % len(self.seeds)]
        self.goal = 'ABC'[scene_seed % 3]
        self.directory = self.root / f'episode-{self.counter:05d}'
        self.directory.mkdir(exist_ok=False)
        self.counter += 1
        self.reset_metadata = self.sim.reset(scene_seed)
        recorder = CompactRecorder(self.sim, self.directory)
        try:
            _, grasp = prepare(self.sim, self.perception, self.residual, recorder)
            observation = self.begin(recorder, self.goal, grasp)
        except ValueError as exc:
            observation = self.begin(recorder, self.goal, self.sim.data.site_xpos[self.sim.tip].copy())
            self.failed = str(exc)
        write(self.directory / 'reset.json', {'reset': self.reset_metadata, 'goal': self.goal,
            'grasp_base_xyz_m': self.grasp.tolist(), 'preparation_error': self.failed})
        self.current_observation = observation
        return observation, {}

    def step(self, action):
        action = np.asarray(action, dtype=float)
        previous = self.delta.copy()
        self.delta += np.clip(action * BOUNDS - self.delta, [-.002] * 3 + [-.0015], [.002] * 3 + [.0015])
        t = self.index
        # V3 still supplies close/lift timing. RL learns only bounded position/jaw offsets.
        u = np.clip((t - 4) / 10., 0., 1.)
        ease = 10 * u**3 - 15 * u**4 + 6 * u**5
        base_grip = .046 * (1 - min(1., (t + 1) / 5.))
        start = float(self.sim.data.time)
        try:
            if self.failed:
                raise ValueError(self.failed)
            q = self.sim.ik(self.grasp + self.delta[:3] + [0., 0., .12 * ease], DOWN, self.sim.target_q)
            self.recorder.move(q, float(np.clip(base_grip + self.delta[3], 0., .05)), .2, 'rl_grasp_lift')
        except (ValueError, RuntimeError) as exc:
            self.failed = str(exc)
        self.index += 1
        # Reward uses physical evidence. SAC receives it as reward, never as actor features.
        contacts = self.sim.contacts()
        height = float(self.sim.data.xpos[self.sim.obj, 2] - self.initial_object_z)
        held = contacts['left'] and contacts['right'] and height > .08
        self.held_s = self.held_s + float(self.sim.data.time) - start if held else 0.
        forbidden = self.recorder.physics_audit['forbidden_steps'] > 0
        done = self.index >= STEPS or self.failed is not None or forbidden
        success = bool(self.held_s >= .4 and not forbidden and not self.failed)
        reward = .5 * float(np.clip(height / .12, 0., 1.))
        reward += .5 * float(held) - .01 * float(np.square(self.delta / BOUNDS).mean())
        reward -= .02 * float(np.square((self.delta - previous) / BOUNDS).mean())
        if done:
            reward += 10. if success else -5.
            if forbidden or self.failed:
                reward -= 10.
        observation = self.observation()
        self.steps.append({'index': t, 'observation': self.current_observation.tolist(),
            'action': action.tolist(), 'applied_residual_m': self.delta.tolist(),
            'reward': reward, 'held_s_for_scoring_only': self.held_s,
            'lift_m_for_scoring_only': height, 'done': done})
        self.current_observation = observation
        info = {'is_success': success, 'skill_success': success, 'forbidden_steps': self.recorder.physics_audit['forbidden_steps']}
        if done:
            info['failure_reason'] = self.failed
        return observation, float(reward), bool(done), False, info

    def save_episode(self, info):
        self.recorder.save()
        write(self.directory / 'rl_steps.json', self.steps)
        write(self.directory / 'result.json', {**info, 'goal': self.goal,
            'scope': 'grasp/lift skill only, not complete placement',
            'decisions': len(self.steps), 'sim_seconds': float(self.sim.data.time)})

    def close(self):
        self.sim.close()


class LoggedGraspEnv(GraspEnv):
    def step(self, action):
        observation, reward, done, truncated, info = super().step(action)
        if done:
            self.save_episode(info)
        return observation, reward, done, truncated, info

    def close(self):
        if self.counter and not (self.directory / 'result.json').exists():
            self.recorder.save()
            write(self.directory / 'rl_steps.json', self.steps)
            write(self.directory / 'partial_result.json', {'status': 'unfinished_at_shutdown',
                'decisions': len(self.steps), 'counts_as_completed_episode': False})
        super().close()
