"""Long-run curriculum with sensor-derived alignment features, reusing RL-01."""
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PREVIOUS = HERE.parent / 'residual-rl-01'
sys.path.insert(0, str(PREVIOUS))
import environment as base
from runtime import RealisticSimulation, models, write, mj, V3
from task import collect_feedback, correct_prediction, grasp_geometry
from vision import estimate
from policy import valid_prediction
from episode import DOWN
from sim import HOME
import gymnasium as gym
import numpy as np
import torch

STEPS = 23  # five open-jaw alignment decisions, then the original 18-step skill


class StressSimulation(RealisticSimulation):
    def __init__(self):
        self.stress_level, self.profile_salt = 0, 0
        super().__init__()

    def reset(self, seed, **kwargs):
        meta = super().reset(seed, **kwargs)
        if not self.realistic or self.stress_level == 0:
            return meta
        rng = np.random.default_rng(int(seed) + self.profile_salt)
        hard = self.stress_level == 2
        friction = float(rng.uniform(.25 if hard else .5, 1.4))
        for geom in range(self.model.ngeom):
            if int(self.model.geom_bodyid[geom]) in self.fingers:
                self.model.geom_friction[geom, 0] = friction
        mass = float(rng.uniform(.025, .12 if hard else .10))
        self.model.body_mass[self.obj] = mass
        self.model.body_inertia[self.obj] = (2 / 3) * mass * self.half_size ** 2
        self.jaw_force = float(rng.uniform(15. if hard else 20., 50.))
        self.model.actuator_ctrlrange[self.ga] = [-self.jaw_force, self.jaw_force]
        self.depth_sigma = float(rng.uniform(.0005, .003 if hard else .002))
        self.delay = int(rng.integers(10, 41 if hard else 31))
        self.pending = [(self.target_q.copy(), self.grip)] * self.delay
        self.model.cam_pos[self.cam] += rng.uniform(-.003 if hard else -.001, .003 if hard else .001, 3)
        saved = self.data.qpos.copy()
        mj.mj_setConst(self.model, self.data)
        self.data.qpos[:] = saved
        mj.mj_forward(self.model, self.data)
        self.profile.update(stress_level=self.stress_level, profile_salt=self.profile_salt,
            finger_friction=friction, object_mass_kg=mass, jaw_force_limit_n=self.jaw_force,
            depth_sigma_m=self.depth_sigma, command_delay_s=self.delay * self.model.opt.timestep,
            camera_position_m=self.model.cam_pos[self.cam].tolist(), calibration_status='synthetic_unmeasured_stress')
        meta['realism'] = self.profile
        return meta


def prepare(sim, perception, residual, recorder, bias_xy):
    frame, _ = recorder.capture_decision('initial')
    initial = correct_prediction(estimate(perception, frame), residual)
    if not valid_prediction(initial):
        raise ValueError('no_confident_initial_target')
    feedback = collect_feedback(sim, recorder, perception, residual, initial,
                               json.loads((V3 / 'feedback_config.json').read_text()))
    if not feedback['accepted']:
        raise ValueError('reobservation_rejected')
    top = np.asarray(feedback['selected_prediction']['top_xyz_m']).copy()
    top[:2] += bias_xy
    center, grasp = grasp_geometry(top)
    qa = sim.ik(grasp + [0., 0., .12], DOWN, HOME)
    qg = sim.ik(grasp, DOWN, qa)
    recorder.move(qa, .046, 2., 'approach')
    recorder.move(qg, .046, 1.8, 'descend')
    return center, grasp


class CurriculumEnv(base.GraspEnv):
    def __init__(self, seeds, directory, validation=False, fixed_bias=None, fixed_level=None, log_episodes=True):
        gym.Env.__init__(self)
        torch.set_num_threads(1)
        self.sim = StressSimulation()
        self.perception, self.residual = models()
        self.seeds, self.root = list(seeds), Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        self.counter, self.case_cursor, self.phase = 0, 0, 0
        self.validation, self.fixed_bias, self.fixed_level = validation, fixed_bias, fixed_level
        self.log_episodes = log_episodes
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (39,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1., 1., (4,), dtype=np.float32)

    def set_phase(self, phase):
        self.phase = int(phase)

    def rewind_cases(self):
        self.case_cursor = 0

    def observation(self):
        frame = self.sim.observe(labels=False)
        predicted = correct_prediction(estimate(self.perception, frame), self.residual)
        visible = valid_prediction(predicted)
        if visible:
            self.last_visual = base.features(predicted)
        original = np.r_[self.last_visual, float(visible), base.state(self.sim),
                         np.eye(3)['ABC'.index(self.goal)], self.delta / base.BOUNDS]
        if self.log_episodes:
            self.actor_frames.append((frame['rgb'].copy(), frame['depth'].copy()))
        top = self.last_visual[:3] * [.1, .15, .03] + [.33, 0., .14]
        _, visual_grasp = grasp_geometry(top)
        # FK of the robot's own TCP and a camera-derived target, never object truth.
        error = (visual_grasp - self.sim.data.site_xpos[self.sim.tip]) / [.04, .04, .16]
        return np.r_[original, np.clip(error, -3., 3.)].astype(np.float32)

    def reset(self, *, seed=None, options=None):
        gym.Env.reset(self, seed=seed)
        index = self.case_cursor % len(self.seeds)
        scene_seed = self.seeds[index]
        self.case_cursor += 1
        self.directory = self.root / f'episode-{self.counter:06d}'
        self.directory.mkdir(exist_ok=False)
        self.actor_frames = []
        salt = 910000000 if self.validation else 710000000 + (self.counter // len(self.seeds)) * 1000003
        rng = np.random.default_rng(scene_seed + salt)
        level = index % 3 if self.validation else self.phase
        rehearsal = (index % 3 == 0) if self.validation else rng.random() < .25
        level = 0 if rehearsal else level
        bound = [0.005, .020, .035][level]
        angle = float(rng.uniform(-np.pi, np.pi))
        radius = float(rng.uniform(.5 * bound, bound))
        bias = np.zeros(2) if rehearsal else radius * np.array([np.cos(angle), np.sin(angle)])
        if self.fixed_bias is not None:
            bias = np.asarray(self.fixed_bias, dtype=float)
        if self.fixed_level is not None:
            level = self.fixed_level
        self.sim.stress_level, self.sim.profile_salt = level, salt
        self.counter += 1
        self.goal = 'ABC'[(index // 3 + index % 3) % 3]
        self.reset_metadata = self.sim.reset(scene_seed)
        recorder = base.CompactRecorder(self.sim, self.directory)
        try:
            _, grasp = prepare(self.sim, self.perception, self.residual, recorder, bias)
            observation = self.begin(recorder, self.goal, grasp)
        except ValueError as exc:
            observation = self.begin(recorder, self.goal, self.sim.data.site_xpos[self.sim.tip].copy())
            self.failed = str(exc)
        self.current_observation = observation
        self.episode_level = level
        write(self.directory / 'reset.json', {'reset': self.reset_metadata, 'goal': self.goal,
            'curriculum_level': level, 'rehearsal': rehearsal, 'initial_grasp_bias_xy_m_scoring_only': bias.tolist(),
            'preparation_error': self.failed, 'grasp_base_xyz_m': self.grasp.tolist()})
        return observation, {}

    @staticmethod
    def potential(observation):
        error_xy = observation[36:38] * .04
        return -4. * min(float(np.linalg.norm(error_xy)) / .035, 2.)

    def step(self, action):
        previous = self.current_observation.copy()
        action = np.asarray(action, dtype=float)
        old_delta = self.delta.copy()
        self.delta += np.clip(action * base.BOUNDS - self.delta, [-.002] * 3 + [-.0015], [.002] * 3 + [.0015])
        t = self.index - 5
        u = np.clip((t - 4) / 10., 0., 1.)
        ease = 10 * u**3 - 15 * u**4 + 6 * u**5
        base_grip = .046 if t < 0 else .046 * (1 - min(1., (t + 1) / 5.))
        start = float(self.sim.data.time)
        try:
            if self.failed:
                raise ValueError(self.failed)
            q = self.sim.ik(self.grasp + self.delta[:3] + [0., 0., .12 * ease], DOWN, self.sim.target_q)
            self.recorder.move(q, float(np.clip(base_grip + self.delta[3], 0., .05)), .2, 'rl_grasp_lift')
        except (ValueError, RuntimeError) as exc:
            self.failed = str(exc)
        self.index += 1
        contacts = self.sim.contacts()
        height = float(self.sim.data.xpos[self.sim.obj, 2] - self.initial_object_z)
        held = contacts['left'] and contacts['right'] and height > .08
        self.held_s = self.held_s + float(self.sim.data.time) - start if held else 0.
        forbidden = self.recorder.physics_audit['forbidden_steps'] > 0
        done = self.index >= STEPS or self.failed is not None or forbidden
        success = bool(self.held_s >= .4 and not forbidden and not self.failed)
        reward = .5 * float(np.clip(height / .12, 0., 1.)) + .5 * float(held)
        reward -= .01 * float(np.square(self.delta / base.BOUNDS).mean())
        reward -= .02 * float(np.square((self.delta - old_delta) / base.BOUNDS).mean())
        if done:
            reward += 10. if success else -5.
            if forbidden or self.failed:
                reward -= 10.
        observation = self.observation()
        self.steps.append({'index': self.index - 1, 'observation': previous.tolist(),
            'action': action.tolist(), 'applied_residual_m': self.delta.tolist(),
            'reward': reward, 'held_s_for_scoring_only': self.held_s,
            'lift_m_for_scoring_only': height, 'done': bool(done)})
        self.current_observation = observation
        info = {'is_success': success, 'skill_success': success,
            'forbidden_steps': self.recorder.physics_audit['forbidden_steps']}
        if done:
            info['failure_reason'] = self.failed
        shaping = (.98 * self.potential(observation) if not done else 0.) - self.potential(previous)
        reward += shaping
        self.steps[-1].update(reward=reward, potential_shaping=shaping)
        info['curriculum_level'] = getattr(self, 'episode_level', 0)
        if done and self.log_episodes:
            self.save_episode(info)
        return observation, float(reward), bool(done), False, info

    def save_episode(self, info):
        super().save_episode(info)
        # Keep exact actor RGB-D on failures and all validation episodes. Successful
        # training retains input features/actions plus initial/feedback RGB-D.
        if self.validation or not info.get('skill_success', False):
            self.save_actor_frames()

    def save_actor_frames(self):
        if self.actor_frames:
            np.savez_compressed(self.directory / 'actor_rgbd.npz',
                rgb=np.stack([f[0] for f in self.actor_frames]),
                depth=np.stack([f[1] for f in self.actor_frames]))

    def close(self):
        if self.counter and not (self.directory / 'result.json').exists():
            self.recorder.save()
            write(self.directory / 'rl_steps.json', self.steps)
            write(self.directory / 'partial_result.json', {'status': 'unfinished_at_shutdown', 'decisions': len(self.steps)})
            self.save_actor_frames()
        super().close()
