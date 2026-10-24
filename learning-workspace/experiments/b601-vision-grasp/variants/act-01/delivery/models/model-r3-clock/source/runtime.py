"""Reuse frozen V3; additions are simulation-only and never import a device driver."""
import hashlib
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
EXPERIMENT = ROOT.parents[1]
V3 = EXPERIMENT / 'versions/V3'
os.environ.setdefault('MUJOCO_GL', 'glfw' if os.name == 'nt' else 'egl')
os.environ.setdefault('OMP_NUM_THREADS', '1')
sys.path.insert(0, str(V3))

import numpy as np
import torch
from sim import Simulation, mj
from task_runner import perception_model
from policy import load


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_v3():
    manifest = V3 / 'release_manifest.json'
    assert digest(manifest) == 'bb6bebf40eb2b660f66600c5be2b41b7e13bee6529bcc042e247a363bcb1342b'
    for name, expected in json.loads(manifest.read_text())['files_sha256'].items():
        assert digest(V3 / name) == expected, name


def models():
    return perception_model(V3 / 'models/perception.pt'), load(V3 / 'models/residual.pt')


class RealisticSimulation(Simulation):
    """Uncalibrated sensitivity envelope, not a measured model of the real arm."""
    def __init__(self, realistic=True, warm_starts=None):
        self.warm_starts = warm_starts
        self.realistic = False
        super().__init__(V3 / 'assets/task_scene.xml')
        self.base_damping = self.model.dof_damping.copy()
        self.base_camera = self.model.cam_pos[self.cam].copy()
        self.realistic = realistic

    def reset(self, seed, **kwargs):
        meta = super().reset(seed, **kwargs)
        self.profile = {}
        if not self.realistic:
            return meta
        rng = np.random.default_rng(int(seed) + 310000000)
        m, d = self.model, self.data
        friction = float(rng.uniform(.65, 1.4))
        torsion = float(rng.uniform(.002, .008))
        for geom in range(m.ngeom):
            if int(m.geom_bodyid[geom]) in self.fingers:
                m.geom_friction[geom] = [friction, torsion, .0002]
                m.geom_solref[geom] = [.006, 1.]
        m.geom_friction[self.og] = [rng.uniform(.5, 1.3), .004, .0002]
        m.body_mass[self.obj] = rng.uniform(.025, .09)
        m.body_inertia[self.obj] = (2 / 3) * m.body_mass[self.obj] * self.half_size ** 2
        m.dof_damping[:] = self.base_damping * rng.uniform(.8, 1.2)
        m.cam_pos[self.cam] = self.base_camera + rng.uniform(-.003, .003, 3)
        self.jaw_force = float(rng.uniform(20., 50.))
        m.actuator_ctrlrange[self.ga] = [-self.jaw_force, self.jaw_force]
        self.delay = int(rng.integers(10, 31))  # 20--60ms at 500Hz
        self.pending = [(self.target_q.copy(), self.grip)] * self.delay
        self.applied_q, self.applied_grip = self.target_q.copy(), self.grip
        self.sensor_rng = np.random.default_rng(int(seed) + 320000000)
        self.depth_sigma = float(rng.uniform(.0005, .0015))
        self.profile = {'finger_friction': friction, 'torsional_friction_m': torsion,
            'rolling_friction_m': .0002, 'jaw_force_limit_n': self.jaw_force,
            'command_delay_s': self.delay * m.opt.timestep, 'joint_slew_rad_s': .45,
            'jaw_slew_m_s': .04, 'depth_sigma_m': self.depth_sigma,
            'depth_dropout': .005, 'rgb_noise_std_255': 1.5,
            'camera_position_m': m.cam_pos[self.cam].tolist(),
            'object_mass_kg': float(m.body_mass[self.obj]),
            'object_friction': m.geom_friction[self.og].tolist(),
            'calibration_status': 'synthetic_unmeasured'}
        if self.warm_starts:
            q = np.asarray(self.warm_starts[int(seed) % len(self.warm_starts)], dtype=float)
            d.qpos[self.qa] = np.clip(q, self.lower, self.upper)
            self.target_q = d.qpos[self.qa].copy()
            self.pending = [(self.target_q.copy(), self.grip)] * self.delay
            self.applied_q = self.target_q.copy()
            self.profile['initial_arm_q_rad'] = self.target_q.tolist()
        saved = d.qpos.copy()
        mj.mj_setConst(m, d)
        d.qpos[:] = saved
        mj.mj_forward(m, d)
        meta['realism'] = self.profile
        return meta

    def step(self):
        if not self.realistic:
            return super().step()
        requested_q, requested_grip = self.target_q.copy(), self.grip
        self.pending.append((requested_q, requested_grip))
        q, g = self.pending.pop(0)
        dt = self.model.opt.timestep
        self.applied_q += np.clip(q - self.applied_q, -.45 * dt, .45 * dt)
        self.applied_grip += float(np.clip(g - self.applied_grip, -.04 * dt, .04 * dt))
        self.target_q, self.grip = self.applied_q, self.applied_grip
        super().step()
        self.target_q, self.grip = requested_q, requested_grip

    def observe(self, *, labels=False):
        frame = super().observe(labels=labels)
        if self.realistic:
            rng = self.sensor_rng
            frame['rgb'] = np.clip(frame['rgb'].astype(float) + rng.normal(0, 1.5, frame['rgb'].shape), 0, 255).astype('uint8')
            depth = frame['depth']
            depth += rng.normal(0, self.depth_sigma, depth.shape).astype('float32')
            depth[rng.random(depth.shape) < .005] = 0
        return frame


def state(sim):
    """Feedback and last requested command only. No stage, clock, contact or truth."""
    return np.r_[sim.data.qpos[sim.qa], sim.data.qvel[sim.va],
                 sim.data.qpos[sim.gq] / .05, sim.target_q, sim.grip / .05].astype('float32')


def write(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
