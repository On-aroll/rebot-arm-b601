"""Standalone MuJoCo physics. No ROS, robot SDK, CAN, or hardware adapter."""
import colorsys
import copy
import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
HOME = np.array([0., .55, .8, 0., .1, 0.])
WIDTH, HEIGHT = 160, 120


class Simulation:
    def __init__(self, scene_path=None):
        self.model = mj.MjModel.from_xml_path(str(scene_path or ROOT / "assets/scene.xml"))
        self.data = mj.MjData(self.model)
        self.probe = mj.MjData(self.model)
        m = self.model
        self.joints = np.array([m.joint(f"joint{i}").id for i in range(1, 7)])
        self.qa = m.jnt_qposadr[self.joints]
        self.va = m.jnt_dofadr[self.joints]
        self.acts = np.array([m.actuator(f"joint{i}_motor").id for i in range(1, 7)])
        self.gq = int(m.jnt_qposadr[m.joint("joint7").id])
        self.gv = int(m.jnt_dofadr[m.joint("joint7").id])
        self.ga = m.actuator("joint7_motor").id
        self.obj = m.body("target").id
        self.og = m.geom("target_geom").id
        self.oq = int(m.jnt_qposadr[m.joint("target_free").id])
        self.tip = m.site("tcp").id
        self.fingers = [m.body(n).id for n in ("gripper_left", "gripper_right")]
        self.cam = m.camera("overhead_rgbd").id
        self.lower, self.upper = m.jnt_range[self.joints].T.copy()
        self.lower[1:3] = 0.  # Match the existing canonical control contract.
        self.kp = np.array([80., 100., 100., 35., 25., 18.])
        self.kd = np.array([8., 10., 10., 4., 3., 2.5])
        self.limits = np.array([36., 36., 36., 14., 14., 14.])
        self.renderer = None
        self.view_renderer = None
        self.reset(0)

    def reset(self, seed, *, absent=False, force_pulse=None):
        pulse = copy.deepcopy(force_pulse)
        if pulse is not None:
            if not isinstance(pulse, dict) or set(pulse) != {"start_s", "duration_s", "force_world_n"}:
                raise ValueError("Invalid force pulse fields")
            try:
                start, duration = float(pulse["start_s"]), float(pulse["duration_s"])
                force = np.asarray(pulse["force_world_n"], dtype=float)
            except (TypeError, ValueError, OverflowError) as exc:
                raise ValueError("Invalid force pulse values") from exc
            if (not np.isfinite([start, duration, start+duration]).all()
                    or start < 0 or not 0 <= duration <= .2 or force.shape != (3,)
                    or not np.isfinite(force).all() or np.linalg.norm(force) > 5.):
                raise ValueError("Force pulse exceeds timing or force bounds")
            pulse = {"start_s": start, "duration_s": duration, "force_world_n": force.tolist()}
        rng = np.random.default_rng(seed)
        m, d = self.model, self.data
        mj.mj_resetData(m, d)
        self.target_q = HOME.copy()
        self.grip = .046
        d.qpos[self.qa] = HOME
        for name in ("joint7", "joint_left", "joint_right"):
            d.qpos[m.jnt_qposadr[m.joint(name).id]] = self.grip
        self.half_size = float(rng.uniform(.018, .023))
        m.geom_size[self.og, :3] = self.half_size
        # Size, mass and inertia are changed together; no physics parameter fitting.
        mass = float(rng.uniform(.025, .06))
        m.body_mass[self.obj] = mass
        m.body_inertia[self.obj] = (2 / 3) * mass * self.half_size ** 2
        m.geom_friction[self.og, 0] = rng.uniform(.8, 1.8)
        xyz = [rng.uniform(.28, .38), rng.uniform(-.105, .105), .1 + self.half_size]
        d.qpos[self.oq:self.oq+3] = xyz if not absent else [2., 0., .12]
        yaw = rng.uniform(-np.pi / 4, np.pi / 4)
        d.qpos[self.oq+3:self.oq+7] = [np.cos(yaw/2), 0., 0., np.sin(yaw/2)]
        m.geom_rgba[self.og] = [*colorsys.hsv_to_rgb(rng.uniform(0., .075), rng.uniform(.7, 1.), rng.uniform(.65, 1.)), 1.]
        m.geom_rgba[m.geom("table_geom").id, :3] = rng.uniform(.25, .48, 3)
        m.light_diffuse[0] = rng.uniform(.5, .9, 3)
        reset_qpos = d.qpos.copy()
        mj.mj_setConst(m, d)
        d.qpos[:] = reset_qpos
        mj.mj_forward(m, d)
        self.initial_z = float(d.xpos[self.obj, 2])
        self._force_pulse = pulse
        self._force_steps = 0
        self._force_impulse = np.zeros(3)
        self._force_start = self._force_end = None
        self._force_pre_xyz = self._force_post_xyz = None
        self._force_settled_xyz = self._force_settled_time = None
        self._force_finger_contact_before_settle = False
        return {"seed": int(seed), "absent": absent, "half_size_m": self.half_size,
                "mass_kg": mass, "friction": float(m.geom_friction[self.og, 0]),
                "initial_xyz": d.xpos[self.obj].tolist(), "yaw_rad": float(yaw)}

    def step(self):
        m, d = self.model, self.data
        tau = d.qfrc_bias[self.va] + self.kp * (self.target_q - d.qpos[self.qa]) - self.kd * d.qvel[self.va]
        d.ctrl[self.acts] = np.clip(tau, -self.limits, self.limits)
        d.ctrl[self.ga] = np.clip(1800 * (self.grip - d.qpos[self.gq]) - 18 * d.qvel[self.gv], -80, 80)
        # Independent environment input: the fixed schedule uses only simulation
        # time. Object positions below are retained solely for external scoring.
        d.xfrc_applied[:] = 0.
        pulse = self._force_pulse
        active = pulse is not None and pulse["start_s"] <= d.time < pulse["start_s"]+pulse["duration_s"]
        if active:
            force = np.asarray(pulse["force_world_n"])
            d.xfrc_applied[self.obj, :3] = force
            if self._force_steps == 0:
                self._force_start = float(d.time)
                self._force_pre_xyz = d.qpos[self.oq:self.oq+3].copy()
            self._force_steps += 1
            self._force_impulse += force * m.opt.timestep
            self._force_end = float(d.time+m.opt.timestep)
        mj.mj_step(m, d)
        if active:
            self._force_post_xyz = d.qpos[self.oq:self.oq+3].copy()
        if pulse is not None and self._force_settled_xyz is None:
            contact = self.contacts()
            self._force_finger_contact_before_settle |= contact["left"] or contact["right"]
        if (self._force_steps and self._force_settled_xyz is None
                and d.time >= pulse["start_s"]+pulse["duration_s"]+.3):
            self._force_settled_xyz = d.qpos[self.oq:self.oq+3].copy()
            self._force_settled_time = float(d.time)
        if not np.isfinite(d.qpos).all() or np.any(d.warning.number):
            raise RuntimeError("Invalid MuJoCo state/warning")

    def environment_audit(self):
        """Independent perturbation/scoring record; never a policy observation."""
        pre, post = self._force_pre_xyz, self._force_post_xyz
        return {"config": copy.deepcopy(self._force_pulse), "applied_steps": self._force_steps,
                "actual_impulse_ns": self._force_impulse.tolist(),
                "actual_start_s": self._force_start, "actual_end_s": self._force_end,
                "truth_for_scoring_only": {
                    "pre_pulse_xyz_m": None if pre is None else pre.tolist(),
                    "post_pulse_xyz_m": None if post is None else post.tolist(),
                    "pulse_displacement_xyz_m": None if pre is None else (post-pre).tolist(),
                    "settle_delay_s": .3, "settled_time_s": self._force_settled_time,
                    "settled_xyz_m": None if self._force_settled_xyz is None else self._force_settled_xyz.tolist(),
                    "settled_displacement_xyz_m": None if self._force_settled_xyz is None
                    else (self._force_settled_xyz-pre).tolist(),
                    "finger_contact_before_settled_snapshot": self._force_finger_contact_before_settle,
                    "final_xyz_m": self.data.qpos[self.oq:self.oq+3].tolist()}}

    def camera_info(self):
        f = .5 * HEIGHT / np.tan(np.deg2rad(self.model.cam_fovy[self.cam]) / 2)
        k = np.array([[f, 0, (WIDTH-1)/2], [0, f, (HEIGHT-1)/2], [0, 0, 1.]])
        t = np.eye(4)
        t[:3, :3] = self.data.cam_xmat[self.cam].reshape(3, 3) @ np.diag([1., -1., -1.])
        t[:3, 3] = self.data.cam_xpos[self.cam]
        return {"K": k.tolist(), "T_base_camera_optical": t.tolist(),
                "width": WIDTH, "height": HEIGHT, "depth_unit": "m",
                "camera_name": "overhead_rgbd", "simulation_time_s": float(self.data.time)}

    def observe(self, *, labels=False):
        mj.mj_forward(self.model, self.data)
        if self.renderer is None:
            self.renderer = mj.Renderer(self.model, height=HEIGHT, width=WIDTH)
        r = self.renderer
        r.update_scene(self.data, camera="overhead_rgbd")
        rgb = r.render().copy()
        r.enable_depth_rendering()
        depth = r.render().copy()
        r.disable_depth_rendering()
        frame = {"rgb": rgb, "depth": depth, "camera": self.camera_info()}
        if labels:
            r.enable_segmentation_rendering()
            seg = r.render().copy()
            r.disable_segmentation_rendering()
            frame["mask"] = (seg[:, :, 0] == self.og) & (seg[:, :, 1] == int(mj.mjtObj.mjOBJ_GEOM))
            # Multisampled ID colours can alias site edges to an unrelated geom ID.
            # Ground-truth volume filtering is used ONLY to construct supervision.
            v, u = np.where(frame["mask"])
            if len(u):
                k = np.asarray(frame["camera"]["K"])
                t = np.asarray(frame["camera"]["T_base_camera_optical"])
                points = np.linalg.solve(k, np.stack([u, v, np.ones(len(u))])) * depth[v, u]
                world = (t[:3, :3] @ points).T + t[:3, 3]
                local = (world-self.data.xpos[self.obj]) @ self.data.xmat[self.obj].reshape(3, 3)
                valid = (np.abs(local) <= self.model.geom_size[self.og]+.002).all(axis=1)
                frame["mask"][v[~valid], u[~valid]] = False
        return frame

    def overview(self):
        if self.view_renderer is None:
            self.view_renderer = mj.Renderer(self.model, height=360, width=480)
        cam = mj.MjvCamera()
        cam.lookat[:] = [.28, 0, .2]
        cam.distance, cam.azimuth, cam.elevation = 1.05, 130, -28
        self.view_renderer.update_scene(self.data, camera=cam)
        return self.view_renderer.render().copy()

    def ik(self, xyz, rotation, seed):
        """Solve in a separate data instance; never teleport the dynamic arm."""
        p = self.probe
        p.qpos[:] = self.data.qpos

        def residual(q):
            p.qpos[self.qa] = q
            mj.mj_forward(self.model, p)
            current = p.site_xmat[self.tip].reshape(3, 3)
            return np.r_[p.site_xpos[self.tip] - xyz,
                         .15 * Rotation.from_matrix(rotation @ current.T).as_rotvec()]

        best = None
        for initial in [seed, HOME, np.array([0., 1., 1.5, 0., .5, 0.])]:
            result = least_squares(residual, np.clip(initial, self.lower+1e-6, self.upper-1e-6),
                                   bounds=(self.lower, self.upper), max_nfev=100, ftol=1e-8, xtol=1e-8, gtol=1e-8)
            error = residual(result.x)
            if best is None or np.linalg.norm(error) < best[0]:
                best = (np.linalg.norm(error), result.x.copy(), error.copy())
            if np.linalg.norm(error[:3]) < .001 and np.linalg.norm(error[3:]) < .005:
                return result.x
        raise ValueError(f"IK rejected for {np.asarray(xyz).tolist()}; q={best[1].tolist()}; residual={best[2].tolist()}")

    def contacts(self):
        touched = set()
        other = []
        for c in self.data.contact:
            bodies = {int(self.model.geom_bodyid[c.geom1]), int(self.model.geom_bodyid[c.geom2])}
            if self.obj in bodies:
                touched.update(bodies.intersection(self.fingers))
            # Table/robot contacts and robot self contacts are forbidden.
            robot = set(range(1, self.obj)) - {self.model.body("table").id}
            if bodies.intersection(robot) and not (self.obj in bodies and bodies.intersection(self.fingers)):
                if c.dist < -.001:
                    other.append([int(c.geom1), int(c.geom2), float(c.dist)])
        return {"left": self.fingers[0] in touched, "right": self.fingers[1] in touched,
                "forbidden": other}

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
        if self.view_renderer is not None:
            self.view_renderer.close()


def deproject(u, v, depth_m, camera):
    if not np.isfinite([u, v, depth_m]).all() or not .05 < depth_m < 2:
        raise ValueError("Invalid pixel/depth")
    k = np.asarray(camera["K"])
    t = np.asarray(camera["T_base_camera_optical"])
    p = np.linalg.solve(k, [u, v, 1.]) * depth_m
    return (t @ np.r_[p, 1.])[:3]
