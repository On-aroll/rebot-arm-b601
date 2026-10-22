"""Forecast robot self-occlusion from calibration, visual target and proprioception.

The independent kinematic model never receives the running simulation or any
object state. Only rendered robot group-1 meshes participate in ray queries.
"""
from itertools import product

import mujoco as mj
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from episode import DOWN
from sim import HOME
from task import grasp_geometry


OFFSETS = [[-.08, 0., .15], [-.06, .06, .12]]
OFFSETS += [list(p) for p in product((-.08, 0., .08), (-.08, 0., .08), (.12, .18))]
ROBOT_GROUP = np.array([0, 1, 0, 0, 0, 0], np.uint8)


class SelfVisibility:
    def __init__(self, scene_path):
        self.model = mj.MjModel.from_xml_path(str(scene_path))
        self.data = mj.MjData(self.model)
        joints = np.asarray([self.model.joint(f"joint{i}").id for i in range(1, 7)])
        self.qa = self.model.jnt_qposadr[joints]
        self.gqa = [self.model.jnt_qposadr[self.model.joint(n).id] for n in ("joint7", "joint_left", "joint_right")]
        self.lower, self.upper = self.model.jnt_range[joints].T.copy()
        self.lower[1:3] = 0.
        self.tip = self.model.site("tcp").id

    def set_robot(self, q, grip):
        q = np.asarray(q, float)
        if (q.shape != (6,) or not np.isfinite(q).all() or np.any(q < self.lower)
                or np.any(q > self.upper) or not 0 <= grip <= .05):
            raise ValueError("invalid_robot_proprioception")
        self.data.qpos[self.qa] = q
        self.data.qpos[self.gqa] = grip
        self.data.qvel[:] = 0.
        mj.mj_forward(self.model, self.data)

    def ik(self, xyz, grip):
        self.set_robot(HOME, grip)

        def residual(q):
            self.set_robot(q, grip)
            current = self.data.site_xmat[self.tip].reshape(3, 3)
            return np.r_[self.data.site_xpos[self.tip]-xyz,
                         .15*Rotation.from_matrix(DOWN@current.T).as_rotvec()]

        best = None
        for initial in (HOME, HOME, np.array([0., 1., 1.5, 0., .5, 0.])):
            fit = least_squares(residual, np.clip(initial, self.lower+1e-6, self.upper-1e-6),
                bounds=(self.lower, self.upper), max_nfev=100, ftol=1e-8, xtol=1e-8, gtol=1e-8)
            error = residual(fit.x)
            if best is None or np.linalg.norm(error) < np.linalg.norm(best[1]):
                best = (fit.x.copy(), error.copy())
            if np.linalg.norm(error[:3]) < .001 and np.linalg.norm(error[3:]) < .005:
                return fit.x
        raise ValueError(f"IK rejected q={best[0].tolist()}; residual={best[1].tolist()}")

    def forecast(self, camera, visual_top, q, grip):
        top = np.asarray(visual_top, float)
        if (top.shape != (3,) or not np.isfinite(top).all() or not .245 <= top[0] <= .50
                or abs(top[1]) > .25 or not .125 <= top[2] <= .16):
            raise ValueError("invalid_visual_target")
        self.set_robot(q, grip)
        half_width = (top[2]-.1)/np.sqrt(2)+.008
        points = np.asarray([top+[x, y, 0.] for y in np.linspace(-half_width, half_width, 9)
                             for x in np.linspace(-half_width, half_width, 9)])
        origin = np.asarray(camera["T_base_camera_optical"], float)[:3, 3].copy()
        vectors = points-origin
        lengths = np.linalg.norm(vectors, axis=1)
        unit = vectors/lengths[:, None]
        hits = np.empty(len(points), np.int32)
        distances = np.empty(len(points), np.float64)
        mj.mj_multiRay(self.model, self.data, origin, unit.ravel(), ROBOT_GROUP,
                      True, -1, hits, distances, None, len(points), 3.)
        assert all(h < 0 or self.model.geom_group[h] == 1 for h in hits)
        blocked = (distances >= 0) & (distances < lengths-.002)
        return {"blocked_rays": int(blocked.sum()), "rays": len(points),
            "clear_fraction": float(1-blocked.mean()), "roi_half_width_m": float(half_width),
            "roi_points_xyz_m": points.tolist(), "ray_origin_xyz_m": origin.tolist(),
            "ray_directions": unit.tolist(), "ray_distances_m": distances.tolist(),
            "ray_geom_ids": hits.tolist()}

    def plan(self, camera, visual_top, current_q, grip):
        _, grasp = grasp_geometry(visual_top)
        candidates = []
        for i, offset in enumerate(OFFSETS):
            tcp = grasp+offset
            item = {"candidate_index": i, "offset_m": offset, "tcp_target_xyz_m": tcp.tolist(),
                    "q_target_rad": None, "forecast": None, "ik_error": None}
            try:
                q = self.ik(tcp, grip)
            except ValueError as exc:
                item["ik_error"] = str(exc)
            else:
                item.update(q_target_rad=q.tolist(), forecast=self.forecast(camera, visual_top, q, grip),
                            maximum_joint_change_rad=float(np.max(np.abs(q-current_q))))
            candidates.append(item)
        ordered = sorted([c for c in candidates if c["q_target_rad"] is not None],
                         key=lambda c: (c["forecast"]["blocked_rays"], c["maximum_joint_change_rad"], c["candidate_index"]))
        selected = []
        for candidate in ordered:
            if any(np.max(np.abs(np.asarray(candidate["q_target_rad"])-candidates[i]["q_target_rad"])) <= .05 for i in selected):
                continue
            selected.append(candidate["candidate_index"])
            if len(selected) == 2:
                break
        return {"candidates": candidates, "ordered_candidate_indices": [c["candidate_index"] for c in ordered],
                "selected_candidate_indices": selected}
