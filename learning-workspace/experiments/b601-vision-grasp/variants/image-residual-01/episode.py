"""Scripted grasp controller driven by a visual estimate, recorded at 20 Hz.

The learned component is perception; IK and the stage controller are explicit.
Only the scorer reads object truth. No welds, attachments, or object pose writes
are used after reset. All commands are MuJoCo joint targets, never motor frames.
"""
import json
from pathlib import Path
import time

import imageio.v2 as imageio
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from sim import HOME, Simulation

DOWN = Rotation.from_euler("y", 1.05).as_matrix()


def run_episode(sim, seed, estimate, directory, *, video=False, absent=False):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    metadata = sim.reset(seed, absent=absent)
    # Scene is reset by the caller immediately before observation too.
    frame = sim.observe()
    Image.fromarray(frame["rgb"]).save(directory / "observation.png")
    np.save(directory / "depth_m.npy", frame["depth"])
    (directory / "camera.json").write_text(json.dumps(frame["camera"], indent=2))
    rows, movie, observations = [], [], []
    begin = time.monotonic()
    result = {"schema": "b601-sim-grasp-episode-v1", "domain": "simulation",
              "hardware_executable": False, "reset": metadata,
              "estimate": estimate, "success": False,
              "controller": "RGB-D perception + pose IK + scripted stages",
              "gripper_unit": "single_finger_model_travel_m",
              "joint_order": [f"joint{i}" for i in range(1, 7)],
              "joint_unit": "rad", "warnings": [], "contact_only": True}
    stage = "observation"
    next_record, next_frame, next_observation = 0., 0., 0.

    def tick():
        nonlocal next_record, next_frame, next_observation
        sim.step()
        contacts = sim.contacts()
        if contacts["forbidden"]:
            raise ValueError(f"Forbidden contact: {contacts['forbidden']}")
        if sim.data.time >= next_record:
            frame_index = None
            if sim.data.time >= next_observation:
                observations.append(sim.observe())
                frame_index = len(observations) - 1
                next_observation += .2
            rows.append({"t_sim_s": float(sim.data.time), "stage": stage,
                         "observation_index": frame_index,
                         "q_command_rad": sim.target_q.tolist(),
                         "q_feedback_rad": sim.data.qpos[sim.qa].tolist(),
                         "qd_feedback_rad_s": sim.data.qvel[sim.va].tolist(),
                         "gripper_command_m": sim.grip,
                         "gripper_feedback_m": float(sim.data.qpos[sim.gq]),
                         "arm_torque_nm": sim.data.ctrl[sim.acts].tolist(),
                         "finger_contact": contacts,
                         "truth_for_scoring_only": {"object_xyz_m": sim.data.xpos[sim.obj].tolist()}})
            next_record += .05
        if video and sim.data.time >= next_frame:
            movie.append(sim.overview())
            next_frame += .1

    def move(q, grip, seconds):
        q0, g0 = sim.target_q.copy(), sim.grip
        # Quintic easing max derivative 1.875; enforce 0.4 rad/s commands.
        seconds = max(seconds, 1.875 * float(np.max(np.abs(q-q0))) / .4)
        steps = int(np.ceil(seconds / sim.model.opt.timestep))
        for i in range(steps):
            u = (i + 1) / steps
            s = 10*u**3 - 15*u**4 + 6*u**5
            sim.target_q = q0 + s * (q-q0)
            sim.grip = float(g0 + s * (grip-g0))
            tick()

    try:
        if estimate is None:
            raise ValueError("No confident visible target")
        top = np.asarray(estimate["top_xyz_m"], dtype=float)
        if top.shape != (3,) or not np.isfinite(top).all() or not (.265 <= top[0] <= .395 and abs(top[1]) <= .125 and .13 <= top[2] <= .15):
            raise ValueError("Visual target outside this experiment's workspace")
        # Estimate the cube center from its observed top and the known table.
        # Align the center of the front pads (local x=-15mm), 4mm above it.
        center = top.copy(); center[2] = (.1 + top[2]) / 2
        grasp = center + DOWN @ np.array([.015, 0., 0.]) + [0., 0., .004]
        approach = grasp.copy(); approach[2] += .13
        lifted = grasp.copy(); lifted[2] += .12
        q_approach = sim.ik(approach, DOWN, HOME)
        q_grasp = sim.ik(grasp, DOWN, q_approach)
        q_lift = sim.ik(lifted, DOWN, q_grasp)
        result["plan"] = {"tcp_grasp_xyz_m": grasp.tolist(), "rotation_base_tcp": DOWN.tolist(),
                          "approach_q_rad": q_approach.tolist(), "grasp_q_rad": q_grasp.tolist(),
                          "lift_q_rad": q_lift.tolist(), "max_command_speed_rad_s": .4}
        stage = "approach"; move(q_approach, .046, 2.)
        stage = "descend"; move(q_grasp, .046, 2.)
        stage = "close"; move(q_grasp, 0., 1.5)
        stage = "lift"; move(q_lift, 0., 2.)
        stage = "hold"
        heights, bilateral = [], []
        for _ in range(round(1. / sim.model.opt.timestep)):
            tick()
            heights.append(float(sim.data.xpos[sim.obj, 2] - sim.initial_z))
            c = sim.contacts(); bilateral.append(c["left"] and c["right"])
        result.update(hold_min_lift_m=min(heights), hold_bilateral_fraction=float(np.mean(bilateral)),
                      success=bool(min(heights) > .06 and np.mean(bilateral) > .9),
                      reason="lift_and_bilateral_contact" if min(heights) > .06 and np.mean(bilateral) > .9 else "miss_or_slip")
    except (ValueError, RuntimeError) as exc:
        result["reason"] = str(exc)
        result["failed_stage"] = stage
    result.update(sim_seconds=float(sim.data.time), wall_seconds=time.monotonic()-begin,
                  record_rows=len(rows), rgbd_frames=len(observations),
                  final_object_xyz_m=sim.data.xpos[sim.obj].tolist())
    if observations:
        np.savez_compressed(directory / "observations.npz",
                            rgb=np.stack([f["rgb"] for f in observations]),
                            depth_m=np.stack([f["depth"] for f in observations]),
                            camera_json=json.dumps([f["camera"] for f in observations]))
    (directory / "trajectory.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows))
    (directory / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    if video and movie:
        imageio.mimsave(directory / "replay.gif", movie, duration=100, loop=0)
        Image.fromarray(movie[-1]).save(directory / "final.png")
    return result
