"""Goal-conditioned visual pick/place with bounded retries, simulation only.

Learned component: policy.correct() chooses a bounded pre-grasp XYZ residual.
Explicit components: task stages, IK, interpolation, jaw feedback checks.
Only Recorder and independent_score consume object/contact ground truth.
"""
import copy
import json
from pathlib import Path
import time

import imageio.v2 as imageio
import numpy as np
from PIL import Image

from episode import DOWN
from feedback import configuration, pair_quality, select_view
from policy import valid_prediction
from sim import HOME
from vision import estimate

GOALS = {"A": [.42, -.16], "B": [.42, 0.], "C": [.42, .16]}


class Recorder:
    def __init__(self, sim, directory, video):
        self.sim, self.directory, self.video = sim, Path(directory), video
        self.rows, self.frames, self.movie = [], [], []
        self.decision_frames, self.decision_metadata = [], []
        self.final_scores = []
        self.physics_audit = {"total_steps": 0, "forbidden_steps": 0, "events": []}
        self.stage = "observe"
        self.next_row = self.next_frame = self.next_movie = 0.

    def tick(self):
        s = self.sim
        s.step()
        contacts = s.contacts()
        # Independent scoring at physics resolution; never sent to the controller.
        self.physics_audit["total_steps"] += 1
        if contacts["forbidden"]:
            self.physics_audit["forbidden_steps"] += 1
            self.physics_audit["events"].append({"t_sim_s": float(s.data.time),
                "stage": self.stage, "contacts": contacts["forbidden"]})
        if s.data.time >= self.next_row:
            index = None
            if s.data.time >= self.next_frame:
                self.frames.append(s.observe(labels=False))
                index = len(self.frames)-1
                self.next_frame += .2
            self.rows.append({"t_sim_s": float(s.data.time), "stage": self.stage,
                "observation_index": index, "q_command_rad": s.target_q.tolist(),
                "q_feedback_rad": s.data.qpos[s.qa].tolist(),
                "qd_feedback_rad_s": s.data.qvel[s.va].tolist(),
                "gripper_command_m": s.grip, "gripper_feedback_m": float(s.data.qpos[s.gq]),
                "arm_torque_nm": s.data.ctrl[s.acts].tolist(),
                "truth_for_scoring_only": {"object_xyz_m": s.data.xpos[s.obj].tolist(),
                                           "contact": contacts}})
            self.next_row += .05
        if self.video and s.data.time >= self.next_movie:
            self.movie.append(s.overview())
            self.next_movie += .1

    def capture_decision(self, stage):
        frame = self.sim.observe(labels=False)
        self.decision_frames.append(frame)
        self.decision_metadata.append({"stage": stage, "t_sim_s": float(self.sim.data.time),
            "q_command_rad": self.sim.target_q.tolist(),
            "q_feedback_rad": self.sim.data.qpos[self.sim.qa].tolist(),
            "gripper_command_m": self.sim.grip,
            "gripper_feedback_m": float(self.sim.data.qpos[self.sim.gq])})
        return frame, len(self.decision_frames)-1

    def move(self, q, grip, seconds, stage):
        self.stage = stage
        s = self.sim
        q = np.asarray(q, float)
        if not np.isfinite(q).all() or (q < s.lower).any() or (q > s.upper).any() or not 0 <= grip <= .05:
            raise ValueError("invalid_joint_or_gripper_target")
        q0, g0 = s.target_q.copy(), s.grip
        seconds = max(seconds, 1.875 * float(np.max(np.abs(q-q0))) / .4)
        count = int(np.ceil(seconds/s.model.opt.timestep))
        for i in range(count):
            u = (i+1)/count
            ease = 10*u**3-15*u**4+6*u**5
            s.target_q = q0+ease*(q-q0)
            s.grip = float(g0+ease*(grip-g0))
            self.tick()

    def save(self):
        (self.directory/"final_score_trace.jsonl").write_text(
            "".join(json.dumps(s)+"\n" for s in self.final_scores))
        if self.decision_frames:
            np.savez_compressed(self.directory/"decision_observations.npz",
                rgb=np.stack([f["rgb"] for f in self.decision_frames]),
                depth_m=np.stack([f["depth"] for f in self.decision_frames]),
                camera_json=json.dumps([f["camera"] for f in self.decision_frames]),
                metadata_json=json.dumps(self.decision_metadata))
        if self.frames:
            np.savez_compressed(self.directory/"observations.npz",
                rgb=np.stack([f["rgb"] for f in self.frames]),
                depth_m=np.stack([f["depth"] for f in self.frames]),
                camera_json=json.dumps([f["camera"] for f in self.frames]))
        (self.directory/"trajectory.jsonl").write_text("".join(json.dumps(r)+"\n" for r in self.rows))
        if self.movie:
            imageio.mimsave(self.directory/"replay.gif", self.movie, duration=100, loop=0)
            Image.fromarray(self.movie[-1]).save(self.directory/"final.png")


def correct_prediction(prediction, policy=None):
    if prediction is None:
        return None
    if policy is not None:
        return policy.correct(prediction)
    prediction = copy.deepcopy(prediction)
    prediction["action_source"] = "visual_geometric_baseline"
    prediction["residual_xyz_m"] = [0., 0., 0.]
    return prediction


def visual_target(perception, frame, policy=None):
    prediction = correct_prediction(estimate(perception, frame), policy)
    if not valid_prediction(prediction):
        return None
    return prediction


def grasp_geometry(top):
    center = np.asarray(top, float).copy()
    center[2] = (.1+center[2])/2
    grasp = center+DOWN @ np.array([.015, 0., 0.])+[0., 0., .004]
    return center, grasp


def collect_feedback(sim, recorder, perception, policy, initial_prediction, config, initial_call=None):
    """Same physical observation sequence for feedback and its causal ablation."""
    _, initial_grasp = grasp_geometry(initial_prediction["top_xyz_m"])
    feedback = {"views": [], "selected_view_index": None, "selected_decision_index": None,
                "selected_raw_prediction": None, "selected_prediction": None,
                "accepted": False, "used_for_plan": False, "plan_delta_xyz_m": [0., 0., 0.]}
    views = [("timed_HOME_refresh", None)] if config["home_refresh_wait_s"] else []
    views += [("offset_vantage", offset) for offset in config["view_offsets_m"]]
    if config["use_home_fallback"]:
        views.append(("home_fallback", None))
    for view_index, (view_type, offset) in enumerate(views):
        # A qualified timed HOME observation is sufficient. Only its visual
        # quality chooses the short path; disturbance/truth are not inputs.
        if feedback["views"] and feedback["views"][0]["view_type"] == "timed_HOME_refresh" and feedback["views"][0]["accepted"]:
            break
        home_fallback = view_type == "home_fallback"
        fixed_home = offset is None
        if home_fallback and select_view(feedback["views"]) is not None:
            break
        target = None if fixed_home else initial_grasp+offset
        view = {"view_index": view_index, "offset_m": offset,
                "view_type": view_type,
                "tcp_target_xyz_m": None if fixed_home else target.tolist(),
                "decision_indices": [], "raw_predictions": [], "quality": [],
                "pair_shift_m": None, "accepted": False, "reason": "unreachable_view"}
        feedback["views"].append(view)
        try:
            q = HOME.copy() if fixed_home else sim.ik(target, DOWN, HOME)
        except ValueError as exc:
            view["ik_error"] = str(exc)
            continue
        view["q_target_rad"] = q.tolist()
        if initial_call is not None and not fixed_home:
            initial_call["used_for_action"] = True
        duration = config["home_refresh_wait_s"] if view_type == "timed_HOME_refresh" else 2.
        recorder.move(q, .046, duration, f"observation_view_{view_index}")
        if view_type != "timed_HOME_refresh":
            recorder.move(q, .046, config["settle_s"], f"observation_settle_{view_index}")
        cameras = []
        for pair_index in range(2):
            if pair_index:
                recorder.move(q, .046, config["pair_gap_s"], f"observation_pair_hold_{view_index}")
            frame, index = recorder.capture_decision(f"pregrasp_view_{view_index}_frame_{pair_index}")
            view["decision_indices"].append(index)
            view["raw_predictions"].append(estimate(perception, frame))
            cameras.append(frame["camera"])
        view.update(pair_quality(view["raw_predictions"], cameras, initial_prediction, config))
    selected = select_view(feedback["views"])
    if selected is not None:
        raw = selected["raw_predictions"][-1]
        corrected = correct_prediction(raw, policy)
        feedback.update(selected_view_index=selected["view_index"],
                        selected_decision_index=selected["decision_indices"][-1],
                        selected_raw_prediction=raw, selected_prediction=corrected,
                        accepted=valid_prediction(corrected))
        feedback["reason"] = "usable_visual_update" if feedback["accepted"] else "corrected_target_invalid"
    else:
        feedback["reason"] = "no_usable_reobservation"
    feedback["update_time_s"] = float(sim.data.time)
    return feedback


def independent_score(sim, goal_xy):
    """Ground-truth scoring only; this result never selects control actions."""
    s = sim
    center = s.data.xpos[s.obj].copy()
    rotation = s.data.xmat[s.obj].reshape(3, 3)
    extent = np.abs(rotation) @ s.model.geom_size[s.og]
    inside = bool(np.all(np.abs(center[:2]-goal_xy)+extent[:2] <= .06))
    settled = abs(center[2]-extent[2]-.1) < .006
    free_j = s.model.joint("target_free").id
    va = int(s.model.jnt_dofadr[free_j])
    velocity = s.data.qvel[va:va+3].copy()
    speed = float(np.linalg.norm(velocity))
    c = s.contacts()
    released = not c["left"] and not c["right"]
    return {"t_sim_s":float(s.data.time), "object_xyz_m": center.tolist(),
            "object_rotation_base":rotation.tolist(),
            "object_half_size_m":s.model.geom_size[s.og].tolist(),
            "object_linear_velocity_m_s":velocity.tolist(),
            "finger_contacts":{"left":bool(c["left"]),"right":bool(c["right"])},
            "inside_region": inside,
            "supported_by_table": bool(settled), "linear_speed_m_s": speed,
            "released": released, "success": bool(inside and settled and speed < .02 and released)}


def run_task(sim, perception, policy, seed, goal, directory, *, video=False,
             absent=False, first_grasp_offset_m=0., max_retries=2,
             observation_mode="none", feedback_config=None, force_pulse=None):
    if goal not in GOALS or not 0 <= max_retries <= 2 or abs(first_grasp_offset_m) > .08:
        raise ValueError("invalid_goal_or_retry_configuration")
    if observation_mode not in ("none", "observe_only", "feedback"):
        raise ValueError("invalid_observation_mode")
    feedback_config = configuration(feedback_config)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    metadata = sim.reset(seed, absent=absent, force_pulse=force_pulse)
    recorder = Recorder(sim, directory, video)
    result = {"schema": "b601-goal-pick-place-v3", "domain": "simulation", "hardware_executable": False,
        "task": {"object": "red_orange_cube", "destination": goal, "region_center_xy_m": GOALS[goal],
                 "region_half_width_m": .06}, "reset": metadata, "attempts": [],
        "controller": "learned_residual" if policy else "visual_geometric_baseline",
        "training_method": "supervised residual correction" if policy else None,
        "observation_mode": observation_mode, "feedback_config": feedback_config,
        "first_grasp_offset_m": first_grasp_offset_m, "max_retries": max_retries,
        "joint_unit": "rad", "gripper_unit": "single_finger_model_travel_m",
        "joint_order": [f"joint{i}" for i in range(1, 7)],
        "success": False, "controller_success": False, "contact_only": True}
    started = time.monotonic()
    try:
        for attempt_id in range(max_retries+1):
            frame, initial_index = recorder.capture_decision(f"initial_attempt_{attempt_id+1}")
            Image.fromarray(frame["rgb"]).save(directory/f"observe-{attempt_id}.png")
            raw = estimate(perception, frame)
            corrected = correct_prediction(raw, policy)
            predicted = corrected if valid_prediction(corrected) else None
            initial_call = None if policy is None or raw is None else {
                "decision_index": initial_index, "role": "initial", "used_for_action": False,
                "raw_prediction": raw, "prediction": corrected}
            attempt = {"attempt": attempt_id+1, "prediction": predicted, "raw_prediction": raw,
                       "initial_decision_index": initial_index,
                       "policy_calls": [] if initial_call is None else [initial_call]}
            result["attempts"].append(attempt)
            if predicted is None:
                result["reason"] = "no_confident_target"
                break
            action_prediction = predicted
            selected_call = None
            if observation_mode != "none":
                feedback = collect_feedback(sim, recorder, perception, policy, predicted,
                                            feedback_config, initial_call)
                attempt["feedback"] = feedback
                if policy is not None and feedback["selected_raw_prediction"] is not None:
                    selected_call = {"decision_index": feedback["selected_decision_index"],
                        "role": "pregrasp_feedback", "used_for_action": False,
                        "raw_prediction": feedback["selected_raw_prediction"],
                        "prediction": feedback["selected_prediction"]}
                    attempt["policy_calls"].append(selected_call)
                if not feedback["accepted"]:
                    attempt["outcome"] = "reobservation_rejected"
                    recorder.move(HOME, .046, 2., "return_for_reobserve")
                    result["reason"] = "retries_exhausted_after_reobservation"
                    continue
                if observation_mode == "feedback":
                    action_prediction = feedback["selected_prediction"]
                    feedback["plan_delta_xyz_m"] = (np.asarray(action_prediction["top_xyz_m"])
                                                     - predicted["top_xyz_m"]).tolist()
            attempt["action_prediction"] = action_prediction
            top = np.asarray(action_prediction["top_xyz_m"], float).copy()
            # Failure injection is after all perception updates so feedback does
            # not accidentally erase the deliberate first grasp error.
            if attempt_id == 0:
                top[1] += first_grasp_offset_m
            center, grasp = grasp_geometry(top)
            approach = grasp+[0., 0., .12]
            lift = grasp+[0., 0., .12]
            destination = grasp.copy(); destination[:2] += np.asarray(GOALS[goal])-center[:2]
            # Place 2mm above the estimated original support level, then release.
            destination[2] += .002
            above_destination = destination+[0., 0., .11]
            qa = sim.ik(approach, DOWN, HOME)
            qg = sim.ik(grasp, DOWN, qa)
            ql = sim.ik(lift, DOWN, qg)
            qt = sim.ik(above_destination, DOWN, ql)
            qp = sim.ik(destination, DOWN, qt)
            attempt["plan"] = {"grasp_xyz_m": grasp.tolist(), "destination_tcp_xyz_m": destination.tolist(),
                "rotation_base_tcp": DOWN.tolist(), "max_command_speed_rad_s": .4,
                "action_decision_time_s": float(sim.data.time),
                "target_top_xyz_m": top.tolist(),
                "approach_q_rad": qa.tolist(), "grasp_q_rad": qg.tolist()}
            if initial_call is not None and observation_mode in ("none", "observe_only"):
                initial_call["used_for_action"] = True
            if observation_mode == "feedback":
                attempt["feedback"]["used_for_plan"] = True
                if selected_call is not None:
                    selected_call["used_for_action"] = True
            recorder.move(qa, .046, 2., "approach")
            recorder.move(qg, .046, 1.8, "descend")
            recorder.move(qg, 0., 1.5, "close")
            recorder.move(ql, 0., 2., "lift")
            # Proprioceptive jaw travel, NOT simulated finger contacts/object truth.
            recorder.move(ql, 0., .4, "check_gripper")
            jaw = float(sim.data.qpos[sim.gq])
            attempt["jaw_feedback_after_lift_m"] = jaw
            if not .008 < jaw < .038:
                attempt["outcome"] = "empty_grasp_or_slip"
                recorder.move(ql, .046, .8, "reopen")
                recorder.move(HOME, .046, 2., "return_for_reobserve")
                result["reason"] = "retries_exhausted"
                continue
            recorder.move(qt, 0., 2., "transport")
            recorder.move(qp, 0., 1.8, "place")
            recorder.move(qp, .046, 1.2, "release")
            recorder.move(qt, .046, 1.8, "retreat")
            recorder.move(HOME, .046, 2., "return_for_verification")
            recorder.move(HOME, .046, .5, "settle")
            final_frame, final_index = recorder.capture_decision(f"verify_attempt_{attempt_id+1}")
            Image.fromarray(final_frame["rgb"]).save(directory/f"verify-{attempt_id}.png")
            # The residual was trained at pickup locations, not destination zones.
            # Verification uses the raw visual estimate and does not extrapolate it.
            final_prediction = visual_target(perception, final_frame)
            attempt["verification_prediction"] = final_prediction
            attempt["verification_decision_index"] = final_index
            verified = False
            if final_prediction:
                final_top = np.asarray(final_prediction["top_xyz_m"])
                verified = bool(np.max(np.abs(final_top[:2]-GOALS[goal])) < .032
                                and .13 <= final_top[2] <= .155
                                and sim.data.qpos[sim.gq] > .04)
            attempt["outcome"] = "visually_verified" if verified else "placement_not_verified"
            if verified:
                result["controller_success"], result["reason"] = True, "visually_verified_placement"
                break
            result["reason"] = "placement_not_verified"
        # Independent final one-second physical audit, never fed back into policy.
        if recorder.rows:
            recorder.stage = "independent_score"
            scores = []
            for _ in range(round(1./sim.model.opt.timestep)):
                recorder.tick()
                scores.append(independent_score(sim, GOALS[goal]))
            recorder.final_scores = scores
            # Contact truth is an independent score, never a control input.
            result["forbidden_contact_samples"] = sum(bool(row["truth_for_scoring_only"]["contact"]["forbidden"])
                                                       for row in recorder.rows)
            result["physical_success"] = all(s["success"] for s in scores) and not recorder.physics_audit["forbidden_steps"]
            result["final_score"] = scores[-1]
            result["success"] = bool(result["controller_success"] and result["physical_success"])
            result["false_success"] = bool(result["controller_success"] and not result["physical_success"])
        else:
            result.update(physical_success=False, false_success=False)
    except (ValueError, RuntimeError) as exc:
        result.update(reason=str(exc), failed_stage=recorder.stage, physical_success=False,
                      false_success=bool(result["controller_success"]))
    result.update(sim_seconds=float(sim.data.time), wall_seconds=time.monotonic()-started,
                  record_rows=len(recorder.rows), rgbd_frames=len(recorder.frames),
                  decision_frames=len(recorder.decision_frames),
                  score_trace_samples=len(recorder.final_scores),
                  physics_tick_audit=recorder.physics_audit,
                  environment_for_scoring_only=sim.environment_audit(),
                  retries=max(0, len(result["attempts"])-1),
                  learned_action_calls=sum(c["used_for_action"] and c["prediction"]["action_source"] ==
                                           "learned_bounded_visuomotor_residual"
                                           for a in result["attempts"] for c in a["policy_calls"]),
                  learned_inference_calls=sum(c["prediction"]["action_source"] ==
                                              "learned_bounded_visuomotor_residual"
                                              for a in result["attempts"] for c in a["policy_calls"]))
    recorder.save()
    (directory/"result.json").write_text(json.dumps(result, indent=2)+"\n")
    return result
