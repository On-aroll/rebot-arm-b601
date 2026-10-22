"""Recompute feedback decisions and audit paired task evidence, independently."""
import argparse
import inspect
import json
from pathlib import Path

import numpy as np

from feedback import pair_quality, select_view
from feedback_runner import complete_summary, group_summary
from policy import BOUND_M, load, valid_prediction
from sim import HOME, ROOT, Simulation, mj
from task import GOALS, collect_feedback, correct_prediction, grasp_geometry, run_task
from task_runner import digest, perception_model, write
from vision import estimate
from score_audit import audit_score_trace


def same(actual, expected):
    if isinstance(expected, dict):
        assert set(actual) == set(expected), (set(actual), set(expected))
        for key in expected:
            same(actual[key], expected[key])
    elif isinstance(expected, list):
        assert len(actual) == len(expected)
        for a, b in zip(actual, expected, strict=True):
            same(a, b)
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        assert np.isclose(actual, expected, atol=1e-8, rtol=1e-7), (actual, expected)
    else:
        assert actual == expected, (actual, expected)


def audit_episode(directory, episode, sim, perception, policy, config):
    disk = json.loads((directory/"result.json").read_text())
    same({k:episode[k] for k in disk}, disk)
    e = disk
    assert not e["hardware_executable"] and e["domain"] == "simulation"
    assert e["success"] == bool(e["controller_success"] and e["physical_success"])
    assert e["false_success"] == bool(e["controller_success"] and not e["physical_success"])
    assert e["retries"] == max(0, len(e["attempts"])-1) <= e["max_retries"] <= 2
    same(e["feedback_config"], config)
    rows = [json.loads(line) for line in (directory/"trajectory.jsonl").read_text().splitlines()]
    assert len(rows) == e["record_rows"]
    physics = e["physics_tick_audit"]
    assert physics["total_steps"] == round(e["sim_seconds"]/sim.model.opt.timestep)
    assert physics["forbidden_steps"] == len(physics["events"])
    assert all(event["contacts"] for event in physics["events"])
    if physics["forbidden_steps"]:
        assert not e["physical_success"] and not e["success"]
    if e["physical_success"]:
        assert e["final_score"]["success"] and not physics["forbidden_steps"]
    audit_score_trace(directory, e, sim.model.opt.timestep)
    if e["reset"]["absent"]:
        assert not rows and not any(a.get("plan") for a in e["attempts"]) and not e["controller_success"]
    max_speed, paired = 0., 0
    if rows:
        with np.load(directory/"observations.npz", allow_pickle=False) as frames:
            cameras = json.loads(str(frames["camera_json"]))
            previous = None
            for row in rows:
                q = np.asarray(row["q_command_rad"])
                assert np.isfinite(q).all() and (q >= sim.lower-1e-8).all() and (q <= sim.upper+1e-8).all()
                assert 0. <= row["gripper_command_m"] <= .05
                if previous is not None:
                    dt = row["t_sim_s"]-previous["t_sim_s"]
                    assert .047 <= dt <= .053
                    speed = float(np.max(np.abs(q-previous["q_command_rad"])) / dt)
                    max_speed = max(max_speed, speed)
                    assert speed <= .401
                index = row["observation_index"]
                if index is not None:
                    assert index == paired
                    assert abs(cameras[index]["simulation_time_s"]-row["t_sim_s"]) < 1e-9
                    if index:
                        assert .197 <= cameras[index]["simulation_time_s"]-cameras[index-1]["simulation_time_s"] <= .203
                    paired += 1
                previous = row
            assert paired == e["rgbd_frames"] == len(frames["rgb"]) == len(frames["depth_m"])
    else:
        assert e["rgbd_frames"] == 0
    if "forbidden_contact_samples" in e:
        assert e["forbidden_contact_samples"] == sum(bool(r["truth_for_scoring_only"]["contact"]["forbidden"]) for r in rows)
    learned_calls, learned_actions, model_calls, gate_pairs = 0, 0, 0, 0
    with np.load(directory/"decision_observations.npz", allow_pickle=False) as frames:
        rgb, depth = frames["rgb"], frames["depth_m"]
        cameras = json.loads(str(frames["camera_json"]))
        metadata = json.loads(str(frames["metadata_json"]))
        assert len(rgb) == len(depth) == len(cameras) == len(metadata) == e["decision_frames"]
        raw_predictions, input_frames = [], []
        for i, camera in enumerate(cameras):
            assert abs(camera["simulation_time_s"]-metadata[i]["t_sim_s"]) < 1e-9
            assert np.isfinite(metadata[i]["q_feedback_rad"]).all()
            if i:
                assert metadata[i]["t_sim_s"] >= metadata[i-1]["t_sim_s"]
            frame = {"rgb":rgb[i], "depth":depth[i], "camera":camera}
            input_frames.append(frame)
            raw_predictions.append(estimate(perception, frame))
        for attempt in e["attempts"]:
            index = attempt["initial_decision_index"]
            raw = raw_predictions[index]
            same(attempt["raw_prediction"], raw)
            corrected = correct_prediction(raw, policy, input_frames[index])
            expected_initial = corrected if valid_prediction(corrected) else None
            same(attempt["prediction"], expected_initial)
            calls = attempt["policy_calls"]
            assert len([c for c in calls if c["role"] == "initial"]) == int(raw is not None and policy is not None)
            for call in calls:
                p = raw_predictions[call["decision_index"]]
                same(call["raw_prediction"], p)
                predicted = correct_prediction(p, policy, input_frames[call["decision_index"]])
                same(call["prediction"], predicted)
                model_calls += 1
                if predicted["action_source"] == "learned_bounded_visuomotor_residual":
                    delta = np.asarray(predicted["residual_xyz_m"])
                    assert np.isfinite(delta).all() and np.max(np.abs(delta)) <= BOUND_M+1e-8
                    assert np.allclose(np.asarray(predicted["raw_top_xyz_m"])+delta, predicted["top_xyz_m"])
                    learned_calls += 1
                    learned_actions += bool(call["used_for_action"])
            feedback = attempt.get("feedback")
            if feedback is not None:
                views = feedback["views"]
                wait = config.get("home_refresh_wait_s", 0.)
                prefix = 1 if wait else 0
                if wait:
                    first = views[0]
                    assert first["view_type"] == "timed_HOME_refresh" and first["view_index"] == 0
                    assert first["offset_m"] is None and first["tcp_target_xyz_m"] is None
                    same(first["q_target_rad"],HOME.tolist())
                    t0 = metadata[index]["t_sim_s"]
                    t1 = metadata[first["decision_indices"][0]]["t_sim_s"]
                    assert abs(t1-t0-wait) < .0021
                    assert all(np.allclose(row["q_command_rad"], HOME,atol=1e-12,rtol=0.)
                               for row in rows if t0 < row["t_sim_s"] <= t1)
                    if first["accepted"]:
                        assert len(views)==1
                if not wait or not views[0]["accepted"]:
                    assert prefix+2 <= len(views) <= prefix+2+int(config["use_home_fallback"])
                    for offset_index,offset in enumerate(config["view_offsets_m"]):
                        view = views[prefix+offset_index]
                        assert view["view_type"]=="offset_vantage"
                        same(view["offset_m"],offset)
                    if len(views)>prefix+2:
                        assert views[-1]["view_type"]=="home_fallback"
                        same(views[-1]["q_target_rad"],HOME.tolist())
                assert [v["view_index"] for v in views]==list(range(len(views)))
                for view in feedback["views"]:
                    indices = view["decision_indices"]
                    if not indices:
                        assert view["reason"] == "unreachable_view" and "ik_error" in view
                        continue
                    assert len(indices) == 2
                    assert abs(metadata[indices[1]]["t_sim_s"]-metadata[indices[0]]["t_sim_s"]-config["pair_gap_s"]) < .0021
                    predictions = [raw_predictions[i] for i in indices]
                    same(view["raw_predictions"], predictions)
                    for i in indices:
                        same(metadata[i]["q_command_rad"], view["q_target_rad"])
                    expected = pair_quality(predictions, [cameras[i] for i in indices], expected_initial, config)
                    same({k:view[k] for k in expected}, expected)
                    gate_pairs += 1
                selected = select_view(feedback["views"])
                assert feedback["selected_view_index"] == (None if selected is None else selected["view_index"])
                expected_calls = 0
                if selected is not None:
                    selected_index = selected["decision_indices"][-1]
                    assert feedback["selected_decision_index"] == selected_index
                    same(feedback["selected_raw_prediction"], raw_predictions[selected_index])
                    same(feedback["selected_prediction"], correct_prediction(raw_predictions[selected_index], policy, input_frames[selected_index]))
                    assert feedback["accepted"] == valid_prediction(feedback["selected_prediction"])
                    expected_calls = int(policy is not None)
                else:
                    assert not feedback["accepted"]
                assert len([c for c in calls if c["role"] == "pregrasp_feedback"]) == expected_calls
                assert feedback["used_for_plan"] == bool(e["observation_mode"] == "feedback" and attempt.get("plan"))
                if policy is not None and wait and views[0]["accepted"] and attempt.get("plan"):
                    initial_call = [c for c in calls if c["role"] == "initial"][0]
                    assert initial_call["used_for_action"] == (e["observation_mode"]=="observe_only")
                if not feedback["accepted"]:
                    assert not attempt.get("plan") and attempt["outcome"] == "reobservation_rejected"
            plan = attempt.get("plan")
            if plan is not None:
                expected_action = feedback["selected_prediction"] if feedback and feedback["used_for_plan"] else expected_initial
                same(attempt["action_prediction"], expected_action)
                top = np.asarray(expected_action["top_xyz_m"], float).copy()
                if attempt["attempt"] == 1:
                    top[1] += e["first_grasp_offset_m"]
                same(plan["target_top_xyz_m"], top.tolist())
                center, grasp = grasp_geometry(top)
                same(plan["grasp_xyz_m"], grasp.tolist())
                sim.probe.qpos[sim.qa] = plan["grasp_q_rad"]
                mj.mj_forward(sim.model, sim.probe)
                assert np.linalg.norm(sim.probe.site_xpos[sim.tip]-grasp) < .0011
                next_attempts = e["attempts"][attempt["attempt"]:]
                next_time = metadata[next_attempts[0]["initial_decision_index"]]["t_sim_s"] if next_attempts else e["sim_seconds"]
                close_rows = [r for r in rows if r["stage"] == "close"
                              and plan["action_decision_time_s"] < r["t_sim_s"] < next_time]
                assert close_rows, "A recorded grasp plan must reach the commanded close stage"
                for row in close_rows:
                    same(row["q_command_rad"], plan["grasp_q_rad"])
                destination = grasp.copy()
                destination[:2] += np.asarray(GOALS[e["task"]["destination"]])-center[:2]
                destination[2] += .002
                same(plan["destination_tcp_xyz_m"], destination.tolist())
                if feedback:
                    same(plan["action_decision_time_s"], feedback["update_time_s"])
                    delta = np.asarray(expected_action["top_xyz_m"])-expected_initial["top_xyz_m"]
                    same(feedback["plan_delta_xyz_m"], delta.tolist())
            verification = attempt.get("verification_decision_index")
            if verification is not None:
                p = correct_prediction(raw_predictions[verification])
                expected = p if valid_prediction(p) else None
                same(attempt["verification_prediction"], expected)
                if attempt["outcome"] == "visually_verified":
                    assert expected is not None
                    top = np.asarray(expected["top_xyz_m"])
                    assert np.max(np.abs(top[:2]-GOALS[e["task"]["destination"]])) < .032
                    assert .13 <= top[2] <= .155 and metadata[verification]["gripper_feedback_m"] > .04
    assert learned_calls == e["learned_inference_calls"]
    assert learned_actions == e["learned_action_calls"]
    env = e["environment_for_scoring_only"]
    if env["config"] is None:
        assert env["applied_steps"] == 0 and env["actual_impulse_ns"] == [0.,0.,0.]
    else:
        expected_impulse = np.asarray(env["config"]["force_world_n"])*env["applied_steps"]*sim.model.opt.timestep
        same(env["actual_impulse_ns"], expected_impulse.tolist())
        if env["applied_steps"]:
            assert abs(env["actual_end_s"]-env["actual_start_s"]-env["applied_steps"]*sim.model.opt.timestep) < 1e-8
            assert abs(env["actual_start_s"]-env["config"]["start_s"]) <= sim.model.opt.timestep+1e-8
        if e["sim_seconds"] > env["config"]["start_s"]+env["config"]["duration_s"]:
            assert abs(env["applied_steps"]*sim.model.opt.timestep-env["config"]["duration_s"]) <= sim.model.opt.timestep+1e-8
    return {"rows":len(rows), "paired_frames":paired, "decision_frames":e["decision_frames"],
            "maximum_command_speed_rad_s":max_speed, "model_calls":model_calls,
            "learned_action_calls":learned_actions, "gate_pairs":gate_pairs,
            "physics_steps":physics["total_steps"], "forbidden_steps":physics["forbidden_steps"]}, rows


def audit(run):
    root = ROOT/"outputs"/run
    frozen = json.loads((root/"frozen_evaluation.json").read_text())
    result = json.loads((root/"summary.json").read_text())
    provenance = json.loads((root/"provenance.json").read_text())
    for name, sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name) == sha
        assert digest(ROOT/name) == sha, f"Source changed after freeze: {name}"
    manifest = json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/name) == sha for name,sha in manifest["files_sha256"].items())
    assert digest(frozen["perception_path"]) == frozen["perception_sha256"]
    assert digest(frozen["policy_path"]) == frozen["policy_sha256"]
    assert digest(root/"prior_seed_ledger.json") == frozen["prior_seed_ledger_sha256"]
    assert digest(root/"config.json") == frozen["config_sha256"]
    ledger = json.loads((root/"prior_seed_ledger.json").read_text())
    for item in ledger["sources"]:
        assert digest(item["path"]) == item["sha256"], item["path"]
    excluded = set(ledger["seeds"])
    for item in frozen["frozen_datasets"]:
        assert digest(item["path"]) == item["sha256"]
        with np.load(item["path"], allow_pickle=False) as data:
            excluded.update(data["seeds"].tolist())
    for seeds in frozen["test_seeds"].values():
        assert not excluded.intersection(seeds)
    # Target selection is structurally separate from independent simulator data.
    for function in (run_task, collect_feedback, correct_prediction, grasp_geometry):
        source = inspect.getsource(function)
        for forbidden in ("sim.obj", "sim.og", "sim.half_size", ".xpos", ".xmat", "sim.contacts("):
            assert forbidden not in source, (function.__name__, forbidden)
    perception, policy = perception_model(frozen["perception_path"]), load(frozen["policy_path"])
    sim = Simulation(ROOT/"assets/task_scene.xml")
    counters, traces = [], {}
    try:
        episodes = [e for groups in result["groups"].values() for group in groups.values() for e in group["episodes"]]
        episodes += result["negative_controls"]
        for episode in episodes:
            directory = root/episode["artifact_directory"]
            metrics, rows = audit_episode(directory, episode, sim, perception, policy, frozen["feedback_config"])
            counters.append(metrics)
            traces[episode["artifact_directory"]] = rows
        prefix_pairs = 0
        for block, groups in result["groups"].items():
            for mode, group in groups.items():
                assert [e["reset"]["seed"] for e in group["episodes"]] == frozen["test_seeds"][block]
                assert [e["task"]["destination"] for e in group["episodes"]] == [list(GOALS)[i%3] for i in range(len(group["episodes"]))]
                same(group["summary"], group_summary(group["episodes"]))
                expected_pulse = frozen["force_pulse"] if block == "disturbed" else None
                assert all(e["environment_for_scoring_only"]["config"] == expected_pulse for e in group["episodes"])
            for reference, candidate in zip(groups["observe_only"]["episodes"], groups["feedback"]["episodes"], strict=True):
                same(reference["reset"], candidate["reset"])
                same(reference["task"], candidate["task"])
                for key in ("config", "applied_steps", "actual_impulse_ns", "actual_start_s", "actual_end_s"):
                    same(reference["environment_for_scoring_only"][key], candidate["environment_for_scoring_only"][key])
                a, b = reference["attempts"][0], candidate["attempts"][0]
                same(a["prediction"], b["prediction"])
                if "feedback" not in a:
                    assert "feedback" not in b and not a["prediction"]
                    continue
                af, bf = a["feedback"], b["feedback"]
                same(af["views"], bf["views"])
                same(af["update_time_s"], bf["update_time_s"])
                end = af["update_time_s"]
                prefix_a = [r for r in traces[reference["artifact_directory"]] if r["t_sim_s"] <= end+1e-9]
                prefix_b = [r for r in traces[candidate["artifact_directory"]] if r["t_sim_s"] <= end+1e-9]
                same(prefix_a, prefix_b)
                prefix_pairs += 1
            same([(e["reset"],e["task"]) for e in groups["v1"]["episodes"]],
                 [(e["reset"],e["task"]) for e in groups["feedback"]["episodes"]])
        expected = complete_summary(json.loads(json.dumps(result)))
        same(result["paired_comparisons"], expected["paired_comparisons"])
        same(result["acceptance_checks"], expected["acceptance_checks"])
        assert result["acceptance_met"] == expected["acceptance_met"]
        report = {"passed":True, "episodes":len(episodes), "matched_motion_prefix_pairs":prefix_pairs,
                  "maximum_command_speed_rad_s":max(c["maximum_command_speed_rad_s"] for c in counters),
                  "sums":{key:sum(c[key] for c in counters) for key in counters[0] if key != "maximum_command_speed_rad_s"},
                  "frozen_dataset_hashes_checked":len(frozen["frozen_datasets"]),
                  "original_asset_files_verified":len(manifest["files_sha256"]),
                  "prior_seed_evidence_files":len(ledger["sources"]),
                  "no_prior_seed_overlap":True, "false_successes":sum(e["false_success"] for e in episodes),
                  "acceptance_met":result["acceptance_met"],
                  "scope":"Audit correctness of recorded evidence; passing this audit does not imply acceptance or hardware readiness."}
        write(root/"audit.json", report)
        print(json.dumps(report), flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    audit(parser.parse_args().run)
