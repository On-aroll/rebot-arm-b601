"""Fixed known-scene physical probes of self-geometry-selected observations."""
import argparse
from datetime import datetime, timezone
import inspect
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np
from PIL import Image

from feedback import pair_quality
from feedback_runner import json_seeds
from policy import load, valid_prediction
from self_visibility import OFFSETS, ROBOT_GROUP, SelfVisibility
from sim import HOME, ROOT, Simulation, mj
from task import Recorder
from task_runner import create_run, digest, perception_model, write
from vision import estimate

CANONICAL = (ROOT/"outputs").resolve().parent
PROTOCOL_SHA = "17f96b571a15676f54b8a87c1ee1dfd06efaf6a5ec9386382da4313b16ad5e71"
SEEDS = [8100000, 11100009, 16400002, 22000018, 28000016, 30000029]
V3_MANIFEST = "bb6bebf40eb2b660f66600c5be2b41b7e13bee6529bcc042e247a363bcb1342b"
MODELS = {"perception.pt": "e911a373325072eb9525031fa57f4270a6f8b6f853a5c6d370eb5ffd5d17bd37",
          "residual.pt": "d2de74cdbe37694cae299abf4b3b90e4dee117bba0a8c74ef658a7f4be5f80b6"}
EXTRA = ("self_visibility.py", "self_visibility_probe.py", "audit_frozen.py", "BRANCH_BASE.json")


def truth_snapshot(sim):
    center = sim.data.xpos[sim.obj].copy()
    top = center.copy()
    top[2] += np.abs(sim.data.xmat[sim.obj].reshape(3, 3)[2]).sum()*sim.half_size
    return {"top_xyz_m": top.tolist(), "center_xyz_m": center.tolist()}


class ProbeRecorder(Recorder):
    def __init__(self, *args):
        super().__init__(*args)
        self.finger_contact_steps = 0
        self.scoring_only = []

    def tick(self):
        super().tick()
        contact = self.sim.contacts()
        self.finger_contact_steps += bool(contact["left"] or contact["right"])

    def capture_decision(self, stage):
        frame, index = super().capture_decision(stage)
        self.scoring_only.append(truth_snapshot(self.sim))
        return frame, index


def build_plans(sim, perception, policy, planner):
    plans, frames = [], []
    for seed in SEEDS:
        sim.reset(seed)
        frame = sim.observe(labels=False)
        raw = estimate(perception, frame)
        anchor = None if not valid_prediction(raw) else policy.correct(raw, frame)
        valid = valid_prediction(anchor)
        q, grip = sim.data.qpos[sim.qa].copy(), float(sim.data.qpos[sim.gq])
        # No reset metadata, seed, true object state or labels enter planner.
        query = None if not valid else planner.plan(frame["camera"], anchor["top_xyz_m"], q, grip)
        plans.append({"seed": seed, "initial_raw_prediction": raw, "anchor_prediction": anchor,
            "camera": frame["camera"], "own_q_rad": q.tolist(), "own_gripper_m": grip,
            "query": query, "reason": "planned" if valid else "no_valid_initial_visual_anchor"})
        frames.append(frame)
    return plans, frames


def capture_pair(recorder, perception, policy, planner, anchor, q, config, stage):
    raw, corrected, cameras, indices, visibility = [], [], [], [], []
    for i in range(2):
        if i:
            recorder.move(q, .046, config["pair_gap_s"], stage+"_hold")
        frame, index = recorder.capture_decision(stage+f"_{i}")
        prediction = estimate(perception, frame)
        raw.append(prediction)
        corrected.append(None if prediction is None else policy.correct(prediction, frame))
        cameras.append(frame["camera"]); indices.append(index)
        own_q = recorder.sim.data.qpos[recorder.sim.qa].copy()
        grip = float(recorder.sim.data.qpos[recorder.sim.gq])
        visibility.append(planner.forecast(frame["camera"], anchor["top_xyz_m"], own_q, grip))
    return {"raw_predictions": raw, "corrected_predictions": corrected, "decision_indices": indices,
            "cameras": cameras, "own_visibility": visibility,
            "quality": pair_quality(raw, cameras, anchor, config)}


def independent_scores(recorder, result, reset):
    output = {"truth_for_scoring_only": recorder.scoring_only, "pairs": {},
              "maximum_object_xy_displacement_m": max((np.linalg.norm(np.asarray(r["truth_for_scoring_only"]["object_xyz_m"][:2])-reset["initial_xyz"][:2]) for r in recorder.rows), default=0.)}
    output["maximum_object_xy_displacement_m"] = float(output["maximum_object_xy_displacement_m"])
    for name in ("home_pair", "view_pair"):
        pair = result[name]
        if pair is None:
            output["pairs"][name] = None
            continue
        scores = []
        for j, index in enumerate(pair["decision_indices"]):
            truth = np.asarray(recorder.scoring_only[index]["top_xyz_m"])
            item = {"decision_index": index}
            for key, pred in (("raw", pair["raw_predictions"][j]), ("V3", pair["corrected_predictions"][j])):
                item[key+"_error_mm"] = None if pred is None else float(np.linalg.norm(np.asarray(pred["top_xyz_m"])-truth)*1000)
            scores.append(item)
        output["pairs"][name] = scores
    return output


def run_case(sim, perception, policy, planner, config, plan, case, directory):
    directory.mkdir()
    reset = sim.reset(case["seed"])
    recorder = ProbeRecorder(sim, directory, False)
    initial, _ = recorder.capture_decision("initial_HOME")
    raw = estimate(perception, initial)
    anchor = None if not valid_prediction(raw) else policy.correct(raw, initial)
    assert raw == plan["initial_raw_prediction"] and anchor == plan["anchor_prediction"]
    result = {**case, "reset": reset, "initial_raw_prediction": raw, "anchor_prediction": anchor,
        "home_pair": None, "view_pair": None, "q_target_rad": None, "would_trigger": False,
        "reason": "no_valid_initial_visual_anchor"}
    if valid_prediction(anchor):
        recorder.move(HOME, .046, config["home_refresh_wait_s"], "timed_HOME_refresh")
        result["home_pair"] = capture_pair(recorder, perception, policy, planner, anchor, HOME, config, "home")
        result["would_trigger"] = not result["home_pair"]["quality"]["accepted"]
        candidate = None if case["candidate_index"] is None else plan["query"]["candidates"][case["candidate_index"]]
        if candidate is None or candidate["q_target_rad"] is None:
            result["reason"] = "unavailable_planned_view"
        else:
            q = np.asarray(candidate["q_target_rad"])
            result.update(q_target_rad=q.tolist(), tcp_target_xyz_m=candidate["tcp_target_xyz_m"],
                          planned_visibility=candidate["forecast"])
            recorder.move(q, .046, 2., "independent_planned_observation")
            recorder.move(q, .046, config["settle_s"], "view_settle")
            result["view_pair"] = capture_pair(recorder, perception, policy, planner, anchor, q, config, "view")
            result["reason"] = result["view_pair"]["quality"]["reason"]
    recorder.save()
    Image.fromarray(initial["rgb"]).save(directory/"initial.png")
    Image.fromarray(recorder.decision_frames[-1]["rgb"]).save(directory/"last.png")
    speeds = [float(np.max(np.abs(np.asarray(b["q_command_rad"])-a["q_command_rad"])/(b["t_sim_s"]-a["t_sim_s"])))
              for a, b in zip(recorder.rows, recorder.rows[1:])]
    result.update(sim_seconds=float(sim.data.time), record_rows=len(recorder.rows), rgbd_frames=len(recorder.frames),
        decision_frames=len(recorder.decision_frames), physics_tick_audit=recorder.physics_audit,
        finger_contact_steps_for_scoring_only=recorder.finger_contact_steps,
        maximum_command_speed_rad_s=max(speeds, default=0.),
        independent_score_only=independent_scores(recorder, result, reset))
    write(directory/"result.json", result)
    return result


def evaluate(plans, results):
    comparisons = []
    for plan in plans:
        cases = [r for r in results if r["seed"] == plan["seed"]]
        assert len(cases) == 4 and all(c["home_pair"] == cases[0]["home_pair"] for c in cases)
        original = [c for c in cases if c["group"] == "original"]
        ranked = [c for c in cases if c["group"] == "ranked"]
        old_forecasts = [c["forecast"]["blocked_rays"] for c in plan["query"]["candidates"][:2] if c["forecast"] is not None] if plan["query"] else []
        best_old_blocked = min(old_forecasts, default=None)

        def reliable(case, require_visibility):
            p = case["view_pair"]
            error = case["independent_score_only"]["pairs"]["view_pair"]
            return bool(p and p["quality"]["accepted"] and valid_prediction(p["corrected_predictions"][-1])
                and error[-1]["V3_error_mm"] <= 5 and (not require_visibility or best_old_blocked is not None
                and p["own_visibility"][-1]["blocked_rays"] <= best_old_blocked))

        before, after = any(reliable(c, False) for c in original), any(reliable(c, True) for c in ranked)
        bad_accepts = [c["candidate_index"] for c in ranked if c["view_pair"] and c["view_pair"]["quality"]["accepted"]
                      and c["independent_score_only"]["pairs"]["view_pair"][-1]["V3_error_mm"] > 5]
        comparisons.append({"seed": plan["seed"], "would_trigger": cases[0]["would_trigger"],
            "original_has_reliable_view": before, "ranked_has_reliable_view": after,
            "new_reliable_scene": not before and after, "lost_reliable_scene": before and not after,
            "best_original_predicted_blocked_rays": best_old_blocked,
            "ranked_accepted_above5mm_candidate_indices": bad_accepts,
            "selected_candidate_indices": [] if not plan["query"] else plan["query"]["selected_candidate_indices"]})
    trigger = [c for c in comparisons if c["would_trigger"]]
    checks = {"all_24_probes_retained": len(results) == 24,
        "zero_forbidden_contacts": all(c["physics_tick_audit"]["forbidden_steps"] == 0 for c in results),
        "zero_finger_contact_steps": all(c["finger_contact_steps_for_scoring_only"] == 0 for c in results),
        "maximum_speed_within_0_40001": all(c["maximum_command_speed_rad_s"] <= .40001 for c in results),
        "object_displacement_at_most2mm": all(c["independent_score_only"]["maximum_object_xy_displacement_m"] <= .002 for c in results),
        "at_least_one_new_reliable_triggered_scene": any(c["new_reliable_scene"] for c in trigger),
        "no_lost_reliable_triggered_scene": not any(c["lost_reliable_scene"] for c in trigger),
        "no_ranked_accepted_above5mm_in_triggered_scene": not any(c["ranked_accepted_above5mm_candidate_indices"] for c in trigger)}
    return {"comparisons": comparisons, "triggered_scenes": len(trigger), "checks": checks,
            "mechanism_gates_passed": all(checks.values()), "requires_full_audit": True,
            "adopted": False, "full_task_executions": 0}


def save_initial(path, frames):
    np.savez_compressed(path, rgb=np.asarray([f["rgb"] for f in frames]),
        depth_m=np.asarray([f["depth"] for f in frames]), camera_json=json.dumps([f["camera"] for f in frames]))


def verify(root, config):
    assert config["seeds"] == SEEDS and config["offsets_m"] == OFFSETS
    assert digest(root/"protocol.md") == config["protocol_sha256"] == PROTOCOL_SHA
    for name, sha in MODELS.items():
        assert digest(root/"frozen"/name) == sha == config["model_sha256"][name]
    assert digest(root/"frozen/feedback_config.json") == config["feedback_config_sha256"]
    assert json.loads((root/"frozen/feedback_config.json").read_text()) == config["feedback_config"]
    for path, sha in config["reference_evidence_sha256"].items():
        assert digest(path) == sha


def main(name):
    v3 = CANONICAL/"versions/V3"
    assert digest(v3/"release_manifest.json") == V3_MANIFEST
    manifest = json.loads((v3/"release_manifest.json").read_text())
    assert all(digest(v3/p) == h for p, h in manifest["files_sha256"].items())
    root = create_run(name, v3/"models/perception.pt")
    provenance = json.loads((root/"provenance.json").read_text())
    for n in EXTRA:
        shutil.copy2(ROOT/n, root/"source"/n); provenance["source_sha256"][n] = digest(ROOT/n)
    write(root/"provenance.json", provenance)
    shutil.copy2(CANONICAL/"SELF_VISIBILITY_DEVELOPMENT_PROTOCOL.md", root/"protocol.md")
    (root/"frozen").mkdir()
    for n in MODELS:
        shutil.copy2(v3/"models"/n, root/"frozen"/n)
    shutil.copy2(v3/"feedback_config.json", root/"frozen/feedback_config.json")
    references = [v3/"release_manifest.json", CANONICAL/"outputs/image-residual-known-tasks-01/audit.json"]
    references += [CANONICAL/"outputs"/r/"summary.json" for r in
        ("geometry-fallback-known-development-01", "goal-geometry-fallback-v3-candidate-01", "goal-image-residual-v4-candidate-01")]
    known = set().union(*(json_seeds(json.loads(p.read_text())) for p in references[2:]))
    assert set(SEEDS) <= known, sorted(set(SEEDS)-known)
    config = {"schema": "b601-self-visibility-known-probe-v1", "hardware_executable": False,
        "development_only": True, "full_task_test": False, "protocol_sha256": digest(root/"protocol.md"),
        "frozen_at_utc": datetime.now(timezone.utc).isoformat(),
        "seeds": SEEDS, "offsets_m": OFFSETS, "model_sha256": MODELS,
        "feedback_config": json.loads((root/"frozen/feedback_config.json").read_text()),
        "feedback_config_sha256": digest(root/"frozen/feedback_config.json"),
        "reference_evidence_sha256": {str(p): digest(p) for p in references}}
    write(root/"config.json", config)
    verify(root, config)
    sim = Simulation(ROOT/"assets/task_scene.xml")
    perception, policy = perception_model(root/"frozen/perception.pt"), load(root/"frozen/residual.pt")
    planner = SelfVisibility(ROOT/"assets/task_scene.xml")
    try:
        plans, frames = build_plans(sim, perception, policy, planner)
        write(root/"plans.json", plans); save_initial(root/"planning_observations.npz", frames)
        cases = []
        for plan in plans:
            selected = [] if plan["query"] is None else plan["query"]["selected_candidate_indices"]
            for group, indices in (("original", [0, 1]), ("ranked", [*selected, *([None]*(2-len(selected)))])):
                for rank, index in enumerate(indices):
                    cases.append({"seed": plan["seed"], "group": group, "rank": rank, "candidate_index": index})
        # Every plan/selection is frozen before any new physical probe.
        write(root/"frozen_probe_plan.json", {"cases": cases, "selected_before_physics_at_utc": datetime.now(timezone.utc).isoformat(),
            "plans_sha256": digest(root/"plans.json"),
            "planning_observations_sha256": digest(root/"planning_observations.npz")})
        result = {"schema": config["schema"], "complete": False, "cases": [],
            "config_sha256": digest(root/"config.json"), "frozen_plan_sha256": digest(root/"frozen_probe_plan.json")}
        by_seed = {p["seed"]: p for p in plans}
        for i, case in enumerate(cases):
            row = run_case(sim, perception, policy, planner, config["feedback_config"], by_seed[case["seed"]], case, root/f"case-{i:03d}")
            result["cases"].append(row); write(root/"summary.json", result)
            print(json.dumps({**case, "would_trigger": row["would_trigger"], "reason": row["reason"],
                "score": row["independent_score_only"]["pairs"]["view_pair"],
                "forbidden_steps": row["physics_tick_audit"]["forbidden_steps"]}), flush=True)
        result.update(complete=True, review=evaluate(plans, result["cases"]))
        write(root/"summary.json", result)
        print(json.dumps(result["review"]), flush=True)
    finally:
        sim.close()


def check_rays(planner, plan, forecast, q, grip):
    planner.set_robot(q, grip)
    point = np.asarray(forecast["ray_origin_xyz_m"])
    actual = []
    for v, expected_id, expected_d in zip(forecast["ray_directions"], forecast["ray_geom_ids"], forecast["ray_distances_m"], strict=True):
        gid = np.empty(1, np.int32)
        distance = mj.mj_ray(planner.model, planner.data, point, np.asarray(v), ROBOT_GROUP, True, -1, gid)
        assert int(gid[0]) == expected_id and abs(distance-expected_d) < 1e-12
        actual.append(distance)
    points = np.asarray(forecast["roi_points_xyz_m"])
    top = np.asarray(plan["anchor_prediction"]["top_xyz_m"])
    half = (top[2]-.1)/np.sqrt(2)+.008
    expected_points = np.asarray([[top[0]+ix*half/4, top[1]+iy*half/4, top[2]]
                                 for iy in range(-4, 5) for ix in range(-4, 5)])
    np.testing.assert_allclose(points, expected_points, rtol=0, atol=1e-15)
    np.testing.assert_allclose(point, np.asarray(plan["camera"]["T_base_camera_optical"])[:3, 3], rtol=0, atol=0)
    np.testing.assert_allclose(np.asarray(forecast["ray_directions"]), (points-point)/np.linalg.norm(points-point, axis=1)[:, None], rtol=0, atol=1e-15)
    lengths = np.linalg.norm(points-point, axis=1)
    expected = sum(d >= 0 and d < length-.002 for d, length in zip(actual, lengths))
    assert expected == forecast["blocked_rays"]
    assert np.allclose(points[:, 2], plan["anchor_prediction"]["top_xyz_m"][2], atol=0, rtol=0)
    return len(actual)


def audit(run):
    root = run.resolve()
    config, result = (json.loads((root/n).read_text()) for n in ("config.json", "summary.json"))
    assert result["complete"] and result["config_sha256"] == digest(root/"config.json")
    verify(root, config)
    provenance = json.loads((root/"provenance.json").read_text())
    for name, sha in provenance["source_sha256"].items():
        assert digest(ROOT/name) == sha == digest(root/"source"/Path(name).name), name
    manifest = json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p) == h for p, h in manifest["files_sha256"].items())
    frozen = json.loads((root/"frozen_probe_plan.json").read_text())
    assert result["frozen_plan_sha256"] == digest(root/"frozen_probe_plan.json")
    assert frozen["plans_sha256"] == digest(root/"plans.json")
    assert frozen["planning_observations_sha256"] == digest(root/"planning_observations.npz")
    assert all(t not in inspect.getsource(run_case)+inspect.getsource(build_plans)+inspect.getsource(capture_pair)
               for t in ("sim.obj", "sim.og", "sim.half_size", ".xpos", ".xmat", "sim.contacts("))
    sim = Simulation(ROOT/"assets/task_scene.xml")
    perception, policy = perception_model(root/"frozen/perception.pt"), load(root/"frozen/residual.pt")
    planner = SelfVisibility(ROOT/"assets/task_scene.xml")
    try:
        plans, frames = build_plans(sim, perception, policy, planner)
        assert plans == json.loads((root/"plans.json").read_text())
        with tempfile.TemporaryDirectory(prefix="b601-self-visibility-audit-") as temporary:
            temporary = Path(temporary)
            save_initial(temporary/"planning.npz", frames)
            with np.load(temporary/"planning.npz", allow_pickle=False) as a, np.load(root/"planning_observations.npz", allow_pickle=False) as b:
                assert a.files == b.files and all(np.array_equal(a[k], b[k]) for k in a.files)
            rays, candidates = 0, 0
            for plan in plans:
                if plan["query"] is None:
                    continue
                for candidate in plan["query"]["candidates"]:
                    candidates += 1
                    if candidate["forecast"]:
                        rays += check_rays(planner, plan, candidate["forecast"], candidate["q_target_rad"], plan["own_gripper_m"])
                        assert np.linalg.norm(planner.data.site_xpos[planner.tip]-candidate["tcp_target_xyz_m"]) < .001
            for i, (case, expected) in enumerate(zip(frozen["cases"], result["cases"], strict=True)):
                plan = next(p for p in plans if p["seed"] == case["seed"])
                directory = temporary/f"case-{i:03d}"
                actual = run_case(sim, perception, policy, planner, config["feedback_config"], plan, case, directory)
                assert actual == expected, i
                original = root/f"case-{i:03d}"
                for n in ("trajectory.jsonl", "final_score_trace.jsonl", "result.json"):
                    assert (directory/n).read_bytes() == (original/n).read_bytes(), (i, n)
                for n in ("observations.npz", "decision_observations.npz"):
                    assert (directory/n).exists() == (original/n).exists()
                    if (directory/n).exists():
                        with np.load(directory/n, allow_pickle=False) as a, np.load(original/n, allow_pickle=False) as b:
                            assert a.files == b.files and all(np.array_equal(a[k], b[k]) for k in a.files), (i, n)
                with np.load(original/"decision_observations.npz", allow_pickle=False) as observations:
                    decision_meta = json.loads(str(observations["metadata_json"]))
                for pair_name in ("home_pair", "view_pair"):
                    pair = actual[pair_name]
                    if pair is None:
                        continue
                    for index, forecast in zip(pair["decision_indices"], pair["own_visibility"], strict=True):
                        own = decision_meta[index]
                        rays += check_rays(planner, plan, forecast, own["q_feedback_rad"], own["gripper_feedback_m"])
                print(json.dumps({"audited_physical_probe": i+1}), flush=True)
            assert evaluate(plans, result["cases"]) == result["review"]
        report = {"passed": True, "physical_probes_exactly_reexecuted": len(result["cases"]),
            "candidate_plans_checked": candidates, "independent_single_rays_checked": rays,
            "exact_RGBD_action_score_reexecution": True, "source_files_checked": len(provenance["source_sha256"]),
            "assets_checked": len(manifest["files_sha256"]),
            "physics_steps_reexecuted": sum(c["physics_tick_audit"]["total_steps"] for c in result["cases"]),
            "summary_sha256": digest(root/"summary.json"), "plans_sha256": digest(root/"plans.json"),
            "eligible_for_new_task_development": result["review"]["mechanism_gates_passed"], "adopted": False,
            "scope": "Six known scenes/24 independent observation probes; every visual/own-state plan, individual rays and exact physical RGBD/action/score rerun. Not a sequential controller, full-task test, fresh acceptance or hardware evidence."}
        write(root/"audit.json", report); print(json.dumps(report), flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--name")
    mode.add_argument("--audit-run", type=Path)
    args = parser.parse_args()
    audit(ROOT/"outputs"/args.audit_run) if args.audit_run else main(args.name)
