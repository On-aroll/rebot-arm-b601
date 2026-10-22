"""Development probe: timed RGB-D refresh while the robot physically holds HOME.

This does not execute a pick/place task. Object truth is recorded only after
inference, for localization scoring. The fixed waits do not read disturbance
state, object state, or scoring results. Existing formal runs stay unchanged.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

import numpy as np

from feedback import configuration, pair_quality
from policy import load, valid_prediction
from sim import HOME, ROOT, Simulation
from task import Recorder, correct_prediction
from task_runner import create_run, digest, perception_datasets, perception_model, write
from vision import estimate


def run_case(sim, perception, policy, *, seed, absent, pulse, wait, feedback_config,
             directory=None):
    reset = sim.reset(seed, absent=absent, force_pulse=pulse)
    recorder = Recorder(sim, directory or Path("."), False)
    initial, initial_index = recorder.capture_decision("initial_HOME")
    initial_raw = estimate(perception, initial)
    initial_prediction = correct_prediction(initial_raw, policy)
    recorder.move(HOME, .046, wait, "hold_HOME_before_refresh")
    predictions, cameras, indices, scores = [], [], [], []
    for i in range(2):
        if i:
            recorder.move(HOME, .046, feedback_config["pair_gap_s"], "HOME_refresh_pair_gap")
        frame, index = recorder.capture_decision(f"HOME_refresh_frame_{i}")
        raw = estimate(perception, frame)
        corrected = correct_prediction(raw, policy)
        predictions.append(raw); cameras.append(frame["camera"]); indices.append(index)
        # Independent scorer only. Neither the wait nor the gate consumes this.
        center = sim.data.xpos[sim.obj].copy()
        rotation = sim.data.xmat[sim.obj].reshape(3, 3)
        top = center.copy(); top[2] += np.abs(rotation[2]) @ np.full(3, sim.half_size)
        scores.append({"t_sim_s":float(sim.data.time), "absent":absent,
            "truth_center_xyz_m":center.tolist(), "truth_rotation":rotation.tolist(),
            "truth_half_size_m":sim.half_size, "truth_bounding_top_center_xyz_m":top.tolist(),
            "raw_error_mm":None if absent or raw is None else float(np.linalg.norm(np.asarray(raw["top_xyz_m"])-top)*1000),
            "corrected_error_mm":None if absent or corrected is None else float(np.linalg.norm(np.asarray(corrected["top_xyz_m"])-top)*1000)})
    # Without an initial target, the last refreshed raw estimate can be used as
    # an anchor for the same visual workspace/pair checks. No physical action
    # follows in this diagnostic; full-task behavior needs its own test.
    anchor = initial_prediction if valid_prediction(initial_prediction) else predictions[-1]
    gate = pair_quality(predictions, cameras, anchor, feedback_config) if valid_prediction(anchor) else None
    frames = recorder.decision_frames
    row = {"seed":seed, "absent":absent, "reset":reset, "wait_s":wait,
        "pulse":pulse, "initial_decision_index":initial_index,
        "initial_raw_prediction":initial_raw, "initial_prediction":initial_prediction,
        "decision_indices":indices, "raw_predictions":predictions, "pair_gate":gate,
        "refresh_anchor":"initial_prediction" if valid_prediction(initial_prediction) else "last_refresh_raw",
        "scoring_only":scores, "environment_for_scoring_only":sim.environment_audit(),
        "physics_tick_audit":recorder.physics_audit, "sim_seconds":float(sim.data.time),
        "record_rows":len(recorder.rows), "rgbd_frames":len(recorder.frames),
        "decision_frames":len(frames), "full_task_test":False, "hardware_executable":False}
    if directory:
        recorder.save(); write(Path(directory)/"result.json",row)
    return row, recorder


def summarize(rows):
    output = {}
    for block in ("static", "disturbed", "empty"):
        for wait in sorted({r["wait_s"] for r in rows}):
            items = [r for r in rows if r["block"]==block and r["wait_s"]==wait]
            if not items:
                continue
            detected = [r for r in items if valid_prediction(r["raw_predictions"][-1])]
            accepted = [r for r in items if r["pair_gate"] and r["pair_gate"]["accepted"]]
            errors = [r["scoring_only"][-1]["corrected_error_mm"] for r in detected if not r["absent"]]
            output[f"{block}-wait-{wait}"] = {"observations":len(items),
                "initial_detected":sum(valid_prediction(r["initial_raw_prediction"]) for r in items),
                "refreshed_detected":len(detected), "pair_accepted":len(accepted),
                "mean_detected_error_mm":float(np.mean(errors)) if errors else None,
                "max_detected_error_mm":max(errors) if errors else None,
                "forbidden_steps":sum(r["physics_tick_audit"]["forbidden_steps"] for r in items)}
    return output


def main(args):
    perception_path, policy_path = args.perception.resolve(), args.policy.resolve()
    assert json.loads((policy_path.parent/"summary.json").read_text())["perception_sha256"]==digest(perception_path)
    # The concurrently running formal test has an immutable history snapshot.
    # Never hash its mutable summary as a supposedly fixed prior record.
    reference = args.reference_run.resolve()
    evidence = [{"path":str(reference/name), "sha256":digest(reference/name)}
                for name in ("prior_seed_ledger.json", "frozen_evaluation.json")]
    ledger = json.loads((reference/"prior_seed_ledger.json").read_text())
    evaluation = json.loads((reference/"frozen_evaluation.json").read_text())
    excluded = set(ledger["seeds"])
    excluded.update(s for seeds in evaluation["test_seeds"].values() for s in seeds)
    datasets = perception_datasets(perception_path)+[
        {"path":str(policy_path.parent/p), "sha256":digest(policy_path.parent/p)} for p in ("training.npz","validation.npz")]
    for item in datasets:
        with np.load(item["path"], allow_pickle=False) as data:
            excluded.update(data["seeds"].tolist())
    seeds = list(range(args.seed_start, args.seed_start+args.scenes))
    empty_seeds = list(range(args.empty_seed_start, args.empty_seed_start+8))
    assert len(set(seeds+empty_seeds))==len(seeds+empty_seeds)
    assert not excluded.intersection(seeds+empty_seeds), "Use fresh development seeds"
    config = configuration(json.loads(args.feedback_config.read_text()))
    pulse = json.loads(args.force_pulse.read_text())
    root = create_run(args.name, perception_path)
    provenance = json.loads((root/"provenance.json").read_text())
    for source in ("home_refresh_probe.py", "home_refresh_audit.py"):
        shutil.copy2(ROOT/source, root/"source"/source)
        provenance["source_sha256"][source] = digest(ROOT/source)
    write(root/"provenance.json", provenance)
    frozen = root/"frozen"; frozen.mkdir()
    shutil.copy2(perception_path,frozen/"perception.pt")
    shutil.copy2(policy_path,frozen/"residual.pt")
    plan = {"schema":"b601-home-refresh-development-v1", "full_task_test":False,
        "hardware_executable":False, "frozen_at_utc":datetime.now(timezone.utc).isoformat(),
        "seeds":seeds, "empty_seeds":empty_seeds, "fixed_waits_s":[.8,1.2],
        "feedback_config":config, "force_pulse":pulse, "reference_evidence":evidence,
        "datasets":datasets, "perception_sha256":digest(frozen/"perception.pt"),
        "policy_sha256":digest(frozen/"residual.pt"),
        "scope":"Timed stationary HOME observation, not physical task success; repeated waits/blocks share scene seeds."}
    write(root/"config.json",plan)
    perception, policy = perception_model(frozen/"perception.pt"), load(frozen/"residual.pt")
    sim = Simulation(ROOT/"assets/task_scene.xml")
    rows = []
    try:
        for block in ("static","disturbed","empty"):
            for wait in plan["fixed_waits_s"]:
                for seed in empty_seeds if block=="empty" else seeds:
                    relative = f"{block}-{wait}-{seed}"
                    directory = root/relative; directory.mkdir()
                    row,_ = run_case(sim, perception, policy, seed=seed, absent=block=="empty",
                        pulse=pulse if block=="disturbed" else None, wait=wait,
                        feedback_config=config, directory=directory)
                    rows.append({**row, "block":block, "artifact_directory":relative})
                write(root/"summary.json",{**plan,"complete":False,"observations":rows,"groups":summarize(rows)})
                print(json.dumps({"block":block,"wait":wait,"group":summarize(rows)[f"{block}-wait-{wait}"]}),flush=True)
        write(root/"summary.json",{**plan,"complete":True,"observations":rows,"groups":summarize(rows)})
    finally:
        sim.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name",required=True)
    p.add_argument("--seed-start",type=int,default=14000000)
    p.add_argument("--empty-seed-start",type=int,default=14100000)
    p.add_argument("--scenes",type=int,default=30)
    p.add_argument("--reference-run",type=Path,default=ROOT/"outputs/goal-egl-coverage-v2-candidate-02")
    p.add_argument("--perception",type=Path,default=ROOT/"outputs/vision-multiview-egl-01/perception.pt")
    p.add_argument("--policy",type=Path,default=ROOT/"outputs/goal-policy-multiview-egl-01/residual.pt")
    p.add_argument("--feedback-config",type=Path,default=ROOT/"configs/feedback-coverage-development-02.json")
    p.add_argument("--force-pulse",type=Path,default=ROOT/"outputs/force-development-01/selected_pulse.json")
    args=p.parse_args()
    if not 6<=args.scenes<=60:
        raise ValueError("Use 6--60 fresh development scenes")
    main(args)
