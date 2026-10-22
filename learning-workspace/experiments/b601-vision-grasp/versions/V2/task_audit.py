"""Audit actual recorded actions, observation pairing, splits and result claims."""
import argparse
import json
from pathlib import Path

import numpy as np

from policy import BOUND_M
from sim import ROOT, Simulation
from task import GOALS
from task_runner import digest


def audit(run):
    root = ROOT/"outputs"/run
    result = json.loads((root/"summary.json").read_text())
    frozen = json.loads((root/"frozen_evaluation.json").read_text())
    assert digest(frozen["policy_path"]) == frozen["policy_sha256"]
    if "baseline_perception_path" in frozen:
        assert digest(frozen["baseline_perception_path"]) == frozen["baseline_perception_sha256"]
    provenance = json.loads((root/"provenance.json").read_text())
    assert digest(provenance["perception_checkpoint"]) == provenance["perception_sha256"]
    for name, sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name) == sha
    policy_root = Path(frozen["policy_path"]).parent
    with np.load(policy_root/"training.npz", allow_pickle=False) as d:
        training_seeds = set(d["seeds"].tolist())
    with np.load(policy_root/"validation.npz", allow_pickle=False) as d:
        validation_seeds = set(d["seeds"].tolist())
    assert not training_seeds.intersection(validation_seeds)
    used_seeds = training_seeds | validation_seeds
    for dataset in frozen.get("frozen_datasets", []):
        assert digest(dataset["path"]) == dataset["sha256"]
        with np.load(dataset["path"], allow_pickle=False) as d:
            used_seeds.update(d["seeds"].tolist())
    sim = Simulation(ROOT/"assets/task_scene.xml")
    try:
        total_rows, total_frames, max_speed, action_calls = 0, 0, 0., 0
        total_false_success = 0
        episodes = sorted(root.glob("*/result.json"))
        for file in episodes:
            e = json.loads(file.read_text())
            assert e["reset"]["seed"] not in used_seeds
            assert not e["hardware_executable"]
            assert e["success"] == bool(e["controller_success"] and e.get("physical_success", False))
            assert e.get("false_success", False) == bool(e["controller_success"] and not e.get("physical_success", False))
            total_false_success += e.get("false_success", False)
            assert e["retries"] <= 2 and len(e["attempts"]) <= 3
            for a in e["attempts"]:
                p = a["prediction"]
                if p and p["action_source"] == "learned_bounded_visuomotor_residual":
                    delta = np.asarray(p["residual_xyz_m"])
                    assert np.isfinite(delta).all() and np.max(np.abs(delta)) <= BOUND_M+1e-8
                    assert np.allclose(np.asarray(p["raw_top_xyz_m"])+delta, p["top_xyz_m"])
                    action_calls += 1
            rows = [json.loads(line) for line in (file.parent/"trajectory.jsonl").read_text().splitlines()]
            assert len(rows) == e["record_rows"]
            if e["reset"]["absent"]:
                assert not rows and not any(a.get("plan") for a in e["attempts"])
            if not rows:
                continue
            with np.load(file.parent/"observations.npz", allow_pickle=False) as data:
                cameras = json.loads(str(data["camera_json"]))
                paired = 0
                previous = None
                for row in rows:
                    q = np.asarray(row["q_command_rad"])
                    assert np.isfinite(q).all() and (q >= sim.lower-1e-7).all() and (q <= sim.upper+1e-7).all()
                    assert 0 <= row["gripper_command_m"] <= .05
                    assert not row["truth_for_scoring_only"]["contact"]["forbidden"]
                    if previous:
                        dt = row["t_sim_s"]-previous["t_sim_s"]
                        assert .047 <= dt <= .053
                        speed = np.max(np.abs(q-np.asarray(previous["q_command_rad"]))) / dt
                        max_speed = max(max_speed, float(speed))
                        assert speed <= .401
                    index = row["observation_index"]
                    if index is not None:
                        assert index == paired
                        assert abs(cameras[index]["simulation_time_s"]-row["t_sim_s"]) < 1e-9
                        paired += 1
                    previous = row
                assert paired == len(data["rgb"]) == len(data["depth_m"]) == e["rgbd_frames"]
                total_rows += len(rows); total_frames += paired
        baseline, learned = result["groups"]["baseline"]["episodes"], result["groups"]["learned"]["episodes"]
        assert [(e["reset"], e["task"]) for e in baseline] == [(e["reset"], e["task"]) for e in learned]
        assert set(e["task"]["destination"] for e in learned) == set(GOALS)
        for group in result["groups"].values():
            assert [(e["reset"], e["task"]) for e in group["episodes"]] == [(e["reset"], e["task"]) for e in baseline]
            assert group["summary"]["successes"] == sum(e["success"] for e in group["episodes"])
        report = {"passed": True, "episodes":len(episodes), "trajectory_rows":total_rows,
                  "paired_rgbd_frames":total_frames, "maximum_command_speed_rad_s":max_speed,
                  "learned_grasp_corrections_checked":action_calls, "false_successes":total_false_success,
                  "training_validation_test_seed_overlap":False,
                  "frozen_dataset_hashes_checked":len(frozen.get("frozen_datasets", []))}
        (root/"audit.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps(report))
    finally:
        sim.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", required=True)
    audit(p.parse_args().run)
