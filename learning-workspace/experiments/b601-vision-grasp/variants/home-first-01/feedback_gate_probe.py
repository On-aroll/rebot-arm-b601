"""Independent development measurements of the pre-grasp observation gate.

Ground truth is used only by this probe's scoring recorder. It is never passed
to the shared observation, gate, residual, or IK code. These scenes are not full
task acceptance tests and must never be reused for final task acceptance.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image

from feedback import configuration
from policy import load, valid_prediction
from sim import ROOT, Simulation
from task import Recorder, collect_feedback, correct_prediction
from task_runner import create_run, digest, perception_datasets, perception_model, write
from vision import estimate


PERCEPTION = ROOT/"outputs/vision-iteration-03/perception.pt"
POLICY = ROOT/"outputs/goal-policy-v2/residual.pt"


def independent_truth(sim):
    """Physical top-face centre for scoring, independent of visibility gating."""
    rotation = sim.data.xmat[sim.obj].reshape(3, 3).copy()
    up_axis = int(np.argmax(np.abs(rotation[2])))
    normal = rotation[:, up_axis] * np.sign(rotation[2, up_axis])
    top = sim.data.xpos[sim.obj].copy()+normal*sim.model.geom_size[sim.og, up_axis]
    return {"top_face_center_xyz_m": top.tolist(),
            "object_center_xyz_m": sim.data.xpos[sim.obj].tolist(),
            "object_rotation": rotation.tolist(),
            "top_face_axis": up_axis,
            "top_face_up_cosine": float(normal[2])}


def score_prediction(prediction, truth):
    if prediction is None:
        return None
    error = np.asarray(prediction["top_xyz_m"])-truth["top_face_center_xyz_m"]
    return {"xyz_error_mm": float(np.linalg.norm(error)*1000),
            "xy_error_mm": float(np.linalg.norm(error[:2])*1000),
            "z_error_mm": float(error[2]*1000),
            "error_above_5mm": bool(np.linalg.norm(error) > .005)}


def error_summary(scores):
    values = [s["xyz_error_mm"] for s in scores if s is not None]
    return {"samples": len(values),
            "mean_mm": float(np.mean(values)) if values else None,
            "median_mm": float(np.median(values)) if values else None,
            "p95_mm": float(np.quantile(values, .95)) if values else None,
            "max_mm": float(np.max(values)) if values else None,
            "above_5mm": sum(v > 5. for v in values)}


def summarize(rows):
    views = [view for row in rows for view in row.get("views", [])]
    accepted = [view for view in views if view["accepted"]]
    rejected = [view for view in views if not view["accepted"]]
    selected = [row for row in rows if row.get("selected_scores") is not None]
    raw = [row["selected_scores"]["raw"] for row in selected]
    corrected = [row["selected_scores"]["corrected"] for row in selected]
    return {
        "scenes": len(rows),
        "initial_detection_misses": sum(not row["initial_detection"] for row in rows),
        "motion_errors": sum("error" in row for row in rows),
        "scenes_with_selected_view": len(selected),
        "retained_scene_fraction_including_initial_misses": len(selected)/len(rows),
        "no_selected_view_scenes": len(rows)-len(selected),
        "initial_raw": error_summary([row.get("initial_scores", {}).get("raw") for row in rows]),
        "initial_corrected": error_summary([row.get("initial_scores", {}).get("corrected") for row in rows]),
        "view_pairs": len(views),
        "accepted_view_pairs": len(accepted),
        "rejected_view_pairs": len(rejected),
        "accepted_pair_frames_raw": error_summary([s["raw"] for v in accepted for s in v["independent_scores"]]),
        "accepted_pair_frames_counterfactual_corrected": error_summary(
            [s["corrected"] for v in accepted for s in v["independent_scores"]]),
        "accepted_pairs_with_any_raw_error_above_5mm": sum(
            any(s["raw"] is not None and s["raw"]["error_above_5mm"] for s in v["independent_scores"])
            for v in accepted),
        "rejected_pairs_with_both_raw_errors_at_most_5mm": sum(
            len(v["independent_scores"]) == 2 and all(s["raw"] is not None and not s["raw"]["error_above_5mm"]
                                                     for s in v["independent_scores"])
            for v in rejected),
        "selected_view_counts": {str(i): sum(row["selected_view_index"] == i for row in selected)
                                 for i in range(3)},
        "selected_raw": error_summary(raw),
        "selected_corrected": error_summary(corrected),
        "selected_correction_regressions": sum(c["xyz_error_mm"] > r["xyz_error_mm"]
                                               for c, r in zip(corrected, raw, strict=True)),
        "physics_steps": sum(row.get("physics_steps", 0) for row in rows),
        "forbidden_physics_steps": sum(row.get("forbidden_physics_steps", 0) for row in rows),
    }


class ProbeRecorder(Recorder):
    """Adds independent per-decision truth without exposing it to control."""
    def __init__(self, sim, directory):
        super().__init__(sim, directory, False)
        self.probe_truth = {}

    def capture_decision(self, stage):
        frame, index = super().capture_decision(stage)
        self.probe_truth[index] = independent_truth(self.sim)
        Image.fromarray(frame["rgb"]).save(self.directory/f"decision-{index:02d}.png")
        Image.fromarray(self.sim.overview()).save(self.directory/f"overview-{index:02d}.png")
        return frame, index


def freeze_probe(args):
    seeds = list(range(args.seed_start, args.seed_start+args.scenes))
    datasets = perception_datasets(args.perception)
    datasets += [{"path": str(args.policy.parent/name), "sha256": digest(args.policy.parent/name)}
                 for name in ("training.npz", "validation.npz")]
    seen = set()
    for dataset in datasets:
        with np.load(dataset["path"], allow_pickle=False) as data:
            seen.update(data["seeds"].tolist())
    if seen.intersection(seeds):
        raise ValueError("Probe seeds overlap model training or validation")
    training = json.loads((args.policy.parent/"summary.json").read_text())
    if training["perception_sha256"] != digest(args.perception):
        raise ValueError("Residual and perception do not match")
    root = create_run(args.name, args.perception)
    for name in ("feedback_gate_probe.py", "feedback.py"):
        shutil.copy2(ROOT/name, root/"source"/name)
    checkpoints = root/"checkpoints"
    checkpoints.mkdir()
    shutil.copy2(args.perception, checkpoints/"perception.pt")
    shutil.copy2(args.policy, checkpoints/"residual.pt")
    provenance = json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"].update({name: digest(root/"source"/name)
                                      for name in ("feedback_gate_probe.py", "feedback.py")})
    provenance.update(policy_sha256=digest(checkpoints/"residual.pt"),
                      perception_sha256=digest(checkpoints/"perception.pt"),
                      frozen_datasets=datasets, development_seeds=seeds,
                      development_only=True, full_task_test=False)
    write(root/"provenance.json", provenance)
    return root, seeds


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", required=True)
    p.add_argument("--seed-start", type=int, default=6100000)
    p.add_argument("--scenes", type=int, default=30)
    p.add_argument("--perception", type=Path, default=PERCEPTION)
    p.add_argument("--policy", type=Path, default=POLICY)
    p.add_argument("--feedback-config", type=Path)
    return p


def main(args):
    config = configuration(json.loads(args.feedback_config.read_text()) if args.feedback_config else None)
    root, seeds = freeze_probe(args)
    write(root/"config.json", {"development_only": True, "seeds": seeds, "feedback_config": config})
    model, policy = perception_model(args.perception), load(args.policy)
    sim = Simulation(ROOT/"assets/task_scene.xml")
    rows = []
    try:
        for seed in seeds:
            sim.reset(seed)
            directory = root/f"case-{seed}"
            directory.mkdir()
            recorder = ProbeRecorder(sim, directory)
            frame, index = recorder.capture_decision("initial_probe")
            raw = estimate(model, frame)
            initial = correct_prediction(raw, policy)
            row = {"seed": seed, "initial_detection": valid_prediction(initial), "views": [],
                   "selected_scores": None, "selected_view_index": None,
                   "initial_scores": {"raw": score_prediction(raw, recorder.probe_truth[index]),
                                      "corrected": score_prediction(initial, recorder.probe_truth[index])}}
            if row["initial_detection"]:
                try:
                    feedback = collect_feedback(sim, recorder, model, policy, initial, config)
                    row["feedback"] = feedback
                    row["selected_view_index"] = feedback["selected_view_index"]
                    for view in feedback["views"]:
                        scored = copy.deepcopy(view)
                        scored["independent_scores"] = [{
                            "raw": score_prediction(p, recorder.probe_truth[i]),
                            "corrected": score_prediction(correct_prediction(p, policy), recorder.probe_truth[i])}
                            for p, i in zip(view["raw_predictions"], view["decision_indices"], strict=True)]
                        row["views"].append(scored)
                    if feedback["accepted"]:
                        truth = recorder.probe_truth[feedback["selected_decision_index"]]
                        row["selected_scores"] = {
                            "raw": score_prediction(feedback["selected_raw_prediction"], truth),
                            "corrected": score_prediction(feedback["selected_prediction"], truth)}
                except (ValueError, RuntimeError) as exc:
                    row["error"] = str(exc)
            row.update(physics_steps=recorder.physics_audit["total_steps"],
                       forbidden_physics_steps=recorder.physics_audit["forbidden_steps"])
            recorder.save()
            write(directory/"truth_for_independent_scoring_only.json", recorder.probe_truth)
            write(directory/"result.json", row)
            rows.append(row)
            write(root/"summary.json", {"development_only": True, "full_task_test": False,
                "seeds": seeds, "feedback_config": config, "complete": len(rows) == len(seeds),
                "summary": summarize(rows), "cases": rows})
            print(json.dumps({"seed": seed, "initial": row["initial_detection"],
                "selected_view": row["selected_view_index"], "score": row["selected_scores"],
                "forbidden_steps": row["forbidden_physics_steps"]}), flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    main(parser().parse_args())
