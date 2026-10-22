"""Bounded, reproducible collect/train/evaluate rounds. SIMULATION ONLY."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import shutil

import numpy as np
import torch

from sim import ROOT, Simulation
from episode import run_episode
from vision import TargetNet, collect, estimate, train, validate


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False)+"\n")


def run(args):
    if not 1 <= args.rounds <= 20 or not 32 <= args.samples <= 4096 or not 1 <= args.episodes <= 100 or not 1 <= args.epochs <= 200:
        raise ValueError("Use 1--20 rounds, 32--4096 samples/round, 1--100 episodes, 1--200 epochs")
    run_dir = ROOT / "outputs" / args.name
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "source").mkdir()
    for name in ("sim.py", "episode.py", "vision.py", "experiment.py"):
        shutil.copy2(ROOT / name, run_dir / "source" / name)
    for name in ("scene.xml", "manifest.json"):
        shutil.copy2(ROOT / "assets" / name, run_dir / "source" / name)
    sim = Simulation()
    torch.manual_seed(42)
    results = {"schema": "b601-vision-training-run-v1", "domain": "simulation",
               "hardware_executable": False, "args": vars(args), "rounds": [],
               "task": "one red/orange cube, fixed overhead RGB-D, known 0.1m table",
               "learned_component": "RGB-D target segmentation",
               "motion_component": "fixed grasp stages + IK + MuJoCo PD/gravity compensation",
               "physics_success": "min lift >60mm and bilateral finger contact >90% over 1s",
               "model_asset_manifest_sha256": hashlib.sha256((ROOT/'assets/manifest.json').read_bytes()).hexdigest()}
    begun = time.monotonic()
    try:
        validation = run_dir / "validation.npz"
        paths, initial, round_offset, sample_offset = [], None, 0, 0
        if args.resume_run:
            parent = ROOT / "outputs" / args.resume_run
            previous = json.loads((parent / "summary.json").read_text())
            if previous["model_asset_manifest_sha256"] != results["model_asset_manifest_sha256"]:
                raise ValueError("Cannot resume across different model assets")
            paths = [Path(p) for p in previous.get("training_datasets", [])]
            if not paths:
                paths = [Path(r["collection"]["path"]) for r in previous["rounds"]]
            if not paths or not all(p.is_file() for p in paths):
                raise ValueError("Resume datasets are missing")
            initial = parent / previous["selected_checkpoint"]
            for path in paths:
                with np.load(path,allow_pickle=False) as data:
                    sample_offset += len(data["seeds"])
            round_offset = previous.get("total_rounds",len(previous["rounds"]))
            shutil.copy2(parent / "validation.npz", validation)
            results["parent_run"] = str(parent)
        else:
            collect(sim, np.arange(2000000, 2000056), validation)
        results["untrained_validation"] = validate(TargetNet().eval(), validation)
        best_score = None
        if initial:
            parent_model = TargetNet()
            parent_model.load_state_dict(torch.load(initial,map_location="cpu",weights_only=True)["state_dict"])
            baseline = validate(parent_model.eval(),validation)
            best_score = (baseline["misses"]+baseline["false_positives"],baseline["xy_p95_mm"] or float("inf"))
            results["resumed_validation"] = baseline
            results["selected_checkpoint"] = str(initial)
        for r in range(args.rounds):
            round_dir = run_dir / f"round-{r+1:02d}"
            round_dir.mkdir()
            dataset = round_dir / "training.npz"
            seed_start = 1000000 + sample_offset + r*args.samples
            collected = collect(sim, np.arange(seed_start, seed_start+args.samples), dataset)
            paths.append(dataset)
            checkpoint = round_dir / "perception.pt"
            model, history = train(paths, checkpoint, epochs=args.epochs, seed=42+r, initial=initial)
            metrics = validate(model, validation)
            trials = []
            for i in range(args.episodes):
                seed = 3000000 + (round_offset+r)*1000 + i
                sim.reset(seed)
                # No segmentation, labels or target pose enter estimate().
                observed = sim.observe(labels=False)
                predicted = estimate(model, observed)
                trial = run_episode(sim, seed, predicted, round_dir / f"episode-{i:03d}", video=(i==0))
                trials.append(trial)
                print(json.dumps({"round":r+1,"episode":i,"success":trial["success"],"reason":trial["reason"]}),flush=True)
            successes = sum(t["success"] for t in trials)
            entry = {"round": r+1, "collection": collected, "training_samples_total": sample_offset+(r+1)*args.samples,
                     "final_training_loss": history[-1]["loss"], "validation": metrics,
                     "physical_successes": successes, "physical_trials": len(trials),
                     "checkpoint": str(checkpoint.relative_to(run_dir)),
                     "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                     "episodes": [{k:t.get(k) for k in ("success","reason","hold_min_lift_m","hold_bilateral_fraction","failed_stage")} for t in trials]}
            # Validation alone selects perception weights; test episodes do not
            # influence training or checkpoint selection. Validation is reused.
            score = (metrics["misses"]+metrics["false_positives"], metrics["xy_p95_mm"] or float("inf"))
            if best_score is None or score < best_score:
                best_score, initial = score, checkpoint
                results["selected_checkpoint"] = str(checkpoint.relative_to(run_dir))
            results["rounds"].append(entry)
            results["training_datasets"] = [str(p) for p in paths]
            results["total_rounds"] = round_offset + r+1
            results["wall_seconds"] = time.monotonic()-begun
            save(run_dir / "summary.json", results)
        save(run_dir / "summary.json", results)
    finally:
        sim.close()
    print(str(run_dir / "summary.json"), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default=time.strftime("run-%Y%m%d-%H%M%S"))
    parser.add_argument("--rounds", type=int, default=2)
    parser.add_argument("--samples", type=int, default=192)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--episodes", type=int, default=8)
    parser.add_argument("--resume-run", help="Previous output directory name; retain its data and selected weights")
    run(parser.parse_args())
