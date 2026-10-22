"""Train, evaluate, or demonstrate the simulation-only goal pick/place task."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil

import numpy as np
import torch

from sim import ROOT, Simulation, mj
from task import GOALS, run_task
from policy import collect, train, load
from vision import TargetNet, inference_configuration

PERCEPTION = ROOT/"outputs/first-training/round-02/perception.pt"
SOURCES = ("sim.py", "vision.py", "vision_runner.py", "viewpoint_probe.py", "episode.py", "policy.py", "task.py", "feedback.py", "feedback_runner.py", "feedback_audit.py", "score_audit.py", "task_runner.py", "task_audit.py", "demo_audit.py", "task.sh")


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False)+"\n")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def create_run(name, perception_path=PERCEPTION):
    if Path(name).name != name or name in (".", ".."):
        raise ValueError("Run name must be one directory name")
    directory = ROOT/"outputs"/name
    directory.mkdir(parents=True, exist_ok=False)
    source = directory/"source"
    source.mkdir()
    hashes = {}
    for name in SOURCES:
        shutil.copy2(ROOT/name, source/name)
        hashes[name] = digest(ROOT/name)
    for name in ("scene.xml", "task_scene.xml", "manifest.json"):
        shutil.copy2(ROOT/"assets"/name, source/name)
        hashes[f"assets/{name}"] = digest(ROOT/"assets"/name)
    manifest = json.loads((ROOT/"assets/manifest.json").read_text())
    if not all(digest(ROOT/"assets"/name) == sha for name, sha in manifest["files_sha256"].items()):
        raise ValueError("Original model asset hash mismatch")
    write(directory/"provenance.json", {"schema":"b601-goal-provenance-v1", "domain":"simulation",
        "hardware_executable": False, "source_sha256": hashes,
        "runtime": {"mujoco_version":mj.__version__,
                    "gl_context_module":getattr(getattr(mj,"GLContext",None),"__module__",None),
                    "requested_mujoco_gl":os.environ.get("MUJOCO_GL")},
        "original_asset_files_verified": len(manifest["files_sha256"]),
        "perception_checkpoint": str(perception_path), "perception_sha256": digest(perception_path)})
    return directory


def perception_model(path=PERCEPTION):
    model = TargetNet()
    checkpoint=torch.load(path, map_location="cpu", weights_only=True)
    model.load_state_dict(checkpoint["state_dict"])
    model.inference_config=inference_configuration(checkpoint.get("inference_config"))
    return model.eval()


def perception_datasets(path):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    paths = checkpoint.get("dataset_paths")
    if paths is None and Path(path).resolve() == PERCEPTION.resolve():
        paths = [ROOT/"outputs/first-training"/f"round-{i:02d}/training.npz" for i in (1, 2)]
        paths.append(ROOT/"outputs/first-training/validation.npz")
    if not paths:
        raise ValueError("Perception checkpoint needs auditable training/validation dataset paths")
    return [{"path": str(Path(p).resolve()), "sha256": digest(p)} for p in paths]


def wilson(successes, total):
    z, p = 1.96, successes/total
    d = 1+z*z/total
    midpoint = (p+z*z/(2*total))/d
    radius = z*np.sqrt(p*(1-p)/total+z*z/(4*total*total))/d
    return [float(midpoint-radius), float(midpoint+radius)]


def summary(episodes):
    n = len(episodes)
    successes = sum(e["success"] for e in episodes)
    return {"trials": n, "successes": successes, "success_rate": successes/n,
        "wilson_95_interval": wilson(successes, n),
        "controller_successes": sum(e["controller_success"] for e in episodes),
        "physical_successes": sum(e.get("physical_success", False) for e in episodes),
        "false_successes": sum(e.get("false_success", False) for e in episodes),
        "total_retries": sum(e["retries"] for e in episodes),
        "mean_task_sim_seconds": float(np.mean([e["sim_seconds"] for e in episodes])),
        "by_goal": {g: {"trials": sum(e["task"]["destination"] == g for e in episodes),
                        "successes": sum(e["success"] and e["task"]["destination"] == g for e in episodes)} for g in GOALS}}


def main(args):
    if args.command == "train" and not 64 <= args.samples <= 4096:
        raise ValueError("Use 64--4096 training scenes")
    if args.command == "evaluate" and not 1 <= args.episodes <= 100:
        raise ValueError("Use 1--100 evaluation episodes per controller")
    perception_path = args.perception.resolve()
    root = create_run(args.name, perception_path)
    write(root/"config.json", {k:str(v) if isinstance(v, Path) else v for k,v in vars(args).items()})
    sim = Simulation(ROOT/"assets/task_scene.xml")
    perception = perception_model(perception_path)
    try:
        if args.command == "train":
            training = collect(sim, perception, np.arange(args.train_seed, args.train_seed+args.samples), root/"training.npz")
            print(json.dumps({"collected_training": training}), flush=True)
            validation = collect(sim, perception, np.arange(args.validation_seed, args.validation_seed+192), root/"validation.npz")
            policy, metrics = train(root/"training.npz", root/"validation.npz", root/"residual.pt")
            write(root/"summary.json", {"schema":"b601-goal-residual-training-v1", "hardware_executable": False,
                "training": training, "validation": validation, "metrics": metrics,
                "checkpoint_sha256": digest(root/"residual.pt"), "perception_sha256": digest(perception_path)})
            print(json.dumps(metrics), flush=True)
            return
        policy_path = ROOT/"outputs"/args.policy_run/"residual.pt"
        policy = load(policy_path)
        training_summary = json.loads((policy_path.parent/"summary.json").read_text())
        if training_summary["perception_sha256"] != digest(perception_path):
            raise ValueError("Residual must be paired with its training perception checkpoint")
        result = {"schema":"b601-goal-evaluation-v1", "domain":"simulation", "hardware_executable":False,
            "policy_path": str(policy_path), "policy_sha256":digest(policy_path),
            "perception_sha256":digest(perception_path), "groups":{}, "negative_controls":[]}
        if args.command == "evaluate":
            result["baseline_perception_path"] = str(args.baseline_perception.resolve())
            result["baseline_perception_sha256"] = digest(args.baseline_perception)
            result["seed_start"] = args.seed_start
            datasets = perception_datasets(perception_path)+perception_datasets(args.baseline_perception)
            datasets += [{"path":str(policy_path.parent/p), "sha256":digest(policy_path.parent/p)}
                         for p in ("training.npz", "validation.npz")]
            result["frozen_datasets"] = list({d["path"]:d for d in datasets}.values())
            seen = set()
            for dataset in result["frozen_datasets"]:
                with np.load(dataset["path"], allow_pickle=False) as d:
                    seen.update(d["seeds"].tolist())
            test_seeds = set(range(args.seed_start, args.seed_start+args.episodes))
            test_seeds.update(range(args.negative_seed_start, args.negative_seed_start+8))
            test_seeds.add(args.recovery_seed)
            if seen & test_seeds:
                raise ValueError("Evaluation scenes overlap perception/policy training or validation")
        # Frozen weights and task source hashes are recorded BEFORE any test.
        if args.command == "demo":
            from feedback import configuration
            feedback_config=configuration(json.loads(args.feedback_config.read_text()) if args.feedback_config else None)
            pulse=json.loads(args.force_pulse.read_text()) if args.force_pulse else None
            (root/"frozen").mkdir()
            shutil.copy2(perception_path,root/"frozen/perception.pt")
            shutil.copy2(policy_path,root/"frozen/residual.pt")
            assert digest(root/"frozen/perception.pt")==result["perception_sha256"]
            assert digest(root/"frozen/residual.pt")==result["policy_sha256"]
            result.update(observation_mode=args.observation_mode,feedback_config=feedback_config,
                          force_pulse=pulse,scene_role=args.scene_role)
        write(root/"frozen_evaluation.json", result)
        if args.command == "demo":
            episode = run_task(sim, perception, policy, args.seed, args.goal, root/"episode",
                video=True, absent=args.absent, first_grasp_offset_m=args.first_grasp_offset,
                max_retries=args.max_retries,observation_mode=args.observation_mode,
                feedback_config=feedback_config,force_pulse=pulse)
            write(root/"summary.json", episode)
            print(json.dumps({k:episode.get(k) for k in ("success","reason","physical_success","controller_success","retries","wall_seconds")}), flush=True)
            return
        baseline_perception = perception_model(args.baseline_perception)
        modes = ("baseline", "vision_only", "learned") if args.ablate_perception else ("baseline", "learned")
        for mode in modes:
            episodes = []
            for i in range(args.episodes):
                seed, goal = args.seed_start+i, list(GOALS)[i%3]
                detector = baseline_perception if mode == "baseline" else perception
                episode = run_task(sim, detector, policy if mode == "learned" else None,
                    seed, goal, root/f"{mode}-{i:03d}", video=mode=="learned" and i<3)
                episodes.append(episode)
                result["groups"][mode] = {"summary":summary(episodes), "episodes":episodes}
                write(root/"summary.json", result)
                print(json.dumps({"mode":mode,"episode":i,"goal":goal,"success":episode["success"],
                                  "reason":episode["reason"],"retries":episode["retries"]}), flush=True)
        for i in range(8):
            episode = run_task(sim, perception, policy, args.negative_seed_start+i, list(GOALS)[i%3],
                               root/f"empty-{i}", absent=True)
            episode["control_case"] = "absent_target"
            result["negative_controls"].append(episode)
            write(root/"summary.json", result)
        # Same perturbation, same scene, with and without retry opportunity.
        for retries in (0, 2):
            episode = run_task(sim, perception, policy, args.recovery_seed, "C", root/f"miss-retries-{retries}",
                first_grasp_offset_m=.065, max_retries=retries, video=True)
            episode["control_case"] = "injected_first_grasp_offset_65mm"
            result["negative_controls"].append(episode)
            write(root/"summary.json", result)
            print(json.dumps({"recovery_retries_allowed":retries,"success":episode["success"],"reason":episode["reason"]}),flush=True)
        result["acceptance_target"] = {"trials_per_controller":30,"learned_success_rate":.8}
        result["acceptance_checks"] = {
            "at_least_30_heldout_scenes":args.episodes>=30,
            "learned_success_rate_at_least_80_percent":result["groups"]["learned"]["summary"]["success_rate"]>=.8,
            "empty_scenes_no_motion":all(not e["record_rows"] for e in result["negative_controls"][:8]),
            "injected_miss_fails_without_retry":not result["negative_controls"][-2]["success"],
            "injected_miss_recovers_with_retry":result["negative_controls"][-1]["success"],
            "no_false_success_claims":not any(e.get("false_success",False) for g in result["groups"].values() for e in g["episodes"])}
        result["acceptance_met"] = all(result["acceptance_checks"].values())
        write(root/"summary.json", result)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    training = commands.add_parser("train")
    training.add_argument("--name", required=True)
    training.add_argument("--samples", type=int, default=768)
    training.add_argument("--perception", type=Path, default=PERCEPTION)
    training.add_argument("--train-seed", type=int, default=4100000)
    training.add_argument("--validation-seed", type=int, default=4200000)
    for command in ("evaluate", "demo"):
        sub = commands.add_parser(command)
        sub.add_argument("--name", required=True)
        sub.add_argument("--policy-run", required=True)
        sub.add_argument("--perception", type=Path, default=PERCEPTION)
        if command == "evaluate":
            sub.add_argument("--episodes", type=int, default=30)
            sub.add_argument("--baseline-perception", type=Path, default=PERCEPTION)
            sub.add_argument("--seed-start", type=int, default=9000000)
            sub.add_argument("--negative-seed-start", type=int, default=9100000)
            sub.add_argument("--recovery-seed", type=int, default=9200000)
            sub.add_argument("--ablate-perception", action="store_true")
        else:
            sub.add_argument("--goal", choices=list(GOALS), required=True)
            sub.add_argument("--seed", type=int, default=8000000)
            sub.add_argument("--absent", action="store_true")
            sub.add_argument("--first-grasp-offset", type=float, default=0.)
            sub.add_argument("--max-retries", type=int, default=2)
            sub.add_argument("--observation-mode",choices=["none","observe_only","feedback"],default="none")
            sub.add_argument("--feedback-config",type=Path)
            sub.add_argument("--force-pulse",type=Path)
            sub.add_argument("--scene-role",choices=["demonstration","posthoc_development"],default="demonstration")
    main(parser.parse_args())
