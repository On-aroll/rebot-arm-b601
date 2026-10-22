"""Freeze and evaluate re-observation against V1 and matched-motion controls.

All three controllers use the same frozen perception and residual checkpoints.
Only the feedback controller consumes the selected fresh observation for grasp
planning. The observe_only controller makes the same observation movements.
"""
import argparse
from collections import Counter
from copy import deepcopy
from datetime import datetime, timezone
import json
from math import comb
from pathlib import Path
import shutil

import numpy as np
import torch

from feedback import DEFAULT_CONFIG, configuration
from policy import load
from sim import ROOT, Simulation
from task import GOALS, run_task
from task_runner import create_run, digest, perception_datasets, perception_model, summary, write

MODES = {"v1": "none", "observe_only": "observe_only", "feedback": "feedback"}
DEFAULT_PERCEPTION = ROOT/"outputs/vision-iteration-03/perception.pt"
DEFAULT_POLICY = ROOT/"outputs/goal-policy-v2/residual.pt"


def json_seeds(value):
    """Extract recorded scenes and reserved evaluation ranges from old evidence."""
    found = set()
    if isinstance(value, dict):
        for key, item in value.items():
            if (key == "seed" or key.endswith("_seed")) and isinstance(item, int):
                found.add(item)
            if (key == "seeds" or key.endswith("_seeds")) and isinstance(item, list):
                found.update(x for x in item if isinstance(x, int))
            if key.endswith("seed_start") and isinstance(item, int):
                count = value.get("episodes", value.get("trials_per_controller", 0))
                if key == "negative_seed_start":
                    count = 8
                if isinstance(count, int) and 0 < count <= 10000:
                    found.update(range(item, item+count))
            found.update(json_seeds(item))
    elif isinstance(value, list):
        for item in value:
            found.update(json_seeds(item))
    return found


def prior_seed_ledger(exclude):
    """Freeze prior development/evaluation/training scenes, including failures.

    Only the seeds array is loaded from NPZ files; RGB-D arrays are not read.
    Prior frozen evaluations reserve their planned scenes even if interrupted.
    """
    excluded = Path(exclude).resolve()
    seen, evidence = set(), []
    json_names = {"config.json", "summary.json", "result.json", "frozen_evaluation.json"}
    for path in sorted((ROOT/"outputs").rglob("*")):
        if not path.is_file() or excluded in path.parents or "source" in path.parts:
            continue
        seeds = set()
        if path.suffix == ".npz":
            with np.load(path, allow_pickle=False) as data:
                if "seeds" in data.files:
                    seeds.update(int(s) for s in data["seeds"].flat)
        elif path.name in json_names:
            seeds = json_seeds(json.loads(path.read_text()))
        if seeds:
            seen.update(seeds)
            evidence.append({"path": str(path.resolve()), "sha256": digest(path),
                             "seeds": sorted(seeds)})
    return {"schema": "b601-prior-seed-ledger-v1", "seeds": sorted(seen),
            "sources": evidence, "excludes_current_run": str(excluded)}


def first_attempt_success(episode):
    return bool(episode["success"] and episode["retries"] == 0)


def episode_feedback_counts(episode):
    counts = Counter()
    for attempt in episode["attempts"]:
        feedback = attempt.get("feedback")
        if not feedback:
            continue
        counts["observation_rounds"] += 1
        views = feedback.get("views", [])
        counts["views"] += len(views)
        counts["accepted_views"] += sum(bool(v["accepted"]) for v in views)
        counts["rejected_views"] += sum(not v["accepted"] for v in views)
        counts["rounds_without_accepted_view"] += feedback.get("selected_view_index") is None
        counts["updates_used_for_plan"] += bool(feedback.get("used_for_plan"))
        if feedback.get("selected_view_index") is None and attempt is not episode["attempts"][-1]:
            counts["gate_recovery_retries"] += 1
    return dict(counts)


def environment_displacement(episode):
    environment = episode.get("environment_for_scoring_only", {})
    truth = environment.get("truth_for_scoring_only", {})
    result = {}
    for key in ("pulse_displacement_xyz_m", "settled_displacement_xyz_m"):
        delta = truth.get(key)
        if delta is not None:
            vector = np.asarray(delta, float)
            result[key] = vector.tolist()
            result[key.replace("xyz_m", "xy_mm")] = float(np.linalg.norm(vector[:2])*1000)
    return result


def group_summary(episodes):
    result = summary(episodes)
    counts = Counter()
    for episode in episodes:
        counts.update(episode_feedback_counts(episode))
    result.update(first_attempt_successes=sum(first_attempt_success(e) for e in episodes),
                  first_attempt_success_rate=float(np.mean([first_attempt_success(e) for e in episodes])),
                  episodes_using_retry=sum(e["retries"] > 0 for e in episodes),
                  failure_reasons=dict(Counter(e["reason"] for e in episodes if not e["success"])),
                  feedback= dict(counts),
                  mean_wall_seconds=float(np.mean([e["wall_seconds"] for e in episodes])),
                  learned_action_calls=sum(e["learned_action_calls"] for e in episodes),
                  learned_inference_calls=sum(e["learned_inference_calls"] for e in episodes))
    displacements = [environment_displacement(e) for e in episodes]
    for key in ("pulse_displacement_xy_mm", "settled_displacement_xy_mm"):
        values = [d[key] for d in displacements if key in d]
        result[key] = {"samples": len(values), "mean": float(np.mean(values)) if values else None,
                       "min": min(values) if values else None, "max": max(values) if values else None}
    return result


def paired_outcomes(reference, candidate, outcome):
    assert len(reference) == len(candidate)
    pairs = [(bool(outcome(a)), bool(outcome(b))) for a, b in zip(reference, candidate)]
    wins = sum(not a and b for a, b in pairs)
    losses = sum(a and not b for a, b in pairs)
    discordant = wins+losses
    p = min(1., 2*sum(comb(discordant, k) for k in range(min(wins, losses)+1))/2**discordant) if discordant else 1.
    return {"pairs": len(pairs), "candidate_wins": wins, "candidate_losses": losses,
            "both_success": sum(a and b for a, b in pairs),
            "both_failure": sum(not a and not b for a, b in pairs),
            "exact_two_sided_mcnemar_p": p,
            "paired_success_rate_difference": (wins-losses)/len(pairs)}


def paired_comparison(reference, candidate):
    return {"full_task": paired_outcomes(reference, candidate, lambda e: e["success"]),
            "first_attempt": paired_outcomes(reference, candidate, first_attempt_success),
            "mean_candidate_minus_reference_retries": float(np.mean([b["retries"]-a["retries"] for a,b in zip(reference,candidate)])),
            "mean_candidate_minus_reference_sim_seconds": float(np.mean([b["sim_seconds"]-a["sim_seconds"] for a,b in zip(reference,candidate)]))}


def all_test_seeds(config):
    blocks = {"static": list(range(config["static_seed_start"], config["static_seed_start"]+config["episodes"])),
              "disturbed": list(range(config["disturbed_seed_start"], config["disturbed_seed_start"]+config["episodes"])),
              "negative": list(range(config["negative_seed_start"], config["negative_seed_start"]+8)),
              "recovery": [config["recovery_seed"]]}
    flat = [seed for block in blocks.values() for seed in block]
    if len(flat) != len(set(flat)):
        raise ValueError("Evaluation seed blocks overlap each other")
    return blocks


def complete_summary(result):
    result["paired_comparisons"] = {}
    for block, groups in result["groups"].items():
        result["paired_comparisons"][block] = {
            f"feedback_vs_{reference}": paired_comparison(groups[reference]["episodes"], groups["feedback"]["episodes"])
            for reference in ("v1", "observe_only")}
    feedback_static = result["groups"]["static"]["feedback"]["summary"]
    feedback_disturbed = result["groups"]["disturbed"]["feedback"]["summary"]
    negative = result["negative_controls"]
    result["acceptance_checks"] = {
        "at_least_30_fresh_scenes_per_block": all(len(seeds)>=30 for name,seeds in result["test_seeds"].items() if name in ("static","disturbed")),
        "balanced_A_B_C": all(len(seeds)%3 == 0 for name,seeds in result["test_seeds"].items() if name in ("static","disturbed")),
        "static_feedback_success_at_least_80_percent": feedback_static["success_rate"] >= .8,
        "disturbed_feedback_success_at_least_80_percent": feedback_disturbed["success_rate"] >= .8,
        "static_feedback_no_fewer_successes_than_v1": feedback_static["successes"] >= result["groups"]["static"]["v1"]["summary"]["successes"],
        "empty_scenes_no_motion": len(negative) >= 8 and all(not e["record_rows"] and not e["success"] for e in negative[:8]),
        "forced_miss_fails_without_retry": len(negative) == 10 and not negative[-2]["success"],
        "forced_miss_recovers_with_retry": len(negative) == 10 and negative[-1]["success"] and negative[-1]["retries"] > 0,
        "no_false_success_claims": not any(e.get("false_success",False) for block in result["groups"].values() for group in block.values() for e in group["episodes"])
            and not any(e.get("false_success",False) for e in negative)}
    result["acceptance_met"] = all(result["acceptance_checks"].values())
    result["adoption_requires_separate_review"] = True
    result["interpretation"] = "Acceptance checks are minimum engineering gates. Paired counts and uncertainty must be reviewed before adoption; n=30 does not establish hardware readiness or a stable performance advantage."
    return result


def main(args):
    if args.command == "demo":
        demo(args)
        return
    if not 30 <= args.episodes <= 300 or args.episodes%3:
        raise ValueError("Use 30--300 scenes per block, divisible by three")
    perception_path, policy_path = args.perception.resolve(), args.policy.resolve()
    training_summary = json.loads((policy_path.parent/"summary.json").read_text())
    if training_summary["perception_sha256"] != digest(perception_path):
        raise ValueError("Residual must be paired with its training perception checkpoint")
    feedback_config = configuration(json.loads(args.feedback_config.read_text()) if args.feedback_config else None)
    force_pulse = json.loads(args.force_pulse.read_text())
    config = {k:str(v.resolve()) if isinstance(v, Path) else v for k,v in vars(args).items()}
    config.update(feedback_config=feedback_config, force_pulse=force_pulse)
    seeds = all_test_seeds(config)
    root = create_run(args.name, perception_path)
    write(root/"config.json", config)
    frozen_dir = root/"frozen"
    frozen_dir.mkdir()
    shutil.copy2(perception_path, frozen_dir/"perception.pt")
    shutil.copy2(policy_path, frozen_dir/"residual.pt")
    datasets = perception_datasets(perception_path)
    datasets.extend({"path":str((policy_path.parent/p).resolve()), "sha256":digest(policy_path.parent/p)} for p in ("training.npz","validation.npz"))
    datasets = list({d["path"]:d for d in datasets}.values())
    ledger = prior_seed_ledger(root)
    write(root/"prior_seed_ledger.json", ledger)
    forbidden = set(ledger["seeds"])
    for dataset in datasets:
        with np.load(dataset["path"], allow_pickle=False) as data:
            forbidden.update(int(s) for s in data["seeds"].flat)
    overlap = forbidden & {s for values in seeds.values() for s in values}
    if overlap:
        write(root/"rejected_overlap.json", {"overlapping_seeds":sorted(overlap)})
        raise ValueError(f"Fresh evaluation overlaps prior training/development/evaluation scenes: {sorted(overlap)[:20]}")
    result = {"schema":"b601-feedback-evaluation-v1", "domain":"simulation", "hardware_executable":False,
        "frozen_at_utc":datetime.now(timezone.utc).isoformat(), "test_seeds":seeds,
        "policy_path":str(frozen_dir/"residual.pt"), "policy_sha256":digest(frozen_dir/"residual.pt"),
        "perception_path":str(frozen_dir/"perception.pt"), "perception_sha256":digest(frozen_dir/"perception.pt"),
        "source_policy_path":str(policy_path), "source_perception_path":str(perception_path),
        "frozen_datasets":datasets, "prior_seed_ledger_sha256":digest(root/"prior_seed_ledger.json"),
        "config_sha256":digest(root/"config.json"), "feedback_config":feedback_config,
        "force_pulse":force_pulse, "modes":MODES,
        "groups":{"static":{},"disturbed":{}}, "negative_controls":[]}
    # No simulation is constructed and no held-out observation is acquired until
    # all sources, weights, datasets, gates, pulse and planned seeds are frozen.
    write(root/"frozen_evaluation.json", result)
    write(root/"summary.json", result)
    sim = Simulation(ROOT/"assets/task_scene.xml")
    perception, policy = perception_model(frozen_dir/"perception.pt"), load(frozen_dir/"residual.pt")
    try:
        for block in ("static", "disturbed"):
            for mode, observation_mode in MODES.items():
                episodes = []
                for i, seed in enumerate(seeds[block]):
                    goal = list(GOALS)[i%3]
                    relative = f"{block}-{mode}-{i:03d}"
                    episode = run_task(sim, perception, policy, seed, goal, root/relative,
                        observation_mode=observation_mode, feedback_config=feedback_config,
                        force_pulse=force_pulse if block == "disturbed" else None,
                        video=mode == "feedback" and i < 3)
                    episodes.append({**episode, "artifact_directory":relative})
                    result["groups"][block][mode] = {"summary":group_summary(episodes), "episodes":episodes}
                    write(root/"summary.json", result)
                    print(json.dumps({"block":block,"mode":mode,"episode":i,"seed":seed,"goal":goal,
                        "success":episode["success"],"reason":episode["reason"],"retries":episode["retries"]}),flush=True)
        for i, seed in enumerate(seeds["negative"]):
            relative = f"empty-{i:02d}"
            episode = run_task(sim, perception, policy, seed, list(GOALS)[i%3], root/relative,
                absent=True, observation_mode="feedback", feedback_config=feedback_config)
            result["negative_controls"].append({**episode,"artifact_directory":relative,"control_case":"absent_target"})
            write(root/"summary.json", result)
        for retries in (0,2):
            relative = f"miss-retries-{retries}"
            episode = run_task(sim, perception, policy, args.recovery_seed, "C", root/relative,
                observation_mode="feedback", feedback_config=feedback_config,
                first_grasp_offset_m=.065, max_retries=retries, video=True)
            result["negative_controls"].append({**episode,"artifact_directory":relative,"control_case":"injected_first_grasp_offset_65mm"})
            write(root/"summary.json", result)
            print(json.dumps({"recovery_retries_allowed":retries,"success":episode["success"],"reason":episode["reason"]}),flush=True)
        complete_summary(result)
        write(root/"summary.json", result)
    finally:
        sim.close()


def demo(args):
    perception_path, policy_path = args.perception.resolve(), args.policy.resolve()
    training_summary = json.loads((policy_path.parent/"summary.json").read_text())
    if training_summary["perception_sha256"] != digest(perception_path):
        raise ValueError("Residual must be paired with its training perception checkpoint")
    config = configuration(json.loads(args.feedback_config.read_text()) if args.feedback_config else None)
    pulse = json.loads(args.force_pulse.read_text()) if args.force_pulse else None
    root = create_run(args.name, perception_path)
    frozen_dir = root/"frozen"
    frozen_dir.mkdir()
    shutil.copy2(perception_path, frozen_dir/"perception.pt")
    shutil.copy2(policy_path, frozen_dir/"residual.pt")
    write(root/"config.json", {"seed":args.seed, "goal":args.goal, "mode":args.mode,
        "feedback_config":config, "force_pulse":pulse, "development_only":True,
        "first_grasp_offset_m":args.first_grasp_offset, "max_retries":args.max_retries})
    write(root/"frozen_evaluation.json", {"development_only":True, "full_task_acceptance":False,
        "perception_path":str(frozen_dir/"perception.pt"), "perception_sha256":digest(frozen_dir/"perception.pt"),
        "policy_path":str(frozen_dir/"residual.pt"), "policy_sha256":digest(frozen_dir/"residual.pt"),
        "config_sha256":digest(root/"config.json")})
    sim = Simulation(ROOT/"assets/task_scene.xml")
    try:
        episode = run_task(sim, perception_model(frozen_dir/"perception.pt"), load(frozen_dir/"residual.pt"),
            args.seed, args.goal, root/"episode", video=not args.no_video, absent=args.absent,
            observation_mode=MODES[args.mode], feedback_config=config, force_pulse=pulse,
            first_grasp_offset_m=args.first_grasp_offset, max_retries=args.max_retries)
        write(root/"summary.json", episode)
        print(json.dumps({k:episode[k] for k in ("success","reason","retries","sim_seconds","wall_seconds")}),flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    evaluate = commands.add_parser("evaluate")
    evaluate.add_argument("--name", required=True)
    evaluate.add_argument("--episodes", type=int, default=30)
    evaluate.add_argument("--perception", type=Path, default=DEFAULT_PERCEPTION)
    evaluate.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    evaluate.add_argument("--feedback-config", type=Path)
    evaluate.add_argument("--force-pulse", type=Path, required=True, help="Frozen development-selected JSON {start_s,duration_s,force_world_n}")
    evaluate.add_argument("--static-seed-start", type=int, default=9600000)
    evaluate.add_argument("--disturbed-seed-start", type=int, default=9900000)
    evaluate.add_argument("--negative-seed-start", type=int, default=9700000)
    evaluate.add_argument("--recovery-seed", type=int, default=9800000)
    demonstration = commands.add_parser("demo")
    demonstration.add_argument("--name", required=True)
    demonstration.add_argument("--seed", type=int, default=8000100)
    demonstration.add_argument("--goal", choices=list(GOALS), default="B")
    demonstration.add_argument("--mode", choices=list(MODES), default="feedback")
    demonstration.add_argument("--perception", type=Path, default=DEFAULT_PERCEPTION)
    demonstration.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    demonstration.add_argument("--feedback-config", type=Path)
    demonstration.add_argument("--force-pulse", type=Path)
    demonstration.add_argument("--first-grasp-offset", type=float, default=0.)
    demonstration.add_argument("--max-retries", type=int, default=2)
    demonstration.add_argument("--absent", action="store_true")
    demonstration.add_argument("--no-video", action="store_true")
    main(parser.parse_args())
