"""Paired physical tasks: retained V3, image residual and zero residual.

Development runs guide the next iteration; only fresh frozen evaluation runs
can satisfy adoption gates. Baseline models and feedback use frozen V3.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

import numpy as np

from feedback import configuration
from feedback_runner import group_summary, paired_comparison, prior_seed_ledger
from policy import load
from sim import ROOT, Simulation
from task import GOALS, run_task
from task_runner import create_run, digest, perception_datasets, perception_model, write

VARIANTS = {"retained_v3":("baseline","feedback"),
            "zero_residual":("zero","feedback"),
            "new_observe_only":("candidate","observe_only"),
            "new_feedback":("candidate","feedback"),
            "retained_two_view":("candidate","feedback")}
EXTRA_SOURCES = ("multiview_runner.py","multiview_audit.py","audit_frozen.py",
                 "observation_dataset.py","observation_data_audit.py","observation_policy.py")
V3_MANIFEST_SHA256 = "bb6bebf40eb2b660f66600c5be2b41b7e13bee6529bcc042e247a363bcb1342b"
BASELINE_CONTROL_SOURCES = ("sim.py","episode.py","feedback.py","vision.py")
FRAME_INTERFACE_SOURCES = ("policy.py","task.py","feedback_audit.py","task_runner.py")


def retained_v3():
    runtime = (ROOT/"outputs").resolve().parent/"versions/V3"
    assert digest(runtime/"release_manifest.json")==V3_MANIFEST_SHA256
    manifest = json.loads((runtime/"release_manifest.json").read_text())
    assert manifest["version"]=="V3" and manifest["observation_mode"]=="feedback"
    assert all(digest(runtime/name)==sha for name,sha in manifest["files_sha256"].items())
    # Shared simulation/vision is exact V3. The explicit image-frame interface
    # has also reproduced all nine V3 physical/action/RGBD reference cases.
    assert all(digest(ROOT/name)==manifest["files_sha256"][name] for name in BASELINE_CONTROL_SOURCES)
    parity=(ROOT/"outputs/image-residual-known-tasks-01").resolve()
    audit=json.loads((parity/"audit.json").read_text())
    assert audit["passed"] and audit["V3_exact_physical_action_RGBD_reexecutions"]==9
    sources=json.loads((parity/"provenance.json").read_text())["source_sha256"]
    assert all(digest(ROOT/name)==sources[name] for name in FRAME_INTERFACE_SOURCES)
    config = configuration(json.loads((runtime/"feedback_config.json").read_text()))
    return runtime,config,{"version":"V3","runtime":str(runtime),
        "release_manifest_sha256":V3_MANIFEST_SHA256,"observation_mode":"feedback",
        "feedback_config":config,
        "frame_interface_parity_audit":str(parity/"audit.json"),"frame_interface_parity_audit_sha256":digest(parity/"audit.json"),
        "frame_interface_source_sha256":{name:sources[name] for name in FRAME_INTERFACE_SOURCES},
        "shared_control_source_sha256":{name:manifest["files_sha256"][name] for name in BASELINE_CONTROL_SOURCES}}


def finalize(result):
    result["paired_comparisons"] = {
        block:{f"new_feedback_vs_{reference}":paired_comparison(groups[reference]["episodes"],groups["new_feedback"]["episodes"])
               for reference in ("retained_v3","zero_residual","new_observe_only","retained_two_view")}
        for block,groups in result["groups"].items()}
    controls = result["negative_controls"]
    empty = [e for e in controls if e["control_case"] == "absent_target"]
    recovery = {e["max_retries"]:e for e in controls if e["control_case"] == "injected_first_grasp_offset_65mm"}
    groups = result["groups"]
    episodes = [e for g in groups.values() for c in g.values() for e in c["episodes"]]+controls
    checks = {"fresh_30_scenes_per_block":all(len(s)>=30 for b,s in result["test_seeds"].items() if b in groups),
              "balanced_A_B_C":all(len(g["episodes"])%3 == 0 and all(
                  sum(e["task"]["destination"]==goal for e in g["episodes"]) == len(g["episodes"])//3 for goal in GOALS)
                  for block in groups.values() for g in block.values()),
              "candidate_80_percent_each_block":all(g["new_feedback"]["summary"]["success_rate"] >= .8 for g in groups.values()),
              "no_static_regression":groups["static"]["new_feedback"]["summary"]["successes"] >= groups["static"]["retained_v3"]["summary"]["successes"],
              "no_disturbed_regression":groups["disturbed"]["new_feedback"]["summary"]["successes"] >= groups["disturbed"]["retained_v3"]["summary"]["successes"],
              "no_static_regression_vs_zero":groups["static"]["new_feedback"]["summary"]["successes"] >= groups["static"]["zero_residual"]["summary"]["successes"],
              "no_disturbed_regression_vs_zero":groups["disturbed"]["new_feedback"]["summary"]["successes"] >= groups["disturbed"]["zero_residual"]["summary"]["successes"],
              "no_static_regression_vs_two_view":groups["static"]["new_feedback"]["summary"]["successes"] >= groups["static"]["retained_two_view"]["summary"]["successes"],
              "no_disturbed_regression_vs_two_view":groups["disturbed"]["new_feedback"]["summary"]["successes"] >= groups["disturbed"]["retained_two_view"]["summary"]["successes"],
              "static_mean_sim_time_not_increased_vs_two_view":groups["static"]["new_feedback"]["summary"]["mean_task_sim_seconds"] <= groups["static"]["retained_two_view"]["summary"]["mean_task_sim_seconds"],
              "empty_no_motion":len(empty)==8 and all(not e["controller_success"] and e["record_rows"]==0 for e in empty),
              "forced_miss_exercised":all(e["attempts"][0].get("outcome")=="empty_grasp_or_slip" and "plan" in e["attempts"][0] for e in recovery.values()) and set(recovery)=={0,2},
              "recovery_pair_passes":set(recovery)=={0,2} and not recovery[0]["success"] and recovery[2]["success"] and recovery[2]["retries"]>=1,
              "no_false_success":not any(e["false_success"] for e in episodes),
              "all_new_score_traces_recorded":all(e["schema"]=="b601-goal-pick-place-v3" and (e["score_trace_samples"]==500 or e["record_rows"]==0) for e in episodes)}
    result["acceptance_checks"] = checks
    result["acceptance_met"] = result["phase"]=="evaluate" and all(checks.values())
    return result


def main(args):
    if args.episodes%3 or not (6 <= args.episodes <= 90):
        raise ValueError("Use 6--90 scenes per block, divisible by three")
    if args.phase=="evaluate" and args.episodes<30:
        raise ValueError("Adoption evaluation requires at least 30 scenes per block")
    if args.phase=="evaluate" and args.protocol is None:
        raise ValueError("Freeze a written evaluation protocol before the trial")
    runtime,baseline_config,baseline = retained_v3()
    paths = {"baseline":(runtime/"models/perception.pt",runtime/"models/residual.pt"),
             "candidate":(args.perception.resolve(),args.policy.resolve()),
             "zero":(args.perception.resolve(),None)}
    for detector,head in paths.values():
        if head is not None:
            assert json.loads((head.parent/"summary.json").read_text())["perception_sha256"] == digest(detector)
    config = configuration(json.loads(args.feedback_config.read_text()) if args.feedback_config else None)
    pulse = json.loads(args.force_pulse.read_text())
    seeds = {"static":list(range(args.static_seed_start,args.static_seed_start+args.episodes)),
             "disturbed":list(range(args.disturbed_seed_start,args.disturbed_seed_start+args.episodes)),
             "negative":list(range(args.negative_seed_start,args.negative_seed_start+8)),"recovery":[args.recovery_seed]}
    flattened = [s for values in seeds.values() for s in values]
    assert len(flattened)==len(set(flattened))
    root = create_run(args.name,paths["candidate"][0])
    provenance = json.loads((root/"provenance.json").read_text())
    for source in EXTRA_SOURCES:
        shutil.copy2(ROOT/source,root/"source"/source)
        provenance["source_sha256"][source] = digest(ROOT/source)
    write(root/"provenance.json",provenance)
    write(root/"config.json",{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()})
    frozen = root/"frozen"; frozen.mkdir()
    models,datasets = {},[]
    for name,(detector,head) in paths.items():
        saved = {}
        for kind,path in (("perception",detector),("policy",head)):
            if path is None:
                saved.update(policy_path=None,policy_sha256=None,source_policy_path=None)
                continue
            destination = frozen/f"{name}-{kind}.pt"
            shutil.copy2(path,destination)
            saved[kind+"_path"],saved[kind+"_sha256"] = str(destination),digest(destination)
            saved["source_"+kind+"_path"] = str(path)
        models[name] = saved
        datasets += perception_datasets(detector)
        if head is not None:
            datasets += [{"path":str(head.parent/p),"sha256":digest(head.parent/p)} for p in ("training.npz","validation.npz")]
    datasets = list({d["path"]:d for d in datasets}.values())
    ledger = prior_seed_ledger(root,args.concurrent_frozen_run)
    if args.concurrent_frozen_run:
        concurrent=args.concurrent_frozen_run.resolve()
        immutable={Path(item["path"]) for item in ledger["sources"] if concurrent in Path(item["path"]).parents}
        assert immutable=={concurrent/"config.json",concurrent/"frozen_evaluation.json"}
        reserved=json.loads((concurrent/"frozen_evaluation.json").read_text())["test_seeds"]
        assert set(s for values in reserved.values() for s in values)<=set(ledger["seeds"])
    write(root/"prior_seed_ledger.json",ledger)
    excluded = set(ledger["seeds"])
    for item in datasets:
        with np.load(item["path"],allow_pickle=False) as data:
            excluded.update(data["seeds"].tolist())
    assert not excluded.intersection(flattened), "Development/evaluation seeds must be fresh"
    result = {"schema":"b601-multiview-task-comparison-v1","phase":args.phase,"domain":"simulation",
              "hardware_executable":False,"frozen_at_utc":datetime.now(timezone.utc).isoformat(),
              "test_seeds":seeds,"models":models,"variants":VARIANTS,"feedback_config":config,"force_pulse":pulse,
              "frozen_datasets":datasets,"prior_seed_ledger_sha256":digest(root/"prior_seed_ledger.json"),
              "config_sha256":digest(root/"config.json"),"groups":{"static":{},"disturbed":{}},"negative_controls":[],
              "adoption_requires_review":True}
    result["retained_baseline"] = baseline
    if args.protocol:
        shutil.copy2(args.protocol.resolve(),root/"protocol.md")
        result["protocol"]={"source_path":str(args.protocol.resolve()),"saved_name":"protocol.md",
                            "sha256":digest(root/"protocol.md")}
    result["variant_feedback_configs"] = {v:(baseline_config if v=="retained_v3" else
        dict(config,home_refresh_wait_s=0.) if v=="retained_two_view" else config) for v in VARIANTS}
    write(root/"frozen_evaluation.json",result)
    loaded = {name:(perception_model(m["perception_path"]),None if m["policy_path"] is None else load(m["policy_path"])) for name,m in models.items()}
    sim = Simulation(ROOT/"assets/task_scene.xml")
    try:
        for block in ("static","disturbed"):
            for variant,(model_name,mode) in VARIANTS.items():
                episodes = []
                for i,seed in enumerate(seeds[block]):
                    relative = f"{block}-{variant}-{i:03d}"
                    episode = run_task(sim,*loaded[model_name],seed,list(GOALS)[i%3],root/relative,
                        observation_mode=mode,feedback_config=result["variant_feedback_configs"][variant],force_pulse=pulse if block=="disturbed" else None,
                        video=variant=="new_feedback" and i<3)
                    episodes.append({**episode,"artifact_directory":relative})
                    result["groups"][block][variant] = {"summary":group_summary(episodes),"episodes":episodes}
                    write(root/"summary.json",result)
                    print(json.dumps({"block":block,"variant":variant,"seed":seed,"success":episode["success"],
                                      "reason":episode["reason"],"retries":episode["retries"]}),flush=True)
        for i,seed in enumerate(seeds["negative"]):
            relative = f"empty-{i:02d}"
            e = run_task(sim,*loaded["candidate"],seed,list(GOALS)[i%3],root/relative,
                         absent=True,observation_mode="feedback",feedback_config=config)
            result["negative_controls"].append({**e,"artifact_directory":relative,"control_case":"absent_target"})
        for retries in (0,2):
            relative = f"miss-retries-{retries}"
            e = run_task(sim,*loaded["candidate"],args.recovery_seed,"C",root/relative,
                         observation_mode="feedback",feedback_config=config,first_grasp_offset_m=.065,
                         max_retries=retries,video=True)
            result["negative_controls"].append({**e,"artifact_directory":relative,"control_case":"injected_first_grasp_offset_65mm"})
        finalize(result)
        write(root/"summary.json",result)
        print(json.dumps({"complete":True,"phase":args.phase,"acceptance_checks":result["acceptance_checks"],
                          "acceptance_met":result["acceptance_met"]}),flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    parser.add_argument("--phase",choices=["development","evaluate"],required=True)
    parser.add_argument("--episodes",type=int,default=30)
    parser.add_argument("--static-seed-start",type=int,required=True)
    parser.add_argument("--disturbed-seed-start",type=int,required=True)
    parser.add_argument("--negative-seed-start",type=int,required=True)
    parser.add_argument("--recovery-seed",type=int,required=True)
    parser.add_argument("--perception",type=Path,default=ROOT/"outputs/vision-multiview-01/perception.pt")
    parser.add_argument("--policy",type=Path,default=ROOT/"outputs/goal-policy-multiview-01/residual.pt")
    parser.add_argument("--feedback-config",type=Path)
    parser.add_argument("--force-pulse",type=Path,default=ROOT/"outputs/force-development-01/selected_pulse.json")
    parser.add_argument("--concurrent-frozen-run",type=Path,
                        help="Reserve a live run using only its immutable config/frozen plan")
    parser.add_argument("--protocol",type=Path,help="Required prewritten protocol for adoption evaluation")
    main(parser.parse_args())
