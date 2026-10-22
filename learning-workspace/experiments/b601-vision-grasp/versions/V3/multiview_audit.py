"""Audit model pairing, exact observations, score traces and comparison design."""
import argparse
import inspect
import json
from pathlib import Path

import numpy as np

from feedback_audit import audit_episode, same
from feedback_runner import group_summary
from multiview_runner import VARIANTS, finalize, retained_v2
from policy import load
from score_audit import audit_score_trace
from sim import ROOT, Simulation
from task import GOALS, collect_feedback, correct_prediction, grasp_geometry, run_task
from task_runner import digest, perception_model, write


def audit(run):
    root = ROOT/"outputs"/run
    frozen = json.loads((root/"frozen_evaluation.json").read_text())
    result = json.loads((root/"summary.json").read_text())
    assert frozen["schema"]==result["schema"]=="b601-multiview-task-comparison-v1"
    for key in frozen:
        if key not in ("groups","negative_controls"):
            same(result[key],frozen[key])
    provenance = json.loads((root/"provenance.json").read_text())
    for name,sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==sha
        assert digest(ROOT/name)==sha, f"Source changed after freeze: {name}"
    manifest = json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p)==sha for p,sha in manifest["files_sha256"].items())
    assert digest(root/"prior_seed_ledger.json")==frozen["prior_seed_ledger_sha256"]
    assert digest(root/"config.json")==frozen["config_sha256"]
    if frozen["phase"]=="evaluate":
        assert frozen.get("protocol"),"Evaluation protocol must be saved before physics"
    if frozen.get("protocol"):
        assert digest(root/frozen["protocol"]["saved_name"])==frozen["protocol"]["sha256"]
    ledger = json.loads((root/"prior_seed_ledger.json").read_text())
    excluded = set(ledger["seeds"])
    for item in ledger["sources"]:
        assert digest(item["path"])==item["sha256"],item["path"]
    if ledger.get("concurrent_reserved_run"):
        concurrent=Path(ledger["concurrent_reserved_run"])
        evidence={Path(item["path"]) for item in ledger["sources"] if concurrent in Path(item["path"]).parents}
        assert evidence=={concurrent/"config.json",concurrent/"frozen_evaluation.json"}
        reserved=json.loads((concurrent/"frozen_evaluation.json").read_text())["test_seeds"]
        assert set(s for values in reserved.values() for s in values)<=excluded
    for item in frozen["frozen_datasets"]:
        assert digest(item["path"])==item["sha256"]
        with np.load(item["path"],allow_pickle=False) as data:
            excluded.update(data["seeds"].tolist())
    flat = [s for values in frozen["test_seeds"].values() for s in values]
    assert len(flat)==len(set(flat)) and not excluded.intersection(flat)
    loaded = {}
    for name,m in frozen["models"].items():
        for kind in ("perception","policy"):
            assert digest(m[kind+"_path"])==m[kind+"_sha256"]
        source_summary = json.loads((Path(m["source_policy_path"]).parent/"summary.json").read_text())
        assert source_summary["perception_sha256"]==m["perception_sha256"]
        loaded[name] = (perception_model(m["perception_path"]),load(m["policy_path"]))
    # Retained baseline identity is explicit, not relabelled candidate weights.
    assert frozen["models"]["baseline"]["perception_sha256"]=="71acc8fa4d5a5af40b8cb78ea7e0dade1181e505d84d514589ad30c2ee822636"
    assert frozen["models"]["baseline"]["policy_sha256"]=="da11e50150cf2215d1942cba8b2e11731bf9a513d30a60944194c8888a0da9ec"
    _,baseline_config,baseline = retained_v2()
    same(frozen["retained_baseline"],baseline)
    same(frozen["variant_feedback_configs"]["retained_v2"],baseline_config)
    same(frozen["variants"],{k:list(v) for k,v in VARIANTS.items()})
    assert VARIANTS["retained_v2"]==("baseline","feedback")
    for function in (run_task,collect_feedback,correct_prediction,grasp_geometry):
        source = inspect.getsource(function)
        assert all(p not in source for p in ("sim.obj","sim.og","sim.half_size",".xpos",".xmat","sim.contacts("))
    counters,traces,score_samples = [],{},0
    sim = Simulation(ROOT/"assets/task_scene.xml")
    prefix_pairs = 0
    try:
        for block,groups in result["groups"].items():
            assert set(groups)==set(VARIANTS)
            for variant,group in groups.items():
                model_name,mode = VARIANTS[variant]
                assert [e["reset"]["seed"] for e in group["episodes"]]==frozen["test_seeds"][block]
                assert [e["task"]["destination"] for e in group["episodes"]]==[list(GOALS)[i%3] for i in range(len(group["episodes"]))]
                for e in group["episodes"]:
                    assert e["observation_mode"]==mode
                    assert e["environment_for_scoring_only"]["config"]==(frozen["force_pulse"] if block=="disturbed" else None)
                    directory = root/e["artifact_directory"]
                    counter,rows = audit_episode(directory,e,sim,*loaded[model_name],frozen["variant_feedback_configs"][variant])
                    counters.append(counter); traces[e["artifact_directory"]] = rows
                    score_samples += e["score_trace_samples"]
                same(group["summary"],group_summary(group["episodes"]))
            for variant,group in groups.items():
                same([(e["reset"],e["task"]) for e in group["episodes"]],
                     [(e["reset"],e["task"]) for e in groups["retained_v2"]["episodes"]])
            for a,b in zip(groups["new_observe_only"]["episodes"],groups["new_feedback"]["episodes"],strict=True):
                for key in ("config","applied_steps","actual_impulse_ns","actual_start_s","actual_end_s"):
                    same(a["environment_for_scoring_only"][key],b["environment_for_scoring_only"][key])
                aa,bb = a["attempts"][0],b["attempts"][0]
                same(aa["prediction"],bb["prediction"])
                if "feedback" not in aa:
                    assert "feedback" not in bb and not aa["prediction"]
                    continue
                same(aa["feedback"]["views"],bb["feedback"]["views"])
                end = aa["feedback"]["update_time_s"]
                same(end,bb["feedback"]["update_time_s"])
                same([r for r in traces[a["artifact_directory"]] if r["t_sim_s"]<=end+1e-9],
                     [r for r in traces[b["artifact_directory"]] if r["t_sim_s"]<=end+1e-9])
                prefix_pairs += 1
        for e in result["negative_controls"]:
            directory = root/e["artifact_directory"]
            counter,_ = audit_episode(directory,e,sim,*loaded["candidate"],frozen["feedback_config"])
            counters.append(counter)
            score_samples += audit_score_trace(directory,e,sim.model.opt.timestep)["samples"]
        expected = finalize(json.loads(json.dumps(result)))
        for key in ("paired_comparisons","acceptance_checks","acceptance_met"):
            same(result[key],expected[key])
        report = {"passed":True,"phase":result["phase"],"episodes":len(counters),
                  "matched_motion_prefix_pairs":prefix_pairs,"score_trace_samples_reconstructed":score_samples,
                  "maximum_command_speed_rad_s":max(c["maximum_command_speed_rad_s"] for c in counters),
                  "sums":{k:sum(c[k] for c in counters) for k in counters[0] if k!="maximum_command_speed_rad_s"},
                  "datasets_checked":len(frozen["frozen_datasets"]),"asset_files_checked":len(manifest["files_sha256"]),
                  "prior_seed_overlap":False,"acceptance_met":result["acceptance_met"],
                  "scope":"Reconstruction of recorded decisions and physical predicates; not independent physics re-execution or hardware validation."}
        write(root/"audit.json",report)
        print(json.dumps(report),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",required=True)
    audit(parser.parse_args().run)
