"""Summarize a completed, audited paired comparison without changing controls."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
from statistics import fmean


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def decision_records(episode):
    records={}
    for attempt in episode["attempts"]:
        records[attempt["initial_decision_index"]]=("initial",attempt["raw_prediction"])
        for view in attempt.get("feedback",{}).get("views",[]):
            for index,prediction in zip(view["decision_indices"],view["raw_predictions"],strict=True):
                assert index not in records
                records[index] = ("feedback",prediction)
        if "verification_decision_index" in attempt:
            index=attempt["verification_decision_index"]
            assert index not in records
            records[index] = ("verification",attempt["verification_prediction"])
    assert set(records)==set(range(episode["decision_frames"]))
    return records


def decision_counts(episodes):
    paths,roles=Counter(),Counter()
    fallback_frames=[];fallback_actions=[];learned_actions=0
    for e in episodes:
        for index,(role,prediction) in decision_records(e).items():
            path=prediction.get("decoder_path","legacy_v2") if prediction else "no_valid_prediction"
            paths[path]+=1;roles[(role,path)]+=1
            if path=="retained_v2_fallback":
                fallback_frames.append({"seed":e["reset"]["seed"],"goal":e["task"]["destination"],
                    "decision_index":index,"role":role,"episode":e["artifact_directory"]})
        for attempt in e["attempts"]:
            for call in attempt["policy_calls"]:
                if call["used_for_action"] and call["prediction"]["action_source"]=="learned_bounded_visuomotor_residual":
                    learned_actions+=1
                    if call["raw_prediction"].get("decoder_path")=="retained_v2_fallback":
                        fallback_actions.append({"seed":e["reset"]["seed"],"goal":e["task"]["destination"],
                            "decision_index":call["decision_index"],"role":call["role"],"episode":e["artifact_directory"]})
    assert learned_actions==sum(e["learned_action_calls"] for e in episodes)
    return {"decision_frames":sum(paths.values()),"prediction_paths":dict(paths),
        "by_role":{role:{path:n for (r,path),n in roles.items() if r==role} for role in ("initial","feedback","verification")},
        "learned_action_calls":learned_actions,"fallback_frames":fallback_frames,
        "fallback_learned_actions":fallback_actions,
        "note":"Verification stores the controller's workspace-filtered prediction; None means no valid prediction at that stage. Frames and policy calls are correlated, not independent scenes."}


def pair_rows(reference,candidate):
    rows=[]
    for a,b in zip(reference,candidate,strict=True):
        assert a["reset"]==b["reset"] and a["task"]==b["task"]
        rows.append({"seed":a["reset"]["seed"],"goal":a["task"]["destination"],
            "reference_success":a["success"],"candidate_success":b["success"],
            "reference_first_success":a["success"] and a["retries"]==0,
            "candidate_first_success":b["success"] and b["retries"]==0,
            "reference_retries":a["retries"],"candidate_retries":b["retries"],
            "sim_seconds_difference":b["sim_seconds"]-a["sim_seconds"],
            "reference_episode":a["artifact_directory"],"candidate_episode":b["artifact_directory"]})
    common=[r for r in rows if r["reference_success"] and r["candidate_success"]]
    return {"rows":rows,"full_gains":[r for r in rows if not r["reference_success"] and r["candidate_success"]],
        "full_losses":[r for r in rows if r["reference_success"] and not r["candidate_success"]],
        "first_gains":[r for r in rows if not r["reference_first_success"] and r["candidate_first_success"]],
        "first_losses":[r for r in rows if r["reference_first_success"] and not r["candidate_first_success"]],
        "common_success_pairs":len(common),
        "common_success_mean_sim_seconds_difference":fmean(r["sim_seconds_difference"] for r in common) if common else None}


def main(args):
    root=(Path(__file__).resolve().parent/"outputs"/args.run).resolve()
    frozen=json.loads((root/"frozen_evaluation.json").read_text())
    result=json.loads((root/"summary.json").read_text())
    audit=json.loads((root/"audit.json").read_text())
    assert audit["passed"] and "acceptance_checks" in result
    groups,counts,paired={}, {}, {}
    episodes=[]
    for block,variants in result["groups"].items():
        groups[block]={};counts[block]={}
        for name,group in variants.items():
            es=group["episodes"]
            assert [e["reset"]["seed"] for e in es]==frozen["test_seeds"][block]
            groups[block][name]={**group["summary"],"first_and_retries_by_goal":{
                goal:{"first":sum(e["success"] and e["retries"]==0 for e in es if e["task"]["destination"]==goal),
                      "retries":sum(e["retries"] for e in es if e["task"]["destination"]==goal)}
                for goal in group["summary"]["by_goal"]}}
            counts[block][name]=decision_counts(es);episodes+=es
        paired[block]={reference:pair_rows(variants[reference]["episodes"],variants["new_feedback"]["episodes"])
            for reference in ("retained_v3","zero_residual","new_observe_only","retained_two_view")}
    controls=result["negative_controls"]
    assert len(episodes)+len(controls)==audit["episodes"]
    failures=[{"episode":e["artifact_directory"],"seed":e["reset"]["seed"],"goal":e["task"]["destination"],
        "reason":e["reason"],"retries":e["retries"],"controller_success":e["controller_success"],
        "physical_success":e["physical_success"],"rows":e["record_rows"],
        "attempt_outcomes":[a.get("outcome","no_action") for a in e["attempts"]]} for e in episodes if not e["success"]]
    report={"posthoc_report":True,"phase":result["phase"],"groups":groups,"paired_details":paired,
        "paired_statistics":result["paired_comparisons"],"decisions":counts,"all_main_task_failures":failures,
        "negative_controls":[{k:e[k] for k in ("artifact_directory","control_case","max_retries","success","reason","retries","record_rows","false_success")} for e in controls],
        "negative_decisions":decision_counts(controls),"acceptance_checks":result["acceptance_checks"],
        "acceptance_met":result["acceptance_met"],"false_successes":sum(e["false_success"] for e in episodes+controls),
        "summary_sha256":digest(root/"summary.json"),"audit_sha256":digest(root/"audit.json"),
        "report_source_sha256":digest(Path(__file__)),"scope":"Posthoc aggregation of all audited recorded trials; no control/model changes, physics re-execution or hardware inference."}
    shutil.copy2(Path(__file__),root/"report_metrics_source.py")
    (root/"report_metrics.json").write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n")
    print(json.dumps({"phase":report["phase"],"episodes":audit["episodes"],"failures":len(failures),
                      "false_successes":report["false_successes"],"acceptance_met":report["acceptance_met"]}),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path,required=True)
    main(parser.parse_args())
