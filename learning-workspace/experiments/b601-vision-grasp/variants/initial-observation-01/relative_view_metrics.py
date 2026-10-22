"""Posthoc localization scoring of all independently audited relative views."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from policy import load
from sim import ROOT
from task_runner import digest, write


def main(args):
    root=(ROOT/"outputs"/args.run).resolve()
    summary=json.loads((root/"summary.json").read_text())
    plan=json.loads((root/"config.json").read_text())
    assert summary["complete"] and json.loads((root/"audit.json").read_text())["passed"]
    assert digest(root/"residual.pt")==plan["model_sha256"]["residual.pt"]
    policy=load(root/"residual.pt")
    rows=[]
    for case,e in zip(plan["cases"],summary["cases"],strict=True):
        view=e["view_pair"];p=None if view is None else view["predictions"][-1]
        corrected=None if p is None else policy.correct(p)
        truth=np.asarray(e["independent_score_only"]["truth_top_xyz_m"])
        error=None if corrected is None else float(np.linalg.norm(np.asarray(corrected["top_xyz_m"])-truth)*1000)
        rows.append({"seed":e["seed"],"offset_index":case["offset_index"],"offset_m":e["offset_m"],
            "home_accepted":bool(e["home_pair"] and e["home_pair"]["quality"]["accepted"]),
            "view_accepted":bool(view and view["quality"]["accepted"]),
            "quality":None if view is None else view["quality"],
            "raw_error_mm":e["independent_score_only"]["raw_error_mm"],
            "corrected_error_mm":error,"corrected_prediction":corrected,
            "record_rows":e["record_rows"],"rgbd_frames":e["rgbd_frames"],"decision_frames":e["decision_frames"],
            "finger_contact_steps":e["finger_contact_steps_for_scoring_only"],
            "forbidden_steps":e["physics_tick_audit"]["forbidden_steps"],
            "physics_steps":e["physics_tick_audit"]["total_steps"],"sim_seconds":e["sim_seconds"]})
    per_seed={}
    for seed in sorted({r["seed"] for r in rows}):
        scenes=[r for r in rows if r["seed"]==seed]
        groups={key:[r for r in scenes if r["view_accepted"] and (r["offset_index"]<2)==old]
                for key,old in (("original",True),("neighbors",False))}
        per_seed[str(seed)]={key:{"accepted":len(values),
            "accepted_at_most_5mm":sum(r["corrected_error_mm"]<=5 for r in values),
            "best_corrected_error_mm":min((r["corrected_error_mm"] for r in values),default=None)} for key,values in groups.items()}
    accepted=[r for r in rows if r["view_accepted"]]
    report={"posthoc_development_only":True,"full_task_test":False,"rows":rows,"per_seed":per_seed,
        "counts":{"probes":len(rows),"accepted_views":len(accepted),
            "accepted_at_most_5mm":sum(r["corrected_error_mm"]<=5 for r in accepted),
            "accepted_above_10mm":sum(r["corrected_error_mm"]>10 for r in accepted),
            **{key:sum(r[key] for r in rows) for key in ("record_rows","rgbd_frames","decision_frames","physics_steps","finger_contact_steps","forbidden_steps")}},
        "summary_sha256":digest(root/"summary.json"),"audit_sha256":digest(root/"audit.json"),
        "source_sha256":digest(Path(__file__)),"residual_sha256":digest(root/"residual.pt"),
        "interpretation":"Known-scene independent pose probes only. Accepted visual pairs are not necessarily accurately localized; keep HOME short path and do not append an ad-hoc pose from these results.16400002 has one accepted neighbor but7.763mm corrected error, so no accepted <=5mm recovery. No task, new-scene policy or adoption improvement shown."}
    shutil.copy2(Path(__file__),root/"metrics_source.py")
    write(root/"metrics.json",report)
    print(json.dumps({"counts":report["counts"],"per_seed":per_seed}),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",required=True)
    main(parser.parse_args())
