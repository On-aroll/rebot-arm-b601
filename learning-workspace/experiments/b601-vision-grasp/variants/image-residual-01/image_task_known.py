"""Known physical regressions: frozen V3, image head and same-vision zero head."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from feedback_audit import audit_episode, same
from image_residual_development import V3
from policy import load
from sim import ROOT, Simulation
from task import run_task
from task_runner import create_run, digest, perception_model, write

CASE_NAMES = ("static_short_path","large_update_fallback","rolling_fallback","empty","miss_no_retry",
              "miss_retry","initial_miss_22000018","prior_11100009","low_confidence_16400002")


def parity(directory, reference, episode):
    old=json.loads((reference/"result.json").read_text())
    same({k:v for k,v in episode.items() if k!="wall_seconds"},{k:v for k,v in old.items() if k!="wall_seconds"})
    for name in ("trajectory.jsonl","final_score_trace.jsonl"):
        assert (directory/name).read_bytes()==(reference/name).read_bytes(),name
    for name in ("observations.npz","decision_observations.npz"):
        assert (directory/name).exists()==(reference/name).exists()
        if (reference/name).exists():
            with np.load(directory/name,allow_pickle=False) as a,np.load(reference/name,allow_pickle=False) as b:
                assert a.files==b.files and all(np.array_equal(a[key],b[key]) for key in a.files),name


def verify_inputs(config):
    assert digest(V3/"release_manifest.json")==config["V3_manifest_sha256"]
    manifest=json.loads((V3/"release_manifest.json").read_text())
    assert all(digest(V3/p)==h for p,h in manifest["files_sha256"].items())
    for path,expected in config["inputs_sha256"].items():assert digest(path)==expected,path
    for model in config["models"].values():
        for kind in ("perception","policy"):
            if model[kind+"_path"] is not None:assert digest(model[kind+"_path"])==model[kind+"_sha256"]


def audit(root):
    root=root.resolve();plan=json.loads((root/"config.json").read_text());summary=json.loads((root/"summary.json").read_text())
    assert summary["complete"] and summary["posthoc_development_only"]
    assert digest(root/"config.json")==summary["config_sha256"]
    for name,expected in json.loads((root/"provenance.json").read_text())["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==digest(ROOT/name)==expected,name
    verify_inputs(plan)
    manifest=json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p)==h for p,h in manifest["files_sha256"].items())
    sim=Simulation(ROOT/"assets/task_scene.xml");counters=[];exact=0
    try:
        for variant,model in plan["models"].items():
            perception=perception_model(model["perception_path"])
            head=None if model["policy_path"] is None else load(model["policy_path"])
            es=summary["groups"][variant];assert len(es)==len(plan["cases"])==9
            for case,e in zip(plan["cases"],es,strict=True):
                directory=root/variant/case["name"]
                same(e,json.loads((directory/"result.json").read_text()))
                assert e["reset"]["seed"]==case["seed"] and e["task"]["destination"]==case["goal"]
                assert e["max_retries"]==case["retries"] and e["first_grasp_offset_m"]==case["offset"]
                same(e["environment_for_scoring_only"]["config"],case["pulse"])
                c,_=audit_episode(directory,e,sim,perception,head,plan["feedback_config"]);counters.append(c)
                if variant=="V3":parity(directory,Path(case["reference"]),e);exact+=1
                if variant=="zero_residual":assert e["learned_action_calls"]==e["learned_inference_calls"]==0
        report={"passed":True,"posthoc_development_only":True,"episodes":len(counters),
            "V3_exact_physical_action_RGBD_reexecutions":exact,"asset_files_checked":len(manifest["files_sha256"]),
            "sums":{k:sum(c[k] for c in counters) for k in counters[0] if k!="maximum_command_speed_rad_s"},
            "maximum_command_speed_rad_s":max(c["maximum_command_speed_rad_s"] for c in counters),
            "scope":"Known physical regression records;9V3cases also compare full re-executed action/score bytes andRGBD arrays. Zero-residual uses identical visual decoder/control with no learned calls. Not fresh adoption evidence."}
        write(root/"audit.json",report);print(json.dumps(report),flush=True)
    finally:sim.close()


def main(args):
    training=(ROOT/"outputs/image-residual-development-01").resolve()
    reviewed=(ROOT/"outputs/image-residual-known-view-review-01").resolve()
    trained=json.loads((training/"summary.json").read_text())
    assert trained["eligible_for_task_development"] and json.loads((training/"audit.json").read_text())["passed"]
    assert json.loads((reviewed/"audit.json").read_text())["passed"]
    oldrun=(ROOT/"outputs/geometry-fallback-known-development-01").resolve()
    assert json.loads((oldrun/"audit.json").read_text())["passed"]
    paths=[training/"summary.json",training/"audit.json",reviewed/"summary.json",reviewed/"audit.json",oldrun/"config.json",oldrun/"summary.json",oldrun/"audit.json"]
    config=json.loads((V3/"feedback_config.json").read_text());cases=[]
    for name in CASE_NAMES:
        source=oldrun/"geometry050-fallback"/name;e=json.loads((source/"result.json").read_text())
        same(e["feedback_config"],config)
        case={"name":name,"seed":e["reset"]["seed"],"goal":e["task"]["destination"],"absent":e["reset"]["absent"],
              "pulse":e["environment_for_scoring_only"]["config"],"offset":e["first_grasp_offset_m"],
              "retries":e["max_retries"],"reference":str(source)}
        cases.append(case)
        paths.extend(source/f for f in ("result.json","trajectory.jsonl","final_score_trace.jsonl","decision_observations.npz","observations.npz") if (source/f).exists())
    root=create_run(args.name,V3/"models/perception.pt")
    provenance=json.loads((root/"provenance.json").read_text())
    for name in ("image_task_known.py","image_residual_development.py","audit_frozen.py"):
        shutil.copy2(ROOT/name,root/"source"/name);provenance["source_sha256"][name]=digest(ROOT/name)
    write(root/"provenance.json",provenance)
    (root/"frozen").mkdir();models={}
    for variant,head in [("V3",V3/"models/residual.pt"),("image_residual",training/"residual.pt"),("zero_residual",None)]:
        model={}
        for kind,source in [("perception",V3/"models/perception.pt"),("policy",head)]:
            if source is None:model.update(policy_path=None,policy_sha256=None);continue
            dest=(root/"frozen"/f"{variant}-{kind}.pt").resolve();shutil.copy2(source,dest)
            model[kind+"_path"],model[kind+"_sha256"]=str(dest),digest(dest)
        models[variant]=model
    plan={"posthoc_development_only":True,"hardware_executable":False,"cases":cases,"feedback_config":config,
          "models":models,"V3_manifest_sha256":digest(V3/"release_manifest.json"),"inputs_sha256":{str(p):digest(p) for p in paths}}
    write(root/"config.json",plan);verify_inputs(plan)
    summary={"complete":False,"posthoc_development_only":True,"groups":{},"config_sha256":digest(root/"config.json")}
    sim=Simulation(ROOT/"assets/task_scene.xml")
    try:
        for variant,model in models.items():
            perception=perception_model(model["perception_path"]);head=None if model["policy_path"] is None else load(model["policy_path"])
            summary["groups"][variant]=[]
            for case in cases:
                e=run_task(sim,perception,head,case["seed"],case["goal"],root/variant/case["name"],
                    absent=case["absent"],force_pulse=case["pulse"],observation_mode="feedback",feedback_config=config,
                    first_grasp_offset_m=case["offset"],max_retries=case["retries"],video=variant=="image_residual")
                summary["groups"][variant].append(e);write(root/"summary.json",summary)
                print(json.dumps({"variant":variant,"case":case["name"],"success":e["success"],"reason":e["reason"],"retries":e["retries"]}),flush=True)
        summary["complete"]=True;write(root/"summary.json",summary)
    finally:sim.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--name");group.add_argument("--audit-run",type=Path)
    args=parser.parse_args()
    audit(ROOT/"outputs"/args.audit_run) if args.audit_run else main(args)
