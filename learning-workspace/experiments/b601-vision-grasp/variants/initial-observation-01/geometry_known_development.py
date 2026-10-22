"""Known physical regressions for matched geometry candidates and V2 parity."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from feedback_audit import audit_episode, same
from multiview_runner import retained_v2
from policy import load
from sim import ROOT, Simulation
from task import run_task
from task_runner import create_run, digest, perception_model, write


def baseline_parity(directory,reference,episode):
    old=json.loads((reference/"result.json").read_text())
    assert {k:v for k,v in episode.items() if k!="wall_seconds"}=={k:v for k,v in old.items() if k!="wall_seconds"}
    for filename in ("trajectory.jsonl","final_score_trace.jsonl"):
        assert (directory/filename).read_bytes()==(reference/filename).read_bytes()
    for filename in ("observations.npz","decision_observations.npz"):
        assert (directory/filename).exists()==(reference/filename).exists()
        if (directory/filename).exists():
            with np.load(directory/filename,allow_pickle=False) as a,np.load(reference/filename,allow_pickle=False) as b:
                assert a.files==b.files
                assert all(np.array_equal(a[key],b[key]) for key in a.files)


def audit(root):
    root=Path(root).resolve()
    plan=json.loads((root/"config.json").read_text())
    summary=json.loads((root/"summary.json").read_text())
    assert summary["complete"] and plan["posthoc_development_only"]
    provenance=json.loads((root/"provenance.json").read_text())
    for name,sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==sha and digest(ROOT/name)==sha
    assert digest(root/"config.json")==summary["config_sha256"]
    _,config,baseline=retained_v2();same(plan["retained_baseline"],baseline)
    sim=Simulation(ROOT/"assets/task_scene.xml")
    counters=[];parity=0
    try:
        for key,model in plan["models"].items():
            for kind in ("perception","policy"):
                assert digest(model[kind+"_path"])==model[kind+"_sha256"]
            loaded=perception_model(model["perception_path"]),load(model["policy_path"])
            episodes=summary["groups"][key]
            assert len(episodes)==len(plan["cases"])
            for case,e in zip(plan["cases"],episodes,strict=True):
                directory=root/key/case["name"]
                same(e,json.loads((directory/"result.json").read_text()))
                assert e["reset"]["seed"]==case["seed"] and e["task"]["destination"]==case["goal"]
                assert e["max_retries"]==case["retries"] and e["first_grasp_offset_m"]==case["offset"]
                same(e["environment_for_scoring_only"]["config"],case["pulse"])
                c,_=audit_episode(directory,e,sim,*loaded,config);counters.append(c)
                if key=="retained_v2" and case["baseline_exact_reference"]:
                    reference=Path(case["source_directory"])
                    assert all(digest(reference/name)==sha for name,sha in case["reference_sha256"].items())
                    baseline_parity(directory,reference,e);parity+=1
        report={"passed":True,"posthoc_development_only":True,"episodes":len(counters),
            "baseline_exact_physical_RGBD_action_reexecutions":parity,
            "sums":{k:sum(c[k] for c in counters) for k in counters[0] if k!="maximum_command_speed_rad_s"},
            "maximum_command_speed_rad_s":max(c["maximum_command_speed_rad_s"] for c in counters),
            "scope":"Known diagnostic recorded decision/action/physical score audit; V2 reference parity includes exact physical re-execution and RGBD/action arrays. Not fresh adoption evidence."}
        write(root/"audit.json",report);print(json.dumps(report),flush=True)
    finally:
        sim.close()


def main(args):
    runtime,config,baseline=retained_v2()
    paths={**({} if args.skip_baseline else {"retained_v2":(runtime/"models/perception.pt",runtime/"models/residual.pt")}),
        **{f"geometry{suffix}":((ROOT/f"outputs/vision-geometry{suffix}-01/perception.pt").resolve(),
            (ROOT/f"outputs/goal-policy-geometry{suffix}-01/residual.pt").resolve()) for suffix in args.suffixes}}
    cases=[]
    for name,relative in [*( (name,"fragment-known-development-01/"+name) for name in
        ("static_short_path","large_update_fallback","rolling_fallback","empty","miss_no_retry","miss_retry")),
        ("initial_miss_22000018","goal-fragment-home-v2-candidate-01/static-new_feedback-018"),
        ("prior_11100009","fragment075-regression-11100009/episode"),
        ("low_confidence_16400002","fragment-regression-16400002/episode")]:
        source=(ROOT/"outputs"/relative).resolve()
        old=json.loads((source/"result.json").read_text())
        cases.append({"name":name,"seed":old["reset"]["seed"],"goal":old["task"]["destination"],
            "absent":old["reset"]["absent"],"pulse":old["environment_for_scoring_only"]["config"],
            "offset":old["first_grasp_offset_m"],"retries":old["max_retries"],
            "source_directory":str(source),"baseline_exact_reference":old["feedback_config"]==config,
            "reference_sha256":{p.name:digest(p) for p in source.iterdir() if p.suffix in (".json",".jsonl",".npz")}})
    root=create_run(args.name,next(iter(paths.values()))[0])
    provenance=json.loads((root/"provenance.json").read_text())
    for name in (Path(__file__).name,"multiview_runner.py","multiview_audit.py"):
        shutil.copy2(ROOT/name,root/"source"/name)
        provenance["source_sha256"][name]=digest(ROOT/name)
    write(root/"provenance.json",provenance)
    (root/"frozen").mkdir()
    models={}
    for key,(detector,head) in paths.items():
        assert json.loads((head.parent/"summary.json").read_text())["perception_sha256"]==digest(detector)
        saved={}
        for kind,path in (("perception",detector),("policy",head)):
            destination=(root/"frozen"/f"{key}-{kind}.pt").resolve()
            shutil.copy2(path,destination)
            saved[kind+"_path"],saved[kind+"_sha256"]=str(destination),digest(destination)
        models[key]=saved
    write(root/"config.json",{"posthoc_development_only":True,"hardware_executable":False,
        "models":models,"cases":cases,"feedback_config":config,"retained_baseline":baseline})
    result={"complete":False,"posthoc_development_only":True,"groups":{},"config_sha256":digest(root/"config.json")}
    sim=Simulation(ROOT/"assets/task_scene.xml")
    try:
        for key,model in models.items():
            loaded=perception_model(model["perception_path"]),load(model["policy_path"])
            result["groups"][key]=[]
            for case in cases:
                directory=root/key/case["name"]
                e=run_task(sim,*loaded,case["seed"],case["goal"],directory,video=True,
                    absent=case["absent"],force_pulse=case["pulse"],observation_mode="feedback",feedback_config=config,
                    first_grasp_offset_m=case["offset"],max_retries=case["retries"])
                result["groups"][key].append(e);write(root/"summary.json",result)
                print(json.dumps({"model":key,"case":case["name"],"success":e["success"],"reason":e["reason"],"retries":e["retries"]}),flush=True)
        result["complete"]=True;write(root/"summary.json",result)
    finally:
        sim.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--name")
    group.add_argument("--audit-run",type=Path)
    parser.add_argument("--suffixes",nargs="+",default=["055","050"])
    parser.add_argument("--skip-baseline",action="store_true")
    args=parser.parse_args()
    audit(args.audit_run) if args.audit_run else main(args)
