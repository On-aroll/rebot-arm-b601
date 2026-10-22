"""Known development cases for HOME short path, fallback, empty and recovery.

All seeds were already observed in development. This is a controller diagnostic,
not a fresh performance evaluation, model selection set, or adoption gate.
"""
import argparse
import json
from pathlib import Path
import shutil

from feedback import configuration
from feedback_audit import audit_episode, same
from policy import load
from sim import ROOT, Simulation
from task import run_task
from task_runner import create_run, digest, perception_model, write


def audit(root):
    root=Path(root).resolve()
    plan=json.loads((root/"config.json").read_text())
    summary=json.loads((root/"summary.json").read_text())
    assert summary["complete"] and plan["posthoc_development_only"]
    provenance=json.loads((root/"provenance.json").read_text())
    for name,sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==sha
        assert digest(ROOT/name)==sha
    assert digest(root/"config.json")==summary["config_sha256"]
    for kind,filename in (("perception","perception.pt"),("policy","residual.pt")):
        assert digest(root/"frozen"/filename)==plan[kind+"_sha256"]
    perception,policy=perception_model(root/"frozen/perception.pt"),load(root/"frozen/residual.pt")
    sim=Simulation(ROOT/"assets/task_scene.xml")
    counters=[]
    try:
        assert len(summary["episodes"])==len(plan["cases"])
        for e,case in zip(summary["episodes"],plan["cases"],strict=True):
            saved=json.loads((root/case["name"]/"result.json").read_text())
            same(e,saved)
            assert e["reset"]["seed"]==case["seed"]
            c,_=audit_episode(root/case["name"],e,sim,perception,policy,plan["feedback_config"])
            counters.append(c)
        e=summary["episodes"]
        assert e[0]["success"] and len(e[0]["attempts"][0]["feedback"]["views"])==1
        assert e[1]["success"] and len(e[1]["attempts"][0]["feedback"]["views"])>1
        assert e[2]["success"] and len(e[2]["attempts"][0]["feedback"]["views"])>1
        assert not e[3]["success"] and e[3]["record_rows"]==0
        assert all(x["attempts"][0]["outcome"]=="empty_grasp_or_slip" for x in e[-2:])
        assert not e[4]["success"] and e[5]["success"] and e[5]["retries"]>=1
        assert not any(x["false_success"] for x in e)
        report={"passed":True,"posthoc_development_only":True,"episodes":len(e),
            "sums":{k:sum(c[k] for c in counters) for k in counters[0] if k!="maximum_command_speed_rad_s"},
            "scope":"Recorded decision/action/score reconstruction on known diagnostic scenes; not adoption evidence."}
        write(root/"audit.json",report);print(json.dumps(report),flush=True)
    finally:
        sim.close()


def main(args):
    perception_path=args.perception.resolve()
    policy_path=args.policy.resolve()
    assert json.loads((policy_path.parent/"summary.json").read_text())["perception_sha256"]==digest(perception_path)
    config=configuration(json.loads(args.feedback_config.read_text()))
    pulse=json.loads((ROOT/"outputs/force-development-01/selected_pulse.json").read_text())
    cases=[{"name":"static_short_path","seed":8100000},
           {"name":"large_update_fallback","seed":8200003,"pulse":pulse},
           {"name":"rolling_fallback","seed":14000015,"pulse":pulse},
           {"name":"empty","seed":14100000,"absent":True},
           {"name":"miss_no_retry","seed":8400000,"offset":.065,"retries":0},
           {"name":"miss_retry","seed":8400000,"offset":.065,"retries":2}]
    root=create_run(args.name,perception_path)
    shutil.copy2(Path(__file__),root/"source/home_first_diagnostic.py")
    provenance=json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"]["home_first_diagnostic.py"]=digest(Path(__file__))
    write(root/"provenance.json",provenance)
    (root/"frozen").mkdir()
    shutil.copy2(perception_path,root/"frozen/perception.pt")
    shutil.copy2(policy_path,root/"frozen/residual.pt")
    plan={"posthoc_development_only":True,"hardware_executable":False,"feedback_config":config,
        "cases":cases,"perception_sha256":digest(perception_path),"policy_sha256":digest(policy_path)}
    write(root/"config.json",plan)
    result={"posthoc_development_only":True,"complete":False,"episodes":[],"config_sha256":digest(root/"config.json")}
    perception,policy=perception_model(root/"frozen/perception.pt"),load(root/"frozen/residual.pt")
    sim=Simulation(ROOT/"assets/task_scene.xml")
    try:
        for case in cases:
            e=run_task(sim,perception,policy,case["seed"],"B",root/case["name"],
                absent=case.get("absent",False),force_pulse=case.get("pulse"),
                observation_mode="feedback",feedback_config=config,
                first_grasp_offset_m=case.get("offset",0.),max_retries=case.get("retries",2),video=True)
            result["episodes"].append(e);write(root/"summary.json",result)
            print(json.dumps({"case":case["name"],"success":e["success"],"retries":e["retries"],"reason":e["reason"]}),flush=True)
        result["complete"]=True;write(root/"summary.json",result)
    finally:
        sim.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument("--name")
    g.add_argument("--audit-run",type=Path)
    p.add_argument("--perception",type=Path,default=ROOT/"outputs/vision-multiview-egl-01/perception.pt")
    p.add_argument("--policy",type=Path,default=ROOT/"outputs/goal-policy-multiview-egl-01/residual.pt")
    p.add_argument("--feedback-config",type=Path,default=ROOT/"configs/home-first-development-01.json")
    args=p.parse_args()
    if args.audit_run:
        audit(args.audit_run)
    else:
        main(args)
