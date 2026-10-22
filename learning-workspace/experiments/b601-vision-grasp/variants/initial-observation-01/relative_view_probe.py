"""Known-scene physical probe of views anchored by the actual visual policy."""
import argparse
import inspect
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np
from PIL import Image

from episode import DOWN
from feedback import pair_quality
from feedback_audit import same
from initial_visibility_probe import ProbeRecorder, score_only
from policy import load, valid_prediction
from sim import HOME, ROOT, Simulation
from task import grasp_geometry
from task_runner import create_run, digest, perception_model, write
from vision import estimate


def capture_pair(recorder,model,q,stage,config):
    predictions,cameras,indices=[],[],[]
    for i in range(2):
        if i:
            recorder.move(q,.046,config["pair_gap_s"],stage+"_hold")
        frame,index=recorder.capture_decision(stage+f"_{i}")
        predictions.append(estimate(model,frame));cameras.append(frame["camera"]);indices.append(index)
    return {"predictions":predictions,"cameras":cameras,"decision_indices":indices}


def run_case(sim,model,policy,config,case,offset,directory,video=False):
    directory.mkdir()
    reset=sim.reset(case["seed"],absent=case["absent"],force_pulse=case["pulse"])
    recorder=ProbeRecorder(sim,directory,video)
    frame,_=recorder.capture_decision("initial_HOME")
    raw=estimate(model,frame)
    anchor=policy.correct(raw) if valid_prediction(raw) else None
    result={"seed":case["seed"],"reset":reset,"offset_m":offset,"initial_raw_prediction":raw,
            "anchor_prediction":anchor,"home_pair":None,"view_pair":None,"q_target_rad":None}
    if valid_prediction(anchor):
        recorder.move(HOME,.046,config["home_refresh_wait_s"],"timed_HOME_refresh")
        home=capture_pair(recorder,model,HOME,"home",config)
        home["quality"]=pair_quality(home["predictions"],home["cameras"],anchor,config)
        result["home_pair"]=home
        _,grasp=grasp_geometry(anchor["top_xyz_m"])
        target=grasp+np.asarray(offset)
        try:
            q=sim.ik(target,DOWN,HOME)
        except ValueError as exc:
            result["ik_error"]=str(exc)
        else:
            result["tcp_target_xyz_m"]=target.tolist();result["q_target_rad"]=q.tolist()
            recorder.move(q,.046,2.,"independent_relative_view")
            recorder.move(q,.046,config["settle_s"],"view_settle")
            view=capture_pair(recorder,model,q,"relative_view",config)
            view["quality"]=pair_quality(view["predictions"],view["cameras"],anchor,config)
            result["view_pair"]=view
    recorder.save()
    Image.fromarray(frame["rgb"]).save(directory/"initial.png")
    Image.fromarray(recorder.decision_frames[-1]["rgb"]).save(directory/"last.png")
    rows=recorder.rows
    speed=[float(np.max(np.abs(np.asarray(b["q_command_rad"])-a["q_command_rad"])/(b["t_sim_s"]-a["t_sim_s"])))
           for a,b in zip(rows,rows[1:])]
    prediction=None if result["view_pair"] is None else result["view_pair"]["predictions"][-1]
    result.update(sim_seconds=float(sim.data.time),record_rows=len(rows),rgbd_frames=len(recorder.frames),
        decision_frames=len(recorder.decision_frames),physics_tick_audit=recorder.physics_audit,
        finger_contact_steps_for_scoring_only=recorder.finger_contact_steps,
        maximum_command_speed_rad_s=max(speed,default=0.),
        independent_score_only=score_only(sim,prediction,reset))
    write(directory/"result.json",result)
    return result


def audit(root):
    root=Path(root).resolve()
    plan=json.loads((root/"config.json").read_text());summary=json.loads((root/"summary.json").read_text())
    assert summary["complete"] and plan["posthoc_development_only"]
    assert digest(root/"config.json")==summary["config_sha256"]
    for name,sha in json.loads((root/"provenance.json").read_text())["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==digest(ROOT/name)==sha
    for filename in ("perception.pt","residual.pt"):
        assert digest(root/filename)==plan["model_sha256"][filename]
    for item in plan["known_reference_evidence"]:
        assert digest(item["path"])==item["sha256"]
    manifest=json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p)==sha for p,sha in manifest["files_sha256"].items())
    assert all(token not in inspect.getsource(run_case) for token in ("sim.obj","sim.og","sim.half_size",".xpos",".xmat","sim.contacts("))
    sim=Simulation(ROOT/"assets/task_scene.xml")
    model,policy=perception_model(root/"perception.pt"),load(root/"residual.pt")
    try:
        with tempfile.TemporaryDirectory(prefix="b601-relative-view-audit-") as temporary:
            for i,(case,expected) in enumerate(zip(plan["cases"],summary["cases"],strict=True)):
                directory=Path(temporary)/f"case-{i:03d}"
                actual=run_case(sim,model,policy,plan["feedback_config"],case,plan["offsets_m"][case["offset_index"]],directory)
                same(actual,expected)
                original=root/f"case-{i:03d}"
                for name in ("trajectory.jsonl","final_score_trace.jsonl"):
                    assert (directory/name).read_bytes()==(original/name).read_bytes()
                for name in ("decision_observations.npz","observations.npz"):
                    assert (directory/name).exists()==(original/name).exists()
                    if (directory/name).exists():
                        with np.load(directory/name,allow_pickle=False) as a,np.load(original/name,allow_pickle=False) as b:
                            assert a.files==b.files and all(np.array_equal(a[k],b[k]) for k in a.files)
        report={"passed":True,"posthoc_development_only":True,"physical_probes_reexecuted":len(summary["cases"]),
            "physics_steps_reexecuted":sum(e["physics_tick_audit"]["total_steps"] for e in summary["cases"]),
            "exact_RGBD_action_reexecution":True,"asset_files_checked":len(manifest["files_sha256"]),
            "scope":"Independent known-scene view probes, not an automatic view policy, sequential controller replacement, full-task success or adoption evidence."}
        write(root/"audit.json",report);print(json.dumps(report),flush=True)
    finally:
        sim.close()


def main(args):
    reference=(ROOT/"outputs/geometry-fallback-known-development-01").resolve()
    source=json.loads((reference/"config.json").read_text())
    assert json.loads((reference/"summary.json").read_text())["complete"]
    assert json.loads((reference/"audit.json").read_text())["passed"]
    models=source["models"]["geometry050-fallback"]
    root=create_run(args.name,Path(models["perception_path"]))
    provenance=json.loads((root/"provenance.json").read_text())
    for name in (Path(__file__).name,"initial_visibility_probe.py"):
        shutil.copy2(ROOT/name,root/"source"/name);provenance["source_sha256"][name]=digest(ROOT/name)
    write(root/"provenance.json",provenance)
    for key,name in (("perception","perception.pt"),("policy","residual.pt")):
        shutil.copy2(models[key+"_path"],root/name)
        assert digest(root/name)==models[key+"_sha256"]
    selected={"static_short_path","large_update_fallback","rolling_fallback","empty",
              "initial_miss_22000018","prior_11100009","low_confidence_16400002"}
    offsets=[[-.08,0.,.15],[-.06,.06,.12],[-.08,-.03,.15],[-.08,.03,.15],[-.06,.03,.12],[-.06,.09,.12]]
    cases=[{**case,"offset_index":i} for case in source["cases"] if case["name"] in selected for i in range(len(offsets))]
    assert len(cases)==42
    plan={"posthoc_development_only":True,"hardware_executable":False,"full_task_test":False,
        "automatic_trigger_implemented":False,"feedback_config":source["feedback_config"],"offsets_m":offsets,"cases":cases,
        "model_sha256":{name:digest(root/name) for name in ("perception.pt","residual.pt")},
        "known_reference_evidence":[{"path":str(reference/name),"sha256":digest(reference/name)} for name in ("config.json","summary.json","audit.json")],
        "selection":"All seven existing non-injected known cases, including empty; two old offsets plus fixed lateral +/-30mm neighbors. All cases retained. No current formal seeds or labels select actions.",
        "important":"Every pose resets independently and starts from timed HOME; later old-view trajectories differ from the sequential controller. Forced probing continues even when HOME is accepted. No grasp/place success claim."}
    write(root/"config.json",plan)
    summary={"complete":False,"posthoc_development_only":True,"config_sha256":digest(root/"config.json"),"cases":[]}
    model,policy=perception_model(root/"perception.pt"),load(root/"residual.pt")
    sim=Simulation(ROOT/"assets/task_scene.xml")
    try:
        for i,case in enumerate(cases):
            result=run_case(sim,model,policy,plan["feedback_config"],case,offsets[case["offset_index"]],root/f"case-{i:03d}",
                            video=case["seed"]==16400002 and case["offset_index"]==0)
            summary["cases"].append(result);write(root/"summary.json",summary)
            pair=result["view_pair"]
            print(json.dumps({"seed":case["seed"],"offset_index":case["offset_index"],
                "home_accepted":bool(result["home_pair"] and result["home_pair"]["quality"]["accepted"]),
                "view_accepted":bool(pair and pair["quality"]["accepted"]),
                "reason":None if pair is None else pair["quality"]["reason"],
                "raw_error_mm":result["independent_score_only"]["raw_error_mm"]}),flush=True)
        summary["complete"]=True;write(root/"summary.json",summary)
    finally:
        sim.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--name")
    group.add_argument("--audit-run",type=Path)
    args=parser.parse_args()
    audit(args.audit_run) if args.audit_run else main(args)
