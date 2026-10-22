"""Posthoc physical visibility probe with object-independent nominal poses."""
import argparse
import inspect
import json
from pathlib import Path
import shutil
import tempfile

import numpy as np
from PIL import Image

from episode import DOWN
from feedback import configuration, quality
from feedback_audit import same
from feedback_runner import prior_seed_ledger
from policy import valid_prediction
from sim import HOME, ROOT, Simulation
from task import Recorder, grasp_geometry
from task_runner import create_run, digest, perception_model, write
from vision import estimate


class ProbeRecorder(Recorder):
    def __init__(self, *args):
        super().__init__(*args)
        self.finger_contact_steps = 0

    def tick(self):
        super().tick()
        contact = self.sim.contacts()
        self.finger_contact_steps += bool(contact["left"] or contact["right"])


def score_only(sim, prediction, reset):
    center = sim.data.xpos[sim.obj].copy()
    top = center.copy()
    top[2] += np.sum(np.abs(sim.data.xmat[sim.obj].reshape(3, 3)[2]))*sim.half_size
    return {"truth_top_xyz_m":top.tolist(),
            "object_xy_displacement_m":float(np.linalg.norm(center[:2]-np.asarray(reset["initial_xyz"])[:2])),
            "raw_error_mm":float(np.linalg.norm(np.asarray(prediction["top_xyz_m"])-top)*1000) if prediction else None}


def run_case(sim, model, config, case, q_target, directory, video=False):
    directory.mkdir()
    reset = sim.reset(case["seed"])
    recorder = ProbeRecorder(sim, directory, video)
    before, _ = recorder.capture_decision("initial_HOME")
    initial = estimate(model, before)
    recorder.move(q_target, .046, 2., "independent_nominal_observation")
    recorder.move(q_target, .046, .3, "observation_hold")
    after, _ = recorder.capture_decision("nominal_observation")
    prediction = estimate(model, after)
    recorder.save()
    Image.fromarray(before["rgb"]).save(directory/"before.png")
    Image.fromarray(after["rgb"]).save(directory/"after.png")
    rows = recorder.rows
    speeds = [float(np.max(np.abs(np.asarray(b["q_command_rad"])-a["q_command_rad"])/(b["t_sim_s"]-a["t_sim_s"])))
              for a,b in zip(rows, rows[1:])]
    result = {"seed":case["seed"],"pose_index":case["pose_index"],"reset":reset,
        "initial_prediction":initial,"prediction":prediction,
        "initial_detected":valid_prediction(initial),"detected":valid_prediction(prediction),
        "initial_quality":quality(initial,before["camera"],config),
        "quality":quality(prediction,after["camera"],config),
        "q_target_rad":np.asarray(q_target).tolist(),"sim_seconds":float(sim.data.time),
        "record_rows":len(rows),"rgbd_frames":len(recorder.frames),
        "decision_frames":len(recorder.decision_frames),"physics_tick_audit":recorder.physics_audit,
        "finger_contact_steps_for_scoring_only":recorder.finger_contact_steps,
        "maximum_command_speed_rad_s":max(speeds,default=0.),
        "independent_score_only":score_only(sim,prediction,reset)}
    write(directory/"result.json",result)
    return result


def audit(root):
    plan=json.loads((root/"config.json").read_text())
    summary=json.loads((root/"summary.json").read_text())
    assert summary["complete"] and plan["posthoc_development_only"]
    assert digest(root/"config.json")==summary["config_sha256"]
    provenance=json.loads((root/"provenance.json").read_text())
    for relative,sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(relative).name)==digest(ROOT/relative)==sha
    assert digest(root/"perception.pt")==plan["perception_sha256"]
    assert digest(plan["validation_dataset"])==plan["validation_dataset_sha256"]
    manifest=json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p)==sha for p,sha in manifest["files_sha256"].items())
    source=inspect.getsource(run_case)
    assert all(s not in source for s in ("sim.obj","sim.og","sim.half_size",".xpos",".xmat","sim.contacts("))
    sim=Simulation(ROOT/"assets/task_scene.xml");model=perception_model(root/"perception.pt")
    try:
        with tempfile.TemporaryDirectory(prefix="b601-visibility-audit-") as temporary:
            for i,(case,expected) in enumerate(zip(plan["cases"],summary["cases"],strict=True)):
                directory=Path(temporary)/f"case-{i:03d}"
                actual=run_case(sim,model,plan["feedback_config"],case,plan["q_targets_rad"][case["pose_index"]],directory)
                same(actual,expected)
                original=root/f"case-{i:03d}"
                for name in ("trajectory.jsonl","final_score_trace.jsonl"):
                    assert (directory/name).read_bytes()==(original/name).read_bytes()
                for name in ("decision_observations.npz","observations.npz"):
                    with np.load(directory/name,allow_pickle=False) as a,np.load(original/name,allow_pickle=False) as b:
                        assert a.files==b.files
                        assert all(np.array_equal(a[k],b[k]) for k in a.files), (i,name)
        report={"passed":True,"posthoc_development_only":True,"physical_cases_reexecuted":len(summary["cases"]),
            "physics_steps_reexecuted":sum(e["physics_tick_audit"]["total_steps"] for e in summary["cases"]),
            "exact_RGBD_and_action_reexecution":True,"asset_files_checked":len(manifest["files_sha256"]),
            "scope":"Exact physical probe replay; not full-task success or an implemented automatic observation trigger."}
        write(root/"audit.json",report);print(json.dumps(report),flush=True)
    finally:
        sim.close()


def main(args):
    perception=args.perception.resolve()
    root=create_run(args.name,perception)
    provenance=json.loads((root/"provenance.json").read_text())
    shutil.copy2(Path(__file__),root/"source/initial_visibility_probe.py")
    provenance["source_sha256"]["initial_visibility_probe.py"]=digest(Path(__file__))
    write(root/"provenance.json",provenance)
    shutil.copy2(perception,root/"perception.pt")
    model=perception_model(root/"perception.pt")
    config=configuration(json.loads((ROOT/"configs/v2-feedback.json").read_text()))
    validation=(ROOT/"outputs/observation-images-egl-01/validation.npz").resolve()
    seeds={22000018,16400002,8100000};rows=[]
    with np.load(validation,allow_pickle=False) as data:
        metadata=json.loads(str(data["metadata_json"]))
        for i,meta in enumerate(metadata):
            view=meta.get("arm_initialization_view","legacy_HOME")
            p=estimate(model,{"rgb":data["rgb"][i],"depth":data["depth_m"][i],"camera":meta["camera"]})
            rows.append({"index":i,"seed":meta["seed"],"view":view,"expected_target":not meta["absent"],
                         "prediction":p,"detected":valid_prediction(p)})
            if not meta["absent"] and view in ("HOME","legacy_HOME") and not valid_prediction(p):
                seeds.add(meta["seed"])
    write(root/"validation_detection_rows.json",rows)
    ledger=prior_seed_ledger(root)
    assert seeds<=set(ledger["seeds"]),"This probe must only reuse known development scenes"
    write(root/"prior_seed_ledger.json",ledger)
    sim=Simulation(ROOT/"assets/task_scene.xml")
    try:
        anchor=[.33,0.,.14];_,grasp=grasp_geometry(anchor)
        offsets=config["view_offsets_m"]
        targets=[sim.ik(grasp+np.asarray(offset),DOWN,HOME).tolist() for offset in offsets]
        cases=[{"seed":seed,"pose_index":i} for seed in sorted(seeds) for i in range(len(targets))]
        plan={"posthoc_development_only":True,"full_task_test":False,"automatic_trigger_implemented":False,
            "hardware_executable":False,"selection":"All positive HOME validation misses plus22000018,16400002 and normal8100000; all retained",
            "validation_dataset":str(validation),"validation_dataset_sha256":digest(validation),
            "perception_sha256":digest(root/"perception.pt"),"feedback_config":config,
            "nominal_anchor_xyz_m":anchor,"view_offsets_m":offsets,"q_targets_rad":targets,"cases":cases}
        write(root/"config.json",plan)
        summary={"schema":"b601-initial-visibility-development-v1","complete":False,
                 "config_sha256":digest(root/"config.json"),"cases":[]}
        for i,case in enumerate(cases):
            result=run_case(sim,model,config,case,targets[case["pose_index"]],root/f"case-{i:03d}",
                            video=case["seed"] in (22000018,16400002) and case["pose_index"]==0)
            summary["cases"].append(result);write(root/"summary.json",summary)
            print(json.dumps({k:result[k] for k in ("seed","pose_index","initial_detected","detected","quality","finger_contact_steps_for_scoring_only")}),flush=True)
        summary["complete"]=True;write(root/"summary.json",summary)
    finally:
        sim.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--name")
    mode.add_argument("--audit-run",type=Path)
    parser.add_argument("--perception",type=Path,default=ROOT.parents[1]/"versions/V2/models/perception.pt")
    args=parser.parse_args()
    audit(args.audit_run.resolve()) if args.audit_run else main(args)
