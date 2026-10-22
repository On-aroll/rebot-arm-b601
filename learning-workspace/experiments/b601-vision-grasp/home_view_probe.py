"""Development-only physical observations at fixed, object-independent arm poses.

No object truth chooses the robot pose, image quality, or selected view. Truth
is read only after each exact decision frame for independent localization error.
Previously rejected test scenes are explicitly post-hoc development cases.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image

from feedback import DEFAULT_CONFIG, pair_quality
from policy import load, valid_prediction
from sim import HOME, ROOT, Simulation
from task import Recorder, correct_prediction
from task_runner import create_run, digest, perception_datasets, perception_model, write
from vision import estimate


POSES = {"HOME":HOME.copy(),"base_plus_1p2":HOME+np.array([1.2,0,0,0,0,0]),
         "base_minus_1p2":HOME+np.array([-1.2,0,0,0,0,0])}


def truth_top_for_score(sim):
    # Scoring output only. Never passed to a pose or prediction function.
    center = sim.data.xpos[sim.obj].copy()
    center[2] += sim.half_size
    return center


def main(args):
    perception_path,policy_path = args.perception.resolve(),args.policy.resolve()
    assert json.loads((policy_path.parent/"summary.json").read_text())["perception_sha256"]==digest(perception_path)
    cases = [(s,"previous_test_now_posthoc_development") for s in (9600008,9600014)]
    cases += [(s,"fresh_development") for s in range(args.seed_start,args.seed_start+args.scenes)]
    assert len({s for s,_ in cases})==len(cases)
    # Reserve every scene in the concurrently running official evaluation.
    reserved = set(range(10000000,10000030))|set(range(10100000,10100030))|set(range(10200000,10200008))|{10300000}
    assert not reserved.intersection(s for s,_ in cases)
    root = create_run(args.name,perception_path)
    for source in ("home_view_probe.py",):
        shutil.copy2(ROOT/source,root/"source"/source)
    provenance = json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"]["home_view_probe.py"]=digest(ROOT/"home_view_probe.py")
    write(root/"provenance.json",provenance)
    frozen = root/"frozen";frozen.mkdir()
    shutil.copy2(perception_path,frozen/"perception.pt");shutil.copy2(policy_path,frozen/"residual.pt")
    datasets = perception_datasets(perception_path)+[
        {"path":str(policy_path.parent/p),"sha256":digest(policy_path.parent/p)} for p in ("training.npz","validation.npz")]
    training_seeds = set()
    for item in datasets:
        with np.load(item["path"],allow_pickle=False) as data:
            training_seeds.update(data["seeds"].tolist())
    assert not training_seeds.intersection(s for s,_ in cases)
    config = {"schema":"b601-fixed-home-observation-development-v1","hardware_executable":False,
              "frozen_at_utc":datetime.now(timezone.utc).isoformat(),"seeds":[s for s,_ in cases],
              "scene_roles":{str(s):role for s,role in cases},"arm_poses_rad":{k:v.tolist() for k,v in POSES.items()},
              "feedback_config":DEFAULT_CONFIG,"perception_sha256":digest(frozen/"perception.pt"),
              "policy_sha256":digest(frozen/"residual.pt"),"datasets":datasets,
              "purpose":"Physical observation development; not full-task success or independent adoption evaluation"}
    write(root/"config.json",config)
    perception,policy = perception_model(frozen/"perception.pt"),load(frozen/"residual.pt")
    sim = Simulation(ROOT/"assets/task_scene.xml")
    rows=[]
    try:
        for seed,role in cases:
            for name,q in POSES.items():
                directory = root/f"seed-{seed}-{name}";directory.mkdir()
                reset = sim.reset(seed)
                recorder = Recorder(sim,directory,False)
                initial,initial_index = recorder.capture_decision("initial_HOME")
                initial_raw = estimate(perception,initial)
                initial_corrected = correct_prediction(initial_raw,policy)
                recorder.move(q,.046,1.,"fixed_observation_move")
                recorder.move(q,.046,.3,"fixed_observation_settle")
                predictions,cameras,scores,indices = [],[],[],[]
                for index in range(2):
                    if index:
                        recorder.move(q,.046,.2,"observation_pair_gap")
                    frame,decision_index = recorder.capture_decision(f"fixed_observation_{index}")
                    predicted = estimate(perception,frame)
                    corrected = correct_prediction(predicted,policy)
                    truth = truth_top_for_score(sim)
                    predictions.append(predicted);cameras.append(frame["camera"]);indices.append(decision_index)
                    scores.append({"t_sim_s":float(sim.data.time),"truth_top_xyz_m":truth.tolist(),
                                   "raw_error_mm":None if predicted is None else float(np.linalg.norm(np.asarray(predicted["top_xyz_m"])-truth)*1000),
                                   "corrected_error_mm":None if corrected is None else float(np.linalg.norm(np.asarray(corrected["top_xyz_m"])-truth)*1000)})
                    Image.fromarray(frame["rgb"]).save(directory/f"decision-{index}.png")
                # This probe always visits every fixed pose. The initial estimate
                # is only used to audit the existing update-distance condition.
                anchor = initial_corrected if valid_prediction(initial_corrected) else None
                gate = pair_quality(predictions,cameras,anchor,DEFAULT_CONFIG) if anchor else None
                recorder.save()
                row = {"seed":seed,"scene_role":role,"view":name,"q_target_rad":q.tolist(),
                       "reset":reset,"initial_raw_prediction":initial_raw,"initial_prediction":initial_corrected,
                       "initial_decision_index":initial_index,"decision_indices":indices,"raw_predictions":predictions,
                       "pair_gate":gate,"scoring_only":scores,"physics_tick_audit":recorder.physics_audit,
                       "sim_seconds":float(sim.data.time),"record_rows":len(recorder.rows),"rgbd_frames":len(recorder.frames),
                       "artifact_directory":directory.name,"full_task_test":False,"hardware_executable":False}
                write(directory/"result.json",row);rows.append(row)
                write(root/"summary.json",{**config,"complete":False,"observations":rows})
                print(json.dumps({"seed":seed,"view":name,"accepted":gate and gate["accepted"],
                                  "raw_error_mm":scores[-1]["raw_error_mm"],"forbidden_steps":recorder.physics_audit["forbidden_steps"]}),flush=True)
        groups = {}
        for name in POSES:
            items=[r for r in rows if r["view"]==name]
            errors=[r["scoring_only"][-1]["corrected_error_mm"] for r in items if r["scoring_only"][-1]["corrected_error_mm"] is not None]
            groups[name]={"scenes":len(items),"detected":len(errors),
                          "accepted_by_existing_pair_gate":sum(bool(r["pair_gate"] and r["pair_gate"]["accepted"]) for r in items),
                          "mean_corrected_error_mm":float(np.mean(errors)) if errors else None,
                          "max_corrected_error_mm":max(errors) if errors else None,
                          "forbidden_steps":sum(r["physics_tick_audit"]["forbidden_steps"] for r in items)}
        write(root/"summary.json",{**config,"complete":True,"by_view":groups,"observations":rows})
        print(json.dumps({"complete":True,"by_view":groups}),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    parser.add_argument("--scenes",type=int,default=6)
    parser.add_argument("--seed-start",type=int,default=7000000)
    parser.add_argument("--perception",type=Path,default=ROOT/"outputs/vision-multiview-01/perception.pt")
    parser.add_argument("--policy",type=Path,default=ROOT/"outputs/goal-policy-multiview-01/residual.pt")
    args=parser.parse_args()
    if not 3<=args.scenes<=30:
        raise ValueError("Use 3--30 fresh development scenes")
    main(args)
