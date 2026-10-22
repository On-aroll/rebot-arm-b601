"""Recompute independent fixed-observation probe evidence."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from feedback import pair_quality
from feedback_audit import same
from policy import load, valid_prediction
from sim import ROOT
from task import correct_prediction
from task_runner import digest, perception_model, write
from vision import estimate


def audit(run):
    root=ROOT/"outputs"/run
    config=json.loads((root/"config.json").read_text())
    result=json.loads((root/"summary.json").read_text())
    assert result["complete"]
    for name,sha in json.loads((root/"provenance.json").read_text())["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==sha
    for kind,filename in (("perception","perception.pt"),("policy","residual.pt")):
        assert digest(root/"frozen"/filename)==config[kind+"_sha256"]
    training=set()
    for item in config["datasets"]:
        assert digest(item["path"])==item["sha256"]
        with np.load(item["path"],allow_pickle=False) as data:
            training.update(data["seeds"].tolist())
    assert not training.intersection(config["seeds"])
    # This immutable ledger predates the probe; post-hoc regression cases are
    # deliberately allowed, but the six fresh development seeds are checked.
    prior_path=ROOT/"outputs/goal-multiview-v2-candidate-01/prior_seed_ledger.json"
    prior=set(json.loads(prior_path.read_text())["seeds"])
    fresh=[s for s in config["seeds"] if config["scene_roles"][str(s)]=="fresh_development"]
    assert not prior.intersection(fresh)
    model,policy=perception_model(root/"frozen/perception.pt"),load(root/"frozen/residual.pt")
    states,paired,decisions,physics=0,0,0,0
    predictions_by_seed={}
    for observation in result["observations"]:
        directory=root/observation["artifact_directory"]
        same(observation,json.loads((directory/"result.json").read_text()))
        rows=[json.loads(s) for s in (directory/"trajectory.jsonl").read_text().splitlines()]
        assert len(rows)==observation["record_rows"]
        with np.load(directory/"decision_observations.npz",allow_pickle=False) as frames:
            cameras=json.loads(str(frames["camera_json"]))
            meta=json.loads(str(frames["metadata_json"]))
            raw=[estimate(model,{"rgb":rgb,"depth":depth,"camera":camera}) for rgb,depth,camera in zip(frames["rgb"],frames["depth_m"],cameras,strict=True)]
            same(raw[0],observation["initial_raw_prediction"])
            initial=correct_prediction(raw[0],policy)
            same(initial,observation["initial_prediction"])
            indices=observation["decision_indices"]
            assert len(indices)==2 and len(raw)==3
            assert abs(meta[indices[1]]["t_sim_s"]-meta[indices[0]]["t_sim_s"]-.2)<.0021
            for i,score in zip(indices,observation["scoring_only"],strict=True):
                same(meta[i]["q_command_rad"],config["arm_poses_rad"][observation["view"]])
                same(score["t_sim_s"],meta[i]["t_sim_s"])
                truth=np.asarray(score["truth_top_xyz_m"])
                nearest=min(rows,key=lambda r:abs(r["t_sim_s"]-meta[i]["t_sim_s"]))
                logged=np.asarray(nearest["truth_for_scoring_only"]["object_xyz_m"])
                logged[2]+=observation["reset"]["half_size_m"]
                assert np.linalg.norm(truth-logged)<.0002
                corrected=correct_prediction(raw[i],policy)
                for name,predicted in (("raw",raw[i]),("corrected",corrected)):
                    expected=None if predicted is None else float(np.linalg.norm(np.asarray(predicted["top_xyz_m"])-truth)*1000)
                    same(score[name+"_error_mm"],expected)
            gate=pair_quality([raw[i] for i in indices],[cameras[i] for i in indices],initial,config["feedback_config"]) if valid_prediction(initial) else None
            same(gate,observation["pair_gate"])
            same([raw[i] for i in indices],observation["raw_predictions"])
            predictions_by_seed.setdefault(observation["seed"],{})[observation["view"]]=raw[indices[-1]]
            decisions+=len(raw)
        with np.load(directory/"observations.npz",allow_pickle=False) as images:
            cameras=json.loads(str(images["camera_json"]))
            assert len(cameras)==len(images["rgb"])==len(images["depth_m"])==observation["rgbd_frames"]
            linked=[r for r in rows if r["observation_index"] is not None]
            for i,row in enumerate(linked):
                assert row["observation_index"]==i
                assert abs(row["t_sim_s"]-cameras[i]["simulation_time_s"])<1e-9
            assert len(linked)==len(cameras)
            paired+=len(cameras)
        for a,b in zip(rows,rows[1:]):
            dt=b["t_sim_s"]-a["t_sim_s"]
            assert .047<=dt<=.053
            assert np.max(np.abs(np.asarray(b["q_command_rad"])-a["q_command_rad"]))/dt<=.401
        assert observation["physics_tick_audit"]["forbidden_steps"]==0
        physics+=observation["physics_tick_audit"]["total_steps"];states+=len(rows)
    comparisons={}
    for seed,views in predictions_by_seed.items():
        comparisons[str(seed)]={name:float(np.linalg.norm(np.asarray(p["top_xyz_m"])-views["HOME"]["top_xyz_m"])*1000) for name,p in views.items() if p is not None and views["HOME"] is not None}
    shutil.copy2(ROOT/"home_view_audit.py",root/"audit_source.py")
    report={"passed":True,"physical_observations":len(result["observations"]),"states":states,"paired_frames":paired,
            "decision_frames":decisions,"physics_steps":physics,"forbidden_steps":0,"fresh_development_seeds":fresh,
            "prior_seed_ledger_sha256":digest(prior_path),"pose_prediction_difference_from_HOME_mm":comparisons,
            "audit_source_sha256":digest(root/"audit_source.py"),
            "scope":"Development observations only, not full-task tests or evidence of improved grasping"}
    write(root/"audit.json",report)
    print(json.dumps(report),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",required=True)
    audit(parser.parse_args().run)
