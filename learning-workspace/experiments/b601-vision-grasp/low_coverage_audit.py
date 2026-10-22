"""Audit quality-rejected image physical development runs, not adoption."""
import argparse
import inspect
import json
from pathlib import Path
import shutil

import numpy as np

from feedback_audit import audit_episode, same
from policy import load
from sim import ROOT, Simulation
from task import run_task
from task_runner import digest, perception_model, summary, write


def audit(run):
    root=ROOT/"outputs"/run
    config=json.loads((root/"config.json").read_text())
    result=json.loads((root/"summary.json").read_text())
    assert result["complete"]
    for name,sha in json.loads((root/"provenance.json").read_text())["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==sha
    for key in ("source_image_audit","validation_images"):
        assert digest(config[key])==config[key+"_sha256"]
    for kind,filename in (("perception","perception.pt"),("policy","residual.pt")):
        assert digest(root/"frozen"/filename)==config[kind+"_sha256"]
    source_rows=json.loads(Path(config["source_image_audit"]).read_text())["validation_rows"]
    with np.load(config["validation_images"],allow_pickle=False) as data:
        metadata=json.loads(str(data["metadata_json"]))
    expected_indices=[i for i,r in enumerate(source_rows) if r["expected_target"] and r["accepted_detection"]
                      and not r["gate"]["accepted"] and ((r["view"]=="legacy_HOME")==(config["source_domain"]=="legacy"))]
    assert expected_indices==config["image_indices"]==[e["source_image_index"] for e in result["episodes"]]
    assert config["actual_gl_context_module"]==("mujoco.egl" if config["source_domain"]=="legacy" else "mujoco.glfw")
    assert len(result["episodes"])==len(expected_indices)
    assert all(p not in inspect.getsource(run_task) for p in ("sim.obj","sim.og","sim.half_size",".xpos",".xmat","sim.contacts("))
    model,policy=perception_model(root/"frozen/perception.pt"),load(root/"frozen/residual.pt")
    sim=Simulation(ROOT/"assets/task_scene.xml")
    counts=[]
    try:
        for e in result["episodes"]:
            i=e["source_image_index"];r=source_rows[i];m=metadata[i]
            assert e["reset"]["seed"]==r["seed"]==m["seed"]
            assert e["reset"]["object_qpos_unchanged_by_robot_initialization"]
            expected_q=m.get("arm_initialization_q_rad",[0.,.55,.8,0.,.1,0.])
            same(e["reset"]["robot_initialization_only_q_rad"],expected_q)
            same(e["attempts"][0]["raw_prediction"],r["prediction"])
            assert e["max_retries"]==e["retries"]==0 and e["observation_mode"]=="none"
            directory=root/e["artifact_directory"]
            c,_=audit_episode(directory,e,sim,model,policy,e["feedback_config"])
            with np.load(directory/"decision_observations.npz",allow_pickle=False) as frames:
                first=json.loads(str(frames["metadata_json"]))[0]
                assert first["t_sim_s"]==0.
                same(first["q_feedback_rad"],expected_q)
            counts.append(c)
        same(result["summary"],summary(result["episodes"]))
        manifest=json.loads((ROOT/"assets/manifest.json").read_text())
        assert all(digest(ROOT/"assets"/p)==sha for p,sha in manifest["files_sha256"].items())
        shutil.copy2(ROOT/"low_coverage_audit.py",root/"audit_source.py")
        report={"passed":True,"episodes":len(counts),"scene_seeds":len(set(config["seeds"])),
                "source_domain":config["source_domain"],"successes":result["summary"]["successes"],
                "sums":{k:sum(c[k] for c in counts) for k in counts[0] if k!="maximum_command_speed_rad_s"},
                "final_physics_score_samples_reconstructed":sum(e["score_trace_samples"] for e in result["episodes"]),
                "audit_source_sha256":digest(root/"audit_source.py"),
                "scope":"Reused image validation cases, selected for gate rejection; physical development evidence only, not held-out task success estimates"}
        write(root/"audit.json",report)
        print(json.dumps(report),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",required=True)
    audit(parser.parse_args().run)
