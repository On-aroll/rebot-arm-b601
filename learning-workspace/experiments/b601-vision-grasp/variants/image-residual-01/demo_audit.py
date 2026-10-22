"""Audit one frozen simulation demo; no performance/adoption interpretation."""
import argparse
import json
from pathlib import Path

from feedback_audit import audit_episode, same
from policy import load
from sim import ROOT, Simulation
from task_runner import digest, perception_model, write


def audit(run):
    root=(ROOT/"outputs"/run).resolve()
    frozen=json.loads((root/"frozen_evaluation.json").read_text())
    e=json.loads((root/"summary.json").read_text())
    provenance=json.loads((root/"provenance.json").read_text())
    same(e,json.loads((root/"episode/result.json").read_text()))
    for name,sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==sha
        assert digest(ROOT/name)==sha
    manifest=json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p)==sha for p,sha in manifest["files_sha256"].items())
    assert digest(root/"frozen/perception.pt")==frozen["perception_sha256"]
    assert digest(root/"frozen/residual.pt")==frozen["policy_sha256"]
    assert e["observation_mode"]==frozen["observation_mode"]
    same(e["environment_for_scoring_only"]["config"],frozen["force_pulse"])
    sim=Simulation(ROOT/"assets/task_scene.xml")
    try:
        counts,_=audit_episode(root/"episode",e,sim,perception_model(root/"frozen/perception.pt"),
            load(root/"frozen/residual.pt"),frozen["feedback_config"])
        report={"passed":True,"success":e["success"],"scene_role":frozen["scene_role"],
            "hardware_executable":False,"counts":counts,"asset_files_checked":len(manifest["files_sha256"]),
            "scope":"One recorded demo decision/action/score reconstruction; not a fresh performance evaluation."}
        write(root/"audit.json",report);print(json.dumps(report),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run",required=True,type=Path)
    audit(p.parse_args().run)
