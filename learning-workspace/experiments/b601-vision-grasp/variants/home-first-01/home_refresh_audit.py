"""Re-execute every stationary refresh probe; compare exact images and records."""
import argparse
import json
from pathlib import Path

import numpy as np

from feedback_audit import same
from home_refresh_probe import run_case, summarize
from policy import load
from sim import HOME, ROOT, Simulation
from task_runner import digest, perception_model, write


def audit(root):
    root = root.resolve()
    plan = json.loads((root/"config.json").read_text())
    summary = json.loads((root/"summary.json").read_text())
    assert summary["complete"] and not plan["full_task_test"]
    for key,value in plan.items():
        same(value, summary[key])
    provenance = json.loads((root/"provenance.json").read_text())
    assert provenance["runtime"]["gl_context_module"]=="mujoco.egl"
    for name,sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==sha
        assert digest(ROOT/name)==sha, f"Source changed: {name}"
    manifest = json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p)==sha for p,sha in manifest["files_sha256"].items())
    for item in plan["reference_evidence"]+plan["datasets"]:
        assert digest(item["path"])==item["sha256"]
    ledger = json.loads(Path(plan["reference_evidence"][0]["path"]).read_text())
    evaluation = json.loads(Path(plan["reference_evidence"][1]["path"]).read_text())
    excluded = set(ledger["seeds"])
    excluded.update(s for seeds in evaluation["test_seeds"].values() for s in seeds)
    for item in plan["datasets"]:
        with np.load(item["path"],allow_pickle=False) as data:
            excluded.update(data["seeds"].tolist())
    assert not excluded.intersection(plan["seeds"]+plan["empty_seeds"])
    for kind,name in (("perception","perception.pt"),("policy","residual.pt")):
        assert digest(root/"frozen"/name)==plan[kind+"_sha256"]
    perception,policy = perception_model(root/"frozen/perception.pt"),load(root/"frozen/residual.pt")
    sim = Simulation(ROOT/"assets/task_scene.xml")
    counts={"observations":0,"rows":0,"paired_frames":0,"decision_frames":0,"physics_steps":0,"forbidden_steps":0}
    expected=[(block,wait,seed) for block in ("static","disturbed","empty")
              for wait in plan["fixed_waits_s"]
              for seed in (plan["empty_seeds"] if block=="empty" else plan["seeds"])]
    assert [(r["block"],r["wait_s"],r["seed"]) for r in summary["observations"]]==expected
    try:
        for row in summary["observations"]:
            directory=root/row["artifact_directory"]
            saved=json.loads((directory/"result.json").read_text())
            same(saved,{k:v for k,v in row.items() if k not in ("block","artifact_directory")})
            rerun,recorder=run_case(sim,perception,policy,seed=row["seed"],absent=row["absent"],
                pulse=row["pulse"],wait=row["wait_s"],feedback_config=plan["feedback_config"])
            same(saved,rerun)
            logged=[json.loads(line) for line in (directory/"trajectory.jsonl").read_text().splitlines()]
            same(logged,recorder.rows)
            assert all(np.array_equal(r["q_command_rad"],HOME) for r in logged)
            for filename,frames,metadata in (("observations.npz",recorder.frames,None),
                     ("decision_observations.npz",recorder.decision_frames,recorder.decision_metadata)):
                with np.load(directory/filename,allow_pickle=False) as data:
                    np.testing.assert_array_equal(data["rgb"],np.stack([f["rgb"] for f in frames]))
                    np.testing.assert_array_equal(data["depth_m"],np.stack([f["depth"] for f in frames]))
                    same(json.loads(str(data["camera_json"])),[f["camera"] for f in frames])
                    if metadata:
                        same(json.loads(str(data["metadata_json"])),metadata)
            counts["observations"]+=1
            counts["rows"]+=len(logged);counts["paired_frames"]+=len(recorder.frames)
            counts["decision_frames"]+=len(recorder.decision_frames)
            counts["physics_steps"]+=recorder.physics_audit["total_steps"]
            counts["forbidden_steps"]+=recorder.physics_audit["forbidden_steps"]
        same(summary["groups"],summarize(summary["observations"]))
        report={"passed":True,**counts,"pixel_exact_reexecution":True,
            "unique_positive_scenes":len(plan["seeds"]),"unique_empty_scenes":len(plan["empty_seeds"]),
            "full_task_test":False,"scope":"Re-execution with frozen probe code, physics, images and inference; no independent physical engine or full-task claim."}
        write(root/"audit.json",report);print(json.dumps(report),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run",required=True,type=Path)
    audit(p.parse_args().run)
