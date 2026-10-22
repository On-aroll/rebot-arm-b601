"""Post-hoc audit: reproduce every empty image and all decoder predictions."""
import argparse
import json
import os
from pathlib import Path
import shutil

import numpy as np
import torch

from feedback_audit import same
from observation_dataset import initial_arm_views
from policy import valid_prediction
from sim import ROOT, Simulation, mj
from task_runner import digest, perception_model, write
from vision import decode_probability, inputs


def audit(run):
    assert os.environ.get("MUJOCO_GL")=="egl" and mj.GLContext.__module__=="mujoco.egl"
    root=(ROOT/"outputs"/run).resolve()
    plan=json.loads((root/"config.json").read_text())
    saved=json.loads((root/"summary.json").read_text())
    assert saved["complete"] and plan["initialization_only"] and not plan["full_task_test"]
    for k,v in plan.items():same(saved[k],v)
    provenance=json.loads((root/"provenance.json").read_text())
    for name,sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==sha
        assert digest(ROOT/name)==sha
    manifest=json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p)==sha for p,sha in manifest["files_sha256"].items())
    assert digest(root/"prior_seed_ledger.json")==plan["prior_seed_ledger_sha256"]
    ledger=json.loads((root/"prior_seed_ledger.json").read_text())
    for item in ledger["sources"]:assert digest(item["path"])==item["sha256"]
    assert not set(plan["seeds"]).intersection(ledger["seeds"])
    assert digest(plan["checkpoint"])==plan["checkpoint_sha256"]
    assert digest(root/"empty_images.npz")==saved["image_dataset_sha256"]
    model=perception_model(plan["checkpoint"])
    sim=Simulation(ROOT/"assets/task_scene.xml")
    rows=[]
    try:
        with np.load(root/"empty_images.npz",allow_pickle=False) as data:
            rgb,depth=data["rgb"],data["depth_m"]
            metadata=json.loads(str(data["metadata_json"]))
            assert len(rgb)==len(depth)==len(metadata)==len(saved["rows"])==3*len(plan["seeds"])
            assert np.array_equal(data["seeds"],np.repeat(plan["seeds"],3))
            for i,meta in enumerate(metadata):
                assert meta["absent"] and meta["physics_steps"]==0
                sim.reset(meta["seed"],absent=True)
                qviews=initial_arm_views(sim,meta["seed"])
                name,q,anchor=qviews[i%3]
                same(meta["view"],name);same(meta["q_initialization_rad"],q.tolist())
                same(meta["independent_nominal_anchor_m"],anchor)
                before=sim.data.qpos[sim.oq:sim.oq+7].copy()
                sim.data.qpos[sim.qa]=q;sim.target_q=q.copy();sim.data.qvel[:]=0.
                mj.mj_forward(sim.model,sim.data)
                assert np.array_equal(before,sim.data.qpos[sim.oq:sim.oq+7]) and sim.data.time==0.
                frame=sim.observe(labels=False)
                np.testing.assert_array_equal(frame["rgb"],rgb[i])
                np.testing.assert_array_equal(frame["depth"],depth[i])
                same(frame["camera"],meta["camera"])
                with torch.no_grad():
                    probability=torch.sigmoid(model(torch.from_numpy(inputs(rgb[i:i+1],depth[i:i+1]))))[0,0].numpy()
                row={"seed":meta["seed"],"view":name,"image_index":i,
                    "predictions":{f"p{c['probability_threshold']}-n{c['min_pixels']}":decode_probability(probability,frame,c) for c in plan["configs"]}}
                same(row,saved["rows"][i]);rows.append(row)
        metrics={}
        for config in plan["configs"]:
            key=f"p{config['probability_threshold']}-n{config['min_pixels']}"
            found=[r for r in rows if valid_prediction(r["predictions"][key])]
            metrics[key]={"images":len(rows),"unique_empty_scenes":len(plan["seeds"]),
                "false_positive_images":len(found),"false_positive_scenes":len({r["seed"] for r in found})}
        same(metrics,saved["metrics"])
        shutil.copy2(Path(__file__),root/"negative_mask_audit_source.py")
        report={"passed":True,"images_reproduced_pixel_exactly":len(rows),
            "unique_empty_scenes":len(plan["seeds"]),"configs_recomputed":len(plan["configs"]),
            "physics_steps":0,"asset_files_checked":len(manifest["files_sha256"]),
            "audit_source_sha256":digest(root/"negative_mask_audit_source.py"),
            "scope":"Post-hoc deterministic image regeneration and inference audit; correlated views, no full-task or hardware claim."}
        write(root/"audit.json",report);print(json.dumps(report),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run",required=True,type=Path)
    audit(p.parse_args().run)
