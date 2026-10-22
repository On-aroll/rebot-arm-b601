"""Fresh empty-scene image development, all declared views and thresholds.

Robot poses are initialization-only; no physical task or motion demonstration.
Every empty image is retained, including any false detection.
"""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np
import torch

from feedback_runner import prior_seed_ledger
from mask_gate_probe import CONFIGS
from observation_dataset import initial_arm_views
from policy import valid_prediction
from sim import ROOT, Simulation, mj
from task_runner import create_run, digest, perception_model, write
from vision import decode_probability, inputs


def main(args):
    checkpoint=args.perception.resolve()
    root=create_run(args.name,checkpoint)
    provenance=json.loads((root/"provenance.json").read_text())
    for name in ("negative_mask_probe.py","mask_gate_probe.py","observation_dataset.py"):
        shutil.copy2(ROOT/name,root/"source"/name)
        provenance["source_sha256"][name]=digest(ROOT/name)
    write(root/"provenance.json",provenance)
    seeds=list(range(args.seed_start,args.seed_start+args.scenes))
    ledger=prior_seed_ledger(root)
    assert not set(seeds).intersection(ledger["seeds"])
    write(root/"prior_seed_ledger.json",ledger)
    plan={"development_only":True,"initialization_only":True,"motion_demonstration":False,
        "full_task_test":False,"hardware_executable":False,"seeds":seeds,"configs":CONFIGS,
        "checkpoint":str(checkpoint),"checkpoint_sha256":digest(checkpoint),
        "prior_seed_ledger_sha256":digest(root/"prior_seed_ledger.json")}
    write(root/"config.json",plan)
    model=perception_model(checkpoint)
    sim=Simulation(ROOT/"assets/task_scene.xml")
    rgb,depth,metadata,rows=[],[],[],[]
    try:
        for j,seed in enumerate(seeds):
            sim.reset(seed,absent=True)
            poses=initial_arm_views(sim,seed)
            for name,q,anchor in poses:
                meta=sim.reset(seed,absent=True)
                object_before=sim.data.qpos[sim.oq:sim.oq+7].copy()
                assert sim.data.time==0.
                sim.data.qpos[sim.qa]=q
                sim.target_q=q.copy()
                sim.data.qvel[:]=0.
                mj.mj_forward(sim.model,sim.data)
                assert np.array_equal(object_before,sim.data.qpos[sim.oq:sim.oq+7])
                frame=sim.observe(labels=False)
                rgb.append(frame["rgb"]);depth.append(frame["depth"])
                metadata.append({**meta,"camera":frame["camera"],"view":name,"q_initialization_rad":q.tolist(),
                    "independent_nominal_anchor_m":anchor,"physics_steps":0})
                with torch.no_grad():
                    probability=torch.sigmoid(model(torch.from_numpy(inputs(frame["rgb"][None],frame["depth"][None]))))[0,0].numpy()
                rows.append({"seed":seed,"view":name,"image_index":len(rgb)-1,
                    "predictions":{f"p{c['probability_threshold']}-n{c['min_pixels']}":decode_probability(probability,frame,c) for c in CONFIGS}})
            if (j+1)%64==0:
                print(json.dumps({"unique_empty_scenes":j+1,"images":len(rgb)}),flush=True)
        np.savez_compressed(root/"empty_images.npz",rgb=np.asarray(rgb),depth_m=np.asarray(depth,np.float32),
            seeds=np.asarray([m["seed"] for m in metadata],np.int64),metadata_json=json.dumps(metadata))
        metrics={}
        for c in CONFIGS:
            key=f"p{c['probability_threshold']}-n{c['min_pixels']}"
            detected=[r for r in rows if valid_prediction(r["predictions"][key])]
            metrics[key]={"images":len(rows),"unique_empty_scenes":len(seeds),
                "false_positive_images":len(detected),"false_positive_scenes":len({r["seed"] for r in detected})}
        write(root/"summary.json",{**plan,"complete":True,"image_dataset_sha256":digest(root/"empty_images.npz"),
            "metrics":metrics,"rows":rows,"interpretation":"Repeated initialized views share seeds; not independent full-task trials or real-world false-positive rate."})
        print(json.dumps(metrics),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name",required=True)
    p.add_argument("--seed-start",type=int,default=19000000)
    p.add_argument("--scenes",type=int,default=384)
    p.add_argument("--perception",type=Path,default=ROOT/"outputs/vision-multiview-egl-01/perception.pt")
    args=p.parse_args()
    if not 32<=args.scenes<=2048:
        raise ValueError("Use 32--2048 fresh empty scenes")
    main(args)
