"""Train bounded residuals from multiview synthetic image supervision."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from policy import features, train, valid_prediction
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import estimate


def collect_images(model, dataset, destination):
    x,y,rows = [],[],[]
    with np.load(dataset, allow_pickle=False) as data:
        rgb,depth = data["rgb"],data["depth_m"]
        seeds = data["seeds"].copy()
        metadata = json.loads(str(data["metadata_json"]))
        for i,meta in enumerate(metadata):
            predicted = estimate(model,{"rgb":rgb[i],"depth":depth[i],"camera":meta["camera"]})
            usable = not meta["absent"] and valid_prediction(predicted)
            if usable:
                xyz = np.asarray(predicted["top_xyz_m"])
                usable = bool(.265 <= xyz[0] <= .395 and abs(xyz[1]) <= .125)
            row = {"seed":meta["seed"], "view":meta.get("arm_initialization_view","legacy_HOME"),
                   "absent":meta["absent"], "detected":predicted is not None, "used_for_supervision":usable}
            if usable:
                # Oracle target is a training/validation label, never inference input.
                truth = np.asarray(meta["initial_xyz"]).copy(); truth[2] += meta["half_size_m"]
                delta = truth-np.asarray(predicted["top_xyz_m"])
                x.append(features(predicted)); y.append(delta)
                row["oracle_residual_m"] = delta.tolist()
            rows.append(row)
    np.savez_compressed(destination, x=np.asarray(x,np.float32),y=np.asarray(y,np.float32),
                        seeds=seeds,metadata_json=json.dumps(rows))
    return {"images":len(rows),"unique_scene_seeds":len(set(seeds.tolist())),
        "training_examples":len(x),"excluded_images":len(rows)-len(x),
        "path":str(destination),"source_images":str(dataset),"source_images_sha256":digest(dataset)}


def main(args):
    perception_path,dataset_root = args.perception.resolve(),args.dataset.resolve()
    with np.load(dataset_root/"training.npz",allow_pickle=False) as a, np.load(dataset_root/"validation.npz",allow_pickle=False) as b:
        assert not set(a["seeds"].tolist()).intersection(b["seeds"].tolist())
    root = create_run(args.name,perception_path)
    shutil.copy2(ROOT/"observation_policy.py",root/"source/observation_policy.py")
    provenance = json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"]["observation_policy.py"] = digest(ROOT/"observation_policy.py")
    write(root/"provenance.json",provenance)
    write(root/"config.json",{"perception":str(perception_path),"perception_sha256":digest(perception_path),
        "dataset":str(dataset_root),"epochs":args.epochs,"method":"supervised multiview visual residual",
        "hardware_executable":False,"uses_executed_demonstrations":False})
    model = perception_model(perception_path)
    training = collect_images(model,dataset_root/"training.npz",root/"training.npz")
    print(json.dumps({"collected_training":training}),flush=True)
    validation = collect_images(model,dataset_root/"validation.npz",root/"validation.npz")
    _,metrics = train(root/"training.npz",root/"validation.npz",root/"residual.pt",epochs=args.epochs)
    write(root/"summary.json",{"schema":"b601-multiview-residual-training-v1","hardware_executable":False,
        "training":training,"validation":validation,"metrics":metrics,
        "checkpoint_sha256":digest(root/"residual.pt"),"perception_sha256":digest(perception_path),
        "interpretation":"Image-supervised residual candidate; requires independent physical full-task evaluation."})
    print(json.dumps(metrics),flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    parser.add_argument("--perception",type=Path,required=True)
    parser.add_argument("--dataset",type=Path,required=True)
    parser.add_argument("--epochs",type=int,default=300)
    args=parser.parse_args()
    if not 1 <= args.epochs <= 600:
        raise ValueError("Use 1--600 training epochs")
    main(args)
