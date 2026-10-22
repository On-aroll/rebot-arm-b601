"""Audit initialized multiview training images and evaluate a frozen detector."""
import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np

from feedback import DEFAULT_CONFIG, quality
from policy import valid_prediction
from sim import ROOT
from task_runner import digest, perception_model, write
from vision import estimate


def audit(run, checkpoint, output_name):
    if Path(output_name).name != output_name or not output_name.endswith(".json"):
        raise ValueError("Use one JSON output filename")
    root = ROOT/"outputs"/run
    if (root/output_name).exists():
        raise FileExistsError(root/output_name)
    summary = json.loads((root/"summary.json").read_text())
    seed_sets, metrics = {}, {}
    test = set(range(9600000,9600030))|set(range(9900000,9900030))|set(range(9700000,9700008))|{9800000}
    for split in ("training","validation"):
        expected = summary[split]
        assert digest(root/f"{split}.npz") == expected["output_sha256"]
        with np.load(root/f"{split}.npz", allow_pickle=False) as data, np.load(expected["base_dataset"], allow_pickle=False) as base:
            rgb, depth, mask = data["rgb"], data["depth_m"], data["mask"]
            meta = json.loads(str(data["metadata_json"]))
            n = len(base["rgb"])
            for name in ("rgb", "depth_m", "mask", "seeds"):
                assert np.array_equal(data[name][:n], base[name])
            assert len(meta) == len(rgb) == len(depth) == len(mask) == expected["images"]
            assert rgb.shape[1:] == (120,160,3) and depth.shape[1:] == mask.shape[1:] == (120,160)
            assert np.isfinite(depth).all()
            negatives = [i for i,m in enumerate(meta) if m["absent"]]
            assert not mask[negatives].any()
            seed_sets[split] = set(data["seeds"].tolist())
            assert not seed_sets[split].intersection(test)
            volume_pixels = 0
            for i,m in enumerate(meta[n:], start=n):
                assert m["physics_steps"] == 0 and not m["motion_demonstration"]
                assert m["camera"]["simulation_time_s"] == 0.
                v,u = np.where(mask[i])
                if not len(u):
                    continue
                k,t = np.asarray(m["camera"]["K"]),np.asarray(m["camera"]["T_base_camera_optical"])
                points = np.linalg.solve(k,np.stack([u,v,np.ones(len(u))]))*depth[i,v,u]
                world = (t[:3,:3]@points).T+t[:3,3]
                angle = m["yaw_rad"]
                rotation = np.array([[np.cos(angle),-np.sin(angle),0],[np.sin(angle),np.cos(angle),0],[0,0,1]])
                local = (world-m["initial_xyz"])@rotation
                assert (np.abs(local)<=m["half_size_m"]+.0020001).all()
                volume_pixels += len(u)
            metrics[split] = {"images":len(rgb), "unique_seeds":len(seed_sets[split]),
                "new_images":len(rgb)-n, "empty_images":len(negatives), "new_label_pixels_volume_checked":volume_pixels,
                "positive_images_with_zero_visible_label":sum(not m["absent"] and not mask[i].any() for i,m in enumerate(meta))}
    assert not seed_sets["training"].intersection(seed_sets["validation"])
    model = perception_model(checkpoint)
    groups = defaultdict(list)
    rows = []
    with np.load(root/"validation.npz", allow_pickle=False) as data:
        rgb,depth,mask = data["rgb"],data["depth_m"],data["mask"]
        metadata = json.loads(str(data["metadata_json"]))
        for i,m in enumerate(metadata):
            prediction = estimate(model,{"rgb":rgb[i],"depth":depth[i],"camera":m["camera"]})
            usable = valid_prediction(prediction)
            gate = quality(prediction,m["camera"],DEFAULT_CONFIG)
            row = {"seed":m["seed"], "view":m.get("arm_initialization_view","legacy_HOME"),
                "expected_target":not m["absent"], "label_pixels":int(mask[i].sum()),
                "accepted_detection":usable, "gate":gate, "prediction":prediction}
            if usable and not m["absent"]:
                truth = np.asarray(m["initial_xyz"]).copy(); truth[2] += m["half_size_m"]
                row["error_mm"] = float(np.linalg.norm(prediction["top_xyz_m"]-truth)*1000)
            groups[row["view"]].append(row); rows.append(row)
    grouped = {}
    for name,items in groups.items():
        errors = [r["error_mm"] for r in items if "error_mm" in r]
        gated = [r["error_mm"] for r in items if "error_mm" in r and r["gate"]["accepted"]]
        grouped[name] = {"images":len(items), "positives":sum(r["expected_target"] for r in items),
            "misses":sum(r["expected_target"] and not r["accepted_detection"] for r in items),
            "false_positives":sum(not r["expected_target"] and r["accepted_detection"] for r in items),
            "quality_accepted_positives":len(gated), "quality_accepted_errors_above_5mm":sum(e>5 for e in gated),
            "detected_mean_error_mm":float(np.mean(errors)) if errors else None,
            "quality_accepted_mean_error_mm":float(np.mean(gated)) if gated else None}
    report = {"passed":True, "data":metrics, "no_train_validation_overlap":True,
        "no_frozen_task_test_overlap":True, "checkpoint":str(checkpoint), "checkpoint_sha256":digest(checkpoint),
        "validation_by_view":grouped, "validation_rows":rows,
        "scope":"Dataset integrity and image-only development evaluation; initialized images are not executed trajectories or full-task tests."}
    write(root/output_name,report)
    print(json.dumps({k:v for k,v in report.items() if k!="validation_rows"}),flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",required=True)
    parser.add_argument("--checkpoint",default="outputs/vision-iteration-03/perception.pt")
    parser.add_argument("--output-name",default="data_and_baseline_audit.json")
    args = parser.parse_args()
    audit(args.run,ROOT/args.checkpoint,args.output_name)
