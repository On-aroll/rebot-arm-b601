"""Train and select a small RGB-D detector on disjoint synthetic scenes."""
import argparse
import copy
import json
from pathlib import Path
import shutil

import numpy as np
import torch
from torch import nn

from policy import valid_prediction
from sim import ROOT, Simulation
from task_runner import create_run, digest, write, perception_datasets
from vision import TargetNet, collect, estimate, inputs


def evaluate(model, data):
    metadata = json.loads(str(data["metadata_json"]))
    rgb, depth = data["rgb"], data["depth_m"]
    rows = []
    for i, meta in enumerate(metadata):
        prediction = estimate(model, {"rgb": rgb[i], "depth": depth[i],
                                      "camera": meta["camera"]})
        accepted = valid_prediction(prediction)
        row = {"seed": meta["seed"], "expected_target": not meta["absent"], "accepted": accepted}
        if accepted and not meta["absent"]:
            truth = np.asarray(meta["initial_xyz"]); truth[2] += meta["half_size_m"]
            row["error_mm"] = float(np.linalg.norm(np.asarray(prediction["top_xyz_m"])-truth)*1000)
        rows.append(row)
    errors = [r["error_mm"] for r in rows if "error_mm" in r]
    return {"samples": len(rows), "positives": sum(r["expected_target"] for r in rows),
        "misses": sum(r["expected_target"] and not r["accepted"] for r in rows),
        "false_positives": sum(not r["expected_target"] and r["accepted"] for r in rows),
        "mean_mm": float(np.mean(errors)) if errors else 1e6,
        "p95_mm": float(np.quantile(errors, .95)) if errors else 1e6, "rows": rows}


def main(args):
    if not 64 <= args.samples <= 4096 or not 1 <= args.epochs <= 100:
        raise ValueError("Bound training size and duration")
    root = create_run(args.name)
    config = {k:str(v) if isinstance(v, Path) else v for k,v in vars(args).items()}
    write(root/"config.json", config)
    if args.reuse_data:
        for name in ("training.npz", "validation.npz"):
            shutil.copy2(args.reuse_data/name, root/name)
        write(root/"reused_datasets.json", {name:{"source":str(args.reuse_data/name), "sha256":digest(root/name)}
                                           for name in ("training.npz", "validation.npz")})
    else:
        sim = Simulation(ROOT/"assets/task_scene.xml")
        try:
            collect(sim, np.arange(args.train_seed, args.train_seed+args.samples), root/"training.npz")
            collect(sim, np.arange(args.validation_seed, args.validation_seed+256), root/"validation.npz")
        finally:
            sim.close()
    with np.load(root/"training.npz", allow_pickle=False) as d:
        x = torch.from_numpy(inputs(d["rgb"], d["depth_m"]))
        y = torch.from_numpy(d["mask"][:, None].astype(np.float32))
    validation = np.load(root/"validation.npz", allow_pickle=False)
    torch.manual_seed(args.training_seed)
    model = TargetNet()
    inherited = []
    if args.initial:
        model.load_state_dict(torch.load(args.initial, map_location="cpu", weights_only=True)["state_dict"])
        inherited = [d["path"] for d in perception_datasets(args.initial)]
        write(root/"initial_checkpoint.json", {"path":str(args.initial.resolve()), "sha256":digest(args.initial)})
    optimizer = torch.optim.Adam(model.parameters(), lr=.001)
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=torch.tensor(35.))
    best, state, selected, history = None, None, None, []
    if args.initial:
        model.eval()
        metrics = evaluate(model, validation)
        best = (metrics["false_positives"], metrics["misses"], metrics["mean_mm"])
        state, selected = copy.deepcopy(model.state_dict()), 0
    for epoch in range(1, args.epochs+1):
        model.train()
        losses = []
        for ids in torch.randperm(len(x)).split(16):
            sample = x[ids]+torch.randn_like(x[ids])*.007
            loss = loss_fn(model(sample), y[ids])
            optimizer.zero_grad(); loss.backward(); optimizer.step()
            losses.append(float(loss.detach()))
        if epoch % 5 == 0 or epoch == args.epochs:
            model.eval()
            metrics = evaluate(model, validation)
            rank = (metrics["false_positives"], metrics["misses"], metrics["mean_mm"])
            if best is None or rank < best:
                best, state, selected = rank, copy.deepcopy(model.state_dict()), epoch
            row = {"epoch": epoch, "loss": float(np.mean(losses)),
                   "validation": {k:v for k,v in metrics.items() if k != "rows"}}
            history.append(row)
            write(root/"history.json", history)
            print(json.dumps(row), flush=True)
    model.load_state_dict(state); model.eval()
    checkpoint = {"schema": "b601-rgbd-segmentation-v1", "state_dict": state,
        "selected_epoch": selected, "training_seed": args.training_seed, "training_samples": len(x),
        "dataset_paths": list(dict.fromkeys(inherited+[str(root/"training.npz"), str(root/"validation.npz")])),
        "input": "RGB[0,1],depth_m[0,2];NCHW", "hardware_executable": False}
    torch.save(checkpoint, root/"perception.pt")
    write(root/"summary.json", {"schema": "b601-vision-selection-v1", "selected_epoch": selected,
        "validation": evaluate(model, validation), "checkpoint_sha256": digest(root/"perception.pt"),
        "datasets_sha256": {p:digest(root/p) for p in ("training.npz", "validation.npz")},
        "config": config, "hardware_executable": False})
    validation.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name", required=True)
    p.add_argument("--samples", type=int, default=1024)
    p.add_argument("--epochs", type=int, default=40)
    p.add_argument("--train-seed", type=int, default=5100000)
    p.add_argument("--validation-seed", type=int, default=5200000)
    p.add_argument("--training-seed", type=int, default=20261021)
    p.add_argument("--initial", type=Path)
    p.add_argument("--reuse-data", type=Path)
    main(p.parse_args())
