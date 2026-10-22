"""Supervised visuomotor residual: visual features -> bounded XYZ correction.

Labels are oracle pre-grasp target corrections in synthetic training scenes.
Inference never receives object truth. This is not reinforcement learning.
"""
import copy
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

from vision import estimate

BOUND_M = .008


def valid_prediction(prediction):
    if prediction is None:
        return False
    top = np.asarray(prediction["top_xyz_m"])
    return bool(np.isfinite(top).all() and .245 <= top[0] <= .50
                and abs(top[1]) <= .25 and .125 <= top[2] <= .16)


def features(prediction):
    p = prediction
    xyz = np.asarray(p["top_xyz_m"], dtype=np.float32)
    return np.r_[(xyz - [.33, 0, .14]) / [.1, .15, .03],
                 (np.asarray(p["pixel_uv"]) - [80, 60]) / [80, 60],
                 p["pixels"] / 120, p["confidence"], p["depth_m"]].astype(np.float32)


class ResidualPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(8, 32), nn.Tanh(), nn.Linear(32, 32),
                                 nn.Tanh(), nn.Linear(32, 3), nn.Tanh())

    def forward(self, x):
        return self.net(x) * BOUND_M

    def correct(self, prediction):
        xyz = np.asarray(prediction["top_xyz_m"])
        if not (.265 <= xyz[0] <= .395 and abs(xyz[1]) <= .125):
            result = copy.deepcopy(prediction)
            result["residual_xyz_m"] = [0., 0., 0.]
            result["action_source"] = "outside_training_workspace_geometry"
            return result
        with torch.no_grad():
            delta = self(torch.from_numpy(features(prediction))[None])[0].numpy()
        result = copy.deepcopy(prediction)
        result["raw_top_xyz_m"] = prediction["top_xyz_m"]
        result["residual_xyz_m"] = delta.tolist()
        result["top_xyz_m"] = (np.asarray(prediction["top_xyz_m"]) + delta).tolist()
        result["action_source"] = "learned_bounded_visuomotor_residual"
        return result


def collect(sim, perception, seeds, path):
    x, y, rows = [], [], []
    for seed in seeds:
        sim.reset(int(seed))
        predicted = estimate(perception, sim.observe(labels=False))
        usable = valid_prediction(predicted)
        row = {"seed": int(seed), "detected": predicted is not None, "workspace_accepted": usable}
        if usable:
            # Training label only; never used by correct() or task execution.
            truth = sim.data.xpos[sim.obj].copy()
            truth[2] += sim.half_size
            delta = truth - predicted["top_xyz_m"]
            x.append(features(predicted)); y.append(delta)
            row["oracle_residual_m"] = delta.tolist()
        rows.append(row)
    np.savez_compressed(path, x=np.asarray(x, np.float32), y=np.asarray(y, np.float32),
                        seeds=np.asarray(seeds), metadata_json=json.dumps(rows))
    return {"scenes": len(rows), "detected_training_examples": len(x),
            "misses": len(rows)-len(x), "path": str(path)}


def errors(model, x, y):
    with torch.no_grad():
        residual = model(x).numpy() if model is not None else np.zeros_like(y)
    e = np.linalg.norm(residual-y, axis=1) * 1000
    return {"mean_mm": float(np.mean(e)), "median_mm": float(np.median(e)),
            "p95_mm": float(np.quantile(e, .95)), "samples": len(e)}


def train(train_path, validation_path, checkpoint, epochs=300):
    torch.manual_seed(20261020)
    with np.load(train_path, allow_pickle=False) as d:
        x, y = torch.from_numpy(d["x"]), torch.from_numpy(d["y"])
    with np.load(validation_path, allow_pickle=False) as d:
        vx, vy = torch.from_numpy(d["x"]), d["y"].copy()
    model = ResidualPolicy()
    optimizer = torch.optim.Adam(model.parameters(), lr=.002, weight_decay=1e-5)
    baseline = errors(None, vx, vy)
    best, state, history = float("inf"), None, []
    for epoch in range(epochs):
        for ids in torch.randperm(len(x)).split(64):
            loss = ((model(x[ids])-y[ids]) * 1000).square().mean()
            optimizer.zero_grad(); loss.backward(); optimizer.step()
        validation = errors(model, vx, vy)
        history.append({"epoch": epoch+1, "training_loss_mm2": float(loss.detach()),
                        "validation": validation})
        if validation["mean_mm"] < best:
            best, state, selected = validation["mean_mm"], copy.deepcopy(model.state_dict()), epoch+1
    model.load_state_dict(state)
    model.eval()
    torch.save({"schema": "b601-bounded-visuomotor-residual-v1", "state_dict": state,
                "bound_m": BOUND_M, "training_method": "synthetic supervised oracle correction",
                "selected_epoch": selected, "hardware_executable": False}, checkpoint)
    result = {"parameters": sum(p.numel() for p in model.parameters()), "selected_epoch": selected,
              "validation_baseline": baseline, "validation_policy": errors(model, vx, vy),
              "epochs": epochs, "training_examples": len(x)}
    Path(checkpoint).with_suffix(".training.json").write_text(json.dumps({**result, "history": history}, indent=2))
    return model, result


def load(path):
    checkpoint = torch.load(path, map_location="cpu", weights_only=True)
    if checkpoint["schema"] != "b601-bounded-visuomotor-residual-v1" or checkpoint["bound_m"] != BOUND_M:
        raise ValueError("Incompatible residual checkpoint")
    model = ResidualPolicy()
    model.load_state_dict(checkpoint["state_dict"])
    return model.eval()
