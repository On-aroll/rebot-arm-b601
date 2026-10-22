"""Train a small RGB-D segmentation network using synthetic camera labels."""
import json
from pathlib import Path

import numpy as np
from scipy import ndimage
import torch
from torch import nn

from sim import deproject

torch.set_num_threads(4)

DEFAULT_INFERENCE_CONFIG = {"probability_threshold": .7, "min_pixels": 25}


def inference_configuration(config=None):
    result = dict(DEFAULT_INFERENCE_CONFIG if config is None else config)
    if (set(result) != set(DEFAULT_INFERENCE_CONFIG)
            or not np.isfinite(list(result.values())).all()
            or not .5 <= result["probability_threshold"] <= .9
            or not isinstance(result["min_pixels"],int)
            or not 8 <= result["min_pixels"] <= 80):
        raise ValueError("invalid_visual_inference_configuration")
    return result


class TargetNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.layers = nn.Sequential(nn.Conv2d(4, 12, 3, padding=1), nn.ReLU(),
                                    nn.Conv2d(12, 12, 3, padding=1), nn.ReLU(),
                                    nn.Conv2d(12, 1, 1))

    def forward(self, x):
        return self.layers(x)


def inputs(rgb, depth):
    rgb = np.asarray(rgb, dtype=np.float32) / 255
    depth = np.clip(np.asarray(depth, dtype=np.float32), 0, 2)
    return np.concatenate([rgb, depth[..., None]], axis=-1).transpose(0, 3, 1, 2)


def collect(sim, seeds, path):
    path = Path(path)
    if path.exists():
        raise FileExistsError(path)
    rgb, depth, masks, metadata = [], [], [], []
    for index, seed in enumerate(seeds):
        meta = sim.reset(int(seed), absent=(index % 7 == 0))
        frame = sim.observe(labels=True)
        rgb.append(frame["rgb"]); depth.append(frame["depth"]); masks.append(frame["mask"])
        metadata.append({**meta, "camera": frame["camera"]})
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, rgb=rgb, depth_m=np.asarray(depth, np.float32), mask=masks,
                        metadata_json=json.dumps(metadata), seeds=np.asarray(seeds, np.int64))
    return {"samples": len(seeds), "path": str(path), "negative_samples": sum(x["absent"] for x in metadata)}


def train(paths, checkpoint, *, epochs, seed, initial=None):
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    model = TargetNet()
    if initial:
        model.load_state_dict(torch.load(initial, map_location="cpu", weights_only=True)["state_dict"])
    batches = [np.load(p, allow_pickle=False) for p in paths]
    x = torch.from_numpy(inputs(np.concatenate([b["rgb"] for b in batches]),
                                np.concatenate([b["depth_m"] for b in batches])))
    y = torch.from_numpy(np.concatenate([b["mask"] for b in batches])[:, None].astype(np.float32))
    for b in batches:
        b.close()
    optimizer = torch.optim.Adam(model.parameters(), lr=.003)
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=torch.tensor(35.))
    history = []
    for epoch in range(epochs):
        losses = []
        model.train()
        for ids in np.array_split(rng.permutation(len(x)), max(1, int(np.ceil(len(x)/16)))):
            # Sensor noise is applied to RGB-D inputs, never to supervision labels.
            sample = x[ids] + torch.randn_like(x[ids]) * .007
            loss = loss_fn(model(sample), y[ids])
            optimizer.zero_grad(); loss.backward(); optimizer.step()
            losses.append(float(loss.detach()))
        history.append({"epoch": epoch+1, "loss": float(np.mean(losses))})
        if (epoch+1) % 5 == 0:
            print(json.dumps({"epoch": epoch+1, "loss": history[-1]["loss"]}), flush=True)
    checkpoint = Path(checkpoint)
    torch.save({"schema": "b601-rgbd-segmentation-v1", "state_dict": model.state_dict(),
                "training_seed": seed, "training_samples": len(x),
                "input": "RGB[0,1],depth_m[0,2];NCHW", "hardware_executable": False}, checkpoint)
    checkpoint.with_suffix(".history.json").write_text(json.dumps(history, indent=2)+"\n")
    return model.eval(), history


def estimate(model, frame):
    """Inference consumes RGB, depth, and calibration only; no scene truth."""
    with torch.no_grad():
        x = torch.from_numpy(inputs(frame["rgb"][None], frame["depth"][None]))
        probability = torch.sigmoid(model(x))[0, 0].numpy()
    return decode_probability(probability,frame,getattr(model,"inference_config",None))


def decode_probability(probability,frame,config=None):
    """Pure image/calibration postprocessing; no labels or scene truth input."""
    config=inference_configuration(config)
    components, count = ndimage.label(probability >= config["probability_threshold"])
    candidates = []
    for label in range(1, count+1):
        mask = components == label
        n = int(mask.sum())
        if not config["min_pixels"] <= n <= 240:
            continue
        v, u = np.where(mask)
        # Reject fragments and implausibly elongated objects for this cube task.
        extent = (u.max()-u.min()+1, v.max()-v.min()+1)
        if max(extent)/min(extent) > 2 or n / (extent[0]*extent[1]) < .4:
            continue
        core = ndimage.binary_erosion(mask)
        if core.sum() < 5:
            continue
        depths = frame["depth"][core]
        if not np.isfinite(depths).all() or np.quantile(depths, .9)-np.quantile(depths, .1) > .012:
            continue
        xyz = deproject(float(u.mean()), float(v.mean()), float(np.median(depths)), frame["camera"])
        candidates.append({"top_xyz_m": xyz.tolist(), "pixel_uv": [float(u.mean()), float(v.mean())],
                           "depth_m": float(np.median(depths)), "pixels": n,
                           "confidence": float(probability[mask].mean()), "source": "trained_rgbd_network"})
    return max(candidates, key=lambda c:c["confidence"]) if candidates else None


def validate(model, dataset):
    data = np.load(dataset, allow_pickle=False)
    metadata = json.loads(str(data["metadata_json"]))
    rows = []
    for index, meta in enumerate(metadata):
        predicted = estimate(model, {"rgb": data["rgb"][index], "depth": data["depth_m"][index], "camera": meta["camera"]})
        expected = not meta["absent"]
        row = {"seed": meta["seed"], "expected_target": expected, "detected": predicted is not None}
        if expected and predicted:
            truth = np.asarray(meta["initial_xyz"])
            row["xy_error_m"] = float(np.linalg.norm(np.asarray(predicted["top_xyz_m"])[:2]-truth[:2]))
        rows.append(row)
    errors = [r["xy_error_m"] for r in rows if "xy_error_m" in r]
    result = {"samples": len(rows), "target_samples": sum(r["expected_target"] for r in rows),
              "misses": sum(r["expected_target"] and not r["detected"] for r in rows),
              "false_positives": sum(not r["expected_target"] and r["detected"] for r in rows),
              "xy_median_mm": float(np.median(errors)*1000) if errors else None,
              "xy_p95_mm": float(np.quantile(errors,.95)*1000) if errors else None,
              "rows": rows}
    data.close()
    return result
