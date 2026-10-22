"""Fixed offline visual-risk experiment; full reconstruction and retraining audit."""
import argparse
from collections import Counter
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.ndimage import map_coordinates
from scipy.stats import rankdata
import torch
from torch import nn

from image_residual import image_patch
from policy import load
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import estimate
from visual_risk import INPUTS, VisualRisk, applicable, load_risk, risk_features

CANONICAL = (ROOT/"outputs").resolve().parent
SPLITS = ("training", "calibration", "validation")
PROTOCOL_SHA = "5811f5d40656587090019563944fbc8a4aebd511647917bf523c3654b41a9b85"
POSITION_SHA = "9307762e59d1a2c82bde7add620ac815a8071dffda087413798c4fb8d7fe4ed2"
PERCEPTION_SHA = "e911a373325072eb9525031fa57f4270a6f8b6f853a5c6d370eb5ffd5d17bd37"
REVIEW_SUMMARY_SHA = "48432d0e52d7d02aa7832bbd8865d86dcc30d34c6b36f58251955267b54daeea"
REVIEW_AUDIT_SHA = "b44959f590cc1698f436d33ef4a62636618a5cc90bb8a34c55e399ad5bd47ca3"
EXTRA = ("visual_risk.py", "visual_risk_development.py", "audit_frozen.py", "BRANCH_BASE.json")
TRAINING = {"seed": 2026102207, "epochs": 300, "batch_size": 64,
            "learning_rate": .001, "weight_decay": 1e-4, "threads": 1,
            "selection": "fixed_final_epoch", "input_dimensions": INPUTS, "parameters": 4545}


def verify_inputs(root, config):
    assert config["training"] == TRAINING and not config["hardware_executable"]
    assert digest(root/"protocol.md") == config["protocol_sha256"] == PROTOCOL_SHA
    assert config["frozen_sha256"]["image-residual.pt"] == POSITION_SHA
    assert config["frozen_sha256"]["perception.pt"] == PERCEPTION_SHA
    for name, sha in config["frozen_sha256"].items():
        assert digest(root/"frozen"/name) == sha, name
    for path, sha in config["evidence_sha256"].items():
        assert digest(path) == sha, path
    review = Path(config["review_run"])
    assert digest(review/"summary.json") == REVIEW_SUMMARY_SHA
    assert digest(review/"audit.json") == REVIEW_AUDIT_SHA
    assert json.loads((review/"audit.json").read_text())["passed"]


def independent_features(model, pred, frame, value):
    """Independent scalar/coverage formulas and SciPy patch interpolation."""
    xyz = np.asarray(pred["top_xyz_m"], dtype=np.float32)
    numeric = np.r_[(xyz-[.33, 0, .14])/[.1, .15, .03],
                    (np.asarray(pred["pixel_uv"])-[80, 60])/[80, 60],
                    pred["pixels"]/120, pred["confidence"], pred["depth_m"]].astype(np.float32)
    assert np.array_equal(numeric, value[:8])
    k = np.asarray(frame["camera"]["K"])
    t = np.asarray(frame["camera"]["T_base_camera_optical"])
    expected_pixels = k[0, 0]*k[1, 1]*(float(pred["top_xyz_m"][2])-.1)**2*abs(t[2, 2])/pred["depth_m"]**2
    assert np.float32(pred["pixels"]/expected_pixels) == value[-1]
    u, v = pred["pixel_uv"]
    dy, dx = np.meshgrid(np.arange(32)-15.5+v, np.arange(32)-15.5+u, indexing="ij")
    channels = [frame["rgb"][:, :, c].astype(np.float32)/255 for c in range(3)]
    channels.append(np.clip((frame["depth"]-pred["depth_m"])/.05, -2, 2))
    patch = np.stack([map_coordinates(c, [dy, dx], order=1, mode="nearest", prefilter=False) for c in channels])
    difference = float(np.abs(patch-image_patch(frame, pred)).max())
    assert difference < 5e-5
    with torch.no_grad():
        tensor = torch.from_numpy(patch)[None]
        embedding = model.image(tensor)[0].numpy()
        delta = model(torch.from_numpy(numeric)[None], tensor)[0].numpy()/.008
    np.testing.assert_allclose(embedding, value[8:136], rtol=0, atol=5e-5)
    np.testing.assert_allclose(delta, value[136:139], rtol=0, atol=5e-5)
    return difference


def reconstruct(root, config, split, position, perception=None):
    review = json.loads((Path(config["review_run"])/"rows.json").read_text())[split]
    x, y, errors, indices, rows = [], [], [], [], []
    max_patch_difference, max_confidence_difference = 0., 0.
    with np.load(Path(config["data_run"])/(split+".npz"), allow_pickle=False) as data:
        metadata = json.loads(str(data["metadata_json"]))
        rgb, depth, seeds = data["rgb"], data["depth_m"], data["seeds"].copy()
        assert len(review) == len(metadata) == len(seeds)
        for i, (meta, old) in enumerate(zip(metadata, review, strict=True)):
            assert old["index"] == i and old["seed"] == meta["seed"] == int(seeds[i])
            assert old["view"] == meta["view"] and old["absent"] == meta["absent"]
            frame = {"rgb": rgb[i], "depth": depth[i], "camera": meta["camera"]}
            pred = old["predictions"]["raw"]
            if perception is not None:
                recomputed = estimate(perception, frame)
                assert (recomputed is None) == (pred is None), (split, i)
                if pred is not None:
                    for key in pred:
                        if key == "confidence":
                            diff = abs(recomputed[key]-pred[key])
                            assert diff < 1e-6, (split, i, key, diff)
                            max_confidence_difference = max(max_confidence_difference, diff)
                        else:
                            assert recomputed[key] == pred[key], (split, i, key)
            # Only RGB-D, calibration and raw visual prediction enter feature inference.
            value = risk_features(position, pred, frame)
            row = {"index": i, "seed": meta["seed"], "view": meta["view"],
                   "absent": meta["absent"], "label_pixels": meta["label_pixels"],
                   "raw_prediction": pred, "applicable": value is not None,
                   "quality": old["quality"], "supervision_usable": False,
                   "reason": "no_detection" if pred is None else "outside_risk_workspace"}
            if value is not None:
                if perception is not None:
                    max_patch_difference = max(max_patch_difference, independent_features(position, pred, frame, value))
                corrected = position.correct(pred, frame)
                np.testing.assert_allclose(corrected["top_xyz_m"], old["predictions"]["image"]["top_xyz_m"], rtol=0, atol=1e-9)
                np.testing.assert_allclose(value[136:139], np.asarray(corrected["residual_xyz_m"])/.008, rtol=0, atol=1e-7)
                row.update(reason="empty_without_position_label" if meta["absent"] else "position_label_available",
                           corrected_prediction=corrected)
                if not meta["absent"]:
                    # Synthetic truth is read only after prediction/features; never an inference argument.
                    truth = meta["truth_for_training_only"]["top_xyz_m"]
                    error = float(np.linalg.norm(np.asarray(corrected["top_xyz_m"])-truth)*1000)
                    assert abs(error-old["image_error_mm"]) < 1e-5, (split, i, error)
                    x.append(value); y.append(float(error > 5)); errors.append(error); indices.append(i)
                    row.update(supervision_usable=True, error_mm=error, unsafe_above5mm=error > 5)
            rows.append(row)
            if (i+1) % 1024 == 0:
                print(json.dumps({"features_split": split, "images": i+1, "audit": perception is not None}), flush=True)
    arrays = {"x": np.asarray(x, np.float32), "y": np.asarray(y, np.float32),
              "error_mm": np.asarray(errors, np.float64), "sample_indices": np.asarray(indices, np.int64), "seeds": seeds}
    diagnostics = {"maximum_independent_patch_difference": max_patch_difference,
                   "maximum_review_confidence_difference": max_confidence_difference}
    return arrays, rows, diagnostics


def train_fixed(data, verbose=False):
    torch.set_num_threads(1)
    torch.use_deterministic_algorithms(True)
    torch.manual_seed(TRAINING["seed"])
    generator = torch.Generator().manual_seed(TRAINING["seed"])
    mean = data["x"].mean(axis=0)
    scale = np.maximum(data["x"].std(axis=0), 1e-4)
    model = VisualRisk(mean, scale)
    assert sum(p.numel() for p in model.parameters()) == TRAINING["parameters"]
    sample_seeds = data["seeds"][data["sample_indices"]]
    counts = Counter(sample_seeds.tolist())
    weights = np.asarray([1/counts[int(s)] for s in sample_seeds], np.float32)
    weights /= weights.mean()
    pos_weight = float(weights[data["y"] == 0].sum()/weights[data["y"] == 1].sum())
    x, y, w = (torch.from_numpy(a) for a in (data["x"], data["y"], weights))
    loss_function = nn.BCEWithLogitsLoss(pos_weight=torch.tensor(pos_weight), reduction="none")
    optimizer = torch.optim.Adam(model.parameters(), lr=TRAINING["learning_rate"], weight_decay=TRAINING["weight_decay"])

    def full_loss():
        with torch.no_grad():
            return float((loss_function(model(x), y)*w).mean())

    history = [{"epoch": 0, "training_weighted_bce": full_loss()}]
    for epoch in range(1, TRAINING["epochs"]+1):
        for ids in torch.randperm(len(x), generator=generator).split(TRAINING["batch_size"]):
            loss = (loss_function(model(x[ids]), y[ids])*w[ids]).mean()
            optimizer.zero_grad(); loss.backward(); optimizer.step()
        history.append({"epoch": epoch, "training_weighted_bce": full_loss()})
        if verbose and epoch % 50 == 0:
            print(json.dumps(history[-1]), flush=True)
    return model.eval(), history, {"row_weights": weights.tolist(), "positive_class_weight": pos_weight,
                                  "training_valid_seeds": len(counts), "equal_total_weight_per_training_seed": True}


def scores(model, data, rows):
    with torch.no_grad():
        learned = torch.sigmoid(model(torch.from_numpy(data["x"]))).numpy().astype(np.float64)
    selected = [rows[i] for i in data["sample_indices"]]
    return {"learned": learned,
            "confidence": np.asarray([1-r["raw_prediction"]["confidence"] for r in selected]),
            "coverage": np.asarray([-r["quality"]["coverage"] for r in selected])}


def threshold(score, labels):
    assert np.isfinite(score).all() and np.any(labels == 1)
    return float(score[labels == 1].min()-1e-6)


def metrics(score, cutoff, data):
    unsafe = data["y"] == 1
    safe = ~unsafe
    accepted = score <= cutoff
    large = data["error_mm"] > 10
    sample_seeds = data["seeds"][data["sample_indices"]]
    n1, n0 = int(unsafe.sum()), int(safe.sum())
    auc = (rankdata(score)[unsafe].sum()-n1*(n1+1)/2)/(n1*n0)
    return {"samples": len(score), "valid_seeds": len(set(sample_seeds.tolist())),
            "safe": n0, "unsafe_above5mm": n1, "unsafe_above10mm": int(large.sum()),
            "accepted_safe": int((accepted & safe).sum()), "accepted_unsafe_above5mm": int((accepted & unsafe).sum()),
            "accepted_unsafe_above10mm": int((accepted & large).sum()),
            "safe_retention": float(accepted[safe].mean()), "unsafe_rejection": float((~accepted)[unsafe].mean()),
            "auc": float(auc), "threshold": cutoff,
            "accepted_unsafe_seeds": sorted(set(sample_seeds[accepted & unsafe].tolist()))}


def evaluate(model, data, rows):
    all_scores = {s: scores(model, data[s], rows[s]) for s in SPLITS}
    cutoffs = {k: threshold(v, data["calibration"]["y"]) for k, v in all_scores["calibration"].items()}
    reports = {}
    for split in SPLITS:
        for j, i in enumerate(data[split]["sample_indices"]):
            rows[split][i]["scores"] = {k: float(v[j]) for k, v in all_scores[split].items()}
            rows[split][i]["accepted_by"] = {k: bool(v[j] <= cutoffs[k]) for k, v in all_scores[split].items()}
        reports[split] = {"images": len(rows[split]), "unique_seeds": len(set(data[split]["seeds"].tolist())),
            "empty_images": sum(r["absent"] for r in rows[split]),
            "positive_unusable_images": sum(not r["absent"] and not r["supervision_usable"] for r in rows[split]),
            "empty_applicable_predictions": sum(r["absent"] and r["applicable"] for r in rows[split]),
            "reason_counts": dict(Counter(r["reason"] for r in rows[split])),
            "models": {k: metrics(v, cutoffs[k], data[split]) for k, v in all_scores[split].items()}}
    cal, val = (reports[s]["models"] for s in ("calibration", "validation"))
    checks = {"calibration_safe_retention_85_percent": cal["learned"]["safe_retention"] >= .85,
        "validation_safe_retention_85_percent": val["learned"]["safe_retention"] >= .85,
        "calibration_accepts_zero_unsafe": cal["learned"]["accepted_unsafe_above5mm"] == 0,
        "validation_rejects_80_percent_unsafe": val["learned"]["unsafe_rejection"] >= .8,
        "validation_rejects_all_above10mm": val["learned"]["accepted_unsafe_above10mm"] == 0}
    for name, group in (("calibration", cal), ("validation", val)):
        checks[name+"_retention_beats_best_simple_by5points"] = (group["learned"]["safe_retention"] >=
            max(group[k]["safe_retention"] for k in ("confidence", "coverage"))+.05)
    return reports, checks, cutoffs


def seed_check(data):
    sets = {s: set(data[s]["seeds"].tolist()) for s in SPLITS}
    assert all(not sets[a] & sets[b] for i, a in enumerate(SPLITS) for b in SPLITS[i+1:])
    return {s: len(sets[s]) for s in SPLITS}


def main(args):
    torch.set_num_threads(1)
    data_root = CANONICAL/"outputs/risk-observation-development-01"
    review = CANONICAL/"outputs/risk-observation-model-review-01"
    assert digest(review/"summary.json") == REVIEW_SUMMARY_SHA
    assert digest(review/"audit.json") == REVIEW_AUDIT_SHA
    root = create_run(args.name, review/"frozen/perception.pt")
    provenance = json.loads((root/"provenance.json").read_text())
    for name in EXTRA:
        shutil.copy2(ROOT/name, root/"source"/name)
        provenance["source_sha256"][name] = digest(ROOT/name)
    write(root/"provenance.json", provenance)
    shutil.copy2(CANONICAL/"VISUAL_RISK_MODEL_PROTOCOL.md", root/"protocol.md")
    (root/"frozen").mkdir()
    for name in ("perception.pt", "image-residual.pt", "feedback_config.json"):
        shutil.copy2(review/"frozen"/name, root/"frozen"/name)
    evidence = [data_root/n for n in ("config.json", "summary.json", "audit.json", "protocol.md")]
    evidence += [data_root/(s+".npz") for s in SPLITS]
    evidence += [review/n for n in ("config.json", "summary.json", "audit.json", "rows.json", "provenance.json")]
    config = {"schema": "b601-visual-risk-development-v1", "hardware_executable": False,
        "data_run": str(data_root), "review_run": str(review), "training": TRAINING,
        "protocol_sha256": digest(root/"protocol.md"),
        "frozen_sha256": {p.name: digest(p) for p in (root/"frozen").iterdir()},
        "evidence_sha256": {str(p): digest(p) for p in evidence},
        "runtime": {"torch_version": str(torch.__version__), "numpy_version": np.__version__,
                    "device": "cpu", "deterministic_algorithms": True}}
    write(root/"config.json", config)
    verify_inputs(root, config)
    position = load(root/"frozen/image-residual.pt")
    for p in position.parameters():
        p.requires_grad_(False)
    data, rows = {}, {}
    for split in SPLITS:
        data[split], rows[split], _ = reconstruct(root, config, split, position)
        np.savez_compressed(root/(split+".npz"), **data[split], metadata_json=json.dumps(rows[split]))
    seeds = seed_check(data)
    model, history, weighting = train_fixed(data["training"], verbose=True)
    write(root/"training_history.json", history)
    write(root/"training_weighting.json", weighting)
    reports, checks, cutoffs = evaluate(model, data, rows)
    torch.save({"schema": "b601-visual-position-risk-v1", "hardware_executable": False,
        "input_dimensions": INPUTS, "state_dict": model.state_dict(), "epochs": TRAINING["epochs"],
        "training_seed": TRAINING["seed"], "position_policy_sha256": POSITION_SHA,
        "perception_sha256": PERCEPTION_SHA, "protocol_sha256": PROTOCOL_SHA,
        "calibration_threshold": cutoffs["learned"], "score_is_calibrated_probability": False}, root/"risk.pt")
    write(root/"rows.json", rows)
    result = {"schema": config["schema"], "complete": True, "development_only": True,
        "reports": reports, "checks": checks, "thresholds": cutoffs,
        "statistical_gates_passed": all(checks.values()), "requires_full_audit_before_any_next_stage": True,
        "adopted": False, "control_changed": False, "new_task_executions": 0,
        "unique_seeds_by_split": seeds, "checkpoint_sha256": digest(root/"risk.pt"),
        "config_sha256": digest(root/"config.json"), "protocol_sha256": PROTOCOL_SHA,
        "feature_data_sha256": {s: digest(root/(s+".npz")) for s in SPLITS},
        "rows_sha256": digest(root/"rows.json"), "history_sha256": digest(root/"training_history.json"),
        "weighting_sha256": digest(root/"training_weighting.json"),
        "scope": "Fixed final 300 epochs, calibration-only zero-unsafe prefix threshold; validation is development, correlated views. Score is not calibrated probability. No position refit, control change, full-task or hardware claim."}
    write(root/"summary.json", result)
    print(json.dumps({"complete": True, "checks": checks, "reports": reports}), flush=True)


def audit(run):
    torch.set_num_threads(1)
    root = run.resolve()
    config = json.loads((root/"config.json").read_text())
    result = json.loads((root/"summary.json").read_text())
    provenance = json.loads((root/"provenance.json").read_text())
    for name, sha in provenance["source_sha256"].items():
        assert digest(ROOT/name) == sha == digest(root/"source"/Path(name).name), name
    manifest = json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p) == sha for p, sha in manifest["files_sha256"].items())
    verify_inputs(root, config)
    assert result["config_sha256"] == digest(root/"config.json")
    for filename, key in (("risk.pt", "checkpoint_sha256"), ("rows.json", "rows_sha256"),
            ("training_history.json", "history_sha256"), ("training_weighting.json", "weighting_sha256")):
        assert digest(root/filename) == result[key]
    model, checkpoint = load_risk(root/"risk.pt")
    assert checkpoint["position_policy_sha256"] == POSITION_SHA and checkpoint["perception_sha256"] == PERCEPTION_SHA
    assert checkpoint["protocol_sha256"] == PROTOCOL_SHA and checkpoint["epochs"] == 300
    assert checkpoint["training_seed"] == TRAINING["seed"] and not checkpoint["score_is_calibrated_probability"]
    position, perception = load(root/"frozen/image-residual.pt"), perception_model(root/"frozen/perception.pt")
    data, rows, diagnostics = {}, {}, {}
    for split in SPLITS:
        data[split], rows[split], diagnostics[split] = reconstruct(root, config, split, position, perception)
        assert digest(root/(split+".npz")) == result["feature_data_sha256"][split]
        with np.load(root/(split+".npz"), allow_pickle=False) as saved:
            assert set(saved.files) == set(data[split]) | {"metadata_json"}
            for k, v in data[split].items():
                assert np.array_equal(saved[k], v), (split, k)
            assert json.loads(str(saved["metadata_json"])) == rows[split]
    assert seed_check(data) == result["unique_seeds_by_split"]
    rebuilt, history, weighting = train_fixed(data["training"], verbose=True)
    assert history == json.loads((root/"training_history.json").read_text())
    assert len(history) == 301 and weighting == json.loads((root/"training_weighting.json").read_text())
    assert all(torch.equal(v, rebuilt.state_dict()[k]) for k, v in model.state_dict().items())
    reports, checks, cutoffs = evaluate(model, data, rows)
    assert reports == result["reports"] and checks == result["checks"] and cutoffs == result["thresholds"]
    assert rows == json.loads((root/"rows.json").read_text())
    assert checkpoint["calibration_threshold"] == cutoffs["learned"]
    assert result["statistical_gates_passed"] == all(checks.values()) and not result["adopted"]
    assert not result["control_changed"] and result["new_task_executions"] == 0
    report = {"passed": True, "images_reconstructed": sum(len(r) for r in rows.values()),
        "valid_risk_features_independently_checked": sum(len(d["x"]) for d in data.values()),
        "training_reproduced_tensor_exact": True, "all_301_training_losses_exact": True,
        "seed_isolation": True, "source_files_checked": len(provenance["source_sha256"]),
        "assets_checked": len(manifest["files_sha256"]), "diagnostics": diagnostics,
        "summary_sha256": digest(root/"summary.json"), "checkpoint_sha256": digest(root/"risk.pt"),
        "eligible_for_next_development": all(checks.values()), "adopted": False,
        "scope": "All raw saved images, original detections (confidence tolerance 1e-6 for single-thread inference), exact 140-feature arrays/labels/seed indices, independent scalars/coverage/SciPy patches, exact deterministic retraining, all scores/calibration thresholds/gates and immutable model/source/asset/data hashes. No new physics or full-task acceptance."}
    write(root/"audit.json", report)
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--name")
    mode.add_argument("--audit-run", type=Path)
    args = parser.parse_args()
    audit(ROOT/"outputs"/args.audit_run) if args.audit_run else main(args)
