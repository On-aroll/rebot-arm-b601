"""Evaluate already fixed position models on every audited risk-data image.

This creates development measurements, without fitting a model or selecting
control thresholds. Truth is read after the model outputs, for scoring only.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import shutil

import numpy as np

from feedback import quality
from policy import load, valid_prediction
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import estimate


def aggregate(rows):
    usable = [r for r in rows if not r["absent"] and r["raw_valid"]]
    result = {"images": len(rows), "unique_seeds": len({r["seed"] for r in rows}),
        "empty_images": sum(r["absent"] for r in rows), "usable_positive_images": len(usable),
        "positive_unusable": sum(not r["absent"] and not r["raw_valid"] for r in rows),
        "empty_valid_predictions": sum(r["absent"] and r["raw_valid"] for r in rows),
        "quality_reasons": dict(Counter(r["quality"]["reason"] for r in rows))}
    for name in ("raw", "V3", "image"):
        errors = [r[name+"_error_mm"] for r in usable]
        result[name] = {"samples": len(errors), "mean_mm": float(np.mean(errors)) if errors else None,
            "p95_mm": float(np.percentile(errors, 95)) if errors else None,
            "maximum_mm": max(errors) if errors else None,
            "above5mm": sum(x > 5 for x in errors), "above10mm": sum(x > 10 for x in errors),
            "corrected_invalid": sum(not r[name+"_valid"] for r in usable)}
    return result


def evaluate(root, config):
    data_root = Path(config["data_run"])
    assert all(digest(Path(name)) == sha for name, sha in config["data_evidence_sha256"].items())
    assert all(digest(root/"frozen"/name) == sha for name, sha in config["models_and_config_sha256"].items())
    perception = perception_model(root/"frozen/perception.pt")
    scalar, image = load(root/"frozen/V3-residual.pt"), load(root/"frozen/image-residual.pt")
    feedback = json.loads((root/"frozen/feedback_config.json").read_text())
    reports, rows_by_split = {}, {}
    for split in ("training", "calibration", "validation"):
        rows = []
        with np.load(data_root/(split+".npz"), allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata_json"]))
            rgb, depth = data["rgb"], data["depth_m"]
            for i, meta in enumerate(metadata):
                frame = {"rgb": rgb[i], "depth": depth[i], "camera": meta["camera"]}
                raw = estimate(perception, frame)
                predictions = {"raw": raw, "V3": None if raw is None else scalar.correct(raw, frame),
                               "image": None if raw is None else image.correct(raw, frame)}
                gate = quality(raw, meta["camera"], feedback)
                # Supervision is not passed into any inference/quality function.
                row = {"index": i, "seed": meta["seed"], "view": meta["view"], "absent": meta["absent"],
                       "label_pixels": meta["label_pixels"], "quality": gate,
                       "initial_forbidden_contact": bool(meta["truth_for_training_only"]["initial_contact"]["forbidden"]),
                       "predictions": predictions}
                for name, prediction in predictions.items():
                    row[name+"_valid"] = valid_prediction(prediction)
                    if prediction is not None and not meta["absent"]:
                        truth = meta["truth_for_training_only"]["top_xyz_m"]
                        row[name+"_error_mm"] = float(np.linalg.norm(np.asarray(prediction["top_xyz_m"])-truth)*1000)
                rows.append(row)
                if (i+1) % 512 == 0:
                    print(json.dumps({"review_split": split, "images": i+1}), flush=True)
        reports[split] = {"all": aggregate(rows),
            "quality_accepted": aggregate([r for r in rows if r["quality"]["accepted"]]),
            "low_confidence_reason": aggregate([r for r in rows if r["quality"]["reason"] == "low_confidence"]),
            "initial_forbidden_contact": aggregate([r for r in rows if r["initial_forbidden_contact"]]),
            "by_view": {v: aggregate([r for r in rows if r["view"] == v]) for v in dict.fromkeys(r["view"] for r in rows)},
            "confidence_bins": {f"{a:g}_to_{min(b,1.):g}": aggregate([r for r in rows
                if r["predictions"]["raw"] is not None and a <= r["predictions"]["raw"]["confidence"] < b])
                for a, b in zip((0., .5, .6, .7, .75, .8, .9), (.5, .6, .7, .75, .8, .9, 1.000001))}}
        rows_by_split[split] = rows
    report = {"schema": "b601-risk-image-fixed-model-review-v1", "development_only": True,
              "model_trained": False, "control_changed": False, "reports": reports,
              "config_sha256": digest(root/"config.json"),
              "scope": "Every audited initialized image scored by fixed V3 and image models. All negative/missing/initial-contact images retained. Per-frame quality is not temporal-pair acceptance. Errors conditional on the same raw-valid positives; other images and corrected-invalid counts reported. No physics or full-task execution. Correlated views are not independent scenes."}
    return report, rows_by_split


def main(args):
    data_root = (ROOT/"outputs"/args.data_run).resolve()
    data_summary = json.loads((data_root/"summary.json").read_text())
    assert data_summary["complete"] and json.loads((data_root/"audit.json").read_text())["passed"]
    canonical = (ROOT/"outputs").resolve().parent
    v3 = canonical/"versions/V3"
    root = create_run(args.name, data_root/"frozen/perception.pt")
    provenance = json.loads((root/"provenance.json").read_text())
    for name in ("risk_image_review.py", "audit_frozen.py"):
        shutil.copy2(ROOT/name, root/"source"/name)
        provenance["source_sha256"][name] = digest(ROOT/name)
    write(root/"provenance.json", provenance)
    (root/"frozen").mkdir()
    sources = {"perception.pt": data_root/"frozen/perception.pt",
               "image-residual.pt": data_root/"frozen/residual.pt",
               "V3-residual.pt": v3/"models/residual.pt", "feedback_config.json": v3/"feedback_config.json"}
    assert digest(sources["V3-residual.pt"]) == "d2de74cdbe37694cae299abf4b3b90e4dee117bba0a8c74ef658a7f4be5f80b6"
    for name, source in sources.items():
        shutil.copy2(source, root/"frozen"/name)
    data_evidence = {str(data_root/name): digest(data_root/name) for name in
        ("summary.json", "audit.json", "config.json", "protocol.md", "training.npz", "calibration.npz", "validation.npz")}
    config = {"data_run": str(data_root), "data_evidence_sha256": data_evidence,
              "models_and_config_sha256": {name: digest(root/"frozen"/name) for name in sources}}
    write(root/"config.json", config)
    report, rows = evaluate(root, config)
    write(root/"summary.json", report); write(root/"rows.json", rows)
    print(json.dumps({s: {k: report["reports"][s][k] for k in ("all", "quality_accepted", "low_confidence_reason")}
                      for s in report["reports"]}), flush=True)


def audit(run):
    root = (ROOT/"outputs"/run).resolve()
    provenance = json.loads((root/"provenance.json").read_text())
    for name, sha in provenance["source_sha256"].items():
        assert digest(ROOT/name) == sha == digest(root/"source"/Path(name).name)
    report, rows = evaluate(root, json.loads((root/"config.json").read_text()))
    assert report == json.loads((root/"summary.json").read_text())
    assert rows == json.loads((root/"rows.json").read_text())
    write(root/"audit.json", {"passed": True, "images_recomputed": sum(len(r) for r in rows.values()),
        "source_files_checked": len(provenance["source_sha256"]), "summary_sha256": digest(root/"summary.json"),
        "rows_sha256": digest(root/"rows.json"),
        "scope": "Exact full saved-image inference/quality/error recomputation and source/model/data hashes; dataset renderer was separately audited. No new physics or full tasks."})
    print(json.dumps({"passed": True, "images_recomputed": sum(len(r) for r in rows.values())}), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--name"); mode.add_argument("--run", help="Audit a finished review")
    parser.add_argument("--data-run", default="risk-observation-development-01")
    args = parser.parse_args()
    audit(args.run) if args.run else main(args)
