"""Describe fixed visual-quality gates versus audited development position errors.

No current formal scenes, model fitting, gate selection, actions or physics.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
from statistics import fmean

import numpy as np

from feedback import quality


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def aggregate(rows):
    usable = [r for r in rows if r["usable_for_supervision"]]
    result = {"images": len(rows), "unique_seeds": len({r["seed"] for r in rows}),
        "empty_images": sum(r["absent"] for r in rows), "usable_positive_images": len(usable),
        "positive_unusable": sum(not r["absent"] and not r["usable_for_supervision"] for r in rows),
        "gate_reasons": dict(Counter(r["quality"]["reason"] for r in rows))}
    for model in ("raw", "V3", "image"):
        errors = [r[model+"_error_mm"] for r in usable]
        result[model] = {"samples": len(errors), "mean_mm": fmean(errors) if errors else None,
                         "above5mm": sum(e > 5 for e in errors), "above10mm": sum(e > 10 for e in errors)}
    return result


def main(name):
    workspace = Path(__file__).resolve().parent
    outputs = (workspace/"outputs").resolve()
    development = outputs/"image-residual-development-01"
    root = outputs/name
    summary = json.loads((development/"summary.json").read_text())
    audit = json.loads((development/"audit.json").read_text())
    assert audit["passed"] and summary["checkpoint_sha256"] == digest(development/"residual.pt")
    v3 = outputs.parent/"versions/V3"
    config = json.loads((v3/"feedback_config.json").read_text())
    source_data = outputs/"observation-images-egl-01"
    evidence = {str(development/filename): digest(development/filename)
                for filename in ("summary.json", "audit.json", "provenance.json", "residual.pt")}
    evidence[str(v3/"feedback_config.json")] = digest(v3/"feedback_config.json")
    reports, all_rows = {}, {}
    for split in ("training", "validation"):
        path = source_data/(split+".npz")
        assert digest(path) == summary["source_data_sha256"][split]
        evidence[str(path)] = digest(path)
        with np.load(path, allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata_json"]))
            seeds = data["seeds"].tolist()
        rows = []
        for row, meta, seed in zip(summary["reports"][split]["rows"], metadata, seeds, strict=True):
            assert row["seed"] == seed == meta["seed"] and row["absent"] == meta["absent"]
            rows.append({**row, "quality": quality(row["raw_prediction"], meta["camera"], config)})
        assert len(rows) == summary["reports"][split]["images"]
        boundaries = [0., .6, .7, .75, .8, .9, 1.000001]
        reports[split] = {"all": aggregate(rows),
            "quality_accepted": aggregate([r for r in rows if r["quality"]["accepted"]]),
            "quality_rejected": aggregate([r for r in rows if not r["quality"]["accepted"]]),
            "low_confidence_only": aggregate([r for r in rows if r["quality"]["reason"] == "low_confidence"]),
            "confidence_bins": {f"{a:g}_to_{min(b,1.):g}": aggregate([r for r in rows
                if r["raw_prediction"] is not None and a <= r["raw_prediction"]["confidence"] < b])
                for a, b in zip(boundaries[:-1], boundaries[1:])}}
        all_rows[split] = rows
    assert not {r["seed"] for r in all_rows["training"]}.intersection(r["seed"] for r in all_rows["validation"])
    root.mkdir(exist_ok=False)
    report = {"schema": "b601-visual-quality-risk-diagnostic-v1", "posthoc_development_only": True,
        "model_trained": False, "control_changed": False, "physics_executed": False,
        "fixed_quality_config": config, "reports": reports, "input_sha256": evidence,
        "source_sha256": digest(Path(__file__)), "quality_source_sha256": digest(workspace/"feedback.py"),
        "scope": "Aggregate existing audited training/development errors against unchanged per-frame V3 quality gates. All 2432 images retained; source NPZ hashes and seeds checked. No formal trial inputs. Views sharing seeds are correlated. Single-frame quality is not temporal pair acceptance. Errors come from the previously audited report; this diagnostic is not independent inference or physics re-execution."}
    (root/"summary.json").write_text(json.dumps(report, indent=2)+"\n")
    (root/"rows.json").write_text(json.dumps(all_rows, indent=2)+"\n")
    shutil.copy2(Path(__file__), root/"diagnostic_source.py")
    shutil.copy2(workspace/"feedback.py", root/"quality_source.py")
    print(json.dumps({split: {k: reports[split][k] for k in
        ("quality_accepted", "quality_rejected", "low_confidence_only")} for split in reports}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True)
    main(parser.parse_args().name)
