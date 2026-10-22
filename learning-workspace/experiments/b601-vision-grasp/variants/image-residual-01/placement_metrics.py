"""Posthoc placement measurements from every audited final score trace.

This measures outcomes; no value from this program enters robot control.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
from statistics import fmean


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(run):
    root = (Path(__file__).resolve().parent / "outputs" / run).resolve()
    summary = json.loads((root / "summary.json").read_text())
    audit = json.loads((root / "audit.json").read_text())
    assert audit["passed"]
    groups, pairs = {}, {}
    for block, variants in summary["groups"].items():
        groups[block] = {}
        for variant, group in variants.items():
            rows = []
            for episode in group["episodes"]:
                path = root / episode["artifact_directory"] / "final_score_trace.jsonl"
                row = {"seed": episode["reset"]["seed"], "goal": episode["task"]["destination"],
                       "success": episode["success"], "episode": episode["artifact_directory"],
                       "trace_sha256": digest(path) if path.exists() else None,
                       "mean_radial_xy_mm": None, "maximum_radial_xy_mm": None}
                if episode["record_rows"]:
                    samples = [json.loads(line) for line in path.read_text().splitlines()]
                    assert len(samples) == episode["score_trace_samples"] == 500
                    distances = [1000 * math.dist(s["object_xyz_m"][:2],
                                 episode["task"]["region_center_xy_m"]) for s in samples]
                    row.update(mean_radial_xy_mm=fmean(distances), maximum_radial_xy_mm=max(distances))
                rows.append(row)
            successful = [row["mean_radial_xy_mm"] for row in rows if row["success"]]
            groups[block][variant] = {"rows": rows, "successes": len(successful),
                "successful_scene_mean_radial_xy_mm": fmean(successful) if successful else None}
        pairs[block] = {}
        candidate = groups[block]["new_feedback"]["rows"]
        for reference in ("retained_v3", "zero_residual", "new_observe_only", "retained_two_view"):
            paired = []
            for a, b in zip(groups[block][reference]["rows"], candidate, strict=True):
                assert (a["seed"], a["goal"]) == (b["seed"], b["goal"])
                if a["success"] and b["success"]:
                    paired.append({"seed": a["seed"], "goal": a["goal"],
                        "candidate_minus_reference_mm": b["mean_radial_xy_mm"]-a["mean_radial_xy_mm"]})
            differences = [row["candidate_minus_reference_mm"] for row in paired]
            pairs[block][reference] = {"common_success_pairs": len(paired), "rows": paired,
                "mean_candidate_minus_reference_mm": fmean(differences) if differences else None,
                "improved_by_over_point1_mm": sum(d < -.1 for d in differences),
                "worsened_by_over_point1_mm": sum(d > .1 for d in differences)}
    report = {"exploratory_posthoc": True, "groups": groups, "paired_common_success": pairs,
        "definition": "Per-scene mean over all 500 final scoring samples of XY distance to requested region centre, in mm. Equal scene weights; pair only common successes. Every failure remains listed.",
        "limitations": "Not a preregistered adoption outcome; no change to task region or control. Correlated scoring samples are not independent scenes.",
        "summary_sha256": digest(root/"summary.json"), "audit_sha256": digest(root/"audit.json"),
        "source_sha256": digest(Path(__file__))}
    shutil.copy2(Path(__file__), root/"placement_metrics_source.py")
    (root/"placement_metrics.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({b: {ref: p["mean_candidate_minus_reference_mm"]
                         for ref, p in comparisons.items()} for b, comparisons in pairs.items()}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    main(parser.parse_args().run)
