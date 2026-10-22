"""Posthoc diagnostics for the rejected fixed risk head; never selects a model or threshold."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def roc(rows, key):
    ordered = sorted(rows, key=lambda r: r["scores"][key], reverse=True)
    bad = sum(r["unsafe_above5mm"] for r in rows)
    good = len(rows)-bad
    fp, tp, x, y = 0, 0, [0.], [0.]
    for i, row in enumerate(ordered):
        tp += row["unsafe_above5mm"]
        fp += not row["unsafe_above5mm"]
        if i == len(ordered)-1 or row["scores"][key] != ordered[i+1]["scores"][key]:
            x.append(fp/good); y.append(tp/bad)
    return x, y


def main(run):
    root = run.resolve()
    config = json.loads((root/"config.json").read_text())
    summary = json.loads((root/"summary.json").read_text())
    audit = json.loads((root/"audit.json").read_text())
    assert audit["passed"] and not audit["eligible_for_next_development"]
    assert audit["summary_sha256"] == sha(root/"summary.json")
    assert summary["rows_sha256"] == sha(root/"rows.json")
    rows = json.loads((root/"rows.json").read_text())
    valid = {s: [r for r in rs if r["supervision_usable"]] for s, rs in rows.items()}
    destination = root/"posthoc-risk-failure"
    destination.mkdir(exist_ok=False)
    copied = destination/"risk_failure_review_source.py"
    shutil.copy2(__file__, copied)
    report = {"posthoc_only": True, "model_or_threshold_selected": False,
        "control_changed": False, "training_restarted": False,
        "summary_sha256": sha(root/"summary.json"), "audit_sha256": sha(root/"audit.json"),
        "rows_sha256": sha(root/"rows.json"), "source_sha256": sha(copied), "splits": {}}
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    history = json.loads((root/"training_history.json").read_text())
    axes[0].semilogy([h["epoch"] for h in history], [h["training_weighted_bce"] for h in history])
    axes[0].set(xlabel="Epoch (fixed final = 300)", ylabel="Weighted training BCE", title="Training fits; transfer still fails")
    colors = {"training": "#888888", "calibration": "#d97706", "validation": "#2563eb"}
    for split, values in valid.items():
        xs, ys = roc(values, "learned")
        metric = summary["reports"][split]["models"]["learned"]
        axes[1].plot(xs, ys, color=colors[split], label=f"{split}: AUC {metric['auc']:.3f}")
        bad = sorted([r for r in values if r["unsafe_above5mm"]], key=lambda r: r["scores"]["learned"])
        safe = [r for r in values if not r["unsafe_above5mm"]]
        report["splits"][split] = {"lowest_scored_unsafe": bad[:3],
            "score_quantiles_safe": np.quantile([r["scores"]["learned"] for r in safe], [0, .05, .5, .95, 1]).tolist(),
            "score_quantiles_unsafe": np.quantile([r["scores"]["learned"] for r in bad], [0, .05, .5, .95, 1]).tolist(),
            "safe_fraction_ranked_below_lowest_unsafe": sum(r["scores"]["learned"] < bad[0]["scores"]["learned"] for r in safe)/len(safe),
            "roc": {"false_positive_rate": xs, "true_positive_rate": ys}}
    axes[1].plot([0, 1], [0, 1], linestyle=":", color="black")
    axes[1].set(xlabel="Safe frames rejected (fraction)", ylabel=">5 mm frames rejected (fraction)", title="Ranking is insufficient for release", xlim=(0, 1), ylim=(0, 1.02))
    axes[1].legend(fontsize=8)
    for unsafe, color, label in ((False, "#16a34a", "Accurate <=5mm"), (True, "#dc2626", "Error >5mm")):
        values = [r["scores"]["learned"] for r in valid["calibration"] if r["unsafe_above5mm"] == unsafe]
        axes[2].hist(np.log10(np.clip(values, 1e-17, 1)), bins=np.linspace(-17, 0, 35), alpha=.6, color=color, label=label)
    axes[2].set(xlabel="log10(learned score); calibration", ylabel="Frames", title="Some inaccurate frames get tiny scores")
    axes[2].legend(fontsize=8)
    fig.suptitle("Rejected visual risk head: fixed calibration cutoff < 0 rejects every frame", fontsize=13)
    fig.tight_layout()
    fig.savefig(destination/"score-diagnostics.png", dpi=150)
    plt.close(fig)

    calibration_bad = min((r for r in valid["calibration"] if r["unsafe_above5mm"]), key=lambda r: r["scores"]["learned"])
    validation_bad = min((r for r in valid["validation"] if r["unsafe_above5mm"]), key=lambda r: r["scores"]["learned"])
    validation_worst = max(valid["validation"], key=lambda r: r["error_mm"])
    calibration_false_alarm = max((r for r in valid["calibration"] if not r["unsafe_above5mm"]), key=lambda r: r["scores"]["learned"])
    examples = [("calibration", calibration_bad), ("validation", validation_bad),
                ("validation", validation_worst), ("calibration", calibration_false_alarm)]
    fig, axes = plt.subplots(2, 4, figsize=(12.8, 7))
    for column, (split, row) in enumerate(examples):
        path = Path(config["data_run"])/(split+".npz")
        assert sha(path) == config["evidence_sha256"][str(path)]
        with np.load(path, allow_pickle=False) as data:
            i = row["index"]
            rgb, depth, mask = data["rgb"][i], data["depth_m"][i], data["mask"][i]
        axes[0, column].imshow(rgb, interpolation="nearest")
        axes[1, column].imshow(depth, cmap="viridis", vmin=.2, vmax=.8, interpolation="nearest")
        for ax in axes[:, column]:
            ax.contour(mask, levels=[.5], colors="white", linewidths=.6)
            ax.plot(*row["raw_prediction"]["pixel_uv"], "+", color="cyan", ms=6)
            ax.set_xticks([]); ax.set_yticks([])
        axes[0, column].set_title(f"{split} {row['seed']}\n{row['view']}", fontsize=10)
        axes[1, column].set_xlabel(f"Error: {row['error_mm']:.3f} mm\nScore: {row['scores']['learned']:.3g}\nConfidence: {row['raw_prediction']['confidence']:.3f}", fontsize=10)
    fig.suptitle("Actual saved RGB-D failures and false alarm (independent labels only)")
    fig.text(.5, .025, "Top: RGB. Bottom: depth 0.2-0.8 m. Cyan: raw detected center. White: supervision mask, shown only for review.", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, .06, 1, .95))
    fig.savefig(destination/"failure-examples.png", dpi=150)
    plt.close(fig)
    report["example_frames"] = [{"split": s, **r} for s, r in examples]
    report["artifacts_sha256"] = {n: sha(destination/n) for n in ("score-diagnostics.png", "failure-examples.png")}
    (destination/"review.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps({"posthoc_only": True, "model_or_threshold_selected": False,
        "review_sha256": sha(destination/"review.json"), "destination": str(destination),
        "safe_fraction_below_lowest_unsafe": {s: r["safe_fraction_ranked_below_lowest_unsafe"] for s, r in report["splits"].items()}}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    args = parser.parse_args()
    main(Path(__file__).resolve().parent/"outputs"/args.run)
