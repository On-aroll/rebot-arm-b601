"""Apply the written image-residual trial review after the full audit.

Writes evidence only. This program never changes the adopted release pointer.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


PROTOCOL_SHA256 = "7c56dca47c336b37b08772663e6ebf62cd326ada2376d3b033fa565e483ab628"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(run):
    root = (Path(__file__).resolve().parent/"outputs"/run).resolve()
    summary = json.loads((root/"summary.json").read_text())
    audit = json.loads((root/"audit.json").read_text())
    metrics = json.loads((root/"report_metrics.json").read_text())
    frozen = json.loads((root/"frozen_evaluation.json").read_text())
    assert summary["phase"] == audit["phase"] == frozen["phase"] == "evaluate"
    assert digest(root/"protocol.md") == frozen["protocol"]["sha256"] == PROTOCOL_SHA256
    assert audit["passed"] and not audit["prior_seed_overlap"]
    assert digest(root/"summary.json") == metrics["summary_sha256"]
    assert digest(root/"audit.json") == metrics["audit_sha256"]
    assert digest(root/"report_metrics_source.py") == metrics["report_source_sha256"]
    expected = sum(len(seeds)*len(frozen["variants"]) for block, seeds in frozen["test_seeds"].items()
                   if block in ("static", "disturbed")) + 10
    assert audit["episodes"] == expected and len(summary["negative_controls"]) == 10
    assert all(len(frozen["test_seeds"][block]) >= 30 for block in ("static", "disturbed"))
    review, preserved, benefits = {}, True, []
    zero_distinguishable = []
    for block, comparisons in metrics["paired_details"].items():
        v3, zero = comparisons["retained_v3"], comparisons["zero_residual"]
        preserved &= not v3["full_losses"]
        def benefit(pair):
            reasons = []
            if len(pair["full_gains"]) > len(pair["full_losses"]):
                reasons.append("net_additional_full_task_success")
            if len(pair["first_gains"]) > len(pair["first_losses"]):
                reasons.append("net_additional_first_attempt_success")
            seconds = pair["common_success_mean_sim_seconds_difference"]
            if seconds is not None and seconds <= -.5:
                reasons.append("common_success_mean_time_reduced_at_least_half_second")
            return reasons
        v3_reasons, zero_reasons = benefit(v3), benefit(zero)
        benefits += [{"block": block, "reason": reason} for reason in v3_reasons]
        zero_distinguishable += [{"block": block, "reason": reason} for reason in zero_reasons]
        review[block] = {"versus_V3": v3, "versus_zero": zero,
                         "task_benefit_reasons_versus_V3": v3_reasons,
                         "task_benefit_reasons_versus_zero": zero_reasons}
    actions = sum(g["new_feedback"]["summary"]["learned_action_calls"] for g in summary["groups"].values())
    checks = {"engineering_acceptance": summary["acceptance_met"] and audit["acceptance_met"],
              "all_V3_full_successes_preserved": bool(preserved),
              "concrete_fresh_task_benefit_over_V3": bool(benefits),
              "image_policy_actually_used_for_action": actions > 0}
    report = {"schema": "b601-image-residual-adoption-review-v1", "checks": checks,
        "recommended_for_release_integration": all(checks.values()),
        "release_integration_complete": False, "adopted": False,
        "benefits_over_V3": benefits, "benefits_over_zero": zero_distinguishable,
        "paired_review": review, "actual_image_model_actions_in_main_feedback_group": actions,
        "all_main_task_failures": metrics["all_main_task_failures"],
        "evidence_sha256": {name: digest(root/name) for name in
            ("protocol.md", "frozen_evaluation.json", "provenance.json", "summary.json", "audit.json", "report_metrics.json")},
        "review_source_sha256": digest(Path(__file__)),
        "limitations": "Finite paired simulation trial; exploratory statistics only. Zero comparison is not an isolated ablation of image features because architecture, capacity and training changed. Adoption additionally requires an independently frozen runtime and verified user entrypoint. No hardware claim."}
    shutil.copy2(Path(__file__), root/"adoption_review_source.py")
    (root/"adoption_review.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps({"checks": checks, "recommended_for_release_integration": all(checks.values()),
                      "benefits_over_V3": benefits, "benefits_over_zero": zero_distinguishable}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True)
    main(parser.parse_args().run)
