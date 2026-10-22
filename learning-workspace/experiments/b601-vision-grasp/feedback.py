"""Image-only quality gates for the fixed overhead camera and known cube task.

The tabletop calibration and cube shape are task assumptions. No simulator
object size, pose, contact, or disturbance state is accepted by this module.
"""
import copy

import numpy as np

from policy import valid_prediction


DEFAULT_CONFIG = {
    "schema": "b601-pregrasp-feedback-v1",
    "table_height_m": .1,
    "view_offsets_m": [[-.08, 0., .15], [-.06, .06, .12]],
    "use_home_fallback": True,
    "settle_s": .3,
    "pair_gap_s": .2,
    "min_confidence": .8,
    "min_coverage": .75,
    "max_coverage": 1.4,
    "max_pair_shift_m": .004,
    "max_update_m": .08,
    "selection": "closest_projected_coverage_to_one",
}


def configuration(config=None):
    result = copy.deepcopy(DEFAULT_CONFIG if config is None else config)
    if set(result) != set(DEFAULT_CONFIG) or result["schema"] != DEFAULT_CONFIG["schema"]:
        raise ValueError("invalid_feedback_configuration")
    offsets = np.asarray(result["view_offsets_m"], float)
    numeric = [result[k] for k in ("table_height_m", "settle_s", "pair_gap_s", "min_confidence",
                                  "min_coverage", "max_coverage", "max_pair_shift_m", "max_update_m")]
    if (offsets.shape != (2, 3) or not np.isfinite(offsets).all()
            or not np.isfinite(numeric).all() or np.max(np.abs(offsets)) > .2
            or not isinstance(result["use_home_fallback"], bool)
            or not .1 <= result["settle_s"] <= 1. or not .1 <= result["pair_gap_s"] <= 1.
            or not 0. <= result["min_confidence"] <= 1.
            or not 0. < result["min_coverage"] < result["max_coverage"] <= 2.
            or not 0. < result["max_pair_shift_m"] <= .01
            or not 0. < result["max_update_m"] <= .1
            or result["selection"] != DEFAULT_CONFIG["selection"]):
        raise ValueError("invalid_feedback_configuration")
    return result


def quality(prediction, camera, config=DEFAULT_CONFIG):
    """Reject small/occluded masks using only raw RGB-D inference/calibration."""
    result = {"accepted": False, "reason": "no_detection"}
    if prediction is None:
        return result
    if not valid_prediction(prediction):
        result["reason"] = "outside_visual_workspace"
        return result
    k = np.asarray(camera["K"], float)
    t = np.asarray(camera["T_base_camera_optical"], float)
    height = float(prediction["top_xyz_m"][2] - config["table_height_m"])
    depth = float(prediction["depth_m"])
    confidence = float(prediction["confidence"])
    # A cube's top face has side length equal to its visually estimated height.
    # cos(normal, optical axis) accounts for a small calibrated camera tilt.
    expected = float(k[0, 0] * k[1, 1] * height**2 * abs(t[2, 2]) / depth**2)
    coverage = float(prediction["pixels"] / expected) if expected > 0. else 0.
    result.update(confidence=confidence, estimated_cube_height_m=height,
                  expected_top_pixels=expected, coverage=coverage)
    if not np.isfinite([height, depth, confidence, expected, coverage]).all() or depth <= 0.:
        result["reason"] = "invalid_visual_geometry"
    elif confidence < config["min_confidence"]:
        result["reason"] = "low_confidence"
    elif not config["min_coverage"] <= coverage <= config["max_coverage"]:
        result["reason"] = "incomplete_or_oversized_mask"
    else:
        result.update(accepted=True, reason="visual_quality_accepted")
    return result


def pair_quality(predictions, cameras, initial_prediction, config=DEFAULT_CONFIG):
    gates = [quality(p, c, config) for p, c in zip(predictions, cameras, strict=True)]
    result = {"quality": gates, "accepted": False, "reason": "frame_quality_rejected",
              "pair_shift_m": None, "update_distance_m": None}
    if len(gates) != 2 or not all(g["accepted"] for g in gates):
        return result
    top = np.asarray(predictions[-1]["top_xyz_m"])
    shift = float(np.linalg.norm(top - predictions[0]["top_xyz_m"]))
    update = float(np.linalg.norm(top - initial_prediction["top_xyz_m"]))
    result.update(pair_shift_m=shift, update_distance_m=update)
    if shift > config["max_pair_shift_m"]:
        result["reason"] = "unstable_observation_pair"
    elif update > config["max_update_m"]:
        result["reason"] = "visual_update_too_large"
    else:
        result.update(accepted=True, reason="stable_visual_pair")
    return result


def select_view(views):
    candidates = [v for v in views if v["accepted"]]
    if not candidates:
        return None
    # Deterministic tie break by the predeclared view index; no truth/error input.
    return min(candidates, key=lambda v: (abs(np.log(v["quality"][-1]["coverage"])), v["view_index"]))
