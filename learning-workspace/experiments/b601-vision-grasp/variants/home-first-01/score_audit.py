"""Reconstruct the final one-second physical predicate from saved raw scoring data.

This checks recorded simulation evidence, not independent physical re-execution.
No result from this module is used to select robot actions.
"""
import json

import numpy as np


def audit_score_trace(directory, episode, timestep):
    if episode["schema"] != "b601-goal-pick-place-v3":
        return {"available":False,"samples":0}
    path = directory/"final_score_trace.jsonl"
    trace = [json.loads(line) for line in path.read_text().splitlines()]
    assert len(trace) == episode["score_trace_samples"]
    if not trace:
        assert not episode["physical_success"]
        assert not episode["record_rows"] or "failed_stage" in episode
        return {"available":True,"samples":0}
    assert len(trace) == round(1./timestep)
    goal = np.asarray(episode["task"]["region_center_xy_m"])
    valid, previous = [], None
    for row in trace:
        center = np.asarray(row["object_xyz_m"])
        rotation = np.asarray(row["object_rotation_base"])
        half_size = np.asarray(row["object_half_size_m"])
        velocity = np.asarray(row["object_linear_velocity_m_s"])
        assert center.shape == velocity.shape == half_size.shape == (3,)
        assert rotation.shape == (3,3)
        assert all(np.isfinite(v).all() for v in (center,rotation,half_size,velocity))
        assert np.allclose(rotation.T@rotation,np.eye(3),atol=1e-7)
        assert np.isclose(np.linalg.det(rotation),1.,atol=1e-7)
        assert np.allclose(half_size,episode["reset"]["half_size_m"],atol=1e-12)
        extent = np.abs(rotation)@half_size
        inside = bool(np.all(np.abs(center[:2]-goal)+extent[:2] <= .06))
        supported = bool(abs(center[2]-extent[2]-.1) < .006)
        speed = float(np.linalg.norm(velocity))
        released = not any(row["finger_contacts"].values())
        assert set(row["finger_contacts"]) == {"left","right"}
        assert row["inside_region"] == inside
        assert row["supported_by_table"] == supported
        assert np.isclose(row["linear_speed_m_s"],speed,atol=1e-12)
        assert row["released"] == released
        success = bool(inside and supported and speed < .02 and released)
        assert row["success"] == success
        if previous is not None:
            assert abs(row["t_sim_s"]-previous-timestep) < 1e-8
        previous = row["t_sim_s"]
        valid.append(success)
    assert abs(trace[-1]["t_sim_s"]-episode["sim_seconds"]) < 1e-8
    assert trace[-1] == episode["final_score"]
    expected = all(valid) and not episode["physics_tick_audit"]["forbidden_steps"]
    assert episode["physical_success"] == expected
    return {"available":True,"samples":len(trace)}
