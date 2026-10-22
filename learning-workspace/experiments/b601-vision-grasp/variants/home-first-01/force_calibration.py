"""Development-only fixed-force calibration while the arm holds HOME.

Every candidate is applied to every declared development scene. No heldout task
scenes, perception predictions, or controller decisions choose the force.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from sim import ROOT, Simulation


SEEDS = list(range(6000000, 6000012))
CANDIDATES = [
    {"name": "no_pulse", "pulse": None},
    *[{"name": name, "pulse": {"start_s": .5, "duration_s": duration,
                                "force_world_n": [0., force, 0.]}}
      for name, force, duration in (("y080n_040ms", .8, .04), ("y080n_060ms", .8, .06),
                                    ("y100n_040ms", 1., .04), ("y100n_060ms", 1., .06),
                                    ("y120n_040ms", 1.2, .04))]]


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2)+"\n")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(name):
    if Path(name).name != name or name in (".", ".."):
        raise ValueError("Use one new output-directory name")
    root = ROOT/"outputs"/name
    root.mkdir(parents=True, exist_ok=False)
    source = root/"source"
    source.mkdir()
    hashes = {}
    for relative in ("sim.py", "force_calibration.py", "assets/task_scene.xml", "assets/scene.xml", "assets/manifest.json"):
        shutil.copy2(ROOT/relative, source/Path(relative).name)
        hashes[relative] = digest(ROOT/relative)
    manifest = json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p) == sha for p, sha in manifest["files_sha256"].items())
    config = {"development_only": True, "full_task_test": False, "hardware_executable": False,
              "seeds": SEEDS, "candidates": CANDIDATES, "duration_s": 1.2,
              "arm_behavior": "hold HOME, gripper open; no policy or IK movement",
              "selection_rule": "Among nonzero pulses: maximize fraction with signed settled y displacement 0.01--0.04m; then minimize absolute median difference from 0.025m; then minimize maximum absolute displacement. Keep all scenes and extremes.",
              "source_sha256": hashes, "original_assets_verified": len(manifest["files_sha256"])}
    write(root/"frozen_config.json", config)
    sim = Simulation(ROOT/"assets/task_scene.xml")
    groups = []
    try:
        for candidate in CANDIDATES:
            cases = []
            for seed in SEEDS:
                directory = root/candidate["name"]/str(seed)
                directory.mkdir(parents=True)
                reset = sim.reset(seed, force_pulse=candidate["pulse"])
                rows, frames = [], []
                next_row = next_frame = 0.
                forbidden_steps = 0
                for _ in range(round(config["duration_s"]/sim.model.opt.timestep)):
                    sim.step()
                    contact = sim.contacts()
                    forbidden_steps += bool(contact["forbidden"])
                    if sim.data.time >= next_row:
                        index = None
                        if sim.data.time >= next_frame:
                            frames.append(sim.observe(labels=False))
                            index = len(frames)-1
                            next_frame += .2
                        rows.append({"t_sim_s": float(sim.data.time), "stage": "hold_home_calibration",
                            "observation_index": index, "q_command_rad": sim.target_q.tolist(),
                            "q_feedback_rad": sim.data.qpos[sim.qa].tolist(),
                            "gripper_command_m": sim.grip,
                            "applied_force_world_n": sim.data.xfrc_applied[sim.obj, :3].tolist(),
                            "truth_for_scoring_only": {
                                "object_xyz_m": sim.data.qpos[sim.oq:sim.oq+3].tolist(), "contact": contact}})
                        next_row += .05
                audit = sim.environment_audit()
                truth = audit["truth_for_scoring_only"]
                displacement = (np.asarray(truth["settled_displacement_xyz_m"])
                                if candidate["pulse"] is not None else
                                np.asarray(truth["final_xyz_m"])-reset["initial_xyz"])
                case = {"seed": seed, "reset": reset, "environment": audit,
                        "settled_displacement_xyz_m": displacement.tolist(),
                        "forbidden_contact_steps": forbidden_steps,
                        "record_rows": len(rows), "rgbd_frames": len(frames)}
                (directory/"trajectory.jsonl").write_text("".join(json.dumps(row)+"\n" for row in rows))
                np.savez_compressed(directory/"observations.npz",
                    rgb=np.stack([frame["rgb"] for frame in frames]),
                    depth_m=np.stack([frame["depth"] for frame in frames]),
                    camera_json=json.dumps([frame["camera"] for frame in frames]))
                write(directory/"result.json", case)
                cases.append(case)
            values = np.asarray([case["settled_displacement_xyz_m"][1] for case in cases])
            group = {"candidate": candidate, "cases": cases, "summary": {
                "samples": len(cases), "within_10_to_40mm": int(np.sum((values >= .01) & (values <= .04))),
                "median_y_mm": float(np.median(values)*1000),
                "minimum_y_mm": float(values.min()*1000), "maximum_y_mm": float(values.max()*1000),
                "maximum_abs_y_mm": float(np.max(np.abs(values))*1000),
                "forbidden_contact_steps": sum(case["forbidden_contact_steps"] for case in cases)}}
            groups.append(group)
            write(root/"summary.json", {"config": config, "groups": groups, "complete": False})
            print(json.dumps({"candidate": candidate["name"], **group["summary"]}), flush=True)
        selected = min(groups[1:], key=lambda group: (
            -group["summary"]["within_10_to_40mm"],
            abs(group["summary"]["median_y_mm"]-25), group["summary"]["maximum_abs_y_mm"]))
        output = {"config": config, "groups": groups, "complete": True,
                  "selected_candidate": selected["candidate"], "selected_summary": selected["summary"]}
        write(root/"summary.json", output)
        print(json.dumps({"selected_candidate": selected["candidate"], "selected_summary": selected["summary"]}), flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="force-development-01")
    run(parser.parse_args().name)
