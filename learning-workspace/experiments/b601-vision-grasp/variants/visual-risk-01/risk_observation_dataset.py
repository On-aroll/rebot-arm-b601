"""Collect and exactly audit independently initialized risk-development images.

The view sampler reads only an independent RNG and robot kinematics. No model
inference, object-truth selection, physics steps or control changes occur.
"""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from episode import DOWN
from feedback import DEFAULT_CONFIG
from feedback_runner import prior_seed_ledger
from sim import HOME, ROOT, Simulation, mj
from task import grasp_geometry
from task_runner import create_run, digest, write


SPLITS = ("training", "calibration", "validation")
EXTRA = ("risk_observation_dataset.py", "audit_frozen.py", "BRANCH_BASE.json")
MODEL_SHA = {"perception": "e911a373325072eb9525031fa57f4270a6f8b6f853a5c6d370eb5ffd5d17bd37",
             "residual": "9307762e59d1a2c82bde7add620ac815a8071dffda087413798c4fb8d7fe4ed2"}


def views(sim, seed):
    rng = np.random.default_rng(seed+23100000)
    anchor = [float(rng.uniform(.28, .38)), float(rng.uniform(-.105, .105)), .14]
    _, grasp = grasp_geometry(anchor)
    offsets = [(f"original_{i}", np.asarray(offset)) for i, offset in enumerate(DEFAULT_CONFIG["view_offsets_m"])]
    offsets += [(f"random_{i}", np.array([rng.uniform(-.08, .04), rng.uniform(-.07, .07),
                                         rng.uniform(.035, .12)])) for i in range(4)]
    result = [{"view": "HOME", "q": HOME.tolist(), "anchor_m": None, "offset_m": None,
               "requested_tcp_m": None, "IK_error": None}]
    for name, offset in offsets:
        target = grasp+offset
        try:
            q, error = sim.ik(target, DOWN, HOME).tolist(), None
        except ValueError as exc:
            q, error = None, str(exc)
        result.append({"view": name, "q": q, "anchor_m": anchor, "offset_m": offset.tolist(),
                       "requested_tcp_m": target.tolist(), "IK_error": error})
    assert len(result) == 7 and sim.data.time == 0.
    return result


def initialize(sim, seed, view):
    meta = sim.reset(seed, absent=seed % 7 == 0)
    before = sim.data.qpos[sim.oq:sim.oq+7].copy()
    q = np.asarray(view["q"])
    assert q.shape == (6,) and (q >= sim.lower).all() and (q <= sim.upper).all()
    sim.data.qpos[sim.qa] = q
    sim.target_q = q.copy()
    sim.data.qvel[:] = 0.
    mj.mj_forward(sim.model, sim.data)
    assert sim.data.time == 0. and np.array_equal(before, sim.data.qpos[sim.oq:sim.oq+7])
    frame = sim.observe(labels=True)
    top = sim.data.xpos[sim.obj].copy(); top[2] += sim.half_size
    result = {**meta, **view, "camera": frame["camera"], "physics_steps": 0,
              "motion_demonstration": False, "object_qpos_unchanged": True,
              "truth_for_training_only": {"top_xyz_m": top.tolist(), "initial_contact": sim.contacts()},
              "label_pixels": int(frame["mask"].sum())}
    if meta["absent"]:
        assert not frame["mask"].any()
    return frame, result


def collect(sim, seeds, path):
    rgb, depth, mask, metadata, image_seeds, failed = [], [], [], [], [], []
    for i, seed in enumerate(seeds):
        sim.reset(seed, absent=seed % 7 == 0)
        for view in views(sim, seed):
            if view["q"] is None:
                failed.append({"seed": seed, **view})
                continue
            frame, meta = initialize(sim, seed, view)
            rgb.append(frame["rgb"]); depth.append(frame["depth"]); mask.append(frame["mask"])
            metadata.append(meta); image_seeds.append(seed)
        if (i+1) % 32 == 0:
            print(json.dumps({"split": path.stem, "scenes": i+1, "images": len(rgb), "failed_IK_views": len(failed)}), flush=True)
    assert len(rgb)+len(failed) == len(seeds)*7
    np.savez_compressed(path, rgb=np.asarray(rgb), depth_m=np.asarray(depth, np.float32),
                        mask=np.asarray(mask), seeds=np.asarray(image_seeds, np.int64),
                        metadata_json=json.dumps(metadata))
    return {"planned_scenes": len(seeds), "planned_view_attempts": len(seeds)*7, "images": len(rgb),
            "empty_images": sum(m["absent"] for m in metadata), "failed_IK_views": failed,
            "positive_images_with_no_visible_label": sum(not m["absent"] and not m["label_pixels"] for m in metadata),
            "output_sha256": digest(path)}


def main(args):
    if not all(1 <= getattr(args, split+"_scenes") <= 2048 for split in SPLITS):
        raise ValueError("Use bounded scene counts per split")
    seeds = {split: list(range(getattr(args, split+"_seed"), getattr(args, split+"_seed")+getattr(args, split+"_scenes"))) for split in SPLITS}
    flat = [s for block in seeds.values() for s in block]
    assert len(flat) == len(set(flat))
    canonical = (ROOT/"outputs").resolve().parent
    paths = {"perception": canonical/"versions/V3/models/perception.pt",
             "residual": ROOT/"outputs/image-residual-development-01/residual.pt"}
    assert all(digest(path) == MODEL_SHA[name] for name, path in paths.items())
    root = create_run(args.name, paths["perception"])
    provenance = json.loads((root/"provenance.json").read_text())
    for name in EXTRA:
        shutil.copy2(ROOT/name, root/"source"/name)
        provenance["source_sha256"][name] = digest(ROOT/name)
    write(root/"provenance.json", provenance)
    ledger = prior_seed_ledger(root, args.concurrent_frozen_run)
    assert not set(flat).intersection(ledger["seeds"]), "New dataset overlaps reserved/development/training scenes"
    write(root/"prior_seed_ledger.json", ledger)
    shutil.copy2(args.protocol, root/"protocol.md")
    (root/"frozen").mkdir()
    for name, path in paths.items():
        shutil.copy2(path, root/"frozen"/(name+".pt"))
    config = {"schema": "b601-risk-observation-data-v1", "hardware_executable": False,
              "seed_blocks": seeds, "model_sha256": MODEL_SHA, "protocol_sha256": digest(root/"protocol.md"),
              "prior_seed_ledger_sha256": digest(root/"prior_seed_ledger.json"),
              "initialization_only": True, "physics_steps": 0, "view_attempts_per_scene": 7,
              "render_backend": mj.GLContext.__module__, "model_inference_during_collection": False}
    write(root/"config.json", config)
    sim = Simulation(ROOT/"assets/task_scene.xml")
    try:
        reports = {split: collect(sim, seeds[split], root/(split+".npz")) for split in SPLITS}
    finally:
        sim.close()
    write(root/"summary.json", {"complete": True, "config_sha256": digest(root/"config.json"),
        "splits": reports, "hardware_executable": False, "model_trained": False,
        "scope": "Initialized supervised images; not motion demonstrations, control executions or full-task tests."})
    print(json.dumps({"complete": True, "images": sum(r["images"] for r in reports.values()),
                      "failed_IK_views": sum(len(r["failed_IK_views"]) for r in reports.values())}), flush=True)


def audit(run):
    root = (ROOT/"outputs"/run).resolve()
    provenance = json.loads((root/"provenance.json").read_text())
    for name, sha in provenance["source_sha256"].items():
        assert digest(ROOT/name) == sha == digest(root/"source"/Path(name).name), name
    assets = json.loads((ROOT/"assets/manifest.json").read_text())["files_sha256"]
    assert all(digest(ROOT/"assets"/name) == sha for name, sha in assets.items())
    config = json.loads((root/"config.json").read_text())
    summary = json.loads((root/"summary.json").read_text())
    assert summary["complete"] and digest(root/"config.json") == summary["config_sha256"]
    assert digest(root/"protocol.md") == config["protocol_sha256"]
    assert digest(root/"prior_seed_ledger.json") == config["prior_seed_ledger_sha256"]
    ledger = json.loads((root/"prior_seed_ledger.json").read_text())
    assert all(digest(Path(source["path"])) == source["sha256"] for source in ledger["sources"])
    seeds = [s for block in config["seed_blocks"].values() for s in block]
    assert len(seeds) == len(set(seeds)) and not set(seeds).intersection(ledger["seeds"])
    assert all(digest(root/"frozen"/(name+".pt")) == sha for name, sha in config["model_sha256"].items())
    sim, metrics = Simulation(ROOT/"assets/task_scene.xml"), {}
    try:
        for split in SPLITS:
            path = root/(split+".npz"); expected = summary["splits"][split]
            assert digest(path) == expected["output_sha256"]
            with np.load(path, allow_pickle=False) as data:
                rgb, depth, masks = data["rgb"], data["depth_m"], data["mask"]
                metadata, image_seeds = json.loads(str(data["metadata_json"])), data["seeds"].tolist()
                assert len(metadata) == len(rgb) == len(depth) == len(masks) == len(image_seeds) == expected["images"]
                assert rgb.shape[1:] == (120, 160, 3) and depth.shape[1:] == masks.shape[1:] == (120, 160)
                assert np.isfinite(depth).all()
                count, failures, pixels, contacts = 0, [], 0, 0
                for seed in config["seed_blocks"][split]:
                    sim.reset(seed, absent=seed % 7 == 0)
                    for view in views(sim, seed):
                        if view["q"] is None:
                            failures.append({"seed": seed, **view}); continue
                        frame, meta = initialize(sim, seed, view)
                        assert meta == metadata[count] and seed == image_seeds[count]
                        assert np.array_equal(frame["rgb"], rgb[count])
                        assert np.array_equal(frame["depth"], depth[count])
                        assert np.array_equal(frame["mask"], masks[count])
                        contacts += bool(meta["truth_for_training_only"]["initial_contact"]["forbidden"])
                        v, u = np.where(masks[count])
                        if len(u):
                            k, t = np.asarray(meta["camera"]["K"]), np.asarray(meta["camera"]["T_base_camera_optical"])
                            points = np.linalg.solve(k, np.stack([u, v, np.ones(len(u))]))*depth[count, v, u]
                            world = (t[:3, :3]@points).T+t[:3, 3]
                            angle = meta["yaw_rad"]
                            rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
                            assert (np.abs((world-meta["initial_xyz"])@rotation) <= meta["half_size_m"]+.0020001).all()
                            pixels += len(u)
                        count += 1
                    if (count+len(failures)) % (32*7) == 0:
                        print(json.dumps({"audit_split": split, "images_reexecuted": count}), flush=True)
                assert count == len(rgb) and failures == expected["failed_IK_views"]
                assert count+len(failures) == expected["planned_view_attempts"]
                metrics[split] = {"images_reexecuted": count, "failed_IK_views_reexecuted": len(failures),
                                  "independent_volume_pixels_checked": pixels, "initialized_images_with_forbidden_contact": contacts}
    finally:
        sim.close()
    report = {"passed": True, "splits": metrics, "source_files_checked": len(provenance["source_sha256"]),
              "asset_files_checked": len(assets), "prior_seed_overlap": False, "physics_steps": 0,
              "scope": "Exact initialized RGB/depth/mask/RNG/IK re-execution, metadata/seed/source/data/model/asset hashes and independent mask volume check. Initial-contact images are retained supervised images, not accepted physical trajectories."}
    write(root/"audit.json", report)
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--name"); mode.add_argument("--audit-run")
    for split, start, count in zip(SPLITS, (31000000, 31100000, 31200000), (512, 256, 256)):
        parser.add_argument("--"+split+"-seed", type=int, default=start)
        parser.add_argument("--"+split+"-scenes", type=int, default=count)
    parser.add_argument("--protocol", type=Path)
    parser.add_argument("--concurrent-frozen-run", type=Path)
    args = parser.parse_args()
    if args.audit_run:
        audit(args.audit_run)
    else:
        if args.protocol is None:
            parser.error("--protocol is required for collection")
        main(args)
