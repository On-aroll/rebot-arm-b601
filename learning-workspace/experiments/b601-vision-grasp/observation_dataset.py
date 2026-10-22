"""Synthetic training images at initialized arm poses; not motion demonstrations.

Every view starts a new simulation reset. Only initial robot joints are set for
rendering, before any physics step. Object pose writes remain in sim.reset().
Labels use the training-only volume-filtered segmentation path. No evaluation
scene or independent task score selects samples, labels, poses, or thresholds.
"""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from episode import DOWN
from feedback import DEFAULT_CONFIG
from sim import HOME, ROOT, Simulation, mj
from task import grasp_geometry
from task_runner import create_run, digest, write


V1_PERCEPTION = ROOT/"outputs/vision-iteration-03/perception.pt"


def initial_arm_views(sim, seed):
    # Independent nominal target anchors diversify occlusion. They are not the
    # current object's truth or a control target for a task execution.
    rng = np.random.default_rng(seed+17100000)
    anchor = [float(rng.uniform(.28,.38)), float(rng.uniform(-.105,.105)), .14]
    _, grasp = grasp_geometry(anchor)
    views = [("HOME", HOME.copy(), None)]
    for index, offset in enumerate(DEFAULT_CONFIG["view_offsets_m"]):
        views.append((f"offset_{index}", sim.ik(grasp+offset, DOWN, HOME), anchor))
    return views


def collect_split(sim, seeds, path, base_path):
    rgb, depth, masks, metadata, image_seeds = [], [], [], [], []
    for index, seed in enumerate(seeds):
        absent = index%7 == 0
        # IK uses its separate probe data; this preliminary reset creates no
        # training frame and performs no task action.
        sim.reset(seed, absent=absent)
        views = initial_arm_views(sim, seed)
        for view_name, q, anchor in views:
            meta = sim.reset(seed, absent=absent)
            before_object = sim.data.qpos[sim.oq:sim.oq+7].copy()
            assert sim.data.time == 0.
            sim.data.qpos[sim.qa] = q
            sim.target_q = q.copy()
            sim.data.qvel[:] = 0.
            mj.mj_forward(sim.model, sim.data)
            assert np.array_equal(before_object, sim.data.qpos[sim.oq:sim.oq+7])
            frame = sim.observe(labels=True)
            if absent:
                assert not frame["mask"].any()
            rgb.append(frame["rgb"]); depth.append(frame["depth"]); masks.append(frame["mask"])
            image_seeds.append(seed)
            metadata.append({**meta, "camera":frame["camera"], "arm_initialization_view":view_name,
                "arm_initialization_q_rad":q.tolist(), "independent_nominal_anchor_m":anchor,
                "physics_steps":0, "motion_demonstration":False, "label_pixels":int(frame["mask"].sum())})
        if (index+1)%32 == 0:
            print(json.dumps({"split":path.stem,"new_scenes":index+1,"new_images":len(rgb)}),flush=True)
    new_images = len(rgb)
    with np.load(base_path, allow_pickle=False) as base:
        base_seeds = base["seeds"].tolist()
        assert not set(base_seeds).intersection(seeds)
        rgb = np.concatenate([base["rgb"], np.asarray(rgb)])
        depth = np.concatenate([base["depth_m"], np.asarray(depth, np.float32)])
        masks = np.concatenate([base["mask"], np.asarray(masks)])
        metadata = json.loads(str(base["metadata_json"]))+metadata
        image_seeds = base_seeds+image_seeds
    np.savez_compressed(path, rgb=rgb, depth_m=depth, mask=masks,
        metadata_json=json.dumps(metadata), seeds=np.asarray(image_seeds, np.int64))
    return {"images":len(rgb), "new_images":new_images, "new_scenes":len(seeds),
            "unique_scene_seeds":len(set(image_seeds)), "negative_images":sum(m["absent"] for m in metadata),
            "base_dataset":str(base_path.resolve()), "base_sha256":digest(base_path),
            "output_sha256":digest(path), "path":str(path)}


def main(args):
    if not 32 <= args.train_scenes <= 2048 or not 32 <= args.validation_scenes <= 512:
        raise ValueError("Use bounded training and validation scene counts")
    training = list(range(args.train_seed,args.train_seed+args.train_scenes))
    validation = list(range(args.validation_seed,args.validation_seed+args.validation_scenes))
    if set(training).intersection(validation):
        raise ValueError("Training and validation scene seeds overlap")
    # Explicitly reserve the still-running full-task evaluation blocks.
    heldout = set(range(9600000,9600030))|set(range(9900000,9900030))|set(range(9700000,9700008))|{9800000}
    if heldout.intersection(training+validation):
        raise ValueError("Dataset overlaps frozen full-task evaluation")
    root = create_run(args.name, V1_PERCEPTION)
    shutil.copy2(ROOT/"observation_dataset.py", root/"source/observation_dataset.py")
    provenance = json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"]["observation_dataset.py"] = digest(ROOT/"observation_dataset.py")
    write(root/"provenance.json", provenance)
    config = {k:str(v.resolve()) if isinstance(v,Path) else v for k,v in vars(args).items()}
    config.update(training_seeds=training, validation_seeds=validation,
                  initialization_only=True, motion_demonstrations=False, full_task_test=False,
                  hardware_executable=False, independent_anchor_rng_offset=17100000,
                  render_backend=mj.GLContext.__module__,
                  view_offsets_m=DEFAULT_CONFIG["view_offsets_m"])
    write(root/"config.json", config)
    sim = Simulation(ROOT/"assets/task_scene.xml")
    try:
        train = collect_split(sim, training, root/"training.npz", args.base/"training.npz")
        validate = collect_split(sim, validation, root/"validation.npz", args.base/"validation.npz")
        with np.load(root/"training.npz", allow_pickle=False) as a, np.load(root/"validation.npz", allow_pickle=False) as b:
            assert not set(a["seeds"].tolist()).intersection(b["seeds"].tolist())
        write(root/"summary.json", {"schema":"b601-multiview-training-images-v1", "complete":True,
            "training":train, "validation":validate, "config":config,
            "model_trained":False, "full_task_test":False, "hardware_executable":False})
        print(json.dumps({"complete":True,"training":train,"validation":validate}),flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True)
    parser.add_argument("--train-scenes", type=int, default=256)
    parser.add_argument("--validation-scenes", type=int, default=128)
    parser.add_argument("--train-seed", type=int, default=6300000)
    parser.add_argument("--validation-seed", type=int, default=6400000)
    parser.add_argument("--base", type=Path, default=ROOT/"outputs/vision-iteration-03")
    main(parser.parse_args())
