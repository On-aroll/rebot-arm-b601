"""Fresh common-seed comparison and negative controls, independent of training."""
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
import torch

from sim import ROOT, Simulation, deproject
from episode import run_episode
from vision import TargetNet, estimate


def load_model(path):
    model = TargetNet()
    model.load_state_dict(torch.load(path, map_location="cpu", weights_only=True)["state_dict"])
    return model.eval()


def verify():
    root = ROOT / "outputs/common-seed-verification"
    root.mkdir(parents=True, exist_ok=False)
    (root / "source").mkdir()
    for name in ("sim.py", "episode.py", "vision.py", "verify.py"):
        shutil.copy2(ROOT / name, root / "source" / name)
    sim = Simulation()
    results = {"schema":"b601-common-seed-verification-v1", "hardware_executable":False,
               "common_seeds":list(range(40000,40008)), "models":[], "negative_controls":[]}
    try:
        # Camera math is checked against geometric truth without learned pixels.
        sim.reset(50000)
        xyz = sim.data.xpos[sim.obj].copy(); xyz[2] += sim.half_size
        info = sim.camera_info(); t = np.asarray(info["T_base_camera_optical"])
        cam = np.linalg.inv(t) @ np.r_[xyz,1.]
        uvw = np.asarray(info["K"]) @ cam[:3]
        back = deproject(*(uvw[:2]/uvw[2]),cam[2],info)
        assert np.allclose(back,xyz,atol=1e-10)
        results["camera_roundtrip_error_m"] = float(np.linalg.norm(back-xyz))
        # Asset snapshot must still match the source hashes captured at creation.
        manifest = json.loads((ROOT/"assets/manifest.json").read_text())
        assert all(hashlib.sha256((ROOT/"assets"/name).read_bytes()).hexdigest()==sha
                   for name,sha in manifest["files_sha256"].items())
        results["asset_files_verified"] = len(manifest["files_sha256"])
        for round_id in (1,2):
            path = ROOT/f"outputs/first-training/round-{round_id:02d}/perception.pt"
            model = load_model(path)
            entry = {"round":round_id, "checkpoint_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),"episodes":[]}
            for seed in results["common_seeds"]:
                sim.reset(seed)
                prediction = estimate(model,sim.observe(labels=False))
                result = run_episode(sim,seed,prediction,root/f"round-{round_id}-seed-{seed}",video=round_id==2 and seed==40000)
                entry["episodes"].append(result)
                print(json.dumps({"round":round_id,"seed":seed,"success":result["success"],"reason":result["reason"]}),flush=True)
            entry["successes"] = sum(x["success"] for x in entry["episodes"])
            results["models"].append(entry)
            (root/"summary.json").write_text(json.dumps(results,indent=2)+"\n")
        for seed in range(50000,50008):
            sim.reset(seed,absent=True)
            predicted = estimate(model,sim.observe(labels=False))
            result = run_episode(sim,seed,predicted,root/f"empty-{seed}",absent=True)
            assert not result["success"] and not result.get("plan"), "Empty scene commanded motion"
            results["negative_controls"].append({"case":"empty_scene","seed":seed,"reason":result["reason"],"motion_rows":result["record_rows"]})
        # Deliberately offset the estimated grasp by 65mm on a real rendered cube.
        sim.reset(40000)
        wrong = estimate(model,sim.observe(labels=False))
        assert wrong is not None
        wrong["top_xyz_m"][1] += -.065 if wrong["top_xyz_m"][1] > 0 else .065
        result = run_episode(sim,40000,wrong,root/"wrong-visual-position",video=True)
        assert not result["success"], "Incorrect visual target spuriously succeeded"
        results["negative_controls"].append({"case":"wrong_visual_position","reason":result["reason"],"success":result["success"]})
        # Audit actual exported trajectories and 5Hz observation pairing.
        paired, rows_checked, max_command_speed = 0,0,0.
        for path in root.glob("*/trajectory.jsonl"):
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            archive = path.parent/"observations.npz"
            if not rows:
                continue
            data = np.load(archive,allow_pickle=False)
            cameras = json.loads(str(data["camera_json"]))
            previous = None
            for row in rows:
                q = np.asarray(row["q_command_rad"])
                assert np.isfinite(q).all() and (q>=sim.lower-1e-7).all() and (q<=sim.upper+1e-7).all()
                assert not row["finger_contact"]["forbidden"]
                if previous:
                    speed = np.max(np.abs(q-np.asarray(previous["q_command_rad"]))) / (row["t_sim_s"]-previous["t_sim_s"])
                    max_command_speed = max(max_command_speed,float(speed))
                    assert speed <= .401
                index = row["observation_index"]
                if index is not None:
                    assert abs(cameras[index]["simulation_time_s"]-row["t_sim_s"])<1e-9
                    paired += 1
                previous = row
                rows_checked += 1
            assert sum(x["observation_index"] is not None for x in rows)==len(data["rgb"])
            data.close()
        results.update(paired_rgbd_frames=paired, trajectory_rows_checked=rows_checked,
                       maximum_exported_command_speed_rad_s=max_command_speed)
        (root/"summary.json").write_text(json.dumps(results,indent=2)+"\n")
        print(json.dumps({"models":[{k:v for k,v in x.items() if k!='episodes'} for x in results['models']],
                          "negative_controls":results["negative_controls"],"paired_rgbd_frames":paired,"rows":rows_checked}),flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    verify()
