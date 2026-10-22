"""Development-only physical observation-pose probe; no task success claims."""
import json

import numpy as np
from PIL import Image

from episode import DOWN
from sim import ROOT, HOME, Simulation
from task import Recorder, visual_target
from task_runner import create_run, perception_model, write


def main():
    root = create_run("viewpoint-development-01", ROOT/"outputs/vision-iteration-03/perception.pt")
    model = perception_model(ROOT/"outputs/vision-iteration-03/perception.pt")
    sim = Simulation(ROOT/"assets/task_scene.xml")
    rows = []
    # Old V0 tests are development data now. No fresh V1 test scene is used.
    try:
        for offset in ([-.08, 0., .15], [-.06, .06, .12]):
            for seed in (9000000, 9000001, 9000002, 9000003, 9000005, 9000006):
                sim.reset(seed)
                p = visual_target(model, sim.observe(labels=False))
                row = {"seed": seed, "tcp_offset_from_estimated_grasp_m": offset,
                       "initial_detection": p is not None}
                if not p:
                    rows.append(row)
                    continue
                directory = root/f"case-{len(rows):02d}"
                directory.mkdir()
                recorder = Recorder(sim, directory, False)
                top = np.asarray(p["top_xyz_m"])
                center = top.copy(); center[2] = (.1+top[2])/2
                target = center+DOWN @ np.array([.015, 0., 0.])+[0., 0., .004]+offset
                try:
                    q = sim.ik(target, DOWN, HOME)
                    recorder.move(q, .046, 2., "observation_vantage")
                    recorder.move(q, .046, .3, "observe_hold")
                    frame = sim.observe(labels=False)
                    prediction = visual_target(model, frame)
                    row.update(visible_at_vantage=prediction is not None, prediction=prediction,
                               tcp_feedback_xyz_m=sim.data.site_xpos[sim.tip].tolist())
                    Image.fromarray(frame["rgb"]).save(directory/"observation.png")
                    Image.fromarray(sim.overview()).save(directory/"overview.png")
                except (ValueError, RuntimeError) as exc:
                    row["error"] = str(exc)
                recorder.save()
                row["forbidden_contact_samples"] = sum(bool(r["truth_for_scoring_only"]["contact"]["forbidden"])
                                                        for r in recorder.rows)
                rows.append(row)
                write(root/"summary.json", {"development_only":True, "full_task_test":False, "cases":rows})
                print(json.dumps({k:v for k,v in row.items() if k not in ("prediction", "tcp_feedback_xyz_m")}), flush=True)
    finally:
        sim.close()


if __name__ == "__main__":
    main()
