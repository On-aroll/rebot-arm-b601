"""Re-render every newly generated multiview image with explicit EGL."""
import argparse
import json
import os
from pathlib import Path
import shutil

import numpy as np

from sim import HOME, ROOT, Simulation, mj
from task_runner import digest, write


def audit(run):
    assert os.environ.get("MUJOCO_GL")=="egl" and mj.GLContext.__module__=="mujoco.egl"
    root=ROOT/"outputs"/run
    summary=json.loads((root/"summary.json").read_text())
    sim=Simulation(ROOT/"assets/task_scene.xml")
    counts={}
    try:
        for split in ("training","validation"):
            f=root/f"{split}.npz"
            assert digest(f)==summary[split]["output_sha256"]
            with np.load(f,allow_pickle=False) as data:
                rgb,depth=data["rgb"],data["depth_m"]
                meta=json.loads(str(data["metadata_json"]))
                indices=[i for i,m in enumerate(meta) if "arm_initialization_q_rad" in m]
                for i in indices:
                    m=meta[i]
                    sim.reset(m["seed"],absent=m["absent"])
                    before=sim.data.qpos[sim.oq:sim.oq+7].copy()
                    q=np.asarray(m["arm_initialization_q_rad"])
                    sim.data.qpos[sim.qa]=q;sim.target_q=q.copy();sim.data.qvel[:]=0.
                    mj.mj_forward(sim.model,sim.data)
                    assert sim.data.time==0. and np.array_equal(before,sim.data.qpos[sim.oq:sim.oq+7])
                    frame=sim.observe(labels=False)
                    assert np.array_equal(frame["rgb"],rgb[i]),(split,i,"RGB reproduction mismatch")
                    assert np.array_equal(frame["depth"],depth[i]),(split,i,"depth reproduction mismatch")
                counts[split]=len(indices)
        shutil.copy2(ROOT/"render_backend_audit.py",root/"render_backend_audit_source.py")
        report={"passed":True,"actual_gl_context_module":mj.GLContext.__module__,
                "images_reproduced_pixel_exactly":counts,"zero_object_pose_writes_after_reset":True,
                "audit_source_sha256":digest(root/"render_backend_audit_source.py"),
                "scope":"All newly generated RGB/depth images reproduced from stored seed and robot initialization under EGL; not physical task testing"}
        write(root/"render_backend_audit.json",report)
        print(json.dumps(report),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",required=True)
    audit(parser.parse_args().run)
