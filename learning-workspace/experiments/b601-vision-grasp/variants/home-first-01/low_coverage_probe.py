"""Physical development tests of already-detected, quality-rejected images.

Reuses vision validation scenes as development data, never as new held-out tests.
Robot joints are initialized at time zero to the image's known robot pose.
Object pose is randomized only by Simulation.reset(), never changed afterward.
Actions use RGB-D + the frozen residual head; oracle labels only score outcomes.
"""
import argparse
import json
import os
from pathlib import Path
import shutil

if os.environ.get("MUJOCO_GL") not in ("egl","glfw"):
    raise RuntimeError("Set MUJOCO_GL=egl for legacy or glfw for new source images before Python starts")

import mujoco as mj
import numpy as np

from feedback_audit import same
from policy import load
from sim import HOME, ROOT, Simulation
from task import GOALS, run_task
from task_runner import create_run, digest, perception_model, summary, write


class InitializedArmSimulation(Simulation):
    initial_robot_q = None

    def reset(self, seed, **kwargs):
        metadata = super().reset(seed, **kwargs)
        q = self.initial_robot_q
        if q is not None:
            assert self.data.time == 0.
            object_before = self.data.qpos[self.oq:self.oq+7].copy()
            assert (q >= self.lower).all() and (q <= self.upper).all()
            self.data.qpos[self.qa] = q
            self.target_q = q.copy()
            self.data.qvel[:] = 0.
            mj.mj_forward(self.model,self.data)
            assert np.array_equal(self.data.qpos[self.oq:self.oq+7],object_before)
            metadata["robot_initialization_only_q_rad"] = q.tolist()
            metadata["object_qpos_unchanged_by_robot_initialization"] = True
        return metadata


def main(args):
    source=ROOT/"outputs/observation-images-01"
    audit_path=source/"data_and_candidate01_audit.json"
    image_audit=json.loads(audit_path.read_text())
    detector=ROOT/"outputs/vision-multiview-01/perception.pt"
    head=ROOT/"outputs/goal-policy-multiview-01/residual.pt"
    assert digest(detector)==image_audit["checkpoint_sha256"]
    assert json.loads((head.parent/"summary.json").read_text())["perception_sha256"]==digest(detector)
    with np.load(source/"validation.npz",allow_pickle=False) as data:
        metadata=json.loads(str(data["metadata_json"]))
    source_view="legacy_HOME" if args.source_domain=="legacy" else None
    expected_backend="egl" if args.source_domain=="legacy" else "glfw"
    assert os.environ["MUJOCO_GL"]==expected_backend
    cases=[(i,r) for i,r in enumerate(image_audit["validation_rows"])
           if r["expected_target"] and r["accepted_detection"] and not r["gate"]["accepted"]
           and ((r["view"]==source_view) if source_view else r["view"]!="legacy_HOME")]
    assert cases
    root=create_run(args.name,detector)
    shutil.copy2(ROOT/"low_coverage_probe.py",root/"source/low_coverage_probe.py")
    provenance=json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"]["low_coverage_probe.py"]=digest(ROOT/"low_coverage_probe.py")
    write(root/"provenance.json",provenance)
    frozen=root/"frozen";frozen.mkdir()
    shutil.copy2(detector,frozen/"perception.pt");shutil.copy2(head,frozen/"residual.pt")
    config={"schema":"b601-rejected-image-physical-development-v1","domain":"simulation","hardware_executable":False,
            "source_image_audit":str(audit_path),"source_image_audit_sha256":digest(audit_path),
            "validation_images":str(source/"validation.npz"),"validation_images_sha256":digest(source/"validation.npz"),
            "perception_sha256":digest(frozen/"perception.pt"),"policy_sha256":digest(frozen/"residual.pt"),
            "seeds":[r["seed"] for _,r in cases],"image_indices":[i for i,_ in cases],
            "selection":"All positive, detected, quality-rejected images in existing image validation; no outcome-based filtering",
            "development_only":True,"max_retries":0,"observation_mode":"none",
            "source_domain":args.source_domain,"requested_backend":os.environ["MUJOCO_GL"],
            "actual_gl_context_module":mj.GLContext.__module__,
            "purpose":"Determine whether low mask coverage predicts physical task failure; not adoption evidence"}
    write(root/"config.json",config)
    model,policy=perception_model(frozen/"perception.pt"),load(frozen/"residual.pt")
    sim=InitializedArmSimulation(ROOT/"assets/task_scene.xml")
    episodes=[]
    try:
        for index,(image_index,row) in enumerate(cases):
            meta=metadata[image_index]
            sim.initial_robot_q=np.asarray(meta.get("arm_initialization_q_rad",HOME),float)
            relative=f"image-{image_index:04d}"
            e=run_task(sim,model,policy,row["seed"],list(GOALS)[index%3],root/relative,
                       max_retries=0,observation_mode="none",video=index<2)
            same(e["attempts"][0]["raw_prediction"],row["prediction"])
            episodes.append({**e,"artifact_directory":relative,"source_image_index":image_index,
                             "source_view":row["view"],"source_quality_gate":row["gate"],
                             "source_raw_error_mm":row["error_mm"]})
            write(root/"summary.json",{**config,"complete":False,"episodes":episodes,"summary":summary(episodes)})
            print(json.dumps({"image":image_index,"seed":row["seed"],"view":row["view"],"coverage":row["gate"]["coverage"],
                              "success":e["success"],"physical_success":e["physical_success"],"reason":e["reason"],
                              "first_outcome":e["attempts"][0].get("outcome")}),flush=True)
        write(root/"summary.json",{**config,"complete":True,"episodes":episodes,"summary":summary(episodes)})
        print(json.dumps({"complete":True,"summary":summary(episodes)}),flush=True)
    finally:
        sim.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    parser.add_argument("--source-domain",choices=["legacy","new"],required=True)
    main(parser.parse_args())
