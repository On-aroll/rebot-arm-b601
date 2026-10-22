"""Check old decoder behavior and exact packaged geometry-probe agreement."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil

import numpy as np
import torch

from feedback_audit import same
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import estimate


def main(args):
    canonical=(ROOT/"outputs").resolve().parent
    runtime=canonical/"versions/V2"
    manifest=json.loads((runtime/"release_manifest.json").read_text())
    assert all(digest(runtime/p)==sha for p,sha in manifest["files_sha256"].items())
    specification=importlib.util.spec_from_file_location("saved_v2_vision",runtime/"vision.py")
    legacy=importlib.util.module_from_spec(specification);specification.loader.exec_module(legacy)
    model_paths={"v2":runtime/"models/perception.pt",
                 "v1":canonical/"outputs/vision-iteration-03/perception.pt",
                 "geometry055":(ROOT/"outputs/vision-geometry055-01/perception.pt").resolve(),
                 "geometry050":(ROOT/"outputs/vision-geometry050-01/perception.pt").resolve()}
    validation=(ROOT/"outputs/observation-images-egl-01/validation.npz").resolve()
    frozen_task=(ROOT/"outputs/goal-fragment-home-v2-candidate-01").resolve()
    decisions=sorted(frozen_task.glob("*/decision_observations.npz"))
    probe=(ROOT/"outputs/component-geometry-development-01").resolve()
    expected=json.loads((probe/"summary.json").read_text())
    root=create_run(args.name,model_paths["v2"])
    provenance=json.loads((root/"provenance.json").read_text())
    shutil.copy2(Path(__file__),root/"source/geometry_decoder_parity.py")
    provenance["source_sha256"]["geometry_decoder_parity.py"]=digest(Path(__file__))
    write(root/"provenance.json",provenance)
    plan={"development_only":True,"hardware_executable":False,
        "models":{k:{"path":str(p),"sha256":digest(p)} for k,p in model_paths.items()},
        "legacy_vision":{"path":str(runtime/"vision.py"),"sha256":digest(runtime/"vision.py")},
        "validation_sha256":digest(validation),"geometry_probe_summary_sha256":digest(probe/"summary.json"),
        "task_decision_files":[{"path":str(p),"sha256":digest(p)} for p in decisions]}
    write(root/"config.json",plan)
    loaded={k:perception_model(p) for k,p in model_paths.items()}
    reference={}
    for key in ("v2","v1"):
        checkpoint=torch.load(model_paths[key],map_location="cpu",weights_only=True)
        model=legacy.TargetNet();model.load_state_dict(checkpoint["state_dict"])
        model.inference_config=checkpoint.get("inference_config")
        reference[key]=model.eval()
    for key in ("geometry055","geometry050"):
        assert all(torch.equal(v,loaded[key].state_dict()[name]) for name,v in loaded["v2"].state_dict().items())
    counts={"v2_validation":0,"v1_validation":0,"v2_task_decisions":0,
            "geometry_dataset_images":0,"geometry_known_decisions":0}
    for split,source in expected["datasets"].items():
        assert digest(source["path"])==source["sha256"]
        with np.load(source["path"],allow_pickle=False) as data:
            rgb,depth=data["rgb"],data["depth_m"]
            metadata=json.loads(str(data["metadata_json"]))
            for i,meta in enumerate(metadata):
                frame={"rgb":rgb[i],"depth":depth[i],"camera":meta["camera"]}
                if split=="validation":
                    for key in ("v1","v2"):
                        same(estimate(loaded[key],frame),legacy.estimate(reference[key],frame))
                        counts[key+"_validation"]+=1
                for key,variant in (("geometry055","geometry_mask_0.55"),("geometry050","geometry_mask_0.5")):
                    same(estimate(loaded[key],frame),expected["rows"][split][variant][i]["prediction"])
                counts["geometry_dataset_images"]+=1
    for path in decisions:
        with np.load(path,allow_pickle=False) as data:
            rgb,depth=data["rgb"],data["depth_m"]
            cameras=json.loads(str(data["camera_json"]))
            for image,d,camera in zip(rgb,depth,cameras,strict=True):
                frame={"rgb":image,"depth":d,"camera":camera}
                same(estimate(loaded["v2"],frame),legacy.estimate(reference["v2"],frame))
                counts["v2_task_decisions"]+=1
    assert counts["v2_task_decisions"]==json.loads((frozen_task/"audit.json").read_text())["sums"]["decision_frames"]
    for known in expected["known_cases"]:
        assert digest(known["posthoc_source"])==known["source_sha256"]
        with np.load(known["posthoc_source"],allow_pickle=False) as data:
            camera=json.loads(str(data["camera_json"]))[0]
            frame={"rgb":data["rgb"][0],"depth":data["depth_m"][0],"camera":camera}
        for key,variant in (("geometry055","geometry_mask_0.55"),("geometry050","geometry_mask_0.5")):
            same(estimate(loaded[key],frame),known["predictions"][variant])
        counts["geometry_known_decisions"]+=1
    report={"passed":True,**counts,"all_segmentation_weights_identical":True,
            "config_sha256":digest(root/"config.json"),
            "scope":"Old V1/V2 inference parity and packaged decoder agreement with audited image probe; not physical tasks."}
    write(root/"summary.json",report);print(json.dumps(report),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    main(parser.parse_args())
