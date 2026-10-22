"""Package unchanged segmentation weights with a development-selected decoder.

This is a postprocessing candidate, not new weight training or adoption.
"""
import argparse
import json
from pathlib import Path
import shutil

import torch

from sim import ROOT
from task_runner import create_run, digest, write
from vision import inference_configuration


def main(args):
    parent=args.parent.resolve()
    config=inference_configuration(json.loads(args.config.read_text()))
    evidence=[ROOT/"outputs/mask-gate-development-01/summary.json",
              ROOT/"outputs/negative-mask-development-01/summary.json"]
    sweep,negative=[json.loads(p.read_text()) for p in evidence]
    key=f"p{config['probability_threshold']}-n{config['min_pixels']}"
    assert sweep["complete"] and negative["complete"]
    assert sweep["checkpoint_sha256"]==negative["checkpoint_sha256"]==digest(parent)
    assert negative["metrics"][key]["false_positive_images"]==0
    selected=sweep["groups"][key];baseline=sweep["groups"]["p0.7-n25"]
    assert selected["miss_or_wrong_above_10mm"]<baseline["miss_or_wrong_above_10mm"]
    assert selected["false_positives"]<=baseline["false_positives"]
    assert selected["wrong_localization_above_10mm"]<=baseline["wrong_localization_above_10mm"]
    root=create_run(args.name,parent)
    shutil.copy2(Path(__file__),root/"source/package_mask_candidate.py")
    provenance=json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"]["package_mask_candidate.py"]=digest(Path(__file__))
    write(root/"provenance.json",provenance)
    checkpoint=torch.load(parent,map_location="cpu",weights_only=True)
    checkpoint.update(inference_config=config,weights_retrained=False,
        derived_from_checkpoint=str(parent),derived_from_checkpoint_sha256=digest(parent),
        candidate_method="unchanged segmentation tensor weights; development-selected probability postprocessing")
    torch.save(checkpoint,root/"perception.pt")
    saved=torch.load(root/"perception.pt",map_location="cpu",weights_only=True)
    original=torch.load(parent,map_location="cpu",weights_only=True)
    assert saved["state_dict"].keys()==original["state_dict"].keys()
    assert all(torch.equal(saved["state_dict"][k],v) for k,v in original["state_dict"].items())
    write(root/"config.json",{"inference_config":config,"parent":str(parent),"hardware_executable":False})
    report={"development_only":True,"hardware_executable":False,"weights_retrained":False,
        "all_segmentation_tensors_identical_to_parent":True,"parent_checkpoint_sha256":digest(parent),
        "checkpoint_sha256":digest(root/"perception.pt"),"inference_config":config,
        "selection_evidence":[{"path":str(p.resolve()),"sha256":digest(p)} for p in evidence],
        "interpretation":"Postprocessing only; requires matched residual retraining, known-failure regression, then fresh full-task evaluation."}
    write(root/"summary.json",report);print(json.dumps(report),flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name",required=True)
    p.add_argument("--parent",type=Path,default=ROOT/"outputs/vision-multiview-egl-01/perception.pt")
    p.add_argument("--config",type=Path,default=ROOT/"configs/vision-fragment-development-01.json")
    main(p.parse_args())
