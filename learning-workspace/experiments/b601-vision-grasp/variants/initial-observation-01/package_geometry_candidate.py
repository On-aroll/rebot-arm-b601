"""Package a declared geometry decoder, preserving all segmentation tensors."""
import argparse
import json
from pathlib import Path
import shutil

import torch

from sim import ROOT
from task_runner import create_run, digest, write
from vision import GEOMETRY_WORKSPACE_M, inference_configuration


def main(args):
    canonical=(ROOT/"outputs").resolve().parent
    parent=canonical/"versions/V2/models/perception.pt"
    evidence=ROOT/"outputs/component-geometry-development-01"
    assert json.loads((evidence/"audit.json").read_text())["passed"]
    root=create_run(args.name,parent)
    provenance=json.loads((root/"provenance.json").read_text())
    shutil.copy2(Path(__file__),root/"source/package_geometry_candidate.py")
    provenance["source_sha256"]["package_geometry_candidate.py"]=digest(Path(__file__))
    write(root/"provenance.json",provenance)
    proposed={"probability_threshold":args.threshold,"min_pixels":25,"geometry_mask":True}
    evidence_items=[evidence/name for name in ("config.json","summary.json","audit.json")]
    if args.geometry_fallback:
        proposed["geometry_fallback"]=True
        known=ROOT/"outputs/geometry-known-development-01"
        assert json.loads((known/"audit.json").read_text())["passed"]
        evidence_items += [known/name for name in ("config.json","summary.json","audit.json")]
        evidence_items += [ROOT/"outputs/geometry-policy-images-development-01/summary.json"]
    config=inference_configuration(proposed)
    plan={"development_only":True,"hardware_executable":False,"weights_retrained":False,
        "parent":str(parent),"parent_sha256":digest(parent),"inference_config":config,
        "geometry_workspace_m":GEOMETRY_WORKSPACE_M,
        "selection_evidence":[{"path":str(path.resolve()),"sha256":digest(path)} for path in evidence_items]}
    write(root/"config.json",plan)
    checkpoint=torch.load(parent,map_location="cpu",weights_only=True)
    checkpoint.update(inference_config=config,inference_geometry_workspace_m=GEOMETRY_WORKSPACE_M,
                      parent_checkpoint_sha256=digest(parent),weights_retrained=False,
                      selection_evidence=plan["selection_evidence"])
    torch.save(checkpoint,root/"perception.pt")
    saved=torch.load(root/"perception.pt",map_location="cpu",weights_only=True)
    original=torch.load(parent,map_location="cpu",weights_only=True)
    assert all(torch.equal(value,saved["state_dict"][name]) for name,value in original["state_dict"].items())
    write(root/"summary.json",{**plan,"complete":True,"checkpoint_sha256":digest(root/"perception.pt"),
        "all_segmentation_tensors_identical_to_parent":True,
        "interpretation":"Decoder candidate only; requires matched residual training and fresh physical full-task evaluation."})
    print(json.dumps({"complete":True,"inference_config":config,"checkpoint_sha256":digest(root/"perception.pt")}),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    parser.add_argument("--threshold",type=float,choices=(.55,.5),required=True)
    parser.add_argument("--geometry-fallback",action="store_true")
    main(parser.parse_args())
