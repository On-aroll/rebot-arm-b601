"""Full image-development calibration of raw confidence versus scored error."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from feedback import configuration, quality
from policy import load, valid_prediction
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import estimate


def main(args):
    parent=ROOT/"outputs/vision-fragment-egl-01/perception.pt"
    head=ROOT/"outputs/goal-policy-fragment-egl-01/residual.pt"
    dataset=ROOT/"outputs/observation-images-egl-01/validation.npz"
    assert json.loads((head.parent/"summary.json").read_text())["perception_sha256"]==digest(parent)
    root=create_run(args.name,parent)
    shutil.copy2(Path(__file__),root/"source/confidence_gate_probe.py")
    provenance=json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"]["confidence_gate_probe.py"]=digest(Path(__file__))
    write(root/"provenance.json",provenance)
    configs={str(c):configuration(dict(json.loads((ROOT/"configs/home-first-development-01.json").read_text()),min_confidence=c))
             for c in (.7,.75,.8,.85)}
    plan={"development_only":True,"hardware_executable":False,"configs":configs,
        "dataset":str(dataset.resolve()),"dataset_sha256":digest(dataset),
        "perception_sha256":digest(parent),"policy_sha256":digest(head),
        "selection_rule":"Report all; maximize accepted positives with corrected error<=5mm without increasing accepted errors>10mm or accepted empty false positives versus0.8. No error-bound or full-task claim."}
    write(root/"config.json",plan)
    model,policy=perception_model(parent),load(head)
    groups={key:[] for key in configs}
    with np.load(dataset,allow_pickle=False) as data:
        rgb,depth=data["rgb"],data["depth_m"]
        metadata=json.loads(str(data["metadata_json"]))
        for i,meta in enumerate(metadata):
            predicted=estimate(model,{"rgb":rgb[i],"depth":depth[i],"camera":meta["camera"]})
            for key,config in configs.items():
                gate=quality(predicted,meta["camera"],config)
                row={"index":i,"seed":meta["seed"],"view":meta.get("arm_initialization_view","legacy_HOME"),
                    "expected_target":not meta["absent"],"gate":gate,"prediction":predicted}
                if not meta["absent"] and valid_prediction(predicted):
                    corrected=policy.correct(predicted)
                    truth=np.asarray(meta["initial_xyz"]).copy();truth[2]+=meta["half_size_m"]
                    row["raw_error_mm"]=float(np.linalg.norm(np.asarray(predicted["top_xyz_m"])-truth)*1000)
                    row["corrected_error_mm"]=float(np.linalg.norm(np.asarray(corrected["top_xyz_m"])-truth)*1000)
                groups[key].append(row)
    summaries={}
    for key,rows in groups.items():
        accepted=[r for r in rows if r["gate"]["accepted"] and r["expected_target"]]
        errors=[r["corrected_error_mm"] for r in accepted]
        summaries[key]={"positive_images":546,"empty_images":94,"accepted_positives":len(accepted),
            "accepted_errors_at_most_5mm":sum(e<=5 for e in errors),
            "accepted_errors_above_5mm":sum(e>5 for e in errors),
            "accepted_errors_above_10mm":sum(e>10 for e in errors),
            "accepted_empty_false_positives":sum(r["gate"]["accepted"] and not r["expected_target"] for r in rows),
            "conditional_corrected_mean_error_mm":float(np.mean(errors)) if errors else None,
            "conditional_max_error_mm":max(errors) if errors else None}
    write(root/"summary.json",{**plan,"complete":True,"groups":summaries,"rows":groups})
    print(json.dumps(summaries),flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name",required=True)
    main(p.parse_args())
