"""Development-only sweep of image postprocessing, with exact legacy parity.

The full existing validation set is reused explicitly for development. Saved
task failures are post-hoc diagnostic inputs, never new heldout scenes.
"""
import argparse
from collections import Counter
import importlib.util
import json
from pathlib import Path
import shutil

import numpy as np
import torch

from feedback_audit import same
from policy import valid_prediction
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import DEFAULT_INFERENCE_CONFIG, decode_probability, inputs


CONFIGS=[{"probability_threshold":p,"min_pixels":n} for p in (.7,.65,.6) for n in (25,20)]


def metrics(rows):
    positive=[r for r in rows if r["expected_target"]]
    negative=[r for r in rows if not r["expected_target"]]
    errors=[r["error_mm"] for r in positive if r["usable"]]
    misses=sum(not r["usable"] for r in positive)
    wrong=sum(r["usable"] and r["error_mm"]>10 for r in positive)
    return {"images":len(rows),"positives":len(positive),"negatives":len(negative),
        "misses":misses,"false_positives":sum(r["usable"] for r in negative),
        "wrong_localization_above_10mm":wrong,"miss_or_wrong_above_10mm":misses+wrong,
        "detected_errors_above_5mm":sum(e>5 for e in errors),
        "conditional_mean_error_mm":float(np.mean(errors)) if errors else None,
        "conditional_p95_error_mm":float(np.quantile(errors,.95)) if errors else None,
        "maximum_error_mm":max(errors) if errors else None}


def main(args):
    checkpoint=args.perception.resolve();dataset=args.dataset.resolve()
    root=create_run(args.name,checkpoint)
    shutil.copy2(Path(__file__),root/"source/mask_gate_probe.py")
    provenance=json.loads((root/"provenance.json").read_text())
    provenance["source_sha256"]["mask_gate_probe.py"]=digest(Path(__file__))
    write(root/"provenance.json",provenance)
    legacy_path=args.legacy_vision.resolve()
    specification=importlib.util.spec_from_file_location("frozen_legacy_vision",legacy_path)
    legacy=importlib.util.module_from_spec(specification);specification.loader.exec_module(legacy)
    models={"retained_v1":perception_model(ROOT/"outputs/vision-iteration-03/perception.pt"),
            "egl_candidate":perception_model(checkpoint)}
    plan={"development_only":True,"hardware_executable":False,"configs":CONFIGS,
        "dataset":str(dataset),"dataset_sha256":digest(dataset),
        "legacy_vision":str(legacy_path),"legacy_vision_sha256":digest(legacy_path),
        "checkpoint":str(checkpoint),"checkpoint_sha256":digest(checkpoint),
        "legacy_parity_models":list(models),
        "selection_rule":"Report all; prefer fewer miss_or_wrong_above_10mm with no increase in empty false positives or >10mm wrong localization; no automatic adoption."}
    write(root/"config.json",plan)
    rows={f"p{c['probability_threshold']}-n{c['min_pixels']}":[] for c in CONFIGS}
    parity=Counter()
    with np.load(dataset,allow_pickle=False) as data:
        rgb,depth=data["rgb"],data["depth_m"]
        metadata=json.loads(str(data["metadata_json"]))
        for i,meta in enumerate(metadata):
            frame={"rgb":rgb[i],"depth":depth[i],"camera":meta["camera"]}
            for name,model in models.items():
                expected=legacy.estimate(model,frame)
                with torch.no_grad():
                    probability=torch.sigmoid(model(torch.from_numpy(inputs(rgb[i:i+1],depth[i:i+1]))))[0,0].numpy()
                actual=decode_probability(probability,frame,DEFAULT_INFERENCE_CONFIG)
                same(actual,expected);parity[name]+=1
                if name!="egl_candidate":
                    continue
                for config in CONFIGS:
                    predicted=decode_probability(probability,frame,config)
                    usable=valid_prediction(predicted)
                    row={"index":i,"seed":meta["seed"],"view":meta.get("arm_initialization_view","legacy_HOME"),
                        "expected_target":not meta["absent"],"usable":usable,"prediction":predicted}
                    if row["expected_target"] and usable:
                        truth=np.asarray(meta["initial_xyz"]).copy();truth[2]+=meta["half_size_m"]
                        row["error_mm"]=float(np.linalg.norm(np.asarray(predicted["top_xyz_m"])-truth)*1000)
                    rows[f"p{config['probability_threshold']}-n{config['min_pixels']}"].append(row)
    diagnostics=[]
    # Every exact decision frame is retained, including missing detections.
    for relative in ("goal-egl-coverage-v2-candidate-02/disturbed-new_feedback-009",
                     "home-first-development-02/static-new_feedback-002"):
        directory=ROOT/"outputs"/relative
        with np.load(directory/"decision_observations.npz",allow_pickle=False) as data:
            rgb,depth=data["rgb"],data["depth_m"]
            cameras=json.loads(str(data["camera_json"]))
            metadata=json.loads(str(data["metadata_json"]))
            for i,meta in enumerate(metadata):
                frame={"rgb":rgb[i],"depth":depth[i],"camera":cameras[i]}
                with torch.no_grad():
                    probability=torch.sigmoid(models["egl_candidate"](torch.from_numpy(inputs(rgb[i:i+1],depth[i:i+1]))))[0,0].numpy()
                diagnostics.append({"source":relative,"source_sha256":digest(directory/"decision_observations.npz"),
                    "decision_index":i,"stage":meta["stage"],"posthoc_development":True,
                    "predictions":{f"p{c['probability_threshold']}-n{c['min_pixels']}":decode_probability(probability,frame,c) for c in CONFIGS}})
    result={**plan,"complete":True,"legacy_parity_frames":dict(parity),
        "groups":{key:metrics(value) for key,value in rows.items()},
        "by_view":{key:{view:metrics([r for r in value if r["view"]==view]) for view in sorted({r["view"] for r in value})} for key,value in rows.items()},
        "validation_rows":rows,"posthoc_diagnostics":diagnostics}
    write(root/"summary.json",result)
    print(json.dumps({"legacy_parity_frames":dict(parity),"groups":result["groups"]}),flush=True)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--name",required=True)
    p.add_argument("--perception",type=Path,default=ROOT/"outputs/vision-multiview-egl-01/perception.pt")
    p.add_argument("--dataset",type=Path,default=ROOT/"outputs/observation-images-egl-01/validation.npz")
    p.add_argument("--legacy-vision",type=Path,default=ROOT/"outputs/goal-egl-coverage-v2-candidate-02/source/vision.py")
    main(p.parse_args())
