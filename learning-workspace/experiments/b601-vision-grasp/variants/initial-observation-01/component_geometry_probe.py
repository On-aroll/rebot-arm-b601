"""Full image-development comparison of calibrated component selection."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np
from scipy import ndimage
import torch

from feedback import configuration, quality
from feedback_audit import same
from policy import load, valid_prediction
from sim import ROOT, deproject
from task_runner import create_run, digest, perception_model, write
from vision import decode_probability, inputs

VARIANTS=("legacy","workspace_rank","geometry_mask_0.6","geometry_mask_0.55","geometry_mask_0.5")


def geometry_mask(frame):
    depth=frame["depth"]
    v,u=np.indices(depth.shape)
    pixels=np.stack([u.ravel(),v.ravel(),np.ones(depth.size)])
    camera=frame["camera"]
    optical=np.linalg.solve(np.asarray(camera["K"]),pixels)*depth.ravel()
    transform=np.asarray(camera["T_base_camera_optical"])
    points=transform[:3,:3]@optical+transform[:3,3,None]
    for index in (0,depth.size//2,depth.size-1):
        if np.isfinite(depth.ravel()[index]) and .05<depth.ravel()[index]<2:
            assert np.allclose(points[:,index],deproject(pixels[0,index],pixels[1,index],depth.ravel()[index],camera),atol=1e-12,rtol=0)
    x,y,z=points
    return (np.isfinite(points).all(axis=0)&(x>=.245)&(x<=.50)&(np.abs(y)<=.25)&(z>=.125)&(z<=.16)).reshape(depth.shape)


def predictions(probability,frame):
    original=decode_probability(probability,frame,{"probability_threshold":.6,"min_pixels":25})
    labels,count=ndimage.label(probability>=.6)
    candidates=[]
    for label in range(1,count+1):
        component=labels==label
        if 25<=component.sum()<=240:
            candidate=decode_probability(np.where(component,probability,0),frame,{"probability_threshold":.6,"min_pixels":25})
            if candidate:
                candidates.append(candidate)
    same(original,max(candidates,key=lambda p:p["confidence"]) if candidates else None)
    valid=[p for p in candidates if valid_prediction(p)]
    result={"legacy":original,"workspace_rank":max(valid,key=lambda p:p["confidence"]) if valid else None}
    masked=np.where(geometry_mask(frame),probability,0)
    for threshold in (.6,.55,.5):
        result[f"geometry_mask_{threshold}"]=decode_probability(masked,frame,{"probability_threshold":threshold,"min_pixels":25})
    return result


def probability(model,frame):
    with torch.no_grad():
        x=torch.from_numpy(inputs(frame["rgb"][None],frame["depth"][None]))
        return torch.sigmoid(model(x))[0,0].numpy()


def metrics(rows):
    positives=[r for r in rows if r["expected_target"]]
    detected=[r for r in positives if r["valid"]]
    errors=[r["corrected_error_mm"] for r in detected]
    return {"positive_images":len(positives),"empty_images":len(rows)-len(positives),
        "unusable_positives":sum(not r["valid"] for r in positives),
        "corrected_errors_above_10mm":sum(e>10 for e in errors),
        "unusable_or_wrong_above_10mm":sum(not r["valid"] for r in positives)+sum(e>10 for e in errors),
        "false_positive_empty_images":sum(r["valid"] and not r["expected_target"] for r in rows),
        "quality_accepted_empty_images":sum(r["quality"]["accepted"] and not r["expected_target"] for r in rows),
        "conditional_corrected_mean_error_mm":float(np.mean(errors)) if errors else None,
        "conditional_corrected_p95_error_mm":float(np.quantile(errors,.95)) if errors else None,
        "valid_positives":len(detected)}


def main(args):
    canonical=ROOT.parents[1]
    detector=canonical/"versions/V2/models/perception.pt"
    head=canonical/"versions/V2/models/residual.pt"
    root=create_run(args.name,detector)
    provenance=json.loads((root/"provenance.json").read_text())
    shutil.copy2(Path(__file__),root/"source/component_geometry_probe.py")
    provenance["source_sha256"]["component_geometry_probe.py"]=digest(Path(__file__))
    write(root/"provenance.json",provenance)
    shutil.copy2(detector,root/"perception.pt");shutil.copy2(head,root/"residual.pt")
    config=configuration(json.loads((ROOT/"configs/v2-feedback.json").read_text()))
    datasets={"validation":(ROOT/"outputs/observation-images-egl-01/validation.npz").resolve(),
              "empty_stress":(ROOT/"outputs/negative-mask-development-01/empty_images.npz").resolve()}
    plan={"development_only":True,"full_task_test":False,"hardware_executable":False,
        "variants":VARIANTS,"perception_sha256":digest(detector),"policy_sha256":digest(head),
        "datasets":{k:{"path":str(p),"sha256":digest(p)} for k,p in datasets.items()},
        "geometry_bounds_m":{"x":[.245,.50],"absolute_y_max":.25,"z":[.125,.16]},
        "selection_rule":"Report all; prefer fewer unusable positives or corrected errors>10mm without increasing>10mm errors or empty false positives. Current residual used only for diagnosis; any adopted decoder needs matched training and fresh full-task evaluation.",
        "feedback_config":config}
    write(root/"config.json",plan)
    model,policy=perception_model(root/"perception.pt"),load(root/"residual.pt")
    results={};rows={}
    for split,path in datasets.items():
        groups={name:[] for name in VARIANTS}
        with np.load(path,allow_pickle=False) as data:
            rgb,depth=data["rgb"],data["depth_m"]
            metadata=json.loads(str(data["metadata_json"]))
            for i,meta in enumerate(metadata):
                frame={"rgb":rgb[i],"depth":depth[i],"camera":meta["camera"]}
                outputs=predictions(probability(model,frame),frame)
                for name,p in outputs.items():
                    row={"index":i,"seed":meta["seed"],"view":meta.get("arm_initialization_view",meta.get("view","legacy_HOME")),
                        "expected_target":not meta["absent"],"prediction":p,"valid":valid_prediction(p),
                        "quality":quality(p,frame["camera"],config)}
                    if row["expected_target"] and row["valid"]:
                        truth=np.asarray(meta["initial_xyz"]).copy();truth[2]+=meta["half_size_m"]
                        row["corrected_error_mm"]=float(np.linalg.norm(np.asarray(policy.correct(p)["top_xyz_m"])-truth)*1000)
                    groups[name].append(row)
        results[split]={name:metrics(group) for name,group in groups.items()}
        rows[split]=groups
        print(json.dumps({"split":split,"metrics":results[split]}),flush=True)
    known=[]
    for name in ("goal-fragment-home-v2-candidate-01/static-new_feedback-018", "fragment-regression-16400002/episode"):
        source=(ROOT/"outputs"/name/"decision_observations.npz").resolve()
        with np.load(source,allow_pickle=False) as data:
            cameras=json.loads(str(data["camera_json"]))
            frame={"rgb":data["rgb"][0],"depth":data["depth_m"][0],"camera":cameras[0]}
        known.append({"posthoc_source":str(source),"source_sha256":digest(source),
                      "predictions":predictions(probability(model,frame),frame)})
    write(root/"summary.json",{**plan,"complete":True,"metrics":results,"rows":rows,"known_cases":known,
        "interpretation":"Reused image development; current-head errors are conditional. No weight training, full-task, fresh adoption or hardware claim."})
    print(json.dumps({"complete":True,"known_cases":known}),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    main(parser.parse_args())
