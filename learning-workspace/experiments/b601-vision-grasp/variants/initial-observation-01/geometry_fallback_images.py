"""Full development inference, decoder parity and matched fallback-head scoring."""
import argparse
import copy
import json
from pathlib import Path
import shutil

import numpy as np

from component_geometry_probe import metrics
from feedback import quality
from feedback_audit import same
from policy import features, load, valid_prediction
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import estimate


def expected_prediction(groups,index):
    geometry=groups["geometry_mask_0.5"][index]["prediction"]
    legacy=groups["legacy"][index]["prediction"]
    if geometry is not None:
        return {**geometry,"decoder_path":"geometry"}
    if valid_prediction(legacy):
        return {**legacy,"decoder_path":"retained_v2_fallback"}
    return None


def main(args):
    probe=(ROOT/"outputs/component-geometry-development-01").resolve()
    assert json.loads((probe/"audit.json").read_text())["passed"]
    data=json.loads((probe/"summary.json").read_text())
    detector=(ROOT/"outputs/vision-geometry050-fallback-01/perception.pt").resolve()
    head=(ROOT/"outputs/goal-policy-geometry050-fallback-01/residual.pt").resolve()
    assert json.loads((head.parent/"summary.json").read_text())["perception_sha256"]==digest(detector)
    model,policy=perception_model(detector),load(head)
    root=create_run(args.name,detector)
    provenance=json.loads((root/"provenance.json").read_text())
    for name in (Path(__file__).name,"component_geometry_probe.py"):
        shutil.copy2(ROOT/name,root/"source"/name)
        provenance["source_sha256"][name]=digest(ROOT/name)
    write(root/"provenance.json",provenance)
    plan={"development_only":True,"hardware_executable":False,
        "probe_summary_sha256":digest(probe/"summary.json"),"probe_audit_sha256":digest(probe/"audit.json"),
        "perception":{"path":str(detector),"sha256":digest(detector)},
        "policy":{"path":str(head),"sha256":digest(head)},"datasets":data["datasets"],
        "feedback_config":data["feedback_config"],
        "primary_selection":"0.50 chosen from previous full validation mean/P95 with equal unusable/>10mm counts and equal known physical regressions; slightly worse maximum retained.",
        "rule":"Use geometry0.50 if detected; otherwise workspace-valid retainedV2 decoder0.60/25. Same downstream visual quality gates."}
    write(root/"config.json",plan)
    results,rows,counts={}, {}, {}
    for split,source in data["datasets"].items():
        assert digest(source["path"])==source["sha256"]
        output=[];x=[];y=[];fallbacks=0
        with np.load(source["path"],allow_pickle=False) as images:
            rgb,depth=images["rgb"],images["depth_m"]
            metadata=json.loads(str(images["metadata_json"]))
            for i,meta in enumerate(metadata):
                frame={"rgb":rgb[i],"depth":depth[i],"camera":meta["camera"]}
                p=estimate(model,frame)
                same(p,expected_prediction(data["rows"][split],i))
                row=copy.deepcopy(data["rows"][split]["geometry_mask_0.5"][i])
                row.update(prediction=p,valid=valid_prediction(p),quality=quality(p,frame["camera"],plan["feedback_config"]))
                if p and p["decoder_path"]=="retained_v2_fallback":
                    fallbacks+=1
                if row["valid"] and row["expected_target"]:
                    truth=np.asarray(meta["initial_xyz"]).copy();truth[2]+=meta["half_size_m"]
                    corrected=policy.correct(p);row["corrected_prediction"]=corrected
                    row["corrected_error_mm"]=float(np.linalg.norm(np.asarray(corrected["top_xyz_m"])-truth)*1000)
                    xyz=np.asarray(p["top_xyz_m"])
                    if .265<=xyz[0]<=.395 and abs(xyz[1])<=.125:
                        x.append(features(p));y.append(truth-xyz)
                else:
                    row.pop("corrected_error_mm",None)
                output.append(row)
        if split=="validation":
            with np.load(head.parent/"validation.npz",allow_pickle=False) as stored:
                assert np.array_equal(stored["x"],np.asarray(x,np.float32))
                assert np.array_equal(stored["y"],np.asarray(y,np.float32))
        results[split]=metrics(output);rows[split]=output;counts[split]={"images":len(output),"fallback_predictions":fallbacks}
        print(json.dumps({"split":split,"metrics":results[split],"counts":counts[split]}),flush=True)
    write(root/"summary.json",{**plan,"complete":True,"decoder_output_parity_passed":True,
        "validation_training_feature_label_parity_passed":True,"metrics":results,"rows":rows,"counts":counts,
        "scope":"Every image inference checked against independently audited component predictions and explicit fallback rule; matched validation feature/label arrays checked. Reused development images, not new task/adoption evidence."})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    main(parser.parse_args())
