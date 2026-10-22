"""Evaluate matched residuals on all previously audited development images."""
import argparse
import copy
import json
from pathlib import Path
import shutil

import numpy as np

from component_geometry_probe import metrics
from feedback_audit import same
from policy import load
from sim import ROOT
from task_runner import create_run, digest, write


def main(args):
    probe=(ROOT/"outputs/component-geometry-development-01").resolve()
    parity=(ROOT/"outputs/geometry-decoder-parity-01").resolve()
    assert json.loads((probe/"audit.json").read_text())["passed"]
    assert json.loads((parity/"summary.json").read_text())["passed"]
    data=json.loads((probe/"summary.json").read_text())
    canonical=(ROOT/"outputs").resolve().parent
    models={"retained_v2":(canonical/"versions/V2/models/perception.pt",canonical/"versions/V2/models/residual.pt","legacy"),
        **{f"geometry{suffix}":((ROOT/f"outputs/vision-geometry{suffix}-01/perception.pt").resolve(),
            (ROOT/f"outputs/goal-policy-geometry{suffix}-01/residual.pt").resolve(),f"geometry_mask_{threshold}")
           for suffix,threshold in (("055",.55),("050",.5))}}
    root=create_run(args.name,models["geometry050"][0])
    provenance=json.loads((root/"provenance.json").read_text())
    for name in (Path(__file__).name,"component_geometry_probe.py"):
        shutil.copy2(ROOT/name,root/"source"/name)
        provenance["source_sha256"][name]=digest(ROOT/name)
    write(root/"provenance.json",provenance)
    plan={"development_only":True,"hardware_executable":False,
        "probe_summary_sha256":digest(probe/"summary.json"),"probe_audit_sha256":digest(probe/"audit.json"),
        "decoder_parity_sha256":digest(parity/"summary.json"),
        "models":{key:{"perception_path":str(detector),"perception_sha256":digest(detector),
            "policy_path":str(head),"policy_sha256":digest(head),"probe_variant":variant}
            for key,(detector,head,variant) in models.items()},
        "selection_rule":"Prefer fewer unusable or >10mm positives, no new empty false positives or known physical regression; use mean/P95 as secondary development evidence. Not adoption evidence."}
    write(root/"config.json",plan)
    results,rows={},{}
    for split,source in data["datasets"].items():
        assert digest(source["path"])==source["sha256"]
        with np.load(source["path"],allow_pickle=False) as images:
            metadata=json.loads(str(images["metadata_json"]))
        groups={}
        for key,(detector,head,variant) in models.items():
            assert json.loads((head.parent/"summary.json").read_text())["perception_sha256"]==digest(detector)
            policy=load(head)
            output=copy.deepcopy(data["rows"][split][variant])
            for row,meta in zip(output,metadata,strict=True):
                if row["expected_target"] and row["valid"]:
                    truth=np.asarray(meta["initial_xyz"]).copy();truth[2]+=meta["half_size_m"]
                    corrected=policy.correct(row["prediction"])
                    row["corrected_prediction"]=corrected
                    row["corrected_error_mm"]=float(np.linalg.norm(np.asarray(corrected["top_xyz_m"])-truth)*1000)
            if key=="retained_v2":
                same(metrics(output),data["metrics"][split]["legacy"])
            groups[key]=output
        rows[split]=groups
        results[split]={key:metrics(value) for key,value in groups.items()}
        print(json.dumps({"split":split,"metrics":results[split]}),flush=True)
    write(root/"summary.json",{**plan,"complete":True,"metrics":results,"rows":rows,
        "scope":"Matched-head rescoring of every audited validation/empty prediction; images and predictions were parity-checked separately. Reused development data, not full tasks or new generalization evidence."})


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name",required=True)
    main(parser.parse_args())
