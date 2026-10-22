"""Score a preselected image head on all42 immutable known-view probes."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np
import torch

from image_residual import ImageResidual
from policy import BOUND_M, load
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import estimate


def score(root, config):
    for path,expected in config["input_sha256"].items():assert digest(path)==expected,path
    for name,expected in config["model_sha256"].items():assert digest(root/"frozen"/name)==expected,name
    ck=torch.load(root/"frozen/image-residual.pt",map_location="cpu",weights_only=True)
    assert ck["schema"]=="b601-image-bounded-residual-v1" and ck["bound_m"]==BOUND_M
    model=ImageResidual();model.load_state_dict(ck["state_dict"]);model.eval()
    scalar=load(root/"frozen/V3-residual.pt");perception=perception_model(root/"frozen/perception.pt")
    assert ck["perception_sha256"]==digest(root/"frozen/perception.pt")
    reference=Path(config["known_probe_run"])
    plan=json.loads((reference/"config.json").read_text())
    summary=json.loads((reference/"summary.json").read_text())
    old_metrics=json.loads((reference/"metrics.json").read_text())
    assert summary["complete"] and json.loads((reference/"audit.json").read_text())["passed"]
    rows=[]
    for i,(case,e,old) in enumerate(zip(plan["cases"],summary["cases"],old_metrics["rows"],strict=True)):
        pair=e["view_pair"];p=None if pair is None else pair["predictions"][-1]
        corrected=image_corrected=None
        if p is not None:
            with np.load(reference/f"case-{i:03d}/decision_observations.npz",allow_pickle=False) as data:
                index=pair["decision_indices"][-1]
                cameras=json.loads(str(data["camera_json"]))
                frame={"rgb":data["rgb"][index],"depth":data["depth_m"][index],"camera":cameras[index]}
            assert estimate(perception,frame)==p,(i,"raw prediction mismatch")
            corrected=scalar.correct(p);image_corrected=model.correct(p,frame)
            assert max(abs(v) for v in image_corrected["residual_xyz_m"])<=BOUND_M+1e-9
        truth=np.asarray(e["independent_score_only"]["truth_top_xyz_m"])
        error=lambda pred:None if pred is None else float(np.linalg.norm(np.asarray(pred["top_xyz_m"])-truth)*1000)
        scalar_error,image_error=error(corrected),error(image_corrected)
        if scalar_error is None:assert old["corrected_error_mm"] is None
        else:assert abs(scalar_error-old["corrected_error_mm"])<1e-8
        rows.append({"seed":e["seed"],"offset_index":case["offset_index"],"offset_m":e["offset_m"],
            "quality_accepted_unchanged":bool(pair and pair["quality"]["accepted"]),
            "quality":None if pair is None else pair["quality"],
            "raw_error_mm":e["independent_score_only"]["raw_error_mm"],
            "V3_error_mm":scalar_error,"image_error_mm":image_error,
            "image_prediction":image_corrected,"source_case":str(reference/f"case-{i:03d}")})
    def group(selected):
        available=[r for r in selected if r["image_error_mm"] is not None]
        stats=lambda k:({"mean_mm":float(np.mean([r[k] for r in available])),
            "max_mm":max(r[k] for r in available),"at_most5mm":sum(r[k]<=5 for r in available),
            "above10mm":sum(r[k]>10 for r in available)} if available else None)
        return {"cases":len(selected),"predictions":len(available),"V3":stats("V3_error_mm"),"image":stats("image_error_mm")}
    return {"development_only":True,"full_task_test":False,"adopted":False,"rows":rows,
        "all_views":group(rows),"quality_accepted":group([r for r in rows if r["quality_accepted_unchanged"]]),
        "per_seed":{str(seed):group([r for r in rows if r["seed"]==seed and r["quality_accepted_unchanged"]]) for seed in sorted({r["seed"] for r in rows})},
        "new_accepted_above10mm":[[r["seed"],r["offset_index"]] for r in rows if r["quality_accepted_unchanged"] and r["image_error_mm"]>10 and r["V3_error_mm"]<=10],
        "fixed_accepted_above10mm":[[r["seed"],r["offset_index"]] for r in rows if r["quality_accepted_unchanged"] and r["V3_error_mm"]>10 and r["image_error_mm"]<=10],
        "model_sha256":config["model_sha256"],"input_sha256":config["input_sha256"],
        "scope":"All42 previously audited independent physical views, preselected image head applied to exact final decision RGB-D; quality/poses unchanged, no physics execution or task success inference."}


def main(args):
    if args.audit_run:
        root=(ROOT/"outputs"/args.audit_run).resolve()
        config=json.loads((root/"config.json").read_text())
        provenance=json.loads((root/"provenance.json").read_text())
        for name,expected in provenance["source_sha256"].items():
            assert digest(root/"source"/Path(name).name)==digest(ROOT/name)==expected
        assert score(root,config)==json.loads((root/"summary.json").read_text())
        write(root/"audit.json",{"passed":True,"known_view_cases":42,"scope":"Exact saved-input model/score reconstruction; not new full tasks or physics re-execution."})
        print(json.dumps({"passed":True,"known_view_cases":42}),flush=True)
        return
    training=(ROOT/"outputs"/args.training).resolve();reference=(ROOT/"outputs/relative-view-development-01").resolve()
    trained=json.loads((training/"summary.json").read_text())
    assert trained["eligible_for_task_development"] and json.loads((training/"audit.json").read_text())["passed"]
    root=create_run(args.name,training/"frozen/perception.pt")
    provenance=json.loads((root/"provenance.json").read_text())
    for name in ("image_residual.py","image_residual_development.py","image_residual_probe_review.py"):
        shutil.copy2(ROOT/name,root/"source"/name);provenance["source_sha256"][name]=digest(ROOT/name)
    write(root/"provenance.json",provenance)
    (root/"frozen").mkdir()
    for src,name in [(training/"frozen/perception.pt","perception.pt"),(training/"frozen/V3-residual.pt","V3-residual.pt"),(training/"residual.pt","image-residual.pt")]:
        shutil.copy2(src,root/"frozen"/name)
    paths=[training/"config.json",training/"summary.json",training/"audit.json",reference/"config.json",reference/"summary.json",reference/"audit.json",reference/"metrics.json"]
    for i in range(42):paths.extend([reference/f"case-{i:03d}/result.json",reference/f"case-{i:03d}/decision_observations.npz"])
    config={"known_probe_run":str(reference),"training_run":str(training),"input_sha256":{str(p):digest(p) for p in paths},
            "model_sha256":{p.name:digest(p) for p in sorted((root/"frozen").iterdir())},"hardware_executable":False}
    write(root/"config.json",config)
    report=score(root,config);write(root/"summary.json",report)
    print(json.dumps({k:report[k] for k in ("all_views","quality_accepted","per_seed","new_accepted_above10mm","fixed_accepted_above10mm")}),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--name");group.add_argument("--audit-run",type=Path)
    parser.add_argument("--training",default="image-residual-development-01")
    main(parser.parse_args())
