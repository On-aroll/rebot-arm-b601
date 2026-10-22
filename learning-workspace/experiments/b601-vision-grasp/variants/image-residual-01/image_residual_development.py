"""Fixed offline image-residual experiment, with full-input reconstruction audit."""
import argparse
import copy
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.ndimage import map_coordinates
import torch

from image_residual import ImageResidual, image_patch, PATCH_SIZE
from policy import BOUND_M, features, load, valid_prediction
from sim import ROOT
from task_runner import create_run, digest, perception_model, write
from vision import estimate

EXTRA = ("image_residual.py", "image_residual_development.py", "BRANCH_BASE.json")
CANONICAL = (ROOT/"outputs").resolve().parent
V3 = CANONICAL/"versions/V3"
DATA = CANONICAL/"outputs/observation-images-egl-01"
PROTOCOL = CANONICAL/"IMAGE_RESIDUAL_DEVELOPMENT_PROTOCOL.md"
EXPECTED_DATA = {"training":"19bc570f868fef38635ed29984e12b1d7eaa971bd307c80ba46ae2bea798a355",
                 "validation":"ad6f98964257dc21214db033e22dfcd95090da74458e4f403d3d7c4fa3647eb2"}


def error_metrics(delta, target):
    error = np.linalg.norm(delta-target, axis=1)*1000
    return {"samples":len(error), "mean_mm":float(error.mean()), "p95_mm":float(np.quantile(error,.95)),
            "max_mm":float(error.max()), "above5mm":int((error>5).sum()), "above10mm":int((error>10).sum())}


def output(model, x, patches):
    with torch.no_grad():
        return torch.cat([model(x[i:i+64], patches[i:i+64]) for i in range(0,len(x),64)]).numpy()


def samples(split, perception):
    path = DATA/f"{split}.npz"
    assert digest(path)==EXPECTED_DATA[split]
    x, y, patches, indices, rows = [], [], [], [], []
    with np.load(path,allow_pickle=False) as data:
        rgb, depth, seeds = data["rgb"], data["depth_m"], data["seeds"].copy()
        metadata = json.loads(str(data["metadata_json"]))
    for i,m in enumerate(metadata):
        frame = {"rgb":rgb[i], "depth":depth[i], "camera":m["camera"]}
        predicted = estimate(perception,frame)
        usable = not m["absent"] and valid_prediction(predicted)
        if usable:
            p = predicted["top_xyz_m"]
            usable = .265 <= p[0] <= .395 and abs(p[1]) <= .125
        row = {"index":i, "seed":m["seed"], "view":m.get("arm_initialization_view","legacy_HOME"),
               "absent":m["absent"], "raw_prediction":predicted, "usable_for_supervision":bool(usable)}
        if usable:
            # Synthetic label only. image_patch and ImageResidual see no metadata.
            truth = np.asarray(m["initial_xyz"]).copy(); truth[2] += m["half_size_m"]
            x.append(features(predicted)); y.append(truth-predicted["top_xyz_m"])
            patches.append(image_patch(frame,predicted)); indices.append(i)
        rows.append(row)
    arrays = {"x":np.asarray(x,np.float32), "y":np.asarray(y,np.float32), "patches":np.asarray(patches,np.float32),
              "sample_indices":np.asarray(indices,np.int64), "seeds":seeds}
    # Exact equality with the adopted scalar head's original supervised data.
    with np.load(V3/"models"/f"{split}.npz",allow_pickle=False) as baseline:
        assert np.array_equal(arrays["x"],baseline["x"])
        assert np.array_equal(arrays["y"],baseline["y"])
        assert np.array_equal(seeds,baseline["seeds"])
    return arrays, rows


def assess(arrays, rows, scalar, model):
    x, p, y = torch.from_numpy(arrays["x"]), torch.from_numpy(arrays["patches"]), arrays["y"]
    with torch.no_grad(): base = scalar(x).numpy()
    candidate = output(model,x,p)
    assert np.abs(candidate).max() <= BOUND_M+1e-9
    for j,i in enumerate(arrays["sample_indices"]):
        rows[i].update(raw_error_mm=float(np.linalg.norm(y[j])*1000),
                       V3_error_mm=float(np.linalg.norm(base[j]-y[j])*1000),
                       image_error_mm=float(np.linalg.norm(candidate[j]-y[j])*1000),
                       image_residual_m=candidate[j].tolist())
    groups = {}
    for view in sorted({r["view"] for r in rows}):
        selected = [r for r in rows if r["view"]==view]
        groups[view] = {"images":len(selected), "positive_unusable":sum(not r["absent"] and not r["usable_for_supervision"] for r in selected),
            "empty_valid_predictions":sum(r["absent"] and valid_prediction(r["raw_prediction"]) for r in selected),
            "V3_mean_mm":float(np.mean([r["V3_error_mm"] for r in selected if "V3_error_mm" in r])) if any("V3_error_mm" in r for r in selected) else None,
            "image_mean_mm":float(np.mean([r["image_error_mm"] for r in selected if "image_error_mm" in r])) if any("image_error_mm" in r for r in selected) else None}
    return {"images":len(rows),"positive_images":sum(not r["absent"] for r in rows),
            "positive_unusable":sum(not r["absent"] and not r["usable_for_supervision"] for r in rows),
            "empty_images":sum(r["absent"] for r in rows),
            "empty_valid_predictions":sum(r["absent"] and valid_prediction(r["raw_prediction"]) for r in rows),
            "raw":error_metrics(np.zeros_like(y),y), "V3":error_metrics(base,y), "image":error_metrics(candidate,y),
            "new_above10mm":[r["index"] for r in rows if r.get("image_error_mm",0)>10 and r["V3_error_mm"]<=10],
            "fixed_above10mm":[r["index"] for r in rows if r.get("V3_error_mm",0)>10 and r["image_error_mm"]<=10],
            "by_view":groups,"rows":rows}


def gates(report, selected):
    a,b = report["V3"], report["image"]
    return {"trained_epoch_selected":selected>0,
            "mean_not_worse":b["mean_mm"]<=a["mean_mm"], "p95_not_worse":b["p95_mm"]<=a["p95_mm"],
            "max_not_worse":b["max_mm"]<=a["max_mm"], "above5_not_increased":b["above5mm"]<=a["above5mm"],
            "above10_not_increased":b["above10mm"]<=a["above10mm"], "no_new_above10":not report["new_above10mm"],
            "meaningful_tail_change":b["above5mm"]<=.8*a["above5mm"] or b["p95_mm"]<=.9*a["p95_mm"]}


def train(args):
    manifest=json.loads((V3/"release_manifest.json").read_text())
    assert digest(V3/"release_manifest.json")=="bb6bebf40eb2b660f66600c5be2b41b7e13bee6529bcc042e247a363bcb1342b"
    assert all(digest(V3/p)==h for p,h in manifest["files_sha256"].items())
    perception_path=V3/"models/perception.pt"
    root=create_run(args.name,perception_path)
    provenance=json.loads((root/"provenance.json").read_text())
    for name in EXTRA:
        shutil.copy2(ROOT/name,root/"source"/name);provenance["source_sha256"][name]=digest(ROOT/name)
    write(root/"provenance.json",provenance)
    shutil.copy2(PROTOCOL,root/"protocol.md")
    (root/"frozen").mkdir()
    for src,dst in [(perception_path,"perception.pt"),(V3/"models/residual.pt","V3-residual.pt")]:
        shutil.copy2(src,root/"frozen"/dst)
    config={"schema":"b601-image-residual-development-v1", "hardware_executable":False,"epochs":300,"seed":20261022,
            "perception_sha256":digest(perception_path), "scalar_policy_sha256":digest(V3/"models/residual.pt"),
            "protocol_sha256":digest(root/"protocol.md"), "datasets_sha256":EXPECTED_DATA,
            "parameters":6419, "patch_size":PATCH_SIZE,"bound_m":BOUND_M,"selection":"minimum validation mean including epoch0"}
    write(root/"config.json",config)
    perception=perception_model(root/"frozen/perception.pt");scalar=load(root/"frozen/V3-residual.pt")
    data,rows={},{}
    for split in EXPECTED_DATA:
        data[split],rows[split]=samples(split,perception)
        np.savez_compressed(root/f"{split}.npz",**data[split],metadata_json=json.dumps(rows[split]))
        print(json.dumps({"collected":split,"images":len(rows[split]),"samples":len(data[split]["x"])}),flush=True)
    assert not set(data["training"]["seeds"].tolist()).intersection(data["validation"]["seeds"].tolist())
    torch.manual_seed(config["seed"])
    model=ImageResidual();model.initialize_scalar(scalar)
    assert sum(p.numel() for p in model.parameters())==config["parameters"]
    x,p,y=[torch.from_numpy(data["training"][k]) for k in ("x","patches","y")]
    vx,vp,vy=[torch.from_numpy(data["validation"][k]) for k in ("x","patches","y")]
    initial=output(model,vx,vp)
    with torch.no_grad(): baseline=scalar(vx).numpy()
    np.testing.assert_allclose(initial,baseline,rtol=0,atol=3e-9)
    history=[{"epoch":0,"validation":error_metrics(initial,vy.numpy())}]
    best=history[0]["validation"]["mean_mm"];state=copy.deepcopy(model.state_dict());selected=0
    optimizer=torch.optim.Adam(model.parameters(),lr=.002,weight_decay=1e-5)
    for epoch in range(1,301):
        losses=[];model.train()
        for ids in torch.randperm(len(x)).split(64):
            loss=((model(x[ids],p[ids])-y[ids])*1000).square().mean()
            optimizer.zero_grad();loss.backward();optimizer.step();losses.append(float(loss.detach()))
        model.eval();metric=error_metrics(output(model,vx,vp),vy.numpy())
        history.append({"epoch":epoch,"training_loss_mm2":float(np.mean(losses)),"validation":metric})
        if metric["mean_mm"]<best:best=metric["mean_mm"];state=copy.deepcopy(model.state_dict());selected=epoch
        if epoch%25==0:print(json.dumps({"epoch":epoch,"validation":metric,"best_epoch":selected}),flush=True)
    model.load_state_dict(state);model.eval()
    torch.save({"schema":"b601-image-bounded-residual-v1","state_dict":state,"bound_m":BOUND_M,
                "perception_sha256":config["perception_sha256"],"selected_epoch":selected,"hardware_executable":False},root/"residual.pt")
    write(root/"training_history.json",history)
    reports={split:assess(data[split],rows[split],scalar,model) for split in EXPECTED_DATA}
    checks=gates(reports["validation"],selected)
    result={"schema":config["schema"],"development_only":True,"selected_epoch":selected,"parameters":config["parameters"],
            "reports":reports,"checks":checks,"eligible_for_task_development":all(checks.values()),"adopted":False,
            "perception_sha256":config["perception_sha256"],"checkpoint_sha256":digest(root/"residual.pt"),
            "source_data_sha256":EXPECTED_DATA,"feature_data_sha256":{s:digest(root/f"{s}.npz") for s in EXPECTED_DATA},
            "protocol_sha256":config["protocol_sha256"],"scope":"Reused-image offline development; no full tasks, risk gate, fresh acceptance, or hardware claim."}
    write(root/"summary.json",result)
    print(json.dumps({"complete":True,"selected_epoch":selected,"validation":{k:v for k,v in reports["validation"].items() if k not in ("rows","by_view")},"checks":checks,"eligible_for_task_development":result["eligible_for_task_development"]}),flush=True)


def audit(root):
    root=root.resolve();config=json.loads((root/"config.json").read_text());result=json.loads((root/"summary.json").read_text())
    provenance=json.loads((root/"provenance.json").read_text())
    for name,sha in provenance["source_sha256"].items():
        assert digest(root/"source"/Path(name).name)==digest(ROOT/name)==sha,name
    assert digest(root/"protocol.md")==config["protocol_sha256"]==result["protocol_sha256"]
    assert digest(root/"frozen/perception.pt")==config["perception_sha256"]
    assert digest(root/"frozen/V3-residual.pt")==config["scalar_policy_sha256"]
    assert digest(root/"residual.pt")==result["checkpoint_sha256"]
    ck=torch.load(root/"residual.pt",map_location="cpu",weights_only=True)
    assert ck["bound_m"]==BOUND_M and ck["perception_sha256"]==config["perception_sha256"] and not ck["hardware_executable"]
    model=ImageResidual();model.load_state_dict(ck["state_dict"]);model.eval()
    scalar=load(root/"frozen/V3-residual.pt");perception=perception_model(root/"frozen/perception.pt")
    seed_sets={};count=0;maximum_patch_difference=0.
    for split in EXPECTED_DATA:
        arrays,rows=samples(split,perception)
        seed_sets[split]=set(arrays["seeds"].tolist())
        assert digest(root/f"{split}.npz")==result["feature_data_sha256"][split]
        with np.load(root/f"{split}.npz",allow_pickle=False) as saved:
            for key,value in arrays.items():assert np.array_equal(value,saved[key]),(split,key)
            assert json.loads(str(saved["metadata_json"]))==rows
        reconstructed=assess(arrays,rows,scalar,model)
        assert reconstructed==result["reports"][split],split
        with np.load(DATA/f"{split}.npz",allow_pickle=False) as source:
            rgb,depth=source["rgb"],source["depth_m"]
        # Independent bilinear interpolation implementation, no model/label input.
        for j,i in enumerate(arrays["sample_indices"]):
            pred=rows[i]["raw_prediction"];u,v=pred["pixel_uv"]
            dy,dx=np.meshgrid(np.arange(32)-15.5+v,np.arange(32)-15.5+u,indexing="ij")
            channels=[rgb[i,:,:,c].astype(np.float32)/255 for c in range(3)]
            channels.append(np.clip((depth[i]-pred["depth_m"])/.05,-2,2))
            expected=np.stack([map_coordinates(c,[dy,dx],order=1,mode="nearest",prefilter=False) for c in channels])
            difference=float(np.abs(expected-arrays["patches"][j]).max())
            maximum_patch_difference=max(maximum_patch_difference,difference)
            assert difference<5e-5,(split,int(i),difference)
            count+=1
    assert not seed_sets["training"].intersection(seed_sets["validation"])
    history=json.loads((root/"training_history.json").read_text())
    selected=min(history,key=lambda h:h["validation"]["mean_mm"])["epoch"]
    assert selected==ck["selected_epoch"]==result["selected_epoch"]
    assert gates(result["reports"]["validation"],selected)==result["checks"]
    assert result["eligible_for_task_development"]==all(result["checks"].values()) and not result["adopted"]
    report={"passed":True,"images_recomputed":sum(r["images"] for r in result["reports"].values()),
            "supervised_patches_independently_checked":count,"maximum_patch_input_difference":maximum_patch_difference,
            "no_train_validation_seed_overlap":True,"source_files_checked":len(provenance["source_sha256"]),
            "scope":"Exact saved-image predictions/features/labels/model outputs plus independent patch interpolation; no physics/full-task/hardware validation."}
    write(root/"audit.json",report);print(json.dumps(report),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--name");group.add_argument("--audit-run",type=Path)
    args=parser.parse_args()
    if args.audit_run:audit(ROOT/"outputs"/args.audit_run)
    else:train(args)
