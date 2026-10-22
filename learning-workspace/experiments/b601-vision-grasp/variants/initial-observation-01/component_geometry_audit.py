"""Posthoc reconstruction of all declared geometry image-development outputs."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from component_geometry_probe import VARIANTS, metrics, predictions, probability
from feedback import quality
from feedback_audit import same
from policy import load, valid_prediction
from sim import ROOT
from task_runner import digest, perception_model, write
from vision import estimate


def audit(root):
    plan=json.loads((root/"config.json").read_text())
    summary=json.loads((root/"summary.json").read_text())
    assert summary["complete"] and plan["development_only"] and not plan["full_task_test"]
    for key,value in plan.items():
        same(summary[key],value)
    provenance=json.loads((root/"provenance.json").read_text())
    for relative,sha in provenance["source_sha256"].items():
        assert digest(ROOT/relative)==digest(root/"source"/Path(relative).name)==sha
    manifest=json.loads((ROOT/"assets/manifest.json").read_text())
    assert all(digest(ROOT/"assets"/p)==sha for p,sha in manifest["files_sha256"].items())
    assert digest(root/"perception.pt")==plan["perception_sha256"]
    assert digest(root/"residual.pt")==plan["policy_sha256"]
    model,policy=perception_model(root/"perception.pt"),load(root/"residual.pt")
    images=0
    for split,source in plan["datasets"].items():
        assert digest(source["path"])==source["sha256"]
        expected=summary["rows"][split]
        assert set(expected)==set(VARIANTS)
        actual={name:[] for name in VARIANTS}
        with np.load(source["path"],allow_pickle=False) as data:
            rgb,depth=data["rgb"],data["depth_m"]
            metadata=json.loads(str(data["metadata_json"]))
            assert all(len(rows)==len(metadata) for rows in expected.values())
            for i,meta in enumerate(metadata):
                frame={"rgb":rgb[i],"depth":depth[i],"camera":meta["camera"]}
                outputs=predictions(probability(model,frame),frame)
                same(outputs["legacy"],estimate(model,frame))
                for name,prediction in outputs.items():
                    row={"index":i,"seed":meta["seed"],"view":meta.get("arm_initialization_view",meta.get("view","legacy_HOME")),
                        "expected_target":not meta["absent"],"prediction":prediction,"valid":valid_prediction(prediction),
                        "quality":quality(prediction,frame["camera"],plan["feedback_config"])}
                    if row["expected_target"] and row["valid"]:
                        truth=np.asarray(meta["initial_xyz"]).copy();truth[2]+=meta["half_size_m"]
                        row["corrected_error_mm"]=float(np.linalg.norm(np.asarray(policy.correct(prediction)["top_xyz_m"])-truth)*1000)
                    same(row,expected[name][i]);actual[name].append(row)
                images+=1
        for name,rows in actual.items():
            same(metrics(rows),summary["metrics"][split][name])
    for known in summary["known_cases"]:
        assert digest(known["posthoc_source"])==known["source_sha256"]
        with np.load(known["posthoc_source"],allow_pickle=False) as data:
            cameras=json.loads(str(data["camera_json"]))
            frame={"rgb":data["rgb"][0],"depth":data["depth_m"][0],"camera":cameras[0]}
        same(predictions(probability(model,frame),frame),known["predictions"])
    saved=root/"component_geometry_audit_source.py";shutil.copy2(Path(__file__),saved)
    report={"passed":True,"posthoc_audit":True,"development_only":True,
        "images_recomputed":images,"variants_per_image":len(VARIANTS),"legacy_model_output_parity_images":images,
        "known_decision_frames_recomputed":len(summary["known_cases"]),"asset_files_checked":len(manifest["files_sha256"]),
        "audit_source_sha256":digest(saved),
        "scope":"Inference and metrics reconstruction from saved development images; not image regeneration, physical tasks or adoption evaluation."}
    write(root/"audit.json",report);print(json.dumps(report),flush=True)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path,required=True)
    audit(parser.parse_args().run.resolve())
