"""Run an archived auditor with its exact saved Python sources and assets."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


def main(args):
    workspace = Path(__file__).resolve().parent
    run = (workspace/"outputs"/args.run).resolve()
    provenance = json.loads((run/"provenance.json").read_text())
    with tempfile.TemporaryDirectory(prefix="b601-frozen-audit-") as temporary:
        root = Path(temporary)
        (root/"assets").mkdir()
        for source in (workspace/"assets").iterdir():
            (root/"assets"/source.name).symlink_to(source, target_is_directory=source.is_dir())
        for name, expected in provenance["source_sha256"].items():
            saved = run/"source"/Path(name).name
            assert hashlib.sha256(saved.read_bytes()).hexdigest() == expected, name
            destination = root/name
            destination.parent.mkdir(parents=True, exist_ok=True)
            if destination.is_symlink():
                destination.unlink()
            shutil.copy2(saved,destination)
        if args.auditor=="component_geometry_audit.py":
            saved=run/"component_geometry_audit_source.py"
            report=json.loads((run/"audit.json").read_text())
            assert hashlib.sha256(saved.read_bytes()).hexdigest()==report["audit_source_sha256"]
            shutil.copy2(saved,root/args.auditor)
        (root/"outputs").symlink_to(workspace/"outputs", target_is_directory=True)
        argument="--audit-run" if args.auditor in ("home_first_diagnostic.py","initial_visibility_probe.py","geometry_known_development.py","image_residual_development.py","image_residual_probe_review.py","image_task_known.py") else "--run"
        subprocess.run([sys.executable,str(root/args.auditor),argument,str(run)],check=True,cwd=root)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",required=True)
    parser.add_argument("--auditor",choices=["feedback_audit.py","multiview_audit.py","home_first_diagnostic.py","home_refresh_audit.py","demo_audit.py","initial_visibility_probe.py","component_geometry_audit.py","geometry_known_development.py","image_residual_development.py","image_residual_probe_review.py","image_task_known.py"],default="feedback_audit.py")
    main(parser.parse_args())
