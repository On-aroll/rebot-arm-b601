"""Run the latest verified simulation release from its frozen code and assets."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", choices=("A", "B", "C"), required=True)
    parser.add_argument("--seed", type=int, default=24000000)
    parser.add_argument("--name")
    parser.add_argument("--disturbed", action="store_true", help="Apply the evaluated simulation force pulse")
    args = parser.parse_args()
    if args.seed < 0:
        parser.error("seed must be nonnegative")
    latest = json.loads((ROOT/"LATEST_VERIFIED.json").read_text())
    runtime = ROOT/latest["runtime_directory"]
    manifest_path = ROOT/latest["runtime_manifest"]
    if digest(manifest_path) != latest["runtime_manifest_sha256"]:
        raise ValueError("Verified release manifest changed")
    manifest = json.loads(manifest_path.read_text())
    if manifest["hardware_executable"] or manifest["version"] != latest["version"]:
        raise ValueError("Invalid simulation release")
    for relative, expected in manifest["files_sha256"].items():
        if digest(runtime/relative) != expected:
            raise ValueError(f"Verified release file changed: {relative}")
    name = args.name or f"{latest['version'].lower()}-{datetime.now(timezone.utc):%Y%m%dT%H%M%S%f}"
    command = [str(ROOT/".venv/bin/python"), "-u", str(runtime/"task_runner.py"),
               "demo", "--name", name, "--goal", args.goal, "--seed", str(args.seed),
               "--policy-run", str(runtime/"models"),
               "--perception", str(runtime/"models/perception.pt"),
               "--observation-mode", "feedback", "--feedback-config", str(runtime/"feedback_config.json")]
    if args.disturbed:
        command += ["--force-pulse", str(runtime/"force_pulse.json")]
    environment = dict(os.environ, MUJOCO_GL="egl", OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1")
    subprocess.run(command, check=True, cwd=runtime, env=environment)
    print(f"Artifacts: {ROOT/'outputs'/name}", flush=True)


if __name__ == "__main__":
    main()
