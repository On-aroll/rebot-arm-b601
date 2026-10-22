#!/usr/bin/env bash
set -euo pipefail
experiment_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$experiment_root"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec .venv/bin/python -u task_runner.py "$@"
