#!/usr/bin/env bash
set -euo pipefail
experiment_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$experiment_root"
if [[ ! -x .venv/bin/python ]]; then
  uv venv --python 3.12 .venv
  uv pip install --python .venv/bin/python -r requirements.lock \
    --extra-index-url https://download.pytorch.org/whl/cpu --index-strategy unsafe-best-match
fi
export MUJOCO_GL="${MUJOCO_GL:-egl}"
exec .venv/bin/python -u experiment.py "$@"
