#!/usr/bin/env bash
set -euo pipefail
experiment_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$experiment_root/.venv/bin/python" "$experiment_root/verified_task.py" "$@"
