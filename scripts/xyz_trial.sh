#!/usr/bin/env bash
set -euo pipefail
REBOT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
REBOT_TRIAL_PY=/usr/bin/python3
for REBOT_ARGUMENT in "$@"; do
    if [[ "$REBOT_ARGUMENT" == --plan ]]; then
        REBOT_TRIAL_PY="${REBOT_SIM_PY:-$HOME/.local/share/robotics/envs/rebot-sim/bin/python}"
    fi
done
export PYTHONPATH="$REBOT_ROOT/scripts:$REBOT_ROOT/.venv/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
# Separate the established Pinocchio/MuJoCo interpreter from the CAN SDK ABI.
if [[ "$REBOT_TRIAL_PY" != /usr/bin/python3 ]]; then
    export PYTHONPATH="$REBOT_ROOT/scripts"
fi
exec "$REBOT_TRIAL_PY" "$REBOT_ROOT/scripts/rebot_xyz_trial.py" "$@"
