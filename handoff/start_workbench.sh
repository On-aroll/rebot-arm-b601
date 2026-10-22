#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../ReBot_Arm_DigitalTwin_RS" && pwd)"
export REBOTARM_GATEWAY_ROOT="$project_root"
cd "$project_root"
exec ./rebotarm start web
