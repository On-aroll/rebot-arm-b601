#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
source scripts/rs_env.sh
node -e 'if(typeof WebSocket!=="function" || typeof fetch!=="function")process.exit(1); console.log("PASS Node",process.version,"WebSocket / fetch")'
python3 deployment/doctor.py
