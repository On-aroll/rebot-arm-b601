#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/rs_env.sh
# This service only observes until the page explicitly prepares/acquires and
# supplies a motion input. It never calls the legacy auto-hold/home launch.
exec 8>"${XDG_RUNTIME_DIR:-/tmp}/rebotarm-csp-twin.lock"
flock -n 8 || { echo 'CSP twin is already running' >&2; exit 1; }
exec python3 -m rebotarmcontroller.csp_twin_driver
