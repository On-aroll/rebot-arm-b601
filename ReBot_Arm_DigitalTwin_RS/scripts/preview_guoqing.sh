#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/rs_env.sh
python3 scripts/preview_writing_ros.py &
writing_preview_pid=$!
trap 'kill "$writing_preview_pid" 2>/dev/null || true' EXIT
rviz2 -d examples/writing/guoqing.rviz
