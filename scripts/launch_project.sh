#!/usr/bin/env bash
# Desktop entry wrapper; keep diagnostics visible after startup.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1
python3 scripts/start_project.py "$@"
RESULT=$?
if [ -t 0 ]; then
  printf '\n按 Enter 关闭此窗口；后台服务会继续运行。\n'
  read -r _ || true
fi
exit "$RESULT"
