#!/usr/bin/env bash
# Explicit terminal entry; project startup never calls this script.
set -e
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
REBOT_SDK_SITE="$ROOT_DIR/.venv/lib/python3.12/site-packages"
if [ ! -d "$REBOT_SDK_SITE/motorbridge" ]; then
  echo '缺少已审核的 motorbridge 环境；不重新安装或连接实机。' >&2
  exit 1
fi
export PYTHONPATH="$ROOT_DIR/scripts:$REBOT_SDK_SITE"
exec /usr/bin/python3 "$ROOT_DIR/scripts/rebot_seeed_trial.py" "$@"
