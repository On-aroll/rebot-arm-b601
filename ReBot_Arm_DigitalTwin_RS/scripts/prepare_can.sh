#!/usr/bin/env bash
# No persistent privilege rules. Polkit asks the local user when needed.
set -Eeuo pipefail
umask 077
CAN_OWNER_DIR="/tmp/rebot-can-owner-${UID}"
mkdir -p "${CAN_OWNER_DIR}"
exec 8>"${CAN_OWNER_DIR}/can0.lock"
flock -n 8 || { echo 'CAN 正由控制器使用，禁止重新配置。' >&2; exit 1; }
/usr/bin/python3 "$(dirname "${BASH_SOURCE[0]}")/can_interface.py" --prepare
