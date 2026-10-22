#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
LAUNCHER="${ROOT_DIR}/launchers/rebotarm-b601-rs.desktop"
DESKTOP_DIR="${XDG_DESKTOP_DIR:-${HOME}/Desktop}"
APP_DIR="${XDG_DATA_HOME:-${HOME}/.local/share}/applications"

mkdir -p "${DESKTOP_DIR}" "${APP_DIR}"
install -m 755 "${LAUNCHER}" "${DESKTOP_DIR}/rebotarm-b601-rs.desktop"
install -m 644 "${LAUNCHER}" "${APP_DIR}/rebotarm-b601-rs.desktop"
chmod +x "${DESKTOP_DIR}/rebotarm-b601-rs.desktop"
echo "桌面启动器已安装：${DESKTOP_DIR}/rebotarm-b601-rs.desktop"
echo "应用菜单启动器已安装：${APP_DIR}/rebotarm-b601-rs.desktop"
