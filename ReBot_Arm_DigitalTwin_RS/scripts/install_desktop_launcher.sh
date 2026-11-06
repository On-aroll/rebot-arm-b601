#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
LAUNCHER="${ROOT_DIR}/launchers/rebotarm-b601-rs.desktop"
DESKTOP_DIR="${XDG_DESKTOP_DIR:-$(xdg-user-dir DESKTOP 2>/dev/null || printf '%s/Desktop' "${HOME}")}"
APP_DIR="${XDG_DATA_HOME:-${HOME}/.local/share}/applications"

mkdir -p "${DESKTOP_DIR}" "${APP_DIR}"
python3 - "${ROOT_DIR}" "${LAUNCHER}" "${DESKTOP_DIR}" "${APP_DIR}" <<'PY'
from pathlib import Path
import sys

root, template, desktop, applications = map(Path, sys.argv[1:])
command = str(root / 'scripts/quick_start.sh')
for character in ('\\', '"', '`', '$'):
    command = command.replace(character, '\\' + character)
command = command.replace('%', '%%')
fields = {'Path': str(root), 'Exec': '"' + command + '" --hardware',
          'Icon': str(root / 'reBotArm_simulator-RS/public/favicon.png')}
lines = []
for line in template.read_text().splitlines():
    key = line.split('=', 1)[0]
    lines.append(key + '=' + fields[key] if key in fields else line)
content = '\n'.join(lines) + '\n'
for directory, mode in ((desktop, 0o755), (applications, 0o644)):
    target = directory / template.name
    target.write_text(content)
    target.chmod(mode)
PY
if command -v gio >/dev/null 2>&1; then
  gio set "${DESKTOP_DIR}/rebotarm-b601-rs.desktop" metadata::trusted true 2>/dev/null || true
fi
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${APP_DIR}"
fi
echo "桌面启动器已安装：${DESKTOP_DIR}/rebotarm-b601-rs.desktop"
echo "应用菜单启动器已安装：${APP_DIR}/rebotarm-b601-rs.desktop"
