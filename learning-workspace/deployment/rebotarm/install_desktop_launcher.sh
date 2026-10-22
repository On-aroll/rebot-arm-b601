#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# The installer writes both the desktop and application menu entries using
# this computer's paths. Re-run the installer to repair the shortcuts.
exec /usr/bin/python3 "${ROOT}/deployment/install.py" --skip-system --no-launch
