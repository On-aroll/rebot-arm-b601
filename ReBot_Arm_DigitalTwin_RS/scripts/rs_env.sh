#!/usr/bin/env bash

set -eo pipefail

REBOTARM_RS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${ROS_DISTRO:-}" ]]; then
  REBOTARM_ROS_DISTRO="${ROS_DISTRO}"
elif [[ -r /etc/os-release ]]; then
  # shellcheck source=/dev/null
  source /etc/os-release
  case "${ID:-}:${VERSION_ID:-}" in
    ubuntu:22.04) REBOTARM_ROS_DISTRO=humble ;;
    *) REBOTARM_ROS_DISTRO=jazzy ;;
  esac
else
  REBOTARM_ROS_DISTRO=jazzy
fi
REBOTARM_ROS_SETUP="/opt/ros/${REBOTARM_ROS_DISTRO}/setup.bash"
REBOTARM_WS_SETUP="${REBOTARM_RS_ROOT}/rebotarm_ros2_RS/install/setup.bash"
REBOTARM_VENV_ACTIVATE="${REBOTARM_RS_ROOT}/rebotarm_ros2_RS/.venv/bin/activate"

if [[ ! -f "${REBOTARM_ROS_SETUP}" ]]; then
  echo "ROS 2 environment not found: ${REBOTARM_ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${REBOTARM_WS_SETUP}" ]]; then
  echo "RS workspace has not been built: ${REBOTARM_WS_SETUP}" >&2
  echo "Run scripts/setup_rs_workspace.sh first." >&2
  exit 1
fi

if [[ ! -f "${REBOTARM_VENV_ACTIVATE}" ]]; then
  echo "RS Python environment has not been created: ${REBOTARM_VENV_ACTIVATE}" >&2
  echo "Run scripts/setup_rs_workspace.sh first." >&2
  exit 1
fi

REBOTARM_RESTORE_NOUNSET=false
if [[ $- == *u* ]]; then
  REBOTARM_RESTORE_NOUNSET=true
  set +u
fi

source "${REBOTARM_ROS_SETUP}"

# All ROS nodes in this project (controller, rosbridge and MuJoCo bridge) run
# on this computer.  Fast DDS otherwise selects a physical interface when a
# process starts; after Wi-Fi roaming/address changes, old and newly-started
# processes can advertise different, unreachable addresses and disappear
# from each other's ROS graph even though WebSocket itself is still online.
# New rosbridge launches bind 127.0.0.1; browser access is local by default.
# Set REBOTARM_ROS_DISCOVERY_RANGE=SUBNET only when using ROS nodes on another
# computer intentionally.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${REBOTARM_ROS_DISCOVERY_RANGE:-LOCALHOST}"

source "${REBOTARM_VENV_ACTIVATE}"
# httpx (used by MCP) rejects IPv6 /128 entries in NO_PROXY. A /128 network
# is exactly one address. Normalize this launch only, preserving other hosts.
for REBOTARM_PROXY_KEY in NO_PROXY no_proxy; do
  if [[ -v "${REBOTARM_PROXY_KEY}" ]]; then
    REBOTARM_PROXY_VALUE="$(python3 - "${REBOTARM_PROXY_KEY}" <<'PY'
import ipaddress
import os
import sys

def normalize(entry):
    try:
        address = ipaddress.IPv6Interface(entry.strip().strip('[]'))
        if address.network.prefixlen == 128:
            return str(address.ip)
    except ValueError:
        pass
    return entry

print(','.join(normalize(entry) for entry in os.environ[sys.argv[1]].split(',')))
PY
)"
    export "${REBOTARM_PROXY_KEY}=${REBOTARM_PROXY_VALUE}"
  fi
done
unset REBOTARM_PROXY_KEY REBOTARM_PROXY_VALUE
REBOTARM_VENV_SITE_PACKAGES="$(python3 -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
REBOTARM_PYTHON_ABI="$(python3 -c 'import sys; print(f"python{sys.version_info.major}.{sys.version_info.minor}")')"
REBOTARM_CMEEL_SITE_PACKAGES="${REBOTARM_VENV_SITE_PACKAGES}/cmeel.prefix/lib/${REBOTARM_PYTHON_ABI}/site-packages"
export PYTHONPATH="${REBOTARM_CMEEL_SITE_PACKAGES}:${REBOTARM_VENV_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"
source "${REBOTARM_WS_SETUP}"

if [[ "${REBOTARM_RESTORE_NOUNSET}" == true ]]; then
  set -u
fi
unset REBOTARM_RESTORE_NOUNSET
unset REBOTARM_CMEEL_SITE_PACKAGES
unset REBOTARM_PYTHON_ABI
unset REBOTARM_VENV_SITE_PACKAGES
