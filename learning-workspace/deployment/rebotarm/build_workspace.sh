#!/usr/bin/env bash
set -Eeo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
# Build in a clean ROS environment, independent of any active user overlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH PYTHONHOME
export ROS_DISTRO=jazzy
source /opt/ros/jazzy/setup.bash
WS="${ROOT}/rebotarm_ros2_RS"
if [[ ! -x "${WS}/.venv/bin/python" ]] || ! "${WS}/.venv/bin/python" -m pip --version >/dev/null 2>&1; then
  /usr/bin/python3 -m venv --system-site-packages "${WS}/.venv"
fi
"${WS}/.venv/bin/python" -m pip install --only-binary=:all: --require-hashes -r "${ROOT}/deployment/requirements.lock"
source "${WS}/.venv/bin/activate"
export PYTHONPATH="${WS}/.venv/lib/python3.12/site-packages/cmeel.prefix/lib/python3.12/site-packages:${WS}/.venv/lib/python3.12/site-packages:${PYTHONPATH:-}"
cd "${WS}"
# Generated launch scripts must use the project Python, which includes the
# system rclpy ABI and the pinned simulation/control libraries.
python3 /usr/bin/colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DPython3_EXECUTABLE="${WS}/.venv/bin/python3" -DPython_EXECUTABLE="${WS}/.venv/bin/python3" \
  -DBUILD_TESTING=OFF
