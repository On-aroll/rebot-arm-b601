#!/usr/bin/env bash
# Fixed local launch actions used by the project startup page.
set -Eeuo pipefail
# OpenCV is linked against the system NumPy ABI, outside the control venv.
if [[ "${1:-}" == camera ]]; then
  REBOTARM_CAMERA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${REBOTARM_ROS_DISCOVERY_RANGE:-LOCALHOST}"
  cd "${REBOTARM_CAMERA_ROOT}"
  exec 7>"${XDG_RUNTIME_DIR:-/tmp}/rebotarm-gemini-color.lock"
  flock -n 7 || { echo 'Gemini color bridge is already running' >&2; exit 1; }
  exec /usr/bin/python3 scripts/orbbec_ros_camera_bridge.py --device auto --fps 8 --jpeg-quality 70
fi
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/rs_env.sh"
cd "${REBOTARM_RS_ROOT}"
case "${1:-}" in
  check)
    ros2 pkg prefix rebotarmcontroller
    ros2 pkg prefix rebotarm_mujoco_rs
    ros2 pkg prefix rosbridge_server
    python3 -c 'import rclpy, mujoco, ruckig; print("ROS 2 / workspace / Python dependencies ready")'
    ;;
  bridge)
    exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 address:=127.0.0.1
    ;;
  simulation)
    export REBOTARM_START_AGENT=false REBOTARM_START_ROSBRIDGE=false
    export REBOTARM_MUJOCO_VIEWER=false MUJOCO_GL=egl
    exec "${REBOTARM_RS_ROOT}/scripts/start_rs_sim.sh"
    ;;
  mcp)
    exec ros2 launch rebotarm_agent rebotarm_mcp.launch.py arm_namespace:=rebotarm_rs motion_mode:=allow host:=127.0.0.1 port:=8081
    ;;
  text)
    export REBOTARM_AGENT_HTTP_HOST=127.0.0.1 REBOTARM_NAMESPACE=rebotarm_rs
    exec "${REBOTARM_RS_ROOT}/scripts/start_rs_text_agent.sh"
    ;;
  *) echo 'Unknown startup component' >&2; exit 2 ;;
esac
