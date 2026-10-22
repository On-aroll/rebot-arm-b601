#!/usr/bin/env bash
# Fixed local launch actions used by the project startup page.
set -Eeuo pipefail
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
