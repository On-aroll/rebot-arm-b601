#!/usr/bin/env bash
set -e
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
UPSTREAM="$ROOT_DIR/external/ReBot_Arm_DigitalTwin_RS"
EXPECTED=aa807cf2affe85b13b86221159871aae7742c01b
if [ "$(git -C "$UPSTREAM" rev-parse HEAD)" != "$EXPECTED" ] || [ -n "$(git -C "$UPSTREAM" status --porcelain --untracked-files=no)" ]; then
  echo 'ROS 上游版本不符或有修改，请先核对锁定版本。' >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/external/ReBot_Arm_DigitalTwin_RS/rebotarm_ros2_RS/install/local_setup.bash"
export ROS_DOMAIN_ID=43 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="$ROOT_DIR/outputs/mit-p5/ros-logs"
exec /usr/bin/python3 "$ROOT_DIR/scripts/rebot_ros_validation.py"
