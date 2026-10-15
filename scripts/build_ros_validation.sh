#!/usr/bin/env bash
# Build only the upstream packages required by the isolated Fake validation.
set -e
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$ROOT_DIR/external/ReBot_Arm_DigitalTwin_RS/rebotarm_ros2_RS"
colcon build --packages-select rebotarm_msgs rebotarmcontroller --symlink-install --executor sequential --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
