#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# reBot Arm B601-RS · 可视化一键启动
#
# 用法:  bash scripts/view.sh [选项]
#        不带参数 = 列出全部选项
#
# 详细说明见 docs/可视化速查_2026-10-06.md
# ---------------------------------------------------------------------------
set -uo pipefail


PROJ="/home/bonvoyage/Desktop/seeed Robotics"
ROS2_WS="$PROJ/external/reBotArmController_ROS2"
MUJOCO_PY="$HOME/.local/share/robotics/py312"
MUJOCO_MODEL="$ROS2_WS/src/rebotarm_mujoco_rs/models/rs_grasp_scene.xml"
SIM_PY="$HOME/.local/share/robotics/envs/rebot-sim/bin/python"
OFFICIAL="$PROJ/external/reBotArm_control_py/example/sim/fk_sim.py"

die() { printf '\n[XX] %s\n' "$*" >&2; exit 1; }

usage() {
  cat <<'EOF'

reBot 可视化 —— 选一个：

  自建（本项目工具）
    dashboard , d    网页控制台，滑块+按钮        http://127.0.0.1:8765
    meshcat   , m    纯 3D 视图（只能拖视角）      http://127.0.0.1:7000/static/
    video     , v    出离线 mp4 到 outputs/（不开窗口）

  官方（Seeed）
    official  , o    官方 Pinocchio 示例           http://127.0.0.1:7000/static/
                     ⚠ 控制入口在【终端】，在提示符「关节角度 >」后敲 6 个数字
    rviz      , r    ROS 2 + RViz 看 URDF         桌面窗口
    mujoco    , j    ROS 2 + MuJoCo 原生窗口      桌面窗口

  例子
    bash scripts/view.sh dashboard
    bash scripts/view.sh official

EOF
}

# ROS 2 公共环境。mujoco 装在系统 Python 3.12 之外的目标目录，必须走 PYTHONPATH。
ros2_env() {
  [ -f /opt/ros/jazzy/setup.bash ] || die "未找到 /opt/ros/jazzy —— ROS 2 尚未安装"
  [ -f "$ROS2_WS/install/setup.bash" ] || die "未找到 $ROS2_WS/install —— 请先 colcon build"

  export ROS_HOME="$PROJ/.ros"
  export ROS_LOG_DIR="$PROJ/.ros/log"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
  export PYTHONPATH="$MUJOCO_PY${PYTHONPATH:+:$PYTHONPATH}"
  mkdir -p "$ROS_HOME"

  set +u
  # shellcheck disable=SC1091
  . /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  . "$ROS2_WS/install/setup.bash"
  set -u
}

case "${1:-}" in
  dashboard|d)
    exec "$HOME/.local/bin/rebot-sim" serve
    ;;

  meshcat|m)
    exec "$HOME/.local/bin/rebot-sim" meshcat
    ;;

  video|v)
    cd "$PROJ" || die "进不去 $PROJ"
    "$HOME/.local/bin/rebot-sim" demo --kind circle --video
    "$HOME/.local/bin/rebot-sim" grasp --video
    echo
    echo "成品在 $PROJ/outputs/ ：circle_demo.mp4 / grasp_demo.mp4"
    ;;

  official|o)
    [ -f "$OFFICIAL" ] || die "未找到官方示例：$OFFICIAL"
    cd "$PROJ" || die "进不去 $PROJ"
    cat <<'TIP'

官方示例启动后，Meshcat 网页只是显示器。
控制入口在【本终端】：等出现「关节角度 > 」后，敲 6 个数字再回车。
  例：0 90 90 0 0 0     （大臂展开，向上前方伸出）
  例：45 90 45 0 0 0    （底座转向侧方）
退出：敲 q 回车

TIP
    exec "$SIM_PY" "$OFFICIAL"
    ;;

  rviz|r)
    ros2_env
    cd "$ROS2_WS" || die "进不去 $ROS2_WS"
    echo "[==] 启动 ROS 2 + RViz（use_rviz:=true）"
    exec ros2 launch rebotarm_bringup fake_rs_bringup.launch.py use_rviz:=true
    ;;

  mujoco|j)
    ros2_env
    [ -d "$MUJOCO_PY/mujoco" ] || die "未找到 $MUJOCO_PY/mujoco
     一次性安装：
       mkdir -p ~/.local/share/robotics
       mv \"$PROJ/downloads/_pkg2\" ~/.local/share/robotics/py312
     或重装：
       pip install --target=$MUJOCO_PY --no-deps mujoco==3.15.0 absl-py etils glfw pyopengl"
    cd "$ROS2_WS" || die "进不去 $ROS2_WS"
    echo "[==] 启动 ROS 2 + MuJoCo 窗口"
    echo "     模型: $MUJOCO_MODEL"
    exec ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py \
      model_path:="$MUJOCO_MODEL" use_viewer:=true
    ;;

  ""|-h|--help|help)
    usage
    ;;

  *)
    printf '\n[XX] 未知选项: %s\n' "$1" >&2
    usage
    exit 1
    ;;
esac
