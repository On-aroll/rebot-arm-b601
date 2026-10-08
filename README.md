# reBot Arm B601-RS 仿真工程

当前可以执行多步骤物理动作程序：抓取、抬升、搬运、放置并抓回原位。支持 Python / JSON 编排、暂停继续、取消、仿真反馈判定与任务记录。已整合本地 Studio 和实机参数读取；仿真轨迹驱动实机尚待接入与验收。

- **[项目重启指南：日常启动、实机连接、排障与当前进度](docs/项目重启指南.md)**
- [实机与模型对应：只读姿态与逐轴映射](docs/实机模型映射_2026-10-08.md)
- [三维目标点：操作与实机边界](docs/三维目标点_2026-10-08.md)
- [电机配置与自动规划](docs/电机配置与自动规划_2026-10-08.md)
- [reBot Studio 控制工作台说明](docs/控制工作台_2026-10-07.md)
- [虚实联调控制台（Studio /console 页）](docs/虚实联调控制台_2026-10-07.md)
- [国庆快乐：一笔三维轨迹](docs/国庆快乐_一笔三维轨迹.md)
- [动作程序使用与数字孪生接入说明](docs/动作程序与数字孪生_2026-10-06.md)
- [可编辑抓取程序](programs/pick_place.json)
- [Python 往返搬运示例](programs/pick_and_return.py)
- [仿真动作验证结果](outputs/program_verification.json)
- [ROS 相机验证结果](outputs/ros_camera_verification.json)
- [同一仿真源的 ROS 状态/TF 验证](outputs/ros_bridge_verification.json)

从本目录启动（服务已运行时无需重复）：

```bash
bash scripts/restart_sim.sh
```

打开 http://127.0.0.1:8765/ （原工作台）或 http://127.0.0.1:8765/zh/studio/console （虚实联调控制台）。另一个终端运行：

```bash
~/.local/share/robotics/envs/rebot-sim/bin/python programs/pick_and_return.py
```

仿真环境安装于 `~/.local/share/robotics`，实机网关使用项目内 `.venv`；代码、模型、缓存、报告在本目录。旧规划与参数研读记录保留在 `docs/`，当前功能以动作程序说明及测试报告为准。

将同一模拟机械臂的实际/目标状态发布到 ROS 2（不连接实机）：

```bash
source /opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=42 ros2 launch "$PWD/scripts/same_sim_ros.launch.py"
```
