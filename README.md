> **2026-10-14 当前入口**：[项目使用手册](docs/项目使用手册.md) 包含启动、网页 XYZ 离线执行、停止、国庆快乐书写、抓取及新电脑部署。主界面只有“离线验证 / 实机操作”；七轴统一确认、电源控制与初始状态检测已实现，真实 XYZ 仍待承重和停止等现场验收。以下旧记录按日期理解，不覆盖该手册的当前流程。

> **2026-10-09 P4 更新**：新一轮进展见 [P4交接](docs/P4交接_2026-10-09.md)。顶部只读连接、MuJoCo 状态同步及 SDK 适配器已实现；用户 3/1/0 增益只保存为待验证草稿。实机写入仍锁定，真实六轴与网页 XYZ 实发未验收；当前 J2 边界读数、现场支撑和停止条件须先核实。没有发送使能或运动。下方 B1/批次 A 记录保留历史含义。

> **2026-10-09 B1 更新**：以 [B1 交接](docs/B1交接_2026-10-09.md) 为最新入口。软件配置不等于真实 MIT 参数备份；旧页面恢复明确只读连接、写入仍锁定。独立六轴 Mock 已验收，仅用合成数据，无真实 CAN 执行；禁止据此开始实机试验。下方旧记录按日期理解。

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


## 批次 A：MIT 控制台与另一台电脑启动

最新边界与验收见 [批次 A 交接](docs/批次A交接_2026-10-08.md)。刷新页面不连接 CAN，真实执行始终 LOCKED。批准软件配置不会写入电机。七轴从摘要展开查看草稿和历史。

推荐 Ubuntu/Kubuntu 24.04、Python 3.12、Node 22.12+ 和 npm，系统需可用 EGL/OpenGL（无显示器也需 Mesa/EGL）。完整网页源码位于 `external/motorbridge-studio`，不是在线官网的 iframe；本地仿真后端必须运行，GitHub Pages 单独托管不能执行 Python 仿真。

```bash
git clone https://github.com/On-aroll/rebot-arm-b601.git
cd rebot-arm-b601
python3 scripts/setup_external.py
python3 -m venv .venv-sim
.venv-sim/bin/python -m pip install -r configs/requirements-lock.txt
cd external/motorbridge-studio
npm ci
VITE_BASE_PATH=/studio/ npm run build
cd ../..
REBOT_PYTHON="$PWD/.venv-sim/bin/python" scripts/restart_sim.sh
```

打开 `http://127.0.0.1:8765/zh/studio/console`。系统缺少 venv、EGL 或 Node 时先安装相应系统依赖；此流程不安装 CAN 驱动、不启动实机网关。也可把 Python 环境放在 `~/.local/share/robotics/envs/rebot-sim`，供多个项目使用；不要覆盖原环境。

依赖按仓库清单锁定；新电脑可重建源码与模型。本轮已在当前 Linux 环境构建和仿真验证，未声称在全新 Windows/macOS 或空白电脑安装验收。私有配置在 `outputs/private_config`，不随 Git 克隆，默认所有 MIT 数值未确认。
