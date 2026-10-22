# ReBot Arm B601-RS：Ubuntu 24.04 一键部署

将这两个对话的现行成果整合为同一个安装包：

- 「生成末端书写国庆快乐的ROS程序」：书写程序、路径高亮、工作台改进、通信恢复、手柄/J7、启动及结束项目。
- 「机械臂仿真」：ROS 2 / MuJoCo / Three.js、实机反馈孪生、坐标/键盘/TCP 控制、示教和程序导入、独立控制算法、视觉/规划及大模型接口。

## 拷贝一个文件，运行一条命令

把 `rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run` 拷贝到另一台电脑，在文件所在目录打开终端：

```bash
bash rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run
```

用普通桌面用户运行；安装系统包时输入 sudo 密码。之后自动完成官方 ROS 软件源和系统包安装、独立 Node 22、带哈希的 Python 依赖安装、ROS 工作空间编译、无硬件软件自检、桌面/应用菜单入口，并打开网页。

**要求：Ubuntu / Kubuntu 24.04，x86_64，系统 Python 3.12，互联网，sudo 权限，至少 6 GiB 可用磁盘。** 建议 4 核 / 8 GiB 内存。安装包已包含项目源码、URDF/STL、MuJoCo 场景、前端库、两个控制 SDK 和固定版本 Node；ROS 系统包和 Python 依赖仍需联网下载，因此不是离线安装包。其他 Linux、ARM64、无图形桌面的物理仿真显示尚未验证。

可选校验（同时拷贝 `REBOTARM_SHA256SUMS`）：

```bash
sha256sum --check --ignore-missing REBOTARM_SHA256SUMS
```

## 安装完成后

应用菜单搜索 **ReBot Arm B601-RS 控制工作台**，或：

```bash
~/.local/share/rebotarm-b601-rs/current/rebotarm-app start
```

浏览器打开 `http://127.0.0.1:3002/startup.html`。默认只启动网页，按页面逐步启动 ROS 联动、仿真或实机只读反馈；安装和软件自检不连接 CAN，不使能电机。使用手柄时在本机浏览器访问 localhost / 127.0.0.1 并确认 standard 映射。

- **安全仿真**：从工作台启动仿真，也可运行 `rebotarm-app simulation`。
- **国庆快乐**：在程序页导入安装目录的 `examples/writing/guoqing_kuaile.json`；三维路径和 ROS/RViz 预览脚本一同提供。实机使用前仍须检查路径和现场条件。
- **实机**：接通 B601-RS 和 USB-CAN，页面启用 can0（1 Mbps），再启动只读反馈。准备、接管和使能保留原来的现场确认与保护。安装器不自动回零、校准或恢复旧运动。
- **相机 / 视觉 / 规划 / LLM**：在新电脑重新授予浏览器相机权限，填写实际服务地址和自己的 API key。包内不含训练好的任务模型。
- **退出**：先支撑并失能，再在网页点击“结束项目”。单独关闭标签页不结束 ROS / MuJoCo 后台。已有退出保护保留；内置浏览器中的最终结束页显示仍有限制。

## 诊断、重试和升级

```bash
# 软件检查，不连接机械臂
~/.local/share/rebotarm-b601-rs/current/rebotarm-app doctor

# DOMAIN 82 内的 Fake ROS 链路验收，不连接实机
~/.local/share/rebotarm-b601-rs/current/rebotarm-app self-test

# 查看计划
bash rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run --dry-run

# 安装但不打开网页
bash rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run --no-launch

# 仅提取源文件
bash rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run --extract ./rebotarm-source
```

默认源码/运行环境在 `~/.local/share/rebotarm-b601-rs/versions/版本-内容摘要/`，当前版本通过 `current` 指向。程序、连接配置、会话和恢复记录存于 `shared/outputs`，重复安装保留。仅自检通过才切换 current；升级前在旧版网页结束项目，安装器检测到旧进程会拒绝更新。失败后可重新运行同一安装文件；完整终端输出会说明失败步骤。

也提供 `.tar.gz`，解压后运行 `bash install.sh`，效果相同。`--skip-system` 仅用于管理员已装好全部系统包的环境。`--prefix` 可更改安装目录，路径不能包含空白或特殊 shell 字符。安装后不要移动版本目录，Python venv 和 ROS 编译产物包含目标电脑的绝对路径；换目录需重新安装。

## 迁移范围与验证边界

2026-10-19构建源码更新：新构建会携带 `configs/csp-sdk-lock.json`、`requirements-vision-offline.txt`、研究/历史Markdown，以及路径评估、J7毫米查表、手眼求解、夹持回放和只读网络审计工具。解包后见 `docs/九项建议离线实施_2026-10-19.md`。手眼工具的独立视觉环境需另行准备。此次只验证了源码暂存包及哈希，未重新发布 `.run`/`.tar.gz`；此前安装文件仍是其原始快照。

打包的是上述两个对话汇集到的完整现行源码，包括未提交但已实现的文件。每个发布文件在 `BUNDLE_MANIFEST.json` 中列有 SHA256；SDK 固定 `6415d43130d1e143c70dc106096a857ac5556f81` 并保留原 Git 完整性检查。安装包不含旧电脑的 .venv、ROS build/install、真实 .env、API key、运行输出、CAN 会话或恢复记录。示教程序应由用户从原工作台导出，再导入新机器；新机械臂重新确认标定和支撑姿态。同一机械臂若存在未完成清理，必须先在原部署核查恢复，不能用新安装规避恢复锁。

既有实机限制继续适用：J3 跟随滞后、偶发通信间隔、长时控制、MIT 实体效果、真实手柄/J7、相机及外部算法还需各自现场验收；封装部署不代表这些物理问题已全部解决。

此包是 ROS 控制与仿真工作台。当前目录原有的 Seeed LeRobot / ACT 安装包仍单独保留，两者不能同时占用同一 CAN 接口。

上游安装依据：[ROS 2 Jazzy 官方 Ubuntu 安装说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)、[Node 22.23.2 官方发布与校验文件](https://nodejs.org/dist/v22.23.2/SHASUMS256.txt)。
