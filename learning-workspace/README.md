# Seeed B601-RS / LeRobot：Linux 一键部署

## ROS 控制台与仿真一键部署包

已将「机械臂仿真」和「生成末端书写国庆快乐的ROS程序」两个对话的成果另外整合为 Ubuntu / Kubuntu 24.04 + ROS 2 Jazzy 安装包，包含网页工作台、MuJoCo、实机反馈、手柄/J7、算法和书写程序。拷贝 `dist/rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run` 到目标电脑后执行：

```bash
bash rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run
```

详细用法见 [ROS 部署说明](deployment/rebotarm/README.md)。下文为原有 LeRobot / ACT 部署包。

把 [Seeed 教程](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_lerobot/) 的安装、诊断、校准、遥操作、数据采集、ACT 训练和模型运行整合为一个部署包。

**已实测环境：原生 Kubuntu 24.04（KDE），x86_64，CPU。** 面向 B601-RS 从臂 + reBot Arm 102 主臂 + 普通 USB RGB 摄像头。安装器提供 Ubuntu / Debian、Fedora、Arch 系依赖适配，其他发行版尚未实测；要求 Linux x86_64 + glibc >= 2.28。详见 [兼容矩阵](docs/SUPPORT.md)。默认 CPU，支持可选 NVIDIA CUDA。软件自检不需要机械臂。

这是联网安装包。它包含固定版本的 Seeed LeRobot 源码构建包，但不包含已学会抓取任务的模型。真实任务仍需录制示范并训练。此包提供 ACT 流程；SmolVLA、Pi0、GR00T、ROS2、深度相机和机械臂物理仿真不在本版安装范围内。

## 1. 一条命令安装

GitHub Release 发布后，可在终端执行以下命令（将 `OWNER/REPO` 替换为实际仓库名，需要系统 Python 3.10+ 与 curl）：

```bash
curl -fsSL https://github.com/OWNER/REPO/releases/latest/download/install-release.py | python3 - --repo OWNER/REPO
```

下载器获取最新版本、核对归档 SHA256、安装到 `~/.local/share/seeed-lerobot/releases/版本/`，通过自检后切换 `current`。配置、数据和模型保存在共享目录，升级时保留。启动菜单：

```bash
~/.local/share/seeed-lerobot/current/robotctl
```

公开仓库可直接下载，私有仓库需要授权；指定版本、GPU、升级和私有仓库用法见 [发布说明](docs/RELEASING.md)。首次 GitHub 发布尚需选定仓库并执行 Actions；下面的手工归档方式可以直接使用。

### 手工下载归档

将 `seeed-lerobot-0.1.0-linux-x86_64.tar.gz` 和 `SHA256SUMS` 放在同一目录，校验并解压：

```bash
sha256sum --check --ignore-missing SHA256SUMS
tar -xzf seeed-lerobot-0.1.0-linux-x86_64.tar.gz
cd seeed-lerobot-linux
bash install.sh
```

使用普通用户运行。安装系统包时会通过 `sudo` 询问 Linux 密码，之后自动完成：

1. 安装系统依赖和串口/摄像头用户组。
2. 下载经过 SHA256 校验的 uv，创建独立的 Python 3.12.12 环境。
3. 按带哈希的锁文件安装 LeRobot、PyTorch 和 Seeed 插件。
4. 创建 `config.json`（重复安装保留现有配置和数据）。
5. 执行软件诊断和离线 ACT 训练/推理自检。

看到“安装完成”后启动菜单：

```bash
./robotctl
```

安装器只在当前项目内建立 Python 环境；系统包通过对应发行版的包管理器安装，用户组由 usermod 配置。安装不会连接、校准或驱动机械臂。若提示用户组变更，请注销并重新登录 Linux 后再连接硬件。

### 系统建议

- 建议至少 4 个 CPU 核、8 GB 内存、40 GB 可用磁盘；真实训练建议更多内存和磁盘。
- 项目放在本地 Linux 文件系统（例如主目录或桌面），以支持可执行权限和软链接。
- 在原生系统中，USB-CAN、主臂和摄像头直接连接电脑的 USB 接口，再检查驱动与用户权限。
- 默认 CPU 可做自检和学习流程，真实 ACT 训练可能很慢。
- 如已由管理员装好系统依赖，可运行 `bash install.sh --skip-system`；此选项跳过系统包和用户组修改。
- 查看安装计划：`bash install.sh --dry-run`。

### 可选：NVIDIA GPU

电脑具备 NVIDIA 显卡且已安装兼容驱动时，使用：

```bash
nvidia-smi
bash install.sh --device cuda
```

CUDA 方案固定为 PyTorch 2.7.1 + CUDA 12.8。AMD / Intel 显卡不能使用此 CUDA 方案。安装器不会安装或修改显卡驱动。自检会验证 `torch.cuda.is_available()`，失败时安装会明确报错，可改用 CPU 方案。

已有 `config.json` 时重新安装会保留训练设备设置。可用 `./robotctl configure` 修改，或单次运行 `./robotctl train --device cuda`。

## 2. 连接机械臂和摄像头

将以下设备直接接到 Kubuntu 电脑的 USB 接口：

| 设备 | Linux 中的典型名称 | 检查方法 |
| --- | --- | --- |
| B601-RS 的 USB-CAN 适配器 | `can0` / `can1` | `ip -details link show`、`lsusb` |
| reBot 102 主臂的 USB 串口 | `/dev/ttyUSB0` | `ls /dev/ttyUSB* /dev/ttyACM*` |
| USB 摄像头 | `/dev/video0` 等 | `v4l2-ctl --list-devices` |

系统必须识别出对应设备。CAN 接口不会因为软件安装完成就凭空出现。适配器还需要相应的 Linux 驱动；例如采用内核驱动的 PCAN-USB 可以检查 `peak_usb`，不兼容时参考厂商驱动说明。

填写实际端口和摄像头：

```bash
./robotctl configure
./robotctl doctor
./robotctl cameras
./robotctl setup-can
```

`setup-can` 通过 sudo 将配置中的 CAN 接口设为 1 Mbit/s，仅配置总线，不发送电机动作。电脑重启或重新插拔适配器后可能需要再次执行。

`doctor` 只读检查软件、端口、权限、CUDA 和桌面会话，不读取电机状态。`cameras` 使用上游工具枚举相机并保存样图。摄像头需要实际采图是否稳定，可在遥操作显示画面时确认。

录制时在 KDE 桌面的 Konsole 终端中操作；只有 SSH 的无桌面会话无法可靠使用键盘录制快捷键。若 Plasma Wayland 下快捷键无效，注销后在登录界面的会话选项中选择 Plasma (X11)（如已安装）。

## 3. 校准 → 示范 → 训练 → 执行

先按 Seeed 教程把主从臂摆到参考零位、夹爪闭合，再校准。校准和以下控制命令会连接真实机械臂，请固定设备、清空运动空间并准备断电开关。

```bash
# 1. 校准
./robotctl calibrate leader
./robotctl calibrate follower

# 2. 主臂控制从臂，确认关节方向和行程
./robotctl teleop

# 3. 录制操作示范，默认 50 个回合
./robotctl record

# 4. 训练 ACT，默认 CPU、20000 步
./robotctl train

# 5. 加载最近一次成功训练的模型，执行并录制一个评估回合
./robotctl run
```

常用操作：

```bash
./robotctl record --resume                 # 向原数据集追加示范
./robotctl train --steps 1000 --batch-size 2
./robotctl train --device cuda
./robotctl run --model /完整路径/pretrained_model --device cpu
./robotctl record --dry-run                # 仅看将要执行的命令
./robotctl train --dry-run
./robotctl self-test                       # 重跑无需硬件的自检
```

录制快捷键沿用 LeRobot：右箭头结束当前回合；左箭头重录当前回合；ESC 结束录制。Ctrl+C 中断当前命令。停止命令不等于实体急停，机械臂掉电后的行为取决于硬件。

真实训练首次会下载 ResNet18 的 ImageNet 初始化权重，需能访问 PyTorch 下载站点；自检使用随机初始化，可在软件安装完成后离线运行。20000 步只是起始配置，不保证任务成功率，需根据真实示范和评估调整。

## 4. 配置与文件位置

`config.json` 使用 JSON，无需编辑代码。`--config /路径/配置.json` 可选择另一份配置，须放在子命令之前。默认只采集一个 640×480、15 FPS 的 USB 摄像头；多个摄像头可以在 `cameras` 中增加，例如：

```json
"cameras": {
  "front": {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 15, "fourcc": "MJPG"},
  "side": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 15, "fourcc": "MJPG"}
}
```

这是配置片段，替换 `config.json` 中对应字段即可。可以用 `/dev/v4l/by-id/...` 稳定路径代替易变的数字编号。同一模型的数据采集和运行须保持相机名称、画面尺寸、摆位和机械臂配置一致；多路 ACT 图像应使用相同尺寸。`display_data=true` 会打开上游 Rerun 可视化，需要可用的桌面图形环境。

| 位置 | 内容 |
| --- | --- |
| `.venv/` | 项目独立 Python 环境 |
| `.tools/` | uv 与安装器管理的 Python |
| `data/datasets/rebot_pick_cube/` | 实际示范数据 |
| `data/outputs/act-时间/` | 训练记录、检查点和权重 |
| `data/evaluations/` | 每次模型运行的独立评估数据 |
| `logs/` | 安装和各操作日志 |
| `.state/latest-model.txt` | 最近一次成功训练的模型路径 |
| `.state/self-test/last-result.json` | 自检结果，不代表真实抓取成功 |

所有相对数据路径以项目目录为基准。默认 `dataset.repo_id=local/...` 只是本地数据集名称，不要求注册 Hugging Face。数据和权重均不自动上传；W&B 关闭。

创建新任务时请同时修改 `dataset.repo_id`、`dataset.root` 和 `dataset.task`。已有数据集不会被首次录制命令覆盖；继续录制须明确使用 `--resume`。每次训练、每次评估都使用新的输出目录。训练成功后会在模型旁保存 `deployment.json`，运行时核对硬件和摄像头配置。

## 5. 排错

| 问题 | 处理 |
| --- | --- |
| 安装下载失败 | 查看 `logs/install-*.log`，检查 PyPI、PyTorch、GitHub Python 下载连接，再重复安装。系统代理可通过 `https_proxy` 配置。 |
| 没有 `can0` | 检查 USB-CAN 连接与适配器驱动；实际可能是 `can1`，用 configure 修改。 |
| 串口权限不足 | 确认已注销重登录，`id -nG` 包含 `dialout`（Arch 系通常为 `uucp`）；检查串口程序占用。 |
| 摄像头黑屏或掉帧 | 检查 USB 接口、数据线和带宽；降低 FPS/分辨率，检查真实 video 节点和是否被其他程序占用。 |
| CPU 训练太慢 | 将训练放到具备 NVIDIA GPU 的 Linux 环境，保留同一份数据集和部署包。 |
| CUDA 不可用 | 检查 `nvidia-smi`、驱动和是否安装 CUDA 版依赖；AMD / Intel 显卡请使用此包的 CPU 方案。 |
| 配置与训练不一致 | 核对模型旁 `deployment.json`。改变相机摆位后即使配置文本一致，也需重新评估。 |
| 安装后移动了项目目录 | Python 环境内可能包含旧绝对路径；将 `.venv` 改名留存后，在新目录重新安装。数据仍可保留；模型可用 `run --model` 重新指定。 |

本包使用 Seeed `DepthCameraSupport` 分支的固定提交，与不断变化的最新版 LeRobot 命令可能不同。请勿在此环境中直接 `pip install -U lerobot`。源码来源和版本见 `vendor/ORIGIN.json`；实际验证范围见 [VERIFICATION.md](VERIFICATION.md)。

## 开发检查与重新打包

```bash
python3 -m unittest discover -s tests -v
bash -n install.sh robotctl scripts/system_deps.sh
python3 scripts/build_bundle.py
```

`dist/` 中的部署包仅包含代码、示例配置、锁文件和上游 wheel，不包含本机环境、用户配置、密钥、采集数据或训练模型。

维护入口：[版本记录](CHANGELOG.md)、[GitHub 发布流程](docs/RELEASING.md)、[兼容矩阵](docs/SUPPORT.md)。部署包装代码采用 Apache-2.0，第三方部分保留各自许可证，见 LICENSE 与 THIRD_PARTY_NOTICES.md。
