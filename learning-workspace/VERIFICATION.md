# 验证记录

日期：2026-10-18。执行环境：原生 Kubuntu 24.04（系统信息 ID=ubuntu、VERSION_ID=24.04）、KDE / X11、x86_64、Python 3.12.12、CPU。

## 已完成

- 通过 SHA256 校验从固定 PyPI wheel 引导 uv 0.13.1。
- 从固定 Seeed 源码提交构建 LeRobot 0.4.4 wheel；源码未修改，来源见 `vendor/ORIGIN.json`。
- 实际按 CPU 锁文件安装 95 个包；`uv pip check` 全部兼容。
- B601-RS 与 reBot 102 插件可导入并正确注册。
- 上游 calibrate、teleoperate、record、train 命令的 `--help` 均可启动。
- 实际生成 2 个回合、24 帧带 H.264 视频和 7 维关节状态/动作的数据集。
- 通过真实 LeRobot 训练入口完成 ACT 两步训练，保存检查点并重新加载，推理输出形状 `[1, 7]` 且所有值有限。
- 自检时启用 `HF_HUB_OFFLINE=1` 和 `HF_DATASETS_OFFLINE=1`，不下载初始化模型、不上传数据。
- 10 项部署与发布边界测试通过：原有流程边界，以及归档可重复构建、排除用户数据、损坏包拒绝解压、路径穿越/符号链接拒绝解压、发行版计划识别、失败升级保留入口和数据并可重试。
- Bash 语法检查和 CPU/CUDA 安装计划检查通过；非法安装设备参数被拒绝。
- 将部署包解压到全新的、含空格的目录，执行 `bash install.sh --skip-system`，独立创建 Python 环境并通过全部软件诊断与训练/推理自检。
- 修改测试目录中的配置并创建数据文件后重复安装，安装成功，配置文件 SHA256 不变，数据内容保持不变。
- 压缩包文件清单已检查，未包含本机虚拟环境、实际用户配置或数据。
- Release 结构实际安装通过：在 `.state/release validation/` 的全新含空格目录中，从本地发布归档模拟 GitHub 附件下载，实际校验/解压、建立版本目录和共享配置/数据链接，实际安装 95 个 CPU 依赖并完成 ACT 训练/推理自检，成功切换 current。下载传输层使用本地替身，uv / Python / 依赖仍按正常安装流程处理。
- GitHub Actions 的两个 YAML 文件已解析并检查结构，Bash 脚本语法与本机安装计划通过。

## 未验证的环境与效果

- B601-RS、主臂、USB-CAN 的实机校准和遥操作，以及摄像头实际数据采集、真实示范和抓取成功率未验证。
- 当前 PCI 设备包含 NVIDIA RTX 3050 Mobile，使用 nouveau 内核驱动，系统未找到 nvidia-smi；CUDA 锁文件已经解析生成，GPU 依赖安装与训练未执行。
- Ubuntu / Kubuntu 22.04 未实测。
- Debian、Fedora、Arch 等系统已做包名与计划适配，未进行实际系统安装；ARM / aarch64 与 Alpine / musl 未适配。
- 尚未在 GitHub 执行 Actions 或正式发布 Release；公开/私有仓库的 Release API 及附件下载链路待目标仓库确定后验证。
- 端到端安装测试使用 `--skip-system`，利用本机已有系统依赖；apt/usermod 步骤尚未在干净的新系统中实跑。
- 自检是较小 ACT 配置上的软件链路验证，不代表机器人已经掌握抓取技能。

运行 `./robotctl self-test` 可以在目标 Kubuntu 电脑中重新执行同样的训练与推理验证。
