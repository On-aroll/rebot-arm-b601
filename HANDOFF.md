# B601-RS 主线与视觉支线换机交接

封存日期：2026-10-22。仓库：`On-aroll/rebot-arm-b601`（保持私有）。

**用户已要求暂停持续实验。默认仿真版本为 V3；没有把实验策略接入实机。接手后先完成新设备安装、同一机械臂历史恢复与只读反馈核对。**

## 1. 目录和阅读顺序

| 路径 | 内容 |
| --- | --- |
| `ReBot_Arm_DigitalTwin_RS/` | 当前主线：3002 网页工作台、ROS 2、MuJoCo、唯一 CAN 执行器、本次封存的全部新功能源码与测试 |
| `learning-workspace/` | 原 `1seeed_Robotics`：LeRobot 部署工具、ROS 工作台安装器及视觉实验 |
| `learning-workspace/experiments/b601-vision-grasp/` | 本会话全部实验分支、冻结 V2/V3、报告和协议 |
| `scripts/`, `external/`, `rebotarm_ros2/` | 早期 Studio/8765 工程与 ROS 工作区，保留历史成果；当前实机入口用上面的 3002 主线 |
| `handoff/evidence/` | 随 Git 提交的实机原始历史、仿真摘要/审计/源快照/模型训练记录和部署验证压缩包及逐文件哈希 |
| `handoff/models/` | 151 份实验检查点及原输出目录映射；默认 V3 模型另在冻结版本中自包含 |
| `handoff/ARTIFACTS.json` | 全量原始输出的压缩包索引和实际上传状态 |
| `handoff/release-assets/` | 本机准备的完整录像、RGB-D、动作与评分原始数据、历史安装包；不直接进入 Git，使用私有 Release 交付 |

先读本文件、`AGENTS.md`，再读主线 `README.md`、`docs/支撑姿态越界恢复与力矩门槛_2026-10-20.md` 和 `docs/自主重复验证_2026-10-20.md`。文档中旧 PID、旧反馈角度和“进行中”是历史记录。本文件与 `ITERATION_STATE.json` 的暂停状态优先；不要据历史记录自动继续运动。

三个主线会话已汇入此快照：

- `01a14fb5-1e5f-76e0-9f2c-b30b9cb88a2e`：调研机械臂自主夹取与焊接路线。
- `01a144e1-f673-79c2-9d4c-539ea322ca55`：生成末端书写国庆快乐的ROS程序。
- `01a13e02-997e-7540-9723-8838e8bec1a4`：机械臂仿真。

## 2. 在另一台设备获取

这是私有仓库，需要同一有权限的账号。新机尚未配置 SSH 时，可先安装并登录 GitHub CLI，再执行 `gh repo clone On-aroll/rebot-arm-b601`；已有 SSH 权限使用下面命令。

```bash
git clone git@github.com:On-aroll/rebot-arm-b601.git
cd rebot-arm-b601
git checkout main
python3 handoff/verify_checkout.py
python3 handoff/prepare_simulation.py
```

不需要 `git submodule` 或 Git LFS。`handoff-2026-10-22` 标签固定封存源码，后续附件上传核验记录查看 main 分支。当前工程和改过的第三方源码已经是实际文件；受执行器完整性检查约束的 CSP SDK 要按下面的锁定提交单独重建。主线原 Git 历史另存 `handoff/provenance/digitaltwin-history.bundle`，可用 `git clone 路径/digitaltwin-history.bundle 独立历史目录` 查看，不需要操作现有工作区。

原始大附件以 [handoff-2026-10-22 Release](https://github.com/On-aroll/rebot-arm-b601/releases/tag/handoff-2026-10-22) 为准。`ARTIFACTS.json` 显示 `prepared_pending_upload` 或 `uploading` 时，说明附件尚未全部发布。只有状态为 `uploaded_and_verified`，且存在服务端 SHA256 对账记录 `handoff/RELEASE_VERIFIED.json`，才表示全部附件发布完成。Git clone 带走源码、默认模型、全部报告和同臂恢复历史；完整原始 RGB-D/录像须另从 Release 下载。

## 3. 主线新机部署（先不连接板子）

已实测主机环境：Ubuntu/Kubuntu 24.04 x86_64，ROS 2 Jazzy，Python 3.12，Node.js 22+，SocketCAN 1 Mbps。22.04/Humble 为兼容路径，须重新回归。若新设备是 ARM 板，不能使用包内 x86_64 Node/旧 `.run` 安装文件，须按目标架构安装依赖；ARM 运行尚未验收。

```bash
cd ReBot_Arm_DigitalTwin_RS
bash scripts/setup_csp_sdk.sh
bash scripts/setup_csp_sdk.sh --check
./setup.sh --check
./setup.sh --yes
./rebotarm doctor
```

CSP SDK 固定 `6415d43130d1e143c70dc106096a857ac5556f81`，以 `configs/csp-sdk-lock.json` 为准。保留这个 checkout 的 `.git`，执行器会核对提交和源码。不要复制旧 venv、ROS build/install 或更换到上游 HEAD。

**同一台机械臂换电脑**，启动任何实机服务前，先在仓库根目录恢复历史：

```bash
sudo apt-get install zstd  # 若系统已有可跳过
python3 handoff/restore_hardware_history.py --same-arm
```

该脚本只解压并核对原始文件，不访问设备；遇到已存在的非空硬件历史会拒绝覆盖。历史可能保留 RECOVERY 锁，不能因为换电脑而跳过。若是另一台机械臂，不要恢复这些设备历史，重新建立它自己的参数快照、标定与支撑记录。

然后仅启动网页：

```bash
bash handoff/start_workbench.sh
# http://127.0.0.1:3002/startup.html
```

包装脚本为新路径设置 `REBOTARM_GATEWAY_ROOT`，防止沿用旧电脑的绝对路径。通过此脚本启动的网页管理后续 ROS/驱动。若在别的终端直接用主线 CLI，也先在主线目录执行 `export REBOTARM_GATEWAY_ROOT="$PWD"`。网页与 ROSBridge 默认本地监听；不要为远程访问直接放开接口。

## 4. 连接板子后的第一步

1. 现场先确认可靠机械支撑、明确失能、供电与 USB-CAN 接线；推荐 USB-CAN 直连电脑。旧电脑不能再同时控制同一 CAN。
2. 在启动台配置 `can0` 为 1 Mbps，启动 ROS 联动，再连接**实机只读反馈**。网页、驱动和仿真就绪不是电机受力的证明。
3. 看连续新鲜反馈、READONLY/IDLE、owner/prepared/goal/reference、各轴模式和恢复状态；有 RECOVERY/FAULT 则先诊断。不要把 10 月 20 日最后一次反馈当现场姿态。
4. 准备、接管、使能或运动属于下一轮现场操作，先预览当前起点与路径。安装、连接、导入程序不会自动执行运动。

只运行当前受管 CSP 链路。不要同时启动旧 `./rebotarm start rs`、8765 网关控制或官方抓取 demo 的独立 SDK 执行器。仿真、ROS `/rebotarm_rs` 与实机 `/rebotarm` 必须辨清。

## 5. 主线已验证什么，还缺什么

- 已实现：六维坐标、键盘/手柄、TCP 拖动、程序导入/示教/暂停继续、J7、六种反馈控制器、视觉/规划/LLM 接口、参数持久化与恢复、唯一 CAN 线程和相关保护。
- 10 月 20 日历史短测：六轴各三轮 2° 往返、程序组合、J7 三轮 0.6↔5 rad、暂停/接管、心跳中断保持及六算法短功能检查有各自通过记录。它们不等于长期稳定性、带载性能、外部 TCP 精度或算法整定完成。
- 最新支撑恢复修复有 469 项全量离线通过记录及构建哈希。10 月 20 日 20:06 已只读加载，连续 40 新帧 READONLY/IDLE，214 份当时既有硬件 JSON 哈希未变；未执行新的支撑恢复动作。
- 最后一次只读实测角度已不同于早先 J2/J3 双超上限姿态。**不可重放旧三段恢复路线**。按当前反馈重新生成预览。
- 初始支撑目标：J2–J6 均 0°，各轴 ±3°（含边界）；J1 保持当前角度，J7 不参与。存档 ID `c12a69ac6aa1e162`，原采样在 `captured_joints`。这是软件目标；当前物理可靠支撑仍要现场确认，不能把零位当无电稳定证明。
- 普通 stop/disable/Esc/重置/服务退出可能撤力。10 月 20 日脚本收尾失能曾直接造成丢失保持力，已记录；不要无人支撑地退出或用停止按钮测试通信。`--handoff-hold` 只在明确验证保持后移交网页，不能把普通停止解释为保持。
- 尚缺：实际支撑姿态恢复与撤力验收、长时/带载稳定性、MIT 实体效果、六算法调定、真实手柄完整验收、J7 开度和 TCP/手眼计量标定、自主抓取到实机迁移。

## 6. 视觉支线的可用版本

目标是“人选择 A/B/C 区，机械臂观察、抓取、放置、检查并有限重试”。当前已经在纯 MuJoCo 仿真形成闭环；主体为规划/IK 与动作阶段，小模型负责视觉定位和最多 ±8 mm 残差修正。不是强化学习或端到端动作策略。

| 版本/实验 | 结论 |
| --- | --- |
| 冻结 V3（默认） | 独立正式静态 30/30、受扰 30/30；对照 V2 为 29/30、30/30。单个配对增益不足以宣称普遍优势。历史已知 16400002 仍失败 |
| 冻结 V2 | 完整保留，可比对和回退 |
| 图像残差策略（6419 参数） | 310 次正式对照与直接/归档审计完成；相对 V3 完整任务、首次成功均无增益，未采用 |
| 视觉风险数据/模型 | 7168 图数据与风险头完整审计通过；阈值拒绝全部，安全样本保留率 0%，未进入动作 |
| self-visibility 分支 | 24 次物理探测完成；两个触发场景均未救回，机制门槛失败。射线审计还在 `mj_ray` 与 `mj_multiRay` 一致性断言失败，原因未调查。保留原始失败，不采用 |

仿真控制只读取 RGB-D、标定和自身反馈，真值只供监督/独立评分。禁止运行中写物体位姿、weld/吸附或隐瞒失败。151 份权重含重复快照，索引列明原输出路径。

```bash
# 仿真独立环境；不用 ROS 工作区 venv
sim=learning-workspace/experiments/b601-vision-grasp
python3.12 -m venv "$sim/.venv"
"$sim/.venv/bin/python" -m pip install --extra-index-url https://download.pytorch.org/whl/cpu -r "$sim/requirements.lock"
python3 handoff/prepare_simulation.py
cd "$sim"
./verified_task.sh --goal B
# 可用 --goal A / C、--seed、--name、--disturbed
```

入口自动核对 V3 冻结清单，使用 EGL/单线程环境。V3 manifest SHA256：`bb6bebf40eb2b660f66600c5be2b41b7e13bee6529bcc042e247a363bcb1342b`。源码与冻结文件未为迁移而改写；仅版本目录外的资产/环境/输出链接恢复为相对路径。

`release_manifest.json`、日志和模型历史指针中的旧电脑绝对路径属于原始证据；当前演示使用相对的 `runtime_directory`，可以换目录运行。历史整批审计若依赖旧路径，应按保留的源快照显式重映射或恢复原目录布局，并保留原证据，不能改写文件后继续声称旧哈希有效。

本次封存只做文件和恢复验证，没有在“暂停”之后重跑任务或训练。

## 7. 附件、回放和历史安装包

`handoff/evidence/simulation-records.tar.zst` 保存全部实验 JSON、模型、源快照、训练配置与审计结果；完整动作/物理评分 JSONL、RGB-D NPZ、PNG/GIF 在逐实验 Release 包中，失败与成功一并保留。每包都有逐文件 SHA256 清单，Release 总表为 `SHA256SUMS`。

GitHub CLI 登录后，在仓库根目录：

```bash
mkdir -p downloaded-assets
gh release download handoff-2026-10-22 --repo On-aroll/rebot-arm-b601 --dir downloaded-assets
(cd downloaded-assets && sha256sum -c SHA256SUMS)
# 仿真包解压到仿真实验目录；包含 outputs/ 前缀
zstd -dc downloaded-assets/sim-goal-geometry-fallback-v3-candidate-01.tar.zst | tar -xf - -C learning-workspace/experiments/b601-vision-grasp
# 主线全量证据建议先解压到独立目录，避免覆盖当前运行数据
mkdir -p historical-main
zstd -dc downloaded-assets/main-outputs.tar.zst | tar -xf - -C historical-main
```

只要恢复摘要：`zstd -dc handoff/evidence/simulation-records.tar.zst | tar -xf - -C learning-workspace/experiments/b601-vision-grasp`。这会恢复到忽略的 outputs 目录，不启动任何任务。

原 `dist` 的 1.0.0 ROS 安装器和 0.1.0 LeRobot 包也保留为历史产物。**它们构建于 10 月 18 日，不包含 10 月 20–22 日的新代码，不能当作本次最新版部署包。** 新设备使用本次源码安装。`learning-workspace/scripts/build_rebotarm_bundle.py` 可以重新构建，但重打包须重新验收，不继承旧安装包验证结论。

Release 中旧版两份说明使用 `ReBotArm-deployment-guide.md` 和 `ReBotArm-validation-report.md`，原中文文件名保存在 `handoff/ARTIFACTS.json` 的 `legacy_filename_aliases`；内容与原文件逐字节相同。

环境、依赖缓存、build/install、node_modules、进程 PID/锁和密钥不迁移；依赖按锁文件/安装脚本重建。API key、真实相机权限、服务地址、标定需要在目标设备重新配置。

## 8. 接手 AI 可直接使用的说明

> 先读根目录 HANDOFF.md、AGENTS.md 和 handoff/ARTIFACTS.json。用户已暂停持续仿真实验，现要在另一设备连接同一 B601-RS 继续。主线在 ReBot_Arm_DigitalTwin_RS，默认纯仿真策略 V3 在 learning-workspace/experiments/b601-vision-grasp。先确认目标 OS/架构并安装、校验锁定 SDK、恢复同臂历史，再只读连接新鲜反馈；旧 PID/反馈/现场授权不代表当前状态。不要直接运行官方抓取 demo、旧实机启动入口或把仿真策略连到 CAN。支撑恢复动作、J7/TCP/手眼标定与 sim2real 尚未验收；新 self-visibility 实验机制失败且审计失败，保留但不采用。首先报告新机软件自检与只读状态，再推进人当场指定的下一步。
