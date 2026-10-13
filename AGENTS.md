> **2026-10-13 单击失能修复**：用户明确要求紧急撤力不再确认“我在旁边”。控制台逐轴红色“立即失能”单击排入 `disable`，可中断查询和未完成的使能流程；使用已知型号及明确轴 ID，其他轴失能仍可点。使能继续保留现场确认。失能后查询暂停，显式刷新仅恢复只读查询。网关仍串行处理已开始的查询，按钮不保证硬件即时撤力；软件回执不证明实际停止。此轮只做虚拟 WebSocket 测试，不发送实机指令或改参数，声音异常仍未定位。

> **2026-10-13 现场 J1 电源试验**：用户已确认现场支撑、J1 方向/零位/限位及独立断电。本次在现有控制台真实发送 J1 `enable`、随后 `disable`，两次网关请求均成功；未发送 MIT 保持或运动目标，未改参数或零位。用户反馈“没感觉到出力变化，无异常”；后续 J1 大角度变化已确认是手动转动。动力效果仍未验收，不把网关成功当作出力或撤力证明。Seeed/P4 只读准备被 J2 原始 −0.032° 低于模型下限 0° 阻断；未裁剪、改零位或放宽限位。+0.5° 运动试验尚未批准。

> **2026-10-13 后续声音与页面修复**：用户补充机械臂持续发声，切断机械臂电源后停止；具体来源与轴尚未定位，不据此认定 J1 已撤力或仍出力。实机试验暂停，保持断电，不继续此前手拨 J2 后上电的建议。机械臂模式已修复已知 RS 七轴只读查询、J1–J3 型号与进入页面时重置增益/目标的问题；原始角度保留，预览不发送实机目标。旧页面不显示未经验证的真实使能状态；电源操作仍使用控制台当次确认入口。

> **2026-10-10 启动与归档更新**：见 [当天总结](docs/工作总结_2026-10-10.md) 和 [重启指南](docs/项目重启指南.md)。日常运行 `python3 scripts/start_project.py` 或双击桌面“reBot 项目启动”，复用或补齐 MuJoCo、网关监听与隔离 ROS Fake，不自动连接实机、使能或续跑任务。接口 DOWN 时显式 `--can-up` 可恢复 1 Mbps 通信；已 UP 的异常接口不自动重置。只读 `--status` 不启动服务。后端改动仍须显式重启，一键启动不加载旧进程内的新代码。

> **2026-10-10 逐轴电源更新**：用户已明确要求新控制台提供真实使能/失能，见 [逐轴真实使能](docs/逐轴真实使能_2026-10-10.md)。七轴卡片的独立按钮经过现场确认后可以通过网关真实发送 `enable` / `disable`；启动、刷新和连接不自动使能。MIT 模式、新鲜位置、六轴原始模型限位及发送前焦点仍检查；不发送 MIT 目标或改设备参数，XYZ 真实轨迹仍锁定。本轮只做模拟网关及页面验收，未发送实机电源指令，真实动力未现场验证。下方只读锁记录保留历史含义。

> **2026-10-09 P4 更新**：新一轮进展见 [P4交接](docs/P4交接_2026-10-09.md)。顶部只读连接、MuJoCo 状态同步及 SDK 适配器已实现；用户 3/1/0 增益只保存为待验证草稿。实机写入仍锁定，真实六轴与网页 XYZ 实发未验收；当前 J2 边界读数、现场支撑和停止条件须先核实。没有发送使能或运动。下方 B1/批次 A 记录保留历史含义。

> **2026-10-09 B1 更新**：以 [B1 交接](docs/B1交接_2026-10-09.md) 为最新入口。软件配置不等于真实 MIT 参数备份；旧页面恢复明确只读连接、写入仍锁定。独立六轴 Mock 已验收，仅用合成数据，无真实 CAN 执行；禁止据此开始实机试验。下方旧记录按日期理解。

> **2026-10-08 批次 A 更新**：当前以 [MIT 控制台交接](docs/批次A交接_2026-10-08.md) 为准。J1–J6 保留 MIT；软件配置保存在私有目录，真实执行 LOCKED；不自动连接 CAN。旧文中总开关/stop 例外、PP/CSP 限速与 wrap 控制建议是历史实现，不代表当前已验收能力。禁止由历史读数推断当前参数或动态映射。

# reBot Arm B601-RS 虚实联调工程 — agent 上手说明

给接手这个项目的 AI / 人看的入口文档。先读这一页，再动代码。
人类视角的进度叙述在 `docs/当前进度.md`，界面细节在 `docs/虚实联调控制台_2026-10-07.md`。

## 这是什么

Seeed **reBot Arm B601-RS**（六轴 + 电动夹爪，RobStride 电机：ID 1–3 为 `rs-06`，
ID 4–7 为 `rs-00`）的本地工程。路线是先把 MuJoCo 物理仿真这条链路做扎实并逐项验收，
再接实机；最终目标是**虚实结合**：实机执行动作程序、实机反馈驱动数字孪生、
并支持手柄直接控制末端执行器位置。

## 当前状态（重要，别越过这条线）

| 能力 | 状态 |
| --- | --- |
| MuJoCo 仿真中的多步骤动作程序（抓取/搬运/放置/书写） | 已完成并验收 |
| 暂停 / 继续 / 取消 / 并发互斥 / 非法程序拒绝 | 已完成并验收 |
| 实机**只读**通信（7 轴位置、限速参数、供电电压） | 已完成并验收 |
| 实机 → 仿真镜像（数字孪生第一形态） | 已完成 |
| 控制台上的七轴独立电机模块、网关连接与总控 | 已完成 |
| 新控制台逐轴真实使能 / 失能 | 使能须当次现场确认，失能单击发送；软件验证通过，真实动力未验收 |
| 模式 / 目标 / 参数及 XYZ 实机轨迹 | 仍锁定，未通过实机验收 |
| 向实机下发过使能 / 失能 | 2026-10-13 已对获准 J1 发送各一次；现场出力 / 撤力待确认 |
| 向实机下发过运动或保持目标 | 尚未发送 |
| 夹爪角度→位移正式标定、TCP 标定 | 未做 |
| XYZ 目标面板驱动实机、手柄遥操作下发、统一执行层 | 未做 |

**红线**：下发任何实机运动指令前必须确认正方向、零位、限位，且现场有人能断电。
这是用户的显式要求，不是建议。任何会解除演练模式、打开总开关、或直接发
`enable` / `pos_vel` / `mit` / `vel` 的改动，都要先征得同意。

## 快速上手

日常重启后优先使用桌面“reBot 项目启动”或以下命令；不要重新安装依赖：

```bash
cd "/home/bonvoyage/Desktop/seeed Robotics"
python3 scripts/start_project.py
# 只读检查：python3 scripts/start_project.py --status
# can0 DOWN 且需要通信：python3 scripts/start_project.py --can-up
```

下方安装/构建流程仅供依赖丢失或首次重建时使用，最新重启方式见 `docs/项目重启指南.md`。

```bash
# 1. 第三方仓库（本仓库不内置，按锁定 commit 重建）
python3 scripts/setup_external.py

# 2. 界面：已有依赖时仅构建；依赖丢失时才 npm ci
cd external/motorbridge-studio
VITE_BASE_PATH=/studio/ /usr/lib/chatgpt/resources/cua_node/bin/node \
  node_modules/vite/bin/vite.js build

# 3. 仿真服务（会核对进程命令行、写死 MUJOCO_GL、回写 pid）
scripts/restart_sim.sh

# 4. 打开
#    http://127.0.0.1:8765/studio/console   虚实联调控制台
#    http://127.0.0.1:8765/                 原工作台
```

实机只读跟随（可选，需要 CAN 线接好、`can0` 已 up）：

```bash
~/.local/bin/rebot-twin-sync --follow --dry-run
```

## 验收怎么跑

`tests/` 下每个脚本都是独立可跑的只读或仿真内检查，结果写进 `outputs/`：

```bash
PY=~/.local/share/robotics/envs/rebot-sim/bin/python
$PY tests/verify_studio_routing.py          # 路由、资源、路径穿越、只读端点语义
$PY tests/verify_gateway_readonly.py        # 网关只读通路 + 与 python-can 交叉对账
$PY tests/verify_studio_integration.py      # 模型一致性与反馈数值边界
$PY tests/verify_program_http.py            # 动作程序 HTTP 链路
```

改 `scripts/*.py` 后要 `scripts/restart_sim.sh` 重启；只改界面源码要重新 build，
改 `dashboard.html/css/js` 则刷新页面即可。

## 目录

```
scripts/     核心代码：仿真服务、动作程序引擎、控制台后端、CAN 调试工具、ROS 桥
programs/    可编辑动作程序（JSON / Python）
models/      自建 MuJoCo 场景
external/    第三方仓库；只有 motorbridge-studio 是我们改过的 fork 且随本仓库提交
tests/       验收脚本
docs/        中文记录，当前进度.md 是入口
outputs/     验收报告、任务落盘、演示产物（多数不入库）
configs/     依赖清单与 external-manifest.json（第三方仓库锁定版本）
```

前端主界面在 `external/motorbridge-studio/src/`，本项目新增的部分：
`components/ConsoleWorkspace.jsx`（控制台）、`components/MotorModuleCard.jsx`
（单电机模块）、`hooks/useMotorGateway.js`（网关总控与三道闸）、
`styles/console-workspace.css`。

## 必须知道的机器事实

这些是实测结论，推翻它们需要新的证据，不要凭常识假设。

- **`get_state()` 与流式帧在本机 RS 固件上失效。** 位置要读参数
  `0x7019 (mechPos)`，走 `robstride_read_param`。官方前端依赖的
  `state_stream` / `set_active_report` 在这台机器上拿不到有用数据。
- **`mechPos` 是未归一化的多圈值**，比较前要 wrap 到 (−180, 180]。
  实测 J6 读到 −196.02°，归一化后 +163.98° 才落在模型限位内。
- **符号约定 = identity**（`q_urdf = q_mechPos`，不取反），零位偏移≈0，减速比 1:1。
  官方文档结论 + 本机手拨探针验证。
- **给未使能的电机发运动帧是静默无操作**：总线返回 ok，固件直接忽略。
  所以顺序必须是先 `enable`、确认受力矩，再下发目标。
- **切换 RobStride 运行模式会让该轴自动去使能且不重新使能**
  （`useGatewayBridge.js:201-224` 有记录）。
- **使能状态读不到**。界面上的「本会话使能」只是页面自己记的，不是电机真实状态。
- **官方前端的 `setTargetFor()` 会在选中电机时写参数 `0x7026`**（主动上报周期，
  会存进电机）并打开主动上报。本项目的 `useMotorGateway.js` 刻意绕开它，
  每次只发 `set_target`，写完 `close_bus` 归还总线。
- **七轴 `limit_spd (0x7017)` 实测均为 1.0 rad/s ≈ 57°/s**，这就是"反应太快"的来源。
  控制台的限速阈值默认 0.3 rad/s，超过就拒绝使能与运动。
- **网关是 Rust 的 `ws_gateway`**，本机由 `motorbridge-gateway --bind 127.0.0.1:9002`
  跑着；`motorbridge` Python 包里的 `gateway` 模块只是用法提示，不是服务端。
- 硬件排障史：长时间无进展的根因曾是一次 **CAN 线自行脱落**；USB 层 −71 错误
  要靠**适配器直连电脑、不经 Hub**解决。

## 历史三道写入闸门（2026-10-07）

此节记录旧实现。当前新控制台只允许独立按钮经当次确认发送 `enable` / `disable`，
不按此节旧总开关/演练流程解锁轨迹或参数。当前校验见顶部逐轴更新及对应文档。

都在 `useMotorGateway.js` 里强制，不是界面自觉：

1. **总开关** `armed` —— 长按 1.5 秒才打开，默认关。
2. **演练模式** `drill` —— 默认开，走完全部校验、把完整 payload 记进日志，但不发帧。
   `stop` 例外，急停永远真发。
3. **限速兵** —— `enable` / `pos_vel` / `mit` / `vel` 之前，该轴 `limit_spd`
   必须读到且不超过阈值；目标还要在模型限位内、与当前位置差值 ≤ 15°。

op 白名单在底层 `send()` 拦截：只读三件套 `set_target` / `robstride_read_param` /
`close_bus`，写入 `enable` / `disable` / `stop` / `ensure_mode` / `pos_vel` / `mit` /
`vel` / `robstride_write_param`。**`set_id`、写零位、`clear_error` 不在名单里。**

## 下一步（按顺序）

1. **首次实机验证**（必须用户在场）：检查当前模式与原始位置，现场支撑及独立断电就绪，
   经新卡片当次确认发送单轴 `enable`，观察真实出力，再确认所选轴 `disable` 与撤力。
   未验收前不发送保持/运动帧，不从断开网页推断已停止；之后才做单轴小幅低速。
2. **统一执行层**：同一轨迹同时驱动实机与仿真，以实机反馈为准。
3. **XYZ 目标面板 + 手柄遥操作**：后端解算已现成（`scripts/rebot_core.py` 的 `ik()`
   与 `check_path()`，`/command` 的 `ik_check` 动作），缺的是执行层与手柄映射。
4. 夹爪角度→位移标定、TCP 标定。

## 环境备注

Kubuntu 24.04 笔记本；Python 环境在 `~/.local/share/robotics`（不是 venv，且系统
没装 `python3-venv`）；`~/.local/bin/` 下有 `rebot-sim`、`rebot-twin-sync`、
`rebot-readstate`、`rebot-can-*` 等入口脚本；本机没有 `curl`、没有独立 `node`、
没有 `gh`。ROS 2 为 Jazzy，状态桥使用 `ROS_DOMAIN_ID=42`。
