# reBot B601 机械臂 · 研读笔记与仿真落地路线

> 对象：矽递（Seeed / 深圳矽递科技股份有限公司）**reBot Arm（B601-RS / B601-DM）**
> 目的：把两台臂适配成一套接口，并先打通轻量仿真
> 编写日期：2026-09-29（v2，登录后补全官方规格）　工作目录：`D:\Desktop\seeed Robotics`
> 本文件在 `docs/`，官方规格书 PDF 存在 `refs/`

**标注约定**（后面每条事实都带标记，便于你判断可信度）

| 标记 | 含义 |
|---|---|
| ✅ | 我在你这台机器上实测过 |
| 📄 | 官方仓库原文 / 官方配置文件，我直接抓取核对 |
| 📕 | 官方规格书 PDF，已下载到 `refs/` 并逐页读过 |
| 🔗 | 官方资料（飞书知识库、Seeed wiki）读到的 |
| ⚠️ | 第三方来源或二手转述，官方页面未直接写明，用前需再核实 |

---

## 0. 一句话结论

飞书那个 wiki 节点**不是技术文档本体**，它只是门户：真正的驱动、URDF、仿真教程全在 `wiki.seeedstudio.com` 和 GitHub `Seeed-Projects/*` 仓库里。
四条会直接影响计划的硬事实：

1. **官方控制库 `reBotArm_control_py` 的 Pinocchio 依赖在 Windows 上装不起来**（`pin`/`eigenpy`/`coal-library`/`cmeel-assimp` 在 PyPI 上没有任何 `win_amd64` 轮子）✅ → 官方那条"URDF + Pinocchio + Meshcat"路线必须走 **WSL2 Ubuntu**，或改用 MuJoCo。
2. **MuJoCo 3.14.0 在你这台机器（Windows + Python 3.12）可以原生跑起来** ✅，而且 reBot 的官方 MJCF 模型已经进了 **MuJoCo Menagerie**（`seeed_rebot_devarm`）📄 → 不装 WSL 也能当天看到机械臂在物理仿真里动。
3. **URDF 的关节限位和官方规格书对不上，6 个关节里 5 个不一致**（最夸张的是 J6：规格书 ±90°，URDF ±180°）📕📄 → 仿真限幅必须自己定义一套 `model_limits` / `spec_limits`，不能直接吃 URDF 的 `<limit>`。详见 §4.1。
4. **官方 FAQ 第一句就劝退虚拟机**："由于虚拟机存在较多坑，所以不建议各位使用虚拟机，首选 ubuntu 物理机" 🔗 → WSL2 只适合**纯仿真**（阶段 3），**接实机不要用 WSL**，尤其 RS 那条原生 CAN。

---

## 1. 我是怎么读到的

- **飞书**：第一轮 `lark-cli` 未配置（`auth status` 返回 `not configured`），只能用内置浏览器以"互联网公开"身份读，而**参数对比表、长 FAQ 这些懒加载内容当时根本没渲染出来**，所以第一版笔记漏了。你登录之后，我改用飞书网页自己的接口 `space/api/wiki/v2/tree/get_info` 拿到**完整节点树 + 每个节点的 obj_token**，才把 §2、§3 补齐。`lark-cli` 至今仍未配置，因此**附件下载、评论、电子表格（BOM/SKU 表）读取**依旧只能靠浏览器手工做。
- **规格书 PDF 不需要飞书权限**：它有公开直链 `https://files.seeedstudio.com/Bazaar/product_pdf/E26041401.pdf`，我下载后逐页读完了 📕。以后要官方数字，先试 `files.seeedstudio.com/Bazaar/product_pdf/<SKU>.pdf` 这个规律（`E26062405` 也可访问；注意同一目录下 `E26062404` 是无关产品 CodeCraft，别照抄编号）。
- **Seeed 公开 wiki 与 GitHub**：直接抓原文，不依赖登录（GitHub API 匿名限额 60 次/小时，这次已经打满过一次）。

---

## 2. 飞书知识库结构地图

知识库名：**Seeed Robotics 产品资料知识库**（互联网公开，最新改动集中在 2026-09）。顶层板块：`reBot Arm`、`soarm101`、`Reachy mini`、`社区demo`、`知识库问答链接`。

登录后（含节点 obj_token）逐个读完，实际内容分布是这样的 🔗：

| 节点 | 类型 | 里面有什么 |
|---|---|---|
| 第零步：导购资料 | 目录 | `导购资料`（**参数对比表 + SKU + 商品页 + GitHub 指引，全库最有信息量的一页**）、`FAQ`、`规格书`、`Rebot Arm RS 一图流` |
| 第一步：组装&初次使用攻略 | 目录 | `电源组装教学`、`接线教学`、`刚拿到手初步上手使用教程`（含 DM/RS 初始化差异表）、`摄像头支架下载`；本页另有一份精简版 RS/DM 对照表 |
| 第二步：使用方式汇总 | 目录 | 只有 1 个子页 `机器人教程中心` |
| 第三步：故障排查 | 目录 | `FAQ`（**正文高约 2.2 万画索，很长**）、`reBot BOM清单和sku对照表`（**是电子表格，obj_type=3，不是文档**）、`RS 电机上位机使用` |
| 社区demo | 文档 | **只是一段征集声明**（收集社区公开 Demo、保留署名、不代表官方用法），**没有任何 Demo 列表或链接**，7月6日后未更新 |

以下几处需要修正或补充 🔗：

- **参数表不在"规格书"页里，在"导购资料"页里**（见 §3）。`规格书` 页其实只挂了 **两份 B601-DM 的 PDF**（`E26062405` 组装版、`E26041401` 散件版），**没有 RS 的规格书**。
- **`机器人教程中心` 的正文就是一句话**："通过访问我们的 wiki，获取完整学习路线和 demo 案例" + 指向 `https://wiki.seeedstudio.com/cn/robotics_page/` 🔗。所以"第二步 使用方式汇总"= 公开 wiki，别在飞书里找仿真教程。
- `RS 电机上位机使用` 正文只有一个链接：`https://gitee.com/robstride/MotorStudio` 🔗（灵足时代电机上位机，用来配电机 ID、写零位）。
- `Rebot Arm RS 一图流` 是 **3 张 1280×720 的图片，没有文字层**，脚本抓不到内容；要看 RS 参数得走 §3 的导购资料表或 OCR。
- 飞书里明确写了 **"最新最全的产品资料请前往 wiki"** —— 飞书这边维护的是**组装/排障/购买**，不是软件接口。

---

## 3. 机型与规格（官方口径，已补齐）

reBot B601 = **开源 6 自由度 + 1 夹爪** 机械臂，关节编号 1–7（7 个执行器），外观与关节布局两款一致，差异全在电机/驱动/电源 🔗。

下面这张表来自飞书 **第零步 → 导购资料** 里的"参数对比"表（📌 注意：它不在"规格书"页里，而在**导购资料**页；未登录时这一整块不渲染，所以第一轮漏掉了）：

| 项目 | B601-DM | B601-RS | 来源 |
|---|---|---|---|
| SKU | **100065783** | **100019336** | 📄 导购资料 |
| 商品页 | `seeedstudio.com/reBot-Arm-B601-DM-p-6740.html` | `seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html` | 📄 导购资料 |
| 自由度 | 6 + 1 | 6 + 1 | 📄 |
| 额定负载 | **1.5 kg** | **2.5 kg** | 📄 |
| **最大负载** | **2.5 kg** | **5 kg** | 📄 |
| 精度 | **0.2 mm** | **0.1 mm** | 📄 |
| **臂展** | **767 mm** | **754.7 mm** | 📄 |
| 通信方式 | CAN Bus via USB-CAN 适配器（两款共用同一行） | | 📄 |
| 工作电源 | **24 V DC 15 A** | **48 V DC 12.5 A** | 📄 |
| 上位机 | PC | PC | 📄 |
| 工作温度 | 0 ℃ ～ 40 ℃ | 同 | 📄（️ 与规格书 PDF 的 −20~50 ℃ 冲突，见 §4.1） |
| 电机 | 达妙科技 DM 行星减速关节电机 | 灵足时代 Robostride 准直驱关节电机 | 📄 |
| 结构 | 工业级 CNC | 同 | 📄 |

**B601-DM 散件版官方规格书**（英文，SKU `E26041401`，更新于 2026-09-11；我已下载并逐页读过 📕 → `refs/E26041401-reBot-Arm-B601-DM-散件版规格书.pdf`）补充：

| 项目 | 值 |
|---|---|
| Reach | **767 mm**；**不带夹爪 607 mm** |
| Payload | **1.5 kg**（不带夹爪；**建议按 70% 行程使用**） |
| Repeatability | **±0.2 mm** |
| 整机重量 | **约 4.7 kg** |
| 舵机 | **DM-3/4340P-2EC ×3 + DM-3/4310-2EC ×4**（夹爪那颗也是 4310，所以 4310 共 4 颗） |
| 关节活动范围 | J1 ±150°；**J2 −220°~0°；J3 −220°~0°**；J4 ±90°；J5 ±90°；**J6 ±90°**；夹爪 −325°~0° |
| 工作温度 | **−20 ~ 50 ℃** |
| 电源 | DC 24 V 15 A（**散件版随包附赠 AC/DC 适配器 24V 15A 3-in-1**，CN/US/EU 插头） |
| 随包硬件 | 含 **DAMIAO USB-CAN Driver Board ×1**、XT30 2+2 电源分离板、若干 XT30 线材、内六角工具组、金属件 21+10 / 3D 打印件 17+5 |
| 官方声明的兼容性 | ROS1、**MoveIt**、ROS2、**MoveIt2**、Python、**LeRobot**、**Isaac Sim**、**Pinocchio**、Open Source（规格书里全部打勾） |

> 之前那条"臂展 767 mm / <0.2 mm 来自第三方 embedsbc"的 ⚠️ 标记可以撤销 —— **官方规格书就是这个数**，且 RS 侧的官方值是 754.7 mm / 0.1 mm。

**DM 也有 USB-CAN 板**这件事值得单独记：官方 config 里 DM 默认 `channel: /dev/ttyACM0`（达妙串口桥），但**散件版盒子里装的是 Damiao USB-CAN Driver Board**，所以 DM 实际有"串口桥"和"原生 CAN"两条链路可选 📕📄。

---

## 4. RS / DM 的真实差异 → 统一接口怎么切

这是"两台都要、抽象成一套接口"的核心。以下全部来自官方仓库 `Seeed-Projects/reBotArm_control_py` 的 `config/rebotarm_rs.yaml` 与 `config/rebotarm_dm.yaml` 原文 📄：

| 抽象点 | RS | DM | 接口设计结论 |
|---|---|---|---|
| `vendor` | `robstride` | `damiao` | 作为后端选择键 |
| `channel` | `can0`（SocketCAN） | `/dev/ttyACM0`（串口桥） | 传输层必须可插拔，不能写死 |
| `urdf_path` | `urdf/RS/urdf/ReBot_Arm_RS.urdf` | `urdf/DM/urdf/ReBot_Arm_DM.urdf` | 模型按机型注入 |
| `end_effector_frame` | **`gripper_end`** | **`end_link`** | ⚠️ 名字不同！正运动学/IK 目标帧必须走配置，绝不能硬编码 |
| `arm_control_mode` 默认 | **`mit`** | **`posvel`** | 两款默认控制模式不同，接口要显式声明而不是靠默认值 |
| `feedback_id` | 全部 `0xFD` | 每关节不同 `0x11/0x12/0x13/0x14…` | 反馈寻址方式不同，属于驱动层差异 |
| 电机型号 | `rs-06` ×3、`rs-00` ×4 | `4340P` ×3、`4310` ×3 | 力矩上限/温降保护策略按机型 |
| MIT 增益 kp/kd | 50/3、150/10、150/10、50/5、50/4、50/4、夹爪 50/4 | 120/8、120/8、120/8、18/2、… | 增益表必须整表随机型切换 |
| POS_VEL 参数 | `vel_kp/pos_kp/vlim`（无 `ki`） | 多带 `vel_ki/pos_ki` | 控制器参数结构不同 → 用"每机型一份 YAML"而不是统一 schema |
| `rate` | 500 | 500 | 控制频率可共用 |
| 重力补偿 | 有 `profiles.basic` / `velocity_lock`（kp 15/2.5、tau_scale 等） | 见 DM 配置 | 顺从/拖拽示教相关，两款分开标定 |

关节限位（URDF 原文，单位 rad）📄：

| | J1 | J2 | J3 | J4 | J5 | J6 | 夹爪 |
|---|---|---|---|---|---|---|---|
| RS | −2.8~2.8 | 0~3.14 | 0~3.14 | ±1.57 | ±1.57 | ±3.14 | `gripper_joint1` 0~0.05 m、`gripper_joint2` 0~0.0715 m（均为移动副 prismatic） |
| DM | ±2.8 | −3.14~0 | −3.14~0 | −1.87~1.57 | ±1.57 | ±3.14 | `finger_left` 0~0.0285 m、`finger_right` −0.0285~0 m（移动副） |

力矩与速度上限（URDF `effort` / `velocity`）：RS = J1–J3 为 36 N·m / 50 rad/s，J4–J6 为 14 N·m / 40 rad/s；DM = J1–J3 为 27 N·m / 50 rad/s，J4–J6 为 7 N·m / 200 rad/s。

> 上表我逐个关节抓原文核对过，与仓库一致 📄。两点接口结论：
> ① **命名体系完全不同** —— RS 连杆 `base_link, link1…link6, gripper_end, gripper_left, gripper_right`；DM 连杆 `base_link, link1…link6, end_link, finger_left_link, finger_right_link`。夹爪关节名也不一样，接口层需要一张关节名映射表，不能假设两台同名。
> ② **J2/J3 符号方向相反**（RS 是 0~3.14，DM 是 −3.14~0）—— 这是"一套接口"最容易翻车的地方：必须显式定义统一正方向，并在机型层做符号/偏置映射，否则同一份轨迹喂给两台臂会做出镜像动作。另外 DM 的 J4–J6 `velocity=200 rad/s` 明显是 CAD 导出占位值（远超实机能力），别拿它当限幅依据。

### 4.1 ⚠️ 规格书限位 ≠ URDF 限位（做仿真限幅前必须先决定信谁）

把官方规格书 📕 和官方 URDF 📄 的 DM 关节范围放到一起，**六个关节里有五个对不上**（180° = π rad）：

| 关节 | 规格书 E26041401 | URDF `ReBot_Arm_DM.urdf` | 差多少 |
|---|---|---|---|
| J1 | ±150° | ±2.8 rad = **±160.4°** | URDF 宽 10.4° |
| J2 | **−220° ~ 0°** | −3.14~0 = **−180° ~ 0°** | URDF **窄 40°** |
| J3 | **−220° ~ 0°** | −3.14~0 = **−180° ~ 0°** | URDF **窄 40°** |
| J4 | ±90° | −1.87~1.57 rad = **−107.1° ~ +90°** | 正侧一致，负侧多 17° |
| J5 | ±90° | ±1.57 rad = ±89.95° | ✅ 基本一致 |
| J6 | **±90°** | ±3.14 rad = **±180°** | **URDF 宽一倍** |
| 夹爪 | −325° ~ 0°（电机轴角） | `finger_left` 0~0.0285 m、`finger_right` −0.0285~0 m（棱柱位移） | 单位都不同，需要换算关系 |

结论与做法：

1. **URDF 是 CAD（SolidWorks）导出的，不是按规格书填的** —— 同仓库里 `ReBot_Arm_RS.csv` 就是 SolidWorks 导出件，`velocity=200 rad/s` 这类占位值也印证了这点。所以 URDF 的 `<limit>` 只能当"模型自带的软限位"，**不能当机械行程**。
2. 统一接口里请分两套常量：`model_limits`（来自 URDF，用于仿真求解器，保证模型不自穿）与 `spec_limits`（来自规格书，用于对用户的可行域裁剪与告警）。**规划/示教时取两者交集**，并在配置里显式记录每个关节用的是哪个。
3. J2/J3 反过来了：URDF 比规格书**窄** 40°，意味着按 URDF 能到的姿态里有一部分是规格允许但模型禁止的（大臂后仰场景），做工作空间分析时会低估能力。
4. 规格书还写了 **"Payload 1.5 kg（不带夹爪），建议按 70% 行程使用"** 📕 —— 这条"降额使用"的口径应该进接口的默认安全参数，而不是等人手动记得。

> 顺带一条同类冲突：**工作温度** 飞书表写 0~40 ℃，规格书 PDF 写 −20~50 ℃ 🔗。这个不影响仿真，但对外材料要用哪个得问产品。

其他实机侧差异（⚠️ 子代理从官方 wiki / HuggingFace 文档读到，接实机前请再核对原文）：

- RS 夹爪是**力受限阻抗**，motorbridge 不暴露 `force_pos`；DM 夹爪走 `force_pos`，`gripper_torque_ratio` 默认 0.07。
- 力矩一关，臂会随重力下垂 —— 任何"断开/急停"逻辑都要考虑支撑动作。
- Windows 上原生 CAN（PEAK PCAN + PCAN-Basic）官方标注为 **experimental**；macOS 需要 MacCAN PCBUSB。也就是说 **RS 的实机链路在 Windows 上是最不稳的一环**。

---

## 5. 软件栈与仓库清单（活跃度已核对）

| 仓库 / 包 | 作用 | 关键约束 | 最近推送 / star 📄 |
|---|---|---|---|
| `Seeed-Projects/reBot-DevArm` | 主仓库（硬件资料、上位机、控制示例） | — | 2026-09-28 / 4.3k |
| `Seeed-Projects/reBotArm_control_py` | **Python 控制 + URDF + Pinocchio/Meshcat 仿真示例** | `requires-python >=3.10,<3.12`；`pin>=3.9.0`、`meshcat>=0.3.2`、`motorbridge>=0.4.9` | 2026-09-11 / 51 |
| `Seeed-Projects/reBot-Isaacsim` | Isaac Sim 6.0 仿真（含 MJCF、USD） | 子模块 `reBotArm_control_py`；`--recurse-submodules` | 2026-09-22 / 12 |
| `Seeed-Projects/lerobot` | LeRobot fork（遥操作/采集/训练） | `pip install -e ".[rebot]"`，相机加 `.[realsense]` | 2026-08-31 / 38 |
| `pip: motorbridge` | 电机驱动库（含 gateway `motorbridge-gateway`、Web 端 motorbridge-studio） | 有 **win_amd64 轮子：cp310/cp311/cp312**，**无 cp313** ✅ | 0.5.6 |
| `google-deepmind/mujoco_menagerie` → `seeed_rebot_devarm` | **官方 MJCF + 网格** | 关节 `joint1..joint6` + `joint_left/joint_right`（`<equality>` 耦合） | 📄 已确认存在 |
| `Yang-Ci/ReBot_Arm_DigitalTwin_RS` | Web 仿真器 / 数字孪生（社区，非 Seeed 官方组织） | Node 18+，含 MuJoCo WASM + ROS 2 Jazzy | 2026-09-21 / 2 |
| `gitee.com/robstride/MotorStudio` | RS 电机上位机（配 ID、写零位） | Windows 桌面工具 | 🔗 飞书 |
| DM 上位机 | `DM_Tools_v1.8.0.1.exe` | Windows | ⚠️ 子代理转述 |

**官方软件栈目标环境**：Ubuntu 24.04/22.04 或 Jetson Orin（JetPack 6.0/6.1，**6.2 不支持**）、Python 3.12、ROS 2（`reBotArmController_ROS2`，包 `rebotarm_bringup` / `rebotarm_moveit_config`，URDF `rebotarm_rs.urdf.xacro`）。

---

## 6. 仿真路线盘点（官方给了 4 条）

| 路线 | 资产在哪 | 现在能做到 | 现在做不到 / 坑 |
|---|---|---|---|
| **URDF + Pinocchio + Meshcat** | `reBotArm_control_py`：`urdf/RS/…`、`urdf/DM/…`、`example/sim/{fk_sim,ik_sim,traj_sim,visualizer}.py`、`config/*.yaml` | 纯软件、无需硬件：FK、带阻尼最小二乘+重试的 IK、笛卡尔轨迹播放，Meshcat 网页查看器 `127.0.0.1:7000` | **Windows 装不了 `pin`** ✅；仓库锁 Python <3.12；示例是键盘驱动，**没有实机→仿真遥操作** |
| **Isaac Sim** | `reBot-Isaacsim`，预烘焙 USD：`usd/RS-rebot-dev-arm/RS-rebot-dev-arm.usda`、`usd/reBot_B601_DM/reBot_B601_DM.usda`，`ROBOT_PRIM_PATH=/World/reBotArm` | 两条终端通过 **UDP JSON（127.0.0.1:5005：`sequence,timestamp,joint_positions[6] rad,gripper_position m`）** 实时镜像；`joint_reader_sender.py` 做实机→仿真只读镜像 60 Hz；`gravity_joint_sender.py` 做重力补偿拖拽示教 | 要求 **RTX 3070+ ≥8 GB、≥32 GB RAM、≥100 GB SSD、Ubuntu 或 Win11+WSL2**；教程里没有 URDF→USD 命令（资产已烘焙）；**不含 Isaac Lab、无 RL / 模仿学习 / 域随机化** |
| **Web 仿真器 / 数字孪生** | `Yang-Ci/ReBot_Arm_DigitalTwin_RS`（社区） | 三套引擎：Three.js r128 浏览器 URDF 查看器（`:3002`）、**MuJoCo WASM**（`:5173`，加载 `models/rs_grasp_scene.xml`）、ROS 2 Jazzy + MoveIt 2 + MCP 文本代理；可通过 **rosbridge `ws://ip:9090`** 和 `MOTORBRIDGE_WS_URL=ws://127.0.0.1:9002` 外部驱动 | 首帧网格约 62 MB；非 Seeed 官方组织维护，star 少；有硬件确认门禁 `I_UNDERSTAND_RS_WILL_MOVE` |
| **LeRobot 生态** | `Seeed-Projects/lerobot` fork | 主从遥操作（leader = **reBot Arm 102**）、`lerobot-record/train/replay`，策略 **ACT / SmolVLA / Pi0 / Pi0.5 / GR00T N1.5**；标定 `lerobot-calibrate` → `~/.cache/huggingface/lerobot/calibration/robots` | 需要实机 + 主臂 + 相机；**训练要 CUDA**，你这台 3050 Laptop 基本只能跑 ACT/SmolVLA 级别；RS 与 DM 的几何/增益/动力学不同，**数据集与策略不能直接互传**（官方明确提示） |
| **MuJoCo（Menagerie）** | `mujoco_menagerie/seeed_rebot_devarm` | **本机原生可跑** ✅；MJCF 限位与 URDF 基本一致（J4 为 −1.79~1.69） | 仓库里那份 MJCF 是**从 Menagerie vendor 来的**（pin 到 commit `da76818e…`），不是独立资产源；`reBotArm_control_py` 本身**不含 MJCF** 📄 |

**仿真↔实机一致性（做 sim-to-real 前必看）** ⚠️（子代理阅读 `reBot-Isaacsim` 仓库文档所得，建议二次确认）：

- 单位约定：手臂关节在 URDF/USD/MJCF/UDP/ROS 里统一用 **rad**，夹爪电机在硬件上是 **rad**，但模型里手指是 **米** —— 仓库里各处 sender 用了 `×0.03`、`×0.007`、`×0.045` 等不同经验缩放。
- 仓库自带的 `VALIDATION.md` 记录了模型与规格的偏差（J1 ±2.8 rad vs 规格 ±150°、J2/J3 180° vs 220°、质量 6.009 kg vs 6.5–6.7 kg、腕部 40 rad/s vs 33），并警告上传的 USD 里 **j2/j3 限位是镜像的**，换资产前必须做符号检查。
- 好消息：同一份文档记录 MuJoCo 3.10.0 与 Pinocchio 3.4.0 的**重力力矩一致性验证到 5.88e-6 N·m**。

---

## 7. 本机环境实测结论 ✅

| 项 | 实测结果 | 影响 |
|---|---|---|
| OS | Windows 11 家庭版 | 官方栈以 Ubuntu 为主，Windows 是二等公民 |
| GPU | RTX 3050 Laptop（另有 Intel Iris Xe 和两个虚拟显卡：ToDesk、GameViewer） | **达不到 Isaac Sim 官方门槛（RTX 3070+ / 8 GB）**，Isaac 路线短期放弃 |
| WSL | **没有任何已安装发行版** | 走 Linux 路线要先 `wsl --install -d Ubuntu-24.04` |
| Python | 3.13.5（默认 `py -3`）、3.10.9（`D:\python 3.10.9`）、uv 托管 3.12.14 | 官方仓库要 `<3.12` → 用 **3.10 或 3.11**，别用默认 3.13 |
| uv | 0.12.8 已装 | 建 venv / 锁依赖很方便 |
| `mujoco` | **3.14.0 在 py3.12 Windows 安装成功、`mj_step` 2000 步 0.004 s 跑通** | 本机当天就能做物理仿真 |
| `motorbridge` | 有 win_amd64 轮子（cp310/311/312），**cp313 无** | 实机驱动在 Windows 上可行（Python 3.12 及以下） |
| `pin`(Pinocchio) | **PyPI 上 `pin`/`eigenpy`/`coal-library`/`cmeel-assimp` 全部没有 win_amd64 轮子**；`--only-binary` 安装直接判不可满足 | 官方 Pinocchio 路线**在原生 Windows 上不可用**，必须 WSL2 或 conda-forge（conda-forge 是否覆盖 win-64 我因网络 SSL 错误未验证，标注为未知） |
| `lerobot` | 0.6.1，`requires_python >=3.12`，纯 Python 轮子 | 能装，但依赖的 torch/CUDA 与相机链路在这台机器上是另一回事 |

---

## 8. 落地路线（按你选的"轻量优先：URDF + Pinocchio/Meshcat（+ MuJoCo）"）

**阶段 0 · 资产落地（半天，纯 Windows）**
把 `reBotArm_control_py` 拉进工作区，确认 `urdf/RS`、`urdf/DM` 两套 URDF + STL 网格、`config/*.yaml`、`example/sim/*` 都在；同时从 Menagerie 取 `seeed_rebot_devarm` 的 MJCF。
验收：本地能列出 RS/DM 两个 URDF 文件路径，且 URDF 里 `package://` 引用为零（官方用的是相对 `../meshes/...`，所以不需要 ROS 工作区就能加载）📄。

**阶段 1 · 本机先跑起来（1 天，Windows 原生）**
用 `uv venv --python 3.12` 建环境，装 `mujoco`，写一个最小加载脚本：读 Menagerie 的 `rebot_devarm.xml`，打印 `nq`、关节名与限位，跑 5 秒 `mj_step` 并输出关节角轨迹。
验收：控制台能看到 `joint1..joint6 / joint_left / joint_right` 与限位，仿真不报错。
理由：这一步不依赖 Pinocchio，先把"模型是对的"锁死，再谈运动学。

**阶段 2 · 统一接口层（2–3 天，跨平台代码）**
按第 4 节的差异表写一个 `rebot/` 包：机型描述全部来自 YAML（`vendor / transport / urdf_path / ee_frame / control_mode / gains / joint_sign_map / limits`），对上层只暴露 `load_model() / fk(q) / ik(target) / move_to(q) / read_state()`。仿真后端先接 MuJoCo，实机后端预留 motorbridge。
**重点**：J2/J3 符号映射和末端帧名（`gripper_end` vs `end_link`）必须在这一层消化掉，上层不许出现机型分支。
验收：同一份 6 关节轨迹 + 同一个 IK 目标，喂给 RS 与 DM 两个配置，仿真里两台臂都朝**语义一致**的方向动（用截图/关节角对比留证）。

**阶段 3 · 补齐官方 Pinocchio 路线（1 天，需要 WSL2）**
`wsl --install -d Ubuntu-24.04` → 在 WSL 里 `uv sync` 官方仓库（Python 3.10/3.11），跑 `example/sim/fk_sim.py`、`ik_sim.py`、`traj_sim.py`，Meshcat 浏览器打开 `127.0.0.1:7000`。
验收：官方三个示例脚本原样跑通，并且用阶段 2 的 MuJoCo 后端对同一组 IK 解做交叉校验（末端位姿误差给个数字）。
这一步顺带把"两台臂的官方仿真口径"和"我们自己的接口"绑在一起，后面换 Isaac 也不用重来。

> ⚠️ 边界：官方 FAQ 明确**不建议用虚拟机**（首选 Ubuntu 物理机）🔗。这条针对的是**接实机**（CAN 时序、串口透传、USB 直通在 VM 里坑多）。所以 WSL2 只用于本阶段的**纯软件仿真**；等真要接 RS 那台臂时，另找 Ubuntu 物理机或双系统，不要用 WSL 硬扛。

**阶段 4 · 再决定要不要上学习链路**
实机 + reBot 102 主臂到位后，才谈 LeRobot 采集/训练；Isaac Sim 换机器或上云。届时把阶段 2 的接口层直接接上 UDP/ROS 镜像即可。

---

## 9. 风险与待你确认（一次性列完，不重复提醒）

**需要你拍板/提供信息的**

1. **仿真限位用哪套口径**：URDF 的 `<limit>` 和规格书差得不少（J6 差一倍，见 §4.1）。我倾向"规划用规格书、求解器用 URDF、取交集告警"，但这会影响工作空间分析结论，你如果知道官方为什么这么填（比如 URDF 是机构极限、规格书是推荐行程），告诉我我就照这个写死。
2. **RS 的官方规格书在哪**：飞书 `规格书` 页只有 DM 的两份 PDF，RS 侧只有一张"一图流"（3 张无文字层图片）和导购资料里的对照表。要做 RS 的精确仿真，得有人把 RS 规格书补出来或指个公开链接。
3. **你手上有实机吗、是哪台**：决定了我该不该现在就为 motorbridge / CAN 预留真机联调路径，还是纯仿真推进。
4. **工作温度对外口径**：飞书表 0~40 ℃ vs 规格书 −20~50 ℃。不影响技术，但两份官方材料自相矛盾，别到时候被客户抓。

**技术风险（已知，按此规避即可）**

5. **RS 的实机链路在 Windows 上不稳**（原生 CAN 官方标 experimental），且官方 FAQ 明确**不建议用虚拟机** 🔗。→ 结论：Windows 只做仿真；接 RS 实机要 Ubuntu 物理机。
6. **Isaac Sim 这台机器不达标**（RTX 3050 Laptop / 无 WSL 发行版 / 官方要求 3070+ 8GB、32GB 内存、100GB SSD）。要走 Isaac 就得换机器或云 GPU。
7. **conda-forge 是否提供 win-64 的 pinocchio 我没能验证**（查询时 SSL 被中断）。若坚持纯 Windows 跑 Pinocchio，这是唯一可能的出路，需要先验证；否则 Pinocchio 只能走 WSL。
8. **官方仓库锁 Python `<3.12`，`lerobot` 要 `>=3.12`** → 两套 venv 分开，别用默认的 3.13（`mujoco`/`motorbridge` 都没有 cp313 的 Windows 轮子 ✅）。
9. **USD 资产另有已记录的偏差**（`reBot-Isaacsim` 的 `VALIDATION.md` 提到质量 6.009 kg vs 规格 6.5–6.7 kg、腕部 40 rad/s vs 33、j2/j3 限位镜像）⚠️。注意规格书 DM 写的是 **约 4.7 kg** 📕，和上面两个数都不同 —— 说明那份偏差记录针对的是 RS。接 Isaac 前重新核一遍。
10. **DM 的软件资产覆盖弱于 RS**：官方 wiki 页面基本只写 RS，`reBot-Isaacsim` 的 DM 教程指向第三方 fork；`config/rebotarm.yaml`（无后缀那份）归属也要确认。你说"两台都要"，DM 侧可能要我们自己补。
11. **`reBot BOM清单和sku对照表` 是飞书电子表格（obj_type=3）**，浏览器里读不出单元格；`lark-cli` 仍未配置，所以这张表我暂时拿不到。好消息是 SKU 号已经从导购资料里拿到了（DM 100065783 / RS 100019336），BOM 明细在 GitHub `reBot-DevArm` 仓库里也有一份。

**已撤销的两条**（第一轮因为没登录而标的不确定的，现在官方数据已确认）

- ~~臂展 767 mm / 重复精度 <0.2 mm 只有第三方来源~~ → 官方规格书 📕 与导购资料 📄 均确认，且补上了 RS 的 754.7 mm / 0.1 mm。
- ~~`社区demo` 里可能有别人做过的仿真 demo~~ → 已读，**只是一段征集声明，没有任何 demo 内容**。

---

## 附录 A · 链接清单

**飞书知识库（`Seeed Robotics 产品资料知识库`，space_id `7654579348120275897`）**

| 页面 | 链接 |
|---|---|
| 入口 `reBot Arm` | `https://seeedstudio.feishu.cn/wiki/Ppwjw3s0iiSk4zkZ7jkcReuontf` |
| **第零步 → 导购资料**（**参数对比表 + SKU + 商品页**） | `.../wiki/PNDlwfAkLimn7IkmzqQcLWkAnZb` |
| 第零步 → FAQ | `.../wiki/X6fMwpyqCi1bCEkv8vdcx2kTnRg` |
| 第零步 → 规格书（只有 DM 两份 PDF） | `.../wiki/AcnDwJBgwiaN3Nk3QXvcZA3wnAg` |
| 第零步 → Rebot Arm RS 一图流（3 张图，无文字层） | `.../wiki/XbglwBIr7i94CbkxUS0cFBPXnBd` |
| 第一步 组装&初次使用（精简版 RS/DM 表 + 流程） | `.../wiki/LZhVwm6UWipu4PkbiYIcLfmrnHg` |
| 刚拿到手初步上手（组装 + 初始化，DM/RS 差异表） | `.../wiki/S6dewDtXxi8UcHkznq5cNrSQnjg` |
| 电源组装 / 接线 / 摄像头支架 | `.../wiki/ESBewOsfYi3bXQkhzUVczXkZnru` · `.../wiki/OP4LwhExjit0Hwk2xZzcIfkFnoe` · `.../wiki/DyTGwGiUWixZXOk0ahDc2NKFnSc` |
| 第二步 使用方式汇总 → 机器人教程中心（=跳转 wiki） | `.../wiki/DxlWw1Kf2iubAvkaMVAc96oUnoc` · `.../wiki/I7zGw6959iJnWekvEY6cCFNnnFd` |
| 第三步 故障排查 → **FAQ（含"不建议用虚拟机"）** | `.../wiki/JRr7wX77gi5HhQkCFn7cbBO8nJh` · `.../wiki/D4KswZTX7iPnF0k6RLncIsCtnpe` |
| 第三步 → reBot BOM清单和sku对照表（**电子表格**） | `.../wiki/NgC5wuPhpioAFPkCsG1cbS5Zn6e` |
| 第三步 → RS 电机上位机使用 | `.../wiki/Ek9fwmV2biOXMAkSzxQcjW1AnAb` |
| 社区demo（仅征集声明，无内容） | `.../wiki/Yl0fwOyRaiIJMIk3n3kclphgnrh` |

**官方规格书 / 商品页（公开，无需登录）**

- DM 散件版规格书（英文，已读 📕）：`https://files.seeedstudio.com/Bazaar/product_pdf/E26041401.pdf` → 本地 `refs/E26041401-reBot-Arm-B601-DM-散件版规格书.pdf`
- DM 组装版规格书（飞书标注 `E26062405`，已下载**未读**）：`https://files.seeedstudio.com/Bazaar/product_pdf/E26062405.pdf` → 本地 `refs/E26062405-reBot-Arm-B601-DM-组装版规格书-未读.pdf`
- 商品页：DM `https://www.seeedstudio.com/reBot-Arm-B601-DM-p-6740.html` ｜ RS（组装版含夹爪套件）`https://www.seeedstudio.com/reBot-Arm-B601-RS-Assembled-Kit-with-Gripper-p-6865.html`
- DM 散件版产品页（规格书内链）：`https://www.seeedstudio.com/reBot-Arm-B601-DM-Bundle.html`
- 安全声明（规格书内链）："Mechanical Arm DIY Kit - Safety Disclaimer and Risk Notice"

**Seeed 公开 wiki（技术文档本体）**
- 总入口：`https://wiki.seeedstudio.com/cn/robotics_page/`
- 快速开始：`/cn/rebot_b601_rs_getting_started/`、`/cn/rebot_b601_dm_getting_started/`
- Pinocchio + Meshcat：`/cn/rebot_arm_b601_rs_pinocchio_meshcat/`、`/cn/rebot_arm_b601_dm_pinocchio_meshcat/`
- Isaac Sim：`/cn/rebot_arm_b601_rs_isaacsim/`、`/cn/rebot_arm_b601_dm_isaacsim/`
- LeRobot：`/cn/rebot_arm_b601_rs_lerobot/`、`/cn/rebot_arm_b601_dm_lerobot/`
- Web 仿真器开发者指南：`/cn/rebot_arm_b601_rs_web_simulator_developer_guide/`、`/cn/rebot_arm_b601_dm_web_simulator_developer_guide/`
- ROS 2 集成：`/cn/rebot_arm_b601_rs_ros2_integration/`（子代理引用，未逐字复核）
- SO-101 对照：`/cn/lerobot_so100m_new/`、`/cn/simulate_soarm101_by_leisaac/`、`/cn/training_soarm101_policy_with_isaacLab/`

**代码仓库**
- `https://github.com/Seeed-Projects/reBot-DevArm` —— 官方在导购资料里描述为：**含 DM/RS 软件算法与社区案例、参数对比、机械结构（STEP）、BOM 表**
- `https://github.com/Seeed-Projects/reBotArm_control_py` —— 控制库 + URDF + Pinocchio/Meshcat 示例
- `https://github.com/Seeed-Projects/reBot-Isaacsim`
- `https://github.com/Seeed-Projects/lerobot`
- `https://github.com/google-deepmind/mujoco_menagerie`（`seeed_rebot_devarm`）
- `https://github.com/Yang-Ci/ReBot_Arm_DigitalTwin_RS`
- `https://gitee.com/robstride/MotorStudio`

---

## 附录 B · 变更记录

- **v1（2026-09-29 上午）**：未登录，用浏览器读飞书公开分享 + 抓 Seeed wiki / GitHub + 本机实测包可用性。飞书侧只读到目录与懒加载出来的少量内容。
- **v2（2026-09-29，你登录飞书后）**：改用飞书网页内部接口拿到完整节点树与 obj_token，逐个读完 `导购资料`、`规格书`、`FAQ`、`社区demo`、`RS 电机上位机使用`；下载并读完官方 DM 散件版规格书 PDF。
  主要变化：① §3 换成官方参数表（补上最大负载、精度、臂展、SKU、商品页、重量、舵机型号、随包硬件）；② 新增 §4.1「规格书限位 ≠ URDF 限位」；③ §2 修正"参数表在第一步"→"在导购资料"，并确认 `社区demo` 无内容；④ §0/§8 补入官方"不建议用虚拟机"的口径，限定 WSL 只用于纯仿真；⑤ §9 撤销两条 ⚠️、新增 4 条待你拍板的问题。
