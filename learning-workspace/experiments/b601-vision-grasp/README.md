# B601-RS 仿真视觉抓取支线

**一小时SAC续研R2：** 147564步，96个新场景354次任务；技能验证30/36对零修正28/36，部分首次抓偏减少，完整成功未提高，不采用为V4。默认V3；[结果和失败](variants/residual-rl-02/REPORT.md)、[推理与复现](variants/residual-rl-02/README.md)、[实机数据](variants/residual-rl-02/REAL_DATA.md)。

**2026-10-24 新主机续研：** 已在Windows跑通冻结V3，新增接触/相机/响应变化、108个训练场景和12个验证场景，以及小型ACT风格动作学习。两轮各30个新场景配对测试，V3均30/30、动作候选均0/30，不采用。默认V3与全部历史失败保留；[续研报告](variants/act-01/REPORT.md)、[复现与交付](variants/act-01/README.md)、[实机数据回传](variants/act-01/REAL_DATA.md)。

**最新验证版本 V3（2026-10-22）：** 310次新正式对照和完整记录审计通过，静态和受扰各30/30；最新V2同场景29/30、30/30。修复一个新初始漏检，受扰首次26/30、重试4次与V2相同，共同成功耗时基本持平。仅1个配对新增成功，不声称稳定的成功率优势。直接运行`./verified_task.sh --goal B`，支持A/B/C；入口核对108个独立冻结文件。结果、限制、成功/失败回放见 [V3报告](GOAL_GEOMETRY_FALLBACK_V3_REPORT.md)。[V2](GOAL_V2_REPORT.md)、[V1](GOAL_V1_REPORT.md)及全部失败候选继续保留。持续任务状态见 [ITERATIONS.md](ITERATIONS.md)，当前下一步见 [NEXT_ITERATION.md](NEXT_ITERATION.md)。

**最近多姿态反馈候选未采用：** 250次测试及审计完成，反馈静态/扰动29/30、29/30，V1为30/30、28/30；静态不退步门槛失败，见 [完整报告](GOAL_MULTIVIEW_V2_REPORT.md)。已修复[训练图像渲染后端差异](RENDER_BACKEND_FINDING.md)，并依据[低覆盖率物理诊断](LOW_COVERAGE_REPORT.md)开始下一候选开发。此前190次反馈候选也未采用，见 [历史报告](GOAL_FEEDBACK_V2_REPORT.md)。

**历史统一EGL候选02未采用：** [250次正式对照与审计](GOAL_EGL_COVERAGE_V2_REPORT.md)完成，反馈静态30/30、受扰29/30，V1两块30/30；受扰首次25对16、重试5对14，但完整任务不退步门槛失败。另一个[HOME优先候选的70次开发](HOME_FIRST_DEVELOPMENT_REPORT.md)减少了观察耗时，却仍未解决当时的漏检回归，因此当轮保留V1，随后继续视觉碎片开发。用户已取消额度停止条件。

**历史视觉碎片与HOME优先V2：** [图像开发](MASK_FRAGMENT_DEVELOPMENT_REPORT.md)、[70次开发](FRAGMENT_HOME_DEVELOPMENT_REPORT.md)及按[预先约定](FRAGMENT_HOME_EVAL_PROTOCOL.md)的310次正式对照已完成。HOME优先相对原两姿态减少观察耗时，新观测更新减少受扰重试。该轮剩余的初始漏检22000018后来在V3开发回归中首次完成；空场景无动作和有限恢复要求继续保留。

**几何视觉有限回退已采用为V3：** [开发报告](GEOMETRY_POLICY_DEVELOPMENT_REPORT.md)保留两个因旧成功退步而未采用的直接裁剪候选，以及有限回退版的完整图像、9次已知回归和70次新开发证据。按[协议](GEOMETRY_FALLBACK_EVAL_PROTOCOL.md)完成310次新正式对照及审计后，独立冻结V3。已知低置信16400002仍失败；[42次观察邻域诊断](RELATIVE_VIEW_REPORT.md)表明视觉门槛通过不保证准确，下一轮研究定位风险和遮挡/形状信息。用户已取消额度停止条件。

**图像策略正式候选未采用：** [310次新正式对照及全量审计](GOAL_IMAGE_RESIDUAL_V4_CANDIDATE_REPORT.md)完成，图像反馈、最新V3及零残差均静态29/30、受扰30/30，首次和重试也持平，工程门槛通过但任务收益条件失败，入口保持V3。6,419参数模型实际参与动作，离线定位改善及旧16400002修复不足以采用；全部训练/开发及失败回放见[开发报告](IMAGE_RESIDUAL_DEVELOPMENT_REPORT.md)。下一轮研究位置风险和学习观察决策。

**风险模型首轮未通过：** [7,168张新初始化观察图及固定模型复核](RISK_OBSERVATION_DATA_REPORT.md)全量审计通过后，按训练前约定训练4,545参数风险头，直接/归档审计和300轮逐参数重训复现均通过。但校准阈值拒绝全部图，准确图保留0%，未接入动作，详见[风险模型报告](VISUAL_RISK_MODEL_REPORT.md)。不在同一验证上调阈值；默认V3，继续研究自身遮挡与观察选择。

**历史 V0：** A/B/C 目标、学习式位置修正、搬运与松手、视觉验收、有限重试和独立物理评分。运行方法见 [GOAL_TASK.md](GOAL_TASK.md)，V0 对照结果见 [GOAL_REPORT.md](GOAL_REPORT.md)。下面保留首阶段抓取实验的原始说明。

本会话的独立实验，2026-10-20 建立。相机看见目标 → 神经网络分割 → 深度反投影 → IK → 接近、闭爪、抬升 → 接触/高度判分 → 保存数据 → 继续训练。

**已经跑通 CPU 上的两轮合成视觉训练与接触物理抓取。** 训练的是视觉分割网络；动作来自显式控制器，尚未学习端到端动作策略。当前任务限定为单个红/橙色方块、固定俯视 RGB-D、已知桌面。没有运行真实相机或机械臂。

## 本轮结果

| 检查 | 结果 |
| --- | --- |
| 第 1 轮 / 第 2 轮训练数据 | 192 / 累计 384 张，包含空场景 |
| 两轮各自的新抓取场景 | 7/8 / 8/8 成功 |
| 同一组 8 个额外场景比较 | 两个模型均 8/8，不能据此宣称显著提升 |
| 第 2 轮固定验证集 | 48 个目标场景中漏检 4；8 个空场景中误检 1 |
| 第 2 轮检出目标 XY 误差 | 中位 1.08 mm；P95 5.04 mm；仅仿真坐标 |
| 另外 8 个空场景检查 | 均无运动 |
| 工作区内故意偏移抓取点 65 mm | 抓空，未被判为成功 |
| 同场景比较及偏移试验记录 | 4,900 条动作状态，1,232 帧配对 RGB-D |

成功要求：抬升后连续 1 秒的最低物体抬升量 >60 mm，双侧手指接触占比 >90%。只靠接触和摩擦，无物体绑定或运行时瞬移。小样本成绩不代表真实成功率，详见 [完整试验记录](REPORT.md)。

## 运行与续训

当前目录有独立 Python 3.12 环境。第一次在别处准备时需要 `uv`、网络和可用的 EGL/Mesa；依赖由锁文件安装。没有安装系统包或修改现有 ROS 环境。

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp

# 新的一次完整实验：采集、训练、物理评估，各轮累积数据
./run.sh --name new-experiment --rounds 2 --samples 192 --epochs 20 --episodes 8

# 从本轮数据与选出的模型继续训练，增加 3 轮新数据
./run.sh --name continued-training --resume-run first-training \
  --rounds 3 --samples 256 --epochs 20 --episodes 8
```

输出目录必须是新名称，旧实验不会覆盖。每次有界运行，保存每轮 checkpoint；本轮没有启动无限后台任务。续训入口已用 `resume-smoke` 实际跑通：保留 384 个旧样本，增加 32 个新样本，1 个训练 epoch、1 次新场景抓取成功。这只是续训链路检查，不用于宣称性能提升。

每轮保留全部失败，不会自动重试到成功后删去失败。模型按固定验证集的漏检/误检和位置 P95 选择，物理测试集不用于训练或选模型；验证集被重复使用，不能把它当最终独立测试。新版本训练/验证/测试 seed 使用分离的百万段。

## 数据和文件

| 文件 | 内容 |
| --- | --- |
| `sim.py`、`assets/scene.xml` | B601 独立 MuJoCo 场景、相机、物理控制、IK |
| `assets/manifest.json` | 76 个复制模型/网格的来源及 SHA256 |
| `vision.py` | 4 通道 RGB-D 小型卷积网络、合成标签、训练与推理 |
| `episode.py` | 分段动作、物理判分、同步录制 |
| `experiment.py` | 有界采集—训练—验证循环，支持续训 |
| `verify.py` | 首轮两个 checkpoint 的同 seed 比较与负对照 |
| `outputs/first-training/summary.json` | 两轮原始汇总与验证数据 |
| `outputs/common-seed-verification/summary.json` | 共同新场景与负对照结果 |
| 每个 episode 的 `observations.npz` | 5 Hz RGB、米制深度、每帧内外参/时间 |
| 每个 episode 的 `trajectory.jsonl` | 20 Hz 关节目标/反馈、速度、模拟力矩、夹爪、接触、图像索引 |
| 每个 episode 的 `result.json` | 场景 seed、估计、计划、成功/失败原因 |

`truth_for_scoring_only` 是单独标出的评估真值，不进入推理或动作规划。第一轮原始抓取试验只有初始相机帧与动作日志；后续同场景验证及续训检查包含完整 5 Hz RGB-D 配对记录。

## 官网 demo 的作用

[Seeed 官方教程](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_grasping_demo/)和[官方源码](https://github.com/Seeed-Projects/reBot-DevArm-Grasp)适合后续接入 RGB-D 相机、YOLO/OBB 抓取姿态和手眼标定。本支线参考其“识别—深度—坐标变换—抓取”的分层方式，自行训练小方块的视觉基线，未运行上游 YOLO、相机 SDK、标定或实机主程序。

官方主程序的 `--dry-run` 文档明确仍初始化并移动机械臂，不能拿它当无运动视觉测试。实机接入应经过现有统一执行器。当前采用固定相机；官网 Eye-in-Hand 标定对应腕部相机，迁移时要按实际安装方式选择变换链。

## 实机迁移

详见 [SIM2REAL.md](SIM2REAL.md)。尤其注意：仿真夹爪是米，现有实机 J7 是电机弧度；模拟力矩/PD 增益不是可直接导入实机的参数。当前数据与 checkpoint 都标记 `hardware_executable=false`。
