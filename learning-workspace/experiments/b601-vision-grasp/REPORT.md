# 首批仿真视觉抓取记录

日期：2026-10-20，Asia/Shanghai。实验目录独立于现有运行中的 ROS/CAN 项目，原模型 76 个来源文件与复制时哈希仍全部一致。本轮未连接真实相机、调用硬件接口或重启已有服务。

## 已经完成

- 在 B601 模型旁加入固定俯视 RGB-D 相机，并用独立 MuJoCo 场景运行接触物理。
- CPU 上自动生成监督图像，训练 4 通道小型卷积网络识别单个红/橙色方块。
- 用视觉预测的目标位置求 IK，执行接近、下降、闭爪、抬升与保持。
- 自动记录成功与失败，保存 checkpoint、数据划分、模型哈希、动作和图像。
- 完成两轮训练、共同新场景比较、空场景/错误目标负对照、追加数据续训链路。

## 抓取重放

第 2 轮模型，额外共同场景 seed=40000。最低抬升 **118.3 mm**，最后 1 秒双侧接触占比 **100%**。

![视觉定位后接触抓取和抬升](outputs/common-seed-verification/round-2-seed-40000/replay.gif)

把同一场景的视觉目标沿 Y 故意偏移 65 mm，仍在工作区内。机械臂抓空，物体没有被带起；最低抬升约 **−0.04 mm**，双侧接触 **0%**，判为失败。

![错误视觉位置导致抓空](outputs/common-seed-verification/wrong-within-workspace/replay.gif)

## 训练和验证

| 指标 | 随机初始化 | 第 1 轮 | 第 2 轮 |
| --- | ---: | ---: | ---: |
| 累计训练图像 | 0 | 192 | 384 |
| 固定验证集目标场景 | 48 | 48 | 48 |
| 漏检 | 48 | 5 | 4 |
| 固定验证集空场景 | 8 | 8 | 8 |
| 误检 | 0 | 1 | 1 |
| 检出目标 XY 误差中位数 | 无检出 | 0.76 mm | 1.08 mm |
| 检出目标 XY 误差 P95 | 无检出 | 6.26 mm | 5.04 mm |
| 每轮不同的新物理场景 | 未执行 | 7/8 | 8/8 |
| 额外相同的 8 个新物理场景 | 未执行 | 8/8 | 8/8 |

固定验证集在训练 seed 范围以外，但被重复用于模型选择。XY 误差只统计检出的目标，不含漏检，不能当作所有场景的精度。两个模型在共同场景均成功，现有样本不足以证明抓取性能显著提高。

第 1 轮独立物理场景 seed=30006 因没有足够置信度而不执行，失败保存在原始结果中。另测 seed=50000–50007 的 8 个空场景均无目标且没有动作记录；这不能抹去验证集里已有的那次误检。工作区外的目标也已检查并拒绝，工作区内错误目标能触发真实抓空失败。

成功标准：连续 1 秒最低抬升 >60 mm，且双侧手指接触占比 >90%；过程若产生禁止碰撞或非有限状态即失败。没有使用 weld、物体绑定或运行时修改物体位姿。

## 录制与复现

共同场景及工作区内偏移试验共保存 **4,900 条 20 Hz 动作/状态记录**、**1,232 帧 5 Hz RGB-D**。图像索引、相机时间和动作时间逐条对应已核对；导出关节指令最大速度约 **0.39998 rad/s**，检查中没有禁止接触。模型文件校验 76 项通过，相机反投影代数往返误差约 8×10⁻¹⁷ m；后者只验证坐标数学，不能当作相机精度。

续训检查 `resume-smoke`：从原 384 个样本及第 2 轮权重继续，加 32 张新图，训练 1 epoch，1 次新物理场景成功。按原验证集比较后保留了原第 2 轮权重，避免因为新轮已经训练就自动采用较差模型。这是功能验证，不是新的成功率估计。

原始资料：

- [首轮完整汇总](outputs/first-training/summary.json)
- [共同场景与负对照汇总](outputs/common-seed-verification/summary.json)
- [成功 episode 结果](outputs/common-seed-verification/round-2-seed-40000/result.json)
- [成功 episode 20 Hz 日志](outputs/common-seed-verification/round-2-seed-40000/trajectory.jsonl)
- [成功 episode 5 Hz RGB-D](outputs/common-seed-verification/round-2-seed-40000/observations.npz)
- [工作区内错误位置结果](outputs/common-seed-verification/wrong-within-workspace/result.json)
- [续训链路记录](outputs/resume-smoke/summary.json)
- [最终源码 SHA256](outputs/final-source-sha256.json)

第 2 轮 checkpoint SHA256：`9d7d8595d62b945cdd7fb4d35fbef4a2266cb53390e8d1e934814ae17b0c5724`。各次运行有当时的 Python 源码快照。首轮训练与控制验证后补齐了 5 Hz 图像录制；共同场景验证及续训检查验证了这版录制逻辑。初始两次几何开发试验使用真值掩码，只存于 `oracle-*`，不计入任何视觉成绩。

## 目前还不能宣称的能力

视觉网络只在仿真单方块任务内训练。没有学习动作策略，没有测试任意物体、真实光照/相机、反光透明物体或真实控制延迟；相机位姿与镜头参数固定。质量和摩擦随机化也没有经过实机辨识。

后续可从已配对的 RGB-D、关节状态与目标指令训练模仿策略。当前阶段应先扩大场景、收集失败、完成真实相机只读对比。具体接口见 [SIM2REAL.md](SIM2REAL.md)。

## 官方 demo 与资料

[Seeed 官方视觉抓取教程](https://wiki.seeedstudio.com/cn/rebot_arm_b601_rs_grasping_demo/)提供 RGB-D 相机、YOLO/OBB、可选 GraspNet 和标定接入路径；[官方仓库](https://github.com/Seeed-Projects/reBot-DevArm-Grasp)可以作为真实视觉采集与几何估计的参考。该 demo 不是自动强化学习系统，本实验没有运行其硬件主程序。官网说明主程序 `--dry-run` 仍可能移动机械臂，单纯看视觉应使用其独立视觉脚本。

本实验实现了独立的神经网络分割、坐标反投影和受限抓取控制。模型/网格来自本机现有 B601 工程快照，完整来源与文件哈希见 [资产清单](assets/manifest.json)；没有声称复制文件对应干净的上游提交或重新授予模型许可。

参考技术依据：[MuJoCo Python API](https://mujoco.readthedocs.io/en/stable/python.html)、[Domain Randomization 论文](https://arxiv.org/abs/1703.06907)。
