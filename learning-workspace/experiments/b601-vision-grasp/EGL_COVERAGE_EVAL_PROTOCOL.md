# 统一 EGL 与覆盖率门限候选：预先约定的正式验收

本协议在 `multiview-egl-development-02` 完成58次开发运行后、正式测试开始前保存。开发结果决定是否继续，不作为正式成功率证据。之前未采用的250次多姿态候选完整保留，见 `GOAL_MULTIVIEW_V2_REPORT.md`。

状态更新：本轮250次已完成并通过记录审计，但受扰反馈29/30低于V1的30/30，未采用。见 `GOAL_EGL_COVERAGE_V2_REPORT.md`。以下保留测试前约定。

## 固定改动

- 全流程显式 `MUJOCO_GL=egl`，并保存实际 `mujoco.egl` 后端。新姿态训练/验证图像均已按此后端重新生成，全部1152张新图像通过像素复现审计；历史混合后端数据保留。
- 视觉：`outputs/vision-multiview-egl-01/perception.pt`，SHA256 `45409a61d623c21781f6ce094a5f710ef9beb2619b0160fc58d36dfdcd3301ab`。
- 匹配的小策略：`outputs/goal-policy-multiview-egl-01/residual.pt`，SHA256 `a876203d0c85e77138eb933b4daa05093a64f632683b9c5e7f226b2839968f1a`。此监督学习有界残差真正参与位置控制，基本运动仍使用 IK。
- `configs/feedback-coverage-development-02.json`：投影覆盖率下限由0.75降为0.25；其余配置不变。该门限是开发假设，不是定位误差保证。来源是已明确标为开发用途的31张低覆盖图像、23个不同场景的物理诊断，不能当作新验收场景。
- 基线身份固定为 V1 视觉 `18fbf99571b242980167095976c6ff45fac923b2aeabb18d3ba1da18e4a995bc`、残差 `ddc80bcb40bf317ff0f04f4edf444cf67b905be4d8010cbb4369a9634ab7f2de`。

## 四个控制器与新场景

`retained_v1` 使用V1原控制；`new_no_feedback` 仅换新视觉/残差；`new_observe_only` 做相同观察及质量判断但使用初始坐标；`new_feedback` 使用选中新观测重新规划。后两者首次更新前的真实动作与图像必须一致。

计划 `goal-egl-coverage-v2-candidate-02`：

| 场景块 | 种子 | 每控制器数量 |
| --- | --- | ---: |
| 静态 | 11000000–11000029 | 30 |
| 固定物理推力扰动 | 11100000–11100029 | 30 |
| 空场景 | 11200000–11200007 | 仅反馈版8次 |
| 首次抓取65mm偏移 | 11300000 | 禁止/允许重试各1次 |

共250次；每块A/B/C各10次。推力仍是0.5秒起Y向0.8N、0.06秒，环境真实施力；不修改物体运行中位姿。所有种子启动时与完整历史账本、训练、验证数据检查交叉。若发现冲突，停止并记录，选择另一个未用范围再冻结；不能重用测试。

## 冻结与门槛

开始前保存模型、配置、源码、数据和76项资产哈希、种子账本。测试过程中不换权重、门限、推力、控制代码，不剔除初始漏检或失败。

- 每块30个新场景，目标区平衡；反馈版两块完整成功率均至少80%。
- 静态和扰动完整成功数均不少于保留V1。
- 8空场景无动作；65mm偏移确实执行并造成首次空抓；禁止重试失败、允许最多两次重试后成功。
- 零错误成功声明；记录20Hz动作、5Hz同步RGB-D、准确决策图像及策略调用；所有有动作运行保存最终1秒500步原始评分，独立复算。
- 审计真实小策略动作、匹配观察前缀、物理接触、最高指令速度、源码/资产/数据/模型身份和种子隔离。

`acceptance_met` 仅表示工程门槛；采用仍需审视首次成功、重试与仿真耗时的实际收益和代价。统计按场景配对，报告区间及探索性配对检验；小样本的零失败不能证明普遍可靠。若未通过，继续保留V1。

此轮同时改变训练后端、模型与门限，不能从本轮比较单独归因各项贡献。相同模型/动作的观察与反馈对照可检验新坐标更新的贡献。后续若需归因，再做独立的冻结消融。

## 可复现命令

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python multiview_runner.py --name my-egl-coverage-evaluation --phase evaluate --episodes 30 --static-seed-start 11000000 --disturbed-seed-start 11100000 --negative-seed-start 11200000 --recovery-seed 11300000 --perception outputs/vision-multiview-egl-01/perception.pt --policy outputs/goal-policy-multiview-egl-01/residual.pt --feedback-config configs/feedback-coverage-development-02.json
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python multiview_audit.py --run my-egl-coverage-evaluation
# 开发源码继续变化后，用保存的精确代码审计：
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python audit_frozen.py --run my-egl-coverage-evaluation --auditor multiview_audit.py
```

上述种子完成后已成为历史数据，重新验证必须选择未用种子；精确重演原试验则保留原种子并明确标注复现。仅纯仿真，结果不证明实机迁移能力。额度停止条件已由用户取消。
