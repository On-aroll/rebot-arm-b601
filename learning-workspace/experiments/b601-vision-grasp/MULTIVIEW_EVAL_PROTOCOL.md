# 多姿态视觉候选：完整任务验收约定

本协议在 `multiview-development-01` 的6场景/块开发测试期间保存，正式验收尚未开始。代码中的门槛已在开发运行开始前冻结。图像验证分数只用于选择视觉和残差权重。

状态更新：该250次验收已完成并通过记录审计，静态不退步门槛失败，未采用；[完整报告](GOAL_MULTIVIEW_V2_REPORT.md)。以下保留原先约定。

## 控制器

| 名称 | 视觉/残差 | 抓前控制 |
| --- | --- | --- |
| retained_v1 | 通过验证的V1旧权重 | 原控制，无额外观察动作 |
| new_no_feedback | 新多姿态视觉+匹配残差 | 原控制，无额外观察动作 |
| new_observe_only | 同一新视觉/残差 | 执行观察及质量门限，抓取仍用本次初始坐标 |
| new_feedback | 同一新视觉/残差 | 相同观察及门限，抓取用选中新坐标重新规划 |

首次更新前的观察/反馈运动和帧必须相同，允许更新之后轨迹不同。保留基线的权重身份在审计中按V1固定SHA256验证。

## 独立场景

正式运行至少每块30场景，A/B/C各10。建议静态seeds10000000–10000029，扰动10100000–10100029，空场景10200000–10200007，恢复10300000（禁止/允许重试各一次）。启动时必须与历史完整种子账本及所有训练/验证数据再次检查不交叉；若已经被使用，选择新种子范围并记录，不能重复包装成新测试。

计划共250次物理运行：30×2块×4控制器+8空场景+2恢复。同样的随机初始物体、质量/摩擦/颜色/照明分布，扰动力仍为独立开发校准选定的0.5s起Y向0.8N、0.06s。没有初始有效检测时明确记录无动作，不能悄悄剔除该试次。

## 门槛与采用

- 每块至少30个新场景，A/B/C平衡。
- 反馈完整成功率两块均至少80%；静态、扰动成功数均不得少于保留V1。
- 8空场景无动作；首次65mm偏移确实执行并导致空抓；禁止重试失败、允许最多两次重试时恢复成功。
- 控制器成功声明与独立物理评分无冲突。
- 所有有动作运行记录最终1秒500步评分原始状态；审计复算每步成功谓词、精确决策图像/小策略/动作关系、配对动作前缀、20Hz状态/5Hz图像、模型/数据/源码/资产哈希及种子。
- 记录首次完成、重试、仿真时间、各目标区成功、质量拒绝及全部失败。报告不确定性和配对胜负，探索性p值不替代验收。

`acceptance_met`只是工程门槛。采用还需查看实际改善与成本：如果完整成功相同，只能在重试/首次完成等实测收益存在且明确说明静态观察时间成本后讨论改进；不能用全部相同的成绩宣称新版本更好。未通过继续保留V1。

## 命令

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python multiview_runner.py --name my-multiview-evaluation --phase evaluate --episodes 30 --static-seed-start 10000000 --disturbed-seed-start 10100000 --negative-seed-start 10200000 --recovery-seed 10300000
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python multiview_audit.py --run my-multiview-evaluation
# 开发源码继续变化后，使用输出目录保存的精确代码审计
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python audit_frozen.py --run my-multiview-evaluation --auditor multiview_audit.py
```

纯仿真范围不变；评分真值只用于监督和独立评分，无真实相机或机械臂执行器。保存动作不证明这些动作可直接输入实机，夹爪单位仍需单独标定。
