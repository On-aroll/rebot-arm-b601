# 视觉碎片修复与HOME优先候选：预先验收约定

2026-10-21，正式试验启动前保存。候选未采用，最新验证版本V1。本文指定候选与比较方式，不预告成功结果。

## 启动前提

先完成并审计`fragment-home-development-01`的70次开发任务。除小样本不满足“30场景”外，现有工程检查必须通过：两块候选成功率≥80%、成功数不少于V1及同模型原两姿态、静态平均仿真耗时不大于原两姿态、空场景无动作、真实抓空且重试恢复、零错误成功及完整评分记录。若失败，保留失败，不启动本约定的正式采用试验；修改候选后须另存约定。

已知11100009及六条机制回归通过记录审计；低置信16400002仍失败，完整保留。它们及全部历史试验现在仅是开发证据，不计入本轮成功率。384新空场景/1,152图像压力试验仅为开发图像证据。

## 固定候选

必须从隔离工作目录`variants/home-first-01`运行，不能用根目录旧代码加载新后处理checkpoint。

- 视觉`outputs/vision-fragment-egl-01/perception.pt`，SHA256 `71acc8fa4d5a5af40b8cb78ea7e0dade1181e505d84d514589ad30c2ee822636`。父EGL网络张量未变，checkpoint增加概率0.60/最小面积25推理配置；旧checkpoint仍默认0.70/25，旧代码默认推理逐帧一致性已检验。
- 小策略`outputs/goal-policy-fragment-egl-01/residual.pt`，SHA256 `da11e50150cf2215d1942cba8b2e11731bf9a513d30a60944194c8888a0da9ec`。与上述视觉哈希配对，监督训练1,443参数、每轴±8mm残差，实际参与抓前位置修正。基本运动仍为IK及显式阶段，非强化学习或端到端控制。
- 配置`configs/fragment-home-first-development-01.json`，SHA256 `a2358b3ef4a9c6c88732598f69ec31785618b6a43202ee836428beb268600936`。HOME等待1.2秒、配对间隔0.2秒、置信度≥0.75、覆盖率0.25–1.4、配对位移≤4mm、更新≤80mm；不合格则原两姿态与HOME回退。完整640复用验证图像选门限，不把门限视为定位误差上界。
- 保留V1视觉SHA256 `18fbf99571b242980167095976c6ff45fac923b2aeabb18d3ba1da18e4a995bc`，策略 `ddc80bcb40bf317ff0f04f4edf444cf67b905be4d8010cbb4369a9634ab7f2de`。
- 全过程`MUJOCO_GL=egl`，保存实际GL上下文、精确源码、模型副本、数据及76资产哈希。正式运行与审计终止前所有运行源码和上游历史账本不变。

## 五个控制器与场景

1. `retained_v1`：V1原控制。
2. `new_no_feedback`：新视觉与新头，原控制。
3. `new_observe_only`：HOME优先及回退观察，使用初始位置规划。
4. `new_feedback`：同样观察路径，使用新位置与小策略更新规划。
5. `retained_two_view`：同新模型、同门限，HOME优先等待为0，保留原两姿态及HOME回退。名称指比较控制器，非V1权重。

观察版与反馈版首次坐标更新前的真实动作、图像及扰动应完全一致。相同模型下第五组比较HOME优先调度的影响；新视觉/新残差/门限相对V1为组合比较，不能单独归因。

计划运行名`goal-fragment-home-v2-candidate-01`：

| 场景块 | 种子 | 每控制器次数 |
| --- | --- | ---: |
| 静态 | 22000000–22000029 | 30 |
| 固定物理推力 | 22100000–22100029 | 30 |
| 空场景 | 22200000–22200007 | 仅候选反馈8次 |
| 首次抓取65mm偏移 | 22300000 | 禁止/允许重试各1次 |

共310次，正场景每块A/B/C各10个。最大两次重试。环境在0.5秒起真实施加Y向0.8N、持续0.06秒，不修改运行中物体位姿。实际物体位移只作独立评分，控制仅使用RGB-D、标定与自身状态。全历史账本及训练/验证NPZ在首次物理步前排查交集，冲突则记录并另选种子重新冻结。

## 采用门槛与结果报告

- 每块30个新场景，反馈完整成功率≥80%；两块完整成功数均不少于V1和原两姿态比较组。
- 静态平均仿真任务耗时不大于同模型原两姿态。另报告共同成功场景的配对耗时，防止快速失败压低均值。
- 8空场景零动作；65mm偏移确实产生首次抓空，禁止重试失败，允许重试后完成。
- 零错误成功；20Hz动作、5Hz配对RGB-D、精确决策图像、小策略每次推理与真正动作调用、最终1秒500步原始评分可复算。
- 审计基线身份、所有数据/模型/代码/资产/种子、匹配观察前缀、物理禁止接触及指令速度≤0.4rad/s。审计是已记录决策/评分重构，不声明独立重跑整个物理轨迹。

门槛通过后还要查看反馈相对V1、只观察、原两姿态的首次成功、总重试及耗时实际收益，才能决定采用。报告所有失败、控制与独立评分的分歧、场景级配对胜负、95% Wilson区间及探索性McNemar检验，不将多组探索检验解释为确认性显著性；小样本零失败不能证明普遍可靠。未通过则保留V1，完整保留本轮证据。

## 复现命令

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/variants/home-first-01
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python multiview_runner.py --name my-fragment-home-evaluation --phase evaluate --episodes 30 --static-seed-start 22000000 --disturbed-seed-start 22100000 --negative-seed-start 22200000 --recovery-seed 22300000 --perception outputs/vision-fragment-egl-01/perception.pt --policy outputs/goal-policy-fragment-egl-01/residual.pt --feedback-config configs/fragment-home-first-development-01.json
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python multiview_audit.py --run my-fragment-home-evaluation
# 后续源码发生改变时，以保存的精确源码审计：
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python audit_frozen.py --run my-fragment-home-evaluation --auditor multiview_audit.py
```

上述种子一旦用于本轮就成为历史数据；另一次新验收必须换未用种子。仅纯仿真，不接硬件，不声明实机能力。持续任务active，额度停止条件已由用户取消。
