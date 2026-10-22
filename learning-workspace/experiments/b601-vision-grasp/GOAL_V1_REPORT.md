# 目标驱动抓取放置 V1：视觉修正与独立验收

## 本轮结论

新视觉模型搭配小策略在 **30 个新的初始场景中完成 30/30**，A/B/C 各 10 次；相同场景的旧视觉几何基线为 **28/30**。仅更换视觉、不启用小策略同样为 **30/30**。

本轮可归因的改进来自视觉；**没有证明位置修正策略提高完整任务成功率**。旧基线失败的两个场景 seed=9300002、9300010 均为初始漏检，无动作。新版本修复了这两例，但只有两例配对差异，精确 McNemar 检验 p=0.5，不能据此声称稳定的统计优势。

30/30 的 Wilson 95% 区间为 **88.65%–100%**，仍是当前有限仿真分布上的估计。所有输出 `hardware_executable=false`，未连接实机。

## 运行与复现

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp

# 已训练版本：指定目标 B。name 必须是未使用过的目录名。
./task.sh demo --name my-v1-B --policy-run goal-policy-v2 --perception outputs/vision-iteration-03/perception.pt --goal B --seed 8000100

# 复现干净标签视觉训练；直接从原视觉模型出发。
.venv/bin/python vision_runner.py --name my-vision --samples 1024 --epochs 20 --initial outputs/first-training/round-02/perception.pt

# 为上述视觉模型重训位置修正。
./task.sh train --name my-policy --samples 1024 --perception outputs/my-vision/perception.pt --train-seed 5400000 --validation-seed 5500000

# 重放本轮固定验收。若据这些场景调参，须另开新测试种子。
./task.sh evaluate --name my-v1-eval --policy-run my-policy --perception outputs/my-vision/perception.pt --episodes 30 --seed-start 9300000 --negative-seed-start 9400000 --recovery-seed 9500000 --ablate-perception
.venv/bin/python task_audit.py --run my-v1-eval
```

验收前冻结了权重、源代码、配置及 7 个独立数据文件的哈希。复现历史精确行为时，以 `outputs/goal-v1-evaluation/source` 为代码依据；后续代码改动应使用新的运行目录和新验收种子。

## 完整任务配对结果

| 指标 | 旧视觉 + 几何 | 新视觉 + 几何 | 新视觉 + 小策略 |
| --- | ---: | ---: | ---: |
| 相同初始场景 | 30 | 30 | 30 |
| 完整任务成功 | 28/30 | 30/30 | 30/30 |
| A / B / C 成功 | 10 / 9 / 9 | 10 / 10 / 10 | 10 / 10 / 10 |
| 普通任务重试总数 | 0 | 0 | 0 |
| 视觉成功但物理失败 | 0 | 0 | 0 |
| 平均仿真时长，含失败及评分 | 28.898s | 30.881s | 30.885s |

新版本均值包含更多实际执行的任务；旧基线的两个漏检场景无运动，因此不能把上述总体均值直接当作效率对比。

种子为 9300000–9300029，按 A/B/C 循环；这批场景没有参与训练、权重选择或观察位置开发。当前相机固定、桌面高度已知，任务只有一个红橙色立方体。随机范围沿用 V0，包括初始位置、尺寸、yaw、颜色、桌面颜色、光照、质量与摩擦。

成功要求控制器视觉验证与独立物理评分同时通过：物体整体位于区域边界内、桌面支撑、连续一秒速度小于 0.02m/s、手指已释放。接触真值现仅记录和独立评分，不参与动作、抓空判断或重试决策。碰撞评分也不再向控制流程发送接触真值。

## 训练过程与发现的标签问题

保留了两轮未采用的候选：

| 候选 | 训练 | 验证漏检 / 219 | 空场景误检 / 37 | 采用情况 |
| --- | --- | ---: | ---: | --- |
| 原模型 | 旧权重 | 16 | 0 | 保留基线 |
| vision-iteration-01 | 从零 40 epoch，选 epoch 30 | 26 | 0 | 未采用 |
| vision-iteration-02 | 从旧权重训练 20 epoch | 16 | 0 | 最佳为初始 epoch 0，未改善 |
| vision-iteration-03 | 干净标签，从旧权重训练 20 epoch | 11 | 0 | 验证选 epoch 20，再进行新任务测试 |

问题是多重采样的分割 ID 颜色在区域标记边缘产生别名：部分边缘被解码为目标 geom。seed=5100001 的 149 个标签像素中有 71 个实际位于桌面/标记处，深度高度约 0.1000–0.1005m。

修正只作用于 `observe(labels=True)`：用训练真值体积过滤明显不在方块表面的标签。1024 个训练样本删除 73,096 个错误标签像素，256 个验证样本删除 18,238 个；184 个空场景的伪标签全部消失。逐像素核对前后 **RGB 和深度完全相同**；`labels=False` 不返回标签、不读取该训练过滤分支。

最终模型直接从原模型与重新生成的干净监督训练，未继承两轮受污染候选的权重。

新增视觉训练 seeds=5100000–5101023，含 877 个有目标、147 个空场景；验证 seeds=5200000–5200255，含 219 个有目标、37 个空场景。旧模型训练/选模数据同样列入测试交集审计。新验证条件定位误差均值 **2.027 → 1.505mm**，P95 **5.175 → 4.079mm**；这些误差只包含通过检测与工作区检查的样本。

## 小策略实际参与的动作

仍为 **1,443 参数 MLP**，输入视觉估计的 8 个特征，输出 XYZ 每轴最多 ±8mm 的抓前位置修正。修正实际进入 TCP 和 IK；姿态、运动阶段、插值及恢复规则仍是显式控制。训练为监督学习，没有训练强化学习或完整端到端动作策略。

新视觉模型对应 `goal-policy-v2`：

- 训练 1024 场景，989 个可用样本；验证 192 场景，185 个可用样本。
- seeds：训练 5400000–5401023，验证 5500000–5500191。
- 按验证均值选择 epoch 290/300。
- 条件三维误差均值 **1.663 → 1.245mm**，中位数 **1.130 → 0.902mm**，P95 **4.285 → 3.439mm**。
- 完整验收中核对了 33 次实际模型修正，均在每轴 ±8mm 内。

视觉网络 1,765 参数，两部分共 3,208 参数，CPU 运行。验证误差下降不能替代完整任务对照，当前消融已显示小策略尚无任务成功率增益。

## 失败恢复与回放

8 个独立空场景 seeds=9400000–9400007：全部无动作、无成功宣告。

seed=9500000、目标 C 的配对试验：第一次抓取位置故意偏移 65mm。禁止重试时抓空并失败；允许最多两次重试时，重新观察后重试 **1 次**，完成放置。这仍只是一个恢复场景，不能代表通用失败恢复能力。

正常目标 C 成功，seed=9300002（旧基线在此漏检）：

![V1 完成 C 区任务](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v1-evaluation/learned-002/replay.gif)

抓偏且禁止重试，明确失败：

![抓偏后失败](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v1-evaluation/miss-retries-0/replay.gif)

同场景允许重试，重新观察后完成：

![重新观察后恢复](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v1-evaluation/miss-retries-2/replay.gif)

## 独立审计与原始证据

100 个 episode：三组共 90 次普通任务、8 个空场景、2 次故障配对。

- 55,877 条 20Hz 动作/状态记录，14,004 帧配对 5Hz RGB-D。
- 最大导出关节目标速度 0.399983rad/s，关节限位、夹爪范围、时间配对与有限重试均通过。
- 0 个错误成功宣告，无审计定义的禁止接触。
- 视觉与策略的训练、验证数据均与测试种子无交集；三组初态与目标完全相同。
- 运行前核对原始 76 个模型/网格哈希，无吸附、weld、物体绑定或运行中物体位姿写入。

视觉 SHA256：`18fbf99571b242980167095976c6ff45fac923b2aeabb18d3ba1da18e4a995bc`。

策略 SHA256：`ddc80bcb40bf317ff0f04f4edf444cf67b905be4d8010cbb4369a9634ab7f2de`。

- [验收汇总](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v1-evaluation/summary.json)
- [测试前冻结](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v1-evaluation/frozen_evaluation.json)
- [审计结果](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v1-evaluation/audit.json)
- [视觉模型与训练配置](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/vision-iteration-03/summary.json)
- [标签完整语料核对](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/vision-iteration-03/label-corpus-audit.json)
- [推理路径核对](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/vision-iteration-03/label-path-audit.json)
- [小策略训练结果](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-policy-v2/summary.json)

## 后续任务

V1 作为最后验证通过的仿真版本保留。下一轮优先做抓前主动观察、遮挡下的质量判断和位置更新，并用物理扰动检验反馈是否减少抓空/重试。初步观察位开发与待验收方案见 [NEXT_ITERATION.md](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/NEXT_ITERATION.md)。

当前没有验证任意物体、杂乱场景、动态跟踪、实机视觉误差或时延。接实机仍需相机/桌面/TCP 标定、夹爪映射及独立执行保护；不得将仿真关节轨迹直接重放到机器人。
