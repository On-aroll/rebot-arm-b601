# B601 目标驱动抓取放置 V0：验收结果

## 结论

首版纯仿真闭环已完成：指定 A/B/C 区域，机械臂从随机初始位置识别方块，完成抓取、搬运、松手、后退、视觉检查；抓空后可返回重新观察并有限重试。

正式共同场景中，几何基线和学习位置修正版本均 **28/30 成功（93.3%）**，达到首版点估计至少 80% 的目标。两组在完全相同的 28 个场景成功，另两个场景均因初始视觉漏检未执行动作。**本轮未证明学习位置修正提高完整任务成功率。**

这是单个红橙色方块、固定相机和已知桌面下的有限任务。没有接入真实相机、ROS、CAN 或机械臂服务；没有证明实机能力或通用自主操作。

## 回放

指令：把方块放到 C 区。正式场景 seed=9000002。

![自主抓取并放到 C 区](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v0-evaluation/learned-002/replay.gif)

第一次抓取位置被故意偏移 65mm：检测抓空，返回重新观察，第二次抓取并放到 C 区。独立场景 seed=9200000。

![抓空后重新观察并恢复](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v0-evaluation/miss-retries-2/replay.gif)

## 完整任务共同场景对照

| 指标 | 视觉几何基线 | 学习位置修正 |
| --- | ---: | ---: |
| 独立初始场景数 | 30 | 相同 30 个 |
| 完整任务成功 | 28/30 | 28/30 |
| A 区 | 10/10 | 10/10 |
| B 区 | 8/10 | 8/10 |
| C 区 | 10/10 | 10/10 |
| 视觉宣告成功、物理评分失败 | 0 | 0 |
| 普通任务中的重试次数 | 0 | 0 |
| 平均任务仿真时长，包含一秒独立评分 | 29.040s | 29.042s |
| 成功率 Wilson 95% 区间 | 78.7%–98.2% | 78.7%–98.2% |

30 个场景的点估计不能当作长期成功率保证，也不能证明成功率置信下界大于 80%。A/B/C 各 10 次；两个失败 seed=9000004、9000007，均为 `no_confident_target`，没有计划或动作记录。这不表示 B 区本身更难：这两次失败发生于选择目标区后、执行动作前的初始识别阶段。

成功同时要求：

- 控制器在返回 HOME 后通过 RGB-D 检查物体位于目标区中心附近、处于桌面高度，且夹爪已打开。
- 独立物理评分连续一秒确认物体整体在区域边界内、由桌面支撑、线速度低于 0.02m/s，且没有手指接触。

目标真值与双指接触没有进入视觉位置修正、规划、夹爪检查或重试决策。它们用于独立评分、记录，以及训练阶段的监督标签；碰撞保护也使用模拟接触信息。

## 小策略训练与它实际控制的内容

预抓取位置修正 MLP 共 **1,443 参数**，输入视觉估计 XYZ、像素中心、像素面积、置信度、深度。输出 XYZ 各轴最多 ±8mm 的修正，实际改变抓取 TCP 和 IK 目标。抓取姿态、任务阶段、路径插值和重试规则仍是显式控制。

训练采用仿真监督学习，标签为目标顶面真值减视觉估计。不是强化学习、ACT，也不是端到端学习完整动作。超出初始训练工作区时退回几何估计；放置验收不使用该修正头外推。

| 项目 | 数量 / 结果 |
| --- | --- |
| 训练场景 | 768；719 个可用视觉样本，49 个漏检或工作区拒绝 |
| 验证场景 | 192；180 个可用视觉样本，12 个漏检或拒绝 |
| 选中权重 | 按验证均值选第 294/300 epoch |
| 验证位置误差均值 | 2.036 → 1.630mm |
| 验证位置误差中位数 | 1.475 → 0.967mm |
| 验证位置误差 P95 | 6.275 → 6.071mm |

上述定位误差只统计通过视觉与工作区检查的验证样本；不包含漏检，不代表任务成功率或实机定位精度。完整任务对照的现有瓶颈是初始感知漏检，位置修正没有解决它。

训练 seeds 4100000–4100767，验证 4200000–4200191，共同测试 9000000–9000029。代码、场景和模型哈希在测试前冻结；测试没有用于选择权重。开发场景与初期模型记录单独保留。

## 负对照与恢复

| 检查 | 结果 |
| --- | --- |
| 8 个无目标场景，seed 9100000–9100007 | 全部无动作、无成功宣告 |
| 第一次抓取偏移 65mm，禁止重试 | 抓空并失败 |
| 同 seed、同偏移，允许最多两次重试 | 第一次抓空，重试一次后成功 |

配对恢复试验使用 seed=9200000。第一次提起后夹爪行程约 2.7×10⁻⁹m，符合空闭合；第二次约 0.0160m，随后完成放置。控制器使用关节行程阈值作疑似抓住判断。一个恢复场景不能证明滑移、遮挡、碰撞或所有空抓都可恢复。

## 数据与审计

审计覆盖 70 个 episode（两组共 60 个正式任务、8 个空场景、2 个恢复配对）：

- **36,294 条 20Hz 动作与状态记录，9,099 帧配对 5Hz RGB-D**。
- 图像索引与相机/动作时间逐条对应；关节目标有限、在限位内。
- 导出指令最大关节速度 **0.399981rad/s**；无检查中定义的禁止接触。
- 31 次学习位置修正均在每轴 ±8mm 内，并核对实际修正后的目标。
- 训练、验证与测试 seed 无重叠；基线与学习版本的初始场景及目标完全一致。
- 原始 76 个模型/网格文件哈希仍与资产清单一致。
- 没有吸附、weld、绑定物体或运行中的物体位姿写入。

策略 SHA256：`29e9c16c45224f9d3f791e32912a7237f087c08999c38dd7ec492a2824966ea6`。

原始证据：

- [正式汇总](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v0-evaluation/summary.json)
- [测试前冻结信息](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v0-evaluation/frozen_evaluation.json)
- [源码与感知模型来源](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v0-evaluation/provenance.json)
- [审计结果](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v0-evaluation/audit.json)
- [训练汇总](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-policy-v1/summary.json)
- [训练好的小策略](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-policy-v1/residual.pt)
- [恢复成功的完整记录](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v0-evaluation/miss-retries-2/result.json)
- [恢复被禁止时的失败记录](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-v0-evaluation/miss-retries-0/result.json)

环境为独立 CPU Python/Torch 2.7.1+cpu、MuJoCo 3.16.0；其余版本见测试目录 `environment.json` 和项目 `requirements.lock`。

## 复现与下一步

运行指令见 [GOAL_TASK.md](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/GOAL_TASK.md)。例如指定 B 区：

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp
./task.sh demo --name my-goal-B --policy-run goal-policy-v1 --goal B --seed 8000100
```

下一步优先解决视觉漏检、扩展物体/背景与失败数据，然后在新的保留场景上检验增益。现有配对图像、状态与动作可以用于后续模仿学习，必须排除评分真值字段并按 episode 划分数据。

实机接入仍须完成相机、桌面、TCP 和夹爪映射标定，并接入现有保护执行层。当前所有输出 `hardware_executable=false`，没有实机执行器。
