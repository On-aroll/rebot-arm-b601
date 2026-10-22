# HOME优先完整任务：70次开发与已知漏检回归

隔离开发目录 `variants/home-first-01`，父目录正式候选02保持原冻结源码。代码与初始来源见候选目录 `PARENT_SNAPSHOT.json` 和 `DEVELOPMENT.md`。没有训练新权重。

固定HOME等1.2秒，再拍间隔0.2秒的两帧；通过原视觉门限就直接用新坐标规划，否则继续原两姿态/HOME回退。所有决策使用RGB-D/标定/自身状态，真值仅记录与评分。对照包括旧V1、新模型原控制、同动作但不更新、HOME优先反馈、相同模型原两姿态反馈。模型动作日志区分固定等待与残差真正用于路径。

## 新开发场景

`outputs/home-first-development-02`：静态16400000–16400005，受扰16500000–16500005，各目标区2场景；8空16600000起；恢复16700000。共70次，但每块只有6个新场景，不能当作正式验收。

| 场景 | 控制器 | 完整/首次完成 | 累计重试 | 平均仿真秒 |
| --- | --- | ---: | ---: | ---: |
| 静态 | V1 | 5/6、5/6 | 0 | 25.555 |
| 静态 | 新原控制 | 5/6、5/6 | 0 | 25.555 |
| 静态 | HOME只观察 | 5/6、5/6 | 0 | 26.721 |
| 静态 | HOME反馈 | 5/6、5/6 | 0 | 26.720 |
| 静态 | 原两姿态反馈 | 5/6、5/6 | 0 | 30.631 |
| 受扰 | V1 | 5/6、2/6 | 3 | 35.814 |
| 受扰 | 新原控制 | 6/6、2/6 | 4 | 44.262 |
| 受扰 | HOME只观察 | 6/6、2/6 | 4 | 47.568 |
| 受扰 | HOME反馈 | 6/6、6/6 | 0 | 33.238 |
| 受扰 | 原两姿态反馈 | 6/6、6/6 | 0 | 37.059 |

静态16400002是所有控制器的同一初始漏检，均0秒无动作。包括这次失败的平均耗时差为3.911秒；在5个共同成功场景上，HOME反馈32.064秒、两姿态36.757秒，节省4.694秒。受扰两反馈完整/首次成绩相同，HOME少3.821仿真秒。相对相同HOME动作但不更新，反馈受扰首次多4次、少4次重试，样本不足以作普遍结论。V1受扰16500005初始漏检，不能用包含它的总体均值比较执行速度。并行运行影响墙钟时间，以上只讨论仿真时间。

8空场景无动作。65mm首次偏移确实空抓，禁重试失败，允许时1次重试完成。全部70次记录错误成功为0。

## 审计

`outputs/home-first-development-02/audit.json`通过：70次、11对相同首次更新前动作/图像，28,000最终评分状态复算，41,561动作、10,414对RGB-D、247决策帧、109模型调用，其中73次小策略用于动作，55质量配对；1,038,325物理步禁止接触0。最高指令速度0.3999773838rad/s，11数据文件、76资产及种子隔离通过。HOME等待时长、恒定关节指令、视觉合格短路径及回退都已按记录复核。

并行父试验仅引用不可变config/frozen_evaluation两项来预留全部69个种子，不冻结变化中的summary/results；已在启动前与审计时核对。第一轮 `home-first-development-01` 因旧种子提取器未解析嵌套test_seeds而缺少第二项证据，人工中断；4完整+1部分运行、全部计划种子保留，详见该目录 `INTERRUPTION.md`，不计入本轮70次或性能结论。

源码已在审计后继续增加单任务CLI入口。以后使用精确保存源码审计：

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/variants/home-first-01
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python audit_frozen.py --run home-first-development-02 --auditor multiview_audit.py
```

## 已知漏检回归：仍失败，不进入正式采用

随后明确将父试验11100009/A作为事后开发回归，按相同推力执行HOME优先任务：`outputs/home-first-regression-11100009`。结果仍是 `no_confident_target`、1次重试后失败，控制器和物理评分都为false。说明HOME优先确实能缩短已见成功案例，但没有修复已知视觉退步。**不直接开启另一轮正式采用测试，不替换V1；下一步先修复视觉碎片/漏检，再回归和新场景验收。**

单任务可复现入口已实际运行（原种子复现应换新输出名；它是事后开发，不是新独立测试）：

```bash
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 ./task.sh demo --name my-home-first-regression --policy-run goal-policy-multiview-egl-01 --perception outputs/vision-multiview-egl-01/perception.pt --goal A --seed 11100009 --observation-mode feedback --feedback-config configs/home-first-development-01.json --force-pulse outputs/force-development-01/selected_pulse.json --scene-role posthoc_development
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python demo_audit.py --run my-home-first-regression
```

单任务保存模型副本、代码/资产/参数/模型哈希、准确决策、20Hz/5Hz和整秒评分及回放；审计范围是保存记录重建。历史6个已知诊断案例也已通过审计，见 `outputs/home-first-known-development-01`，不能计入上述新6场景结果。
