# 视觉碎片修复与HOME优先：70次完整任务开发

2026-10-21完成并通过记录审计。最新采用仍V1；本轮仅6个不同正场景/块，不能替代至少30场景/块的正式采用试验。

## 固定候选与场景

隔离源码`variants/home-first-01`，输出`outputs/fragment-home-development-01`。模型、图像开发和已知失败回归见 [视觉碎片开发报告](MASK_FRAGMENT_DEVELOPMENT_REPORT.md)。分割概率0.60/面积25，匹配重训残差，观察置信度0.75，HOME等待1.2秒，其他门限及真实推力不变。每模型读取自己的推理配置，V1默认行为和权重身份保留。

新静态21600000–21600005、扰动21700000–21700005，A/B/C各2个；空场景21800000–21800007；恢复21900000。五组每块共享初态/目标/环境推力，不共享后续控制结果。完整历史账本、训练/验证交集在首次物理步前排查，无交集。

## 实际结果

| 场景块/指标 | V1原控制 | 新模型原控制 | 新模型只观察 | HOME优先反馈 | 同新模型原两姿态反馈 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 静态完整成功 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| 静态无需重试完成 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| 静态累计重试 | 0 | 0 | 0 | 0 | 0 |
| 静态平均仿真秒 | 31.019 | 31.015 | 32.415 | 32.411 | 37.058 |
| 受扰完整成功 | 6/6 | 6/6 | 6/6 | 6/6 | 6/6 |
| 受扰无需重试完成 | 2/6 | 2/6 | 2/6 | 3/6 | 3/6 |
| 受扰累计重试 | 4 | 4 | 4 | 3 | 3 |
| 受扰平均仿真秒 | 43.947 | 43.943 | 45.530 | 41.999 | 46.045 |

HOME优先比同模型原两姿态平均少4.647静态秒、4.046受扰秒；全部正任务成功，所以配对共同成功均值相同。对V1静态多1.392秒、受扰少1.948秒。对同观察路径的只观察版，受扰多1个无需重试完成、少1次重试、平均少3.531秒；场景配对胜1负0，探索性双侧McNemar p=1，不能宣称显著改善。每块6/6的95% Wilson区间约61.0%–100%。未证明完整成功率优于V1。

受扰21700002、21700004、21700005两种反馈组均在第一次观察轮拒绝更新、第二轮重新定位后成功，未删去这些重试。21700002明确是位移超过80mm更新上限；最终配置保持冻结，没有临时放宽。真实推力实际冲量0.048N·s，推力结束后0.3秒固定快照的XY位移范围0.400–94.878mm、均值55.288mm；该快照不保证物体已经静止，不能称为所有场景的最终位移。

8个空场景均`no_confident_target`、0动作行。65mm抓偏实际造成首次抓空：禁止重试失败，允许两次上限时1次重试成功。0错误成功声明。另保留事后已知场景16400002的低置信失败，不能把本轮新场景零失败扩展为无漏检。

## 审计

`outputs/fragment-home-development-01/audit.json`通过：70次、12对更新前匹配观察/动作前缀；47,835状态行、11,989同步RGB-D、320精确决策图像、121模型推理调用、87次真正参与动作的小策略调用、85个观察配对。31,000个最终评分状态复算，1,195,003物理步、0禁止步骤；最高指令速度0.399982406rad/s。11份数据、76资产、模型/源码/全历史种子账本一致。审计为记录决策/动作/评分重构，不是独立重新执行整个物理轨迹，也不是硬件验证。

全部开发工程检查通过，唯一未满足项是每块不足30场景，`acceptance_met=false`符合开发运行定义。下一步执行 [正式约定](FRAGMENT_HOME_EVAL_PROTOCOL.md) 的310次新对照，最终采用必须看完整结果及审计。协议在正式运行前另存于`outputs/fragment-home-protocol-01`，SHA256 `88d1d66bfc4bd0c22fd7f970ffa670dd0205bc1926cc4265c4c34730ed2d0e1d`。

## 复现与回放

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/variants/home-first-01
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python multiview_runner.py --name my-fragment-home-development --phase development --episodes 6 --static-seed-start 21600000 --disturbed-seed-start 21700000 --negative-seed-start 21800000 --recovery-seed 21900000 --perception outputs/vision-fragment-egl-01/perception.pt --policy outputs/goal-policy-fragment-egl-01/residual.pt --feedback-config configs/fragment-home-first-development-01.json
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python audit_frozen.py --run my-fragment-home-development --auditor multiview_audit.py
```

原种子已用，只能作为明确标注的精确复现；新验证必须换种子。开发成功回放：`static-new_feedback-000/replay.gif`、`disturbed-new_feedback-000/replay.gif`；抓空失败/恢复回放：`miss-retries-0/replay.gif`、`miss-retries-2/replay.gif`，均在本轮输出目录。只运行纯仿真，持续目标active、额度停止条件取消。
