# 目标驱动抓取放置V2：新观测参与规划，受扰后减少重试

2026-10-22，310次预先冻结对照与完整记录审计通过，采用为本支线最新纯仿真版本V2。V1模型、原验收和指针副本继续保留。持续迭代目标active，用户取消了额度停止条件。

## 结论与一条运行命令

人选择A/B/C后，机器人通过RGB-D观察、调用学习残差、IK接近与抓取、搬运松手、视觉检查，最多重试两次。新静态30场景完成29/30，受扰30场景完成30/30；同场景V1为28/30、29/30。受扰无需重试完成27次，对V1和同路径只观察的10次；总重试3对20。改进支持采用V2，仍有一条初始漏检失败，不能声称完全可靠或具备实机能力。

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp
./verified_task.sh --goal B
# 可复现的受扰演示；name必须未使用过：
./verified_task.sh --goal C --seed 24000001 --name my-v2-C --disturbed
```

入口读取`LATEST_VERIFIED.json`，先核对独立冻结的`versions/V2/release_manifest.json`以及108个代码/模型/配置/资产及残差训练数据文件，再从V2源码运行。固定EGL，输出到规范根目录`outputs/<name>`。新开发源码不会改变该冻结版本。默认seed=24000000是演示，不计入独立验收成功率；显式`--seed`指定其他初态。当前是单个红橙色方块、固定俯视RGB-D、已知桌面和固定区域，非自由语言/多物体通用机器人。

## 完整配对结果

每块30个不同新初态，A/B/C各10个，五个控制器共享同块初态与真实环境推力。所有初始漏检、重试、失败都保留。

| 场景块/指标 | V1原控制 | 新模型原控制 | 新模型只观察 | V2 HOME优先反馈 | 同新模型原两姿态反馈 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 静态完整成功 | 28/30 | 29/30 | 29/30 | 29/30 | 29/30 |
| 静态无需重试完成 | 28 | 29 | 29 | 29 | 29 |
| 静态累计重试 | 1 | 0 | 0 | 0 | 0 |
| 静态平均仿真秒，含失败 | 29.988 | 30.006 | 31.360 | 31.358 | 35.881 |
| 受扰完整成功 | 29/30 | 30/30 | 30/30 | 30/30 | 30/30 |
| 受扰无需重试完成 | 10 | 10 | 10 | 27 | 27 |
| 受扰累计重试 | 20 | 20 | 20 | 3 | 3 |
| 受扰平均仿真秒，含失败 | 43.201 | 44.134 | 46.364 | 34.418 | 38.979 |

完整任务成功要求控制器视觉确认与独立物理评分均通过。物体整体位于目标区、桌面支撑、已松手且连续1秒速度<0.02m/s，无禁止物理步骤。不是仅判断抬起或最后一个时刻。

V2对V1完整任务每块配对胜1负0，探索性双侧McNemar p=1，不能把+1次解释为稳定的成功率优势。静态29/30的95% Wilson区间83.33%–99.41%；受扰30/30为88.65%–100%。静态A/B/C成功9/10、10/10、10/10，受扰均10/10。

## 新观测与耗时的实际贡献

与相同新模型、相同观察路径但使用初始坐标的`new_observe_only`比较，V2受扰无需重试配对胜17负0，探索性p=0.00001526；总重试少17次、平均少11.945仿真秒。完整成功率同为30/30，故已证明的是这组场景上的重试和耗时收益，而非完整成功率提升。测试涉及多组探索比较，没有作为多重校正的确认性统计试验。

与同模型原两姿态反馈比较，V2完整成功、首次与重试相同；平均少4.523静态秒、4.561受扰秒，支持HOME优先的观察调度。排除双方失败后，29个共同成功静态场景少4.679秒；受扰30个全部共同成功，差值不变。

相对V1，V2静态总体平均多1.370秒、28个共同成功多1.399秒，是新增观察的代价。受扰总体平均少8.783秒、29个共同成功少10.132秒。该权衡与受扰大幅减少重试一起支持采用，不只依据80%门槛。

新视觉、后处理、匹配残差和观察门限相对V1是组合改动，不能把静态+1次单独归因某一项。小策略真正参与动作，但本轮没有关闭残差的消融，仍不能声称残差本身提高完整任务成功率。

## 模型、观察和物理扰动

- 视觉1,765参数，继承统一EGL训练权重。本轮分割张量未再训练，checkpoint封装概率阈值0.60/最小面积25；旧模型默认0.70/25及V1行为保留。完整640复用图像开发、1,280帧旧代码默认一致性和384新空场景/1,152图像逐像素审计见 [图像开发报告](MASK_FRAGMENT_DEVELOPMENT_REPORT.md)。
- 残差1,443参数，与新视觉哈希配对监督重训；每轴最多±8mm。实际输入只有视觉估计特征，真值仅生成训练标签与独立评分。基本运动仍为IK、显式阶段与有限恢复；非强化学习、非端到端策略。
- HOME等待1.2秒，间隔0.2秒的两帧通过质量和稳定性检查后更新；不合格则原两姿态与HOME回退。置信度0.75、覆盖率0.25–1.4、配对位移≤4mm、更新≤80mm。门限不是定位误差保证。
- 在0.5秒起施加Y向0.8N、持续0.06秒，实际冲量0.048N·s。30场景脉冲窗口XY位移均值17.125mm、范围1.680–30.715mm；结束后0.3秒固定快照均值45.690mm、范围0.634–119.556mm。该快照不保证物体已静止，不能叫最终稳定位置。物理位移真值不进入控制。

V2候选59个正任务完整成功；静态29次与受扰31次小策略实际位置动作调用共60次。受扰3条在第一轮观察门限拒绝后重新定位；“无需重试”指整个任务尝试，不等同于实际夹爪抓了几次。

## 所有失败、恢复和回放

- 新静态22000018/A：四组新模型均初始`no_confident_target`，0动作；V2明确失败，没有通过其他观测姿态救回。这是下一轮优先问题。
- V1静态22000009/A初始漏检；22000018/A实际已放入区域，但末端视觉漏检、重新观察仍未确认，故完整任务失败、物理评分成功。这种保守漏报没有记作任务成功。
- V1受扰22100004/B：1次重试后仍无有效视觉目标，完整任务失败。
- V2受扰22100004、22100006、22100028：首轮观察拒绝，重新定位1次后完成，全部保留。V2无抓空后删样本或临时放宽门限。
- 8新空场景22200000–22200007：无成功宣告、0动作行。恢复22300000/C的65mm首次抓偏真实产生空抓；禁止重试失败、允许时1次重试完成。仅一个恢复场景，不能外推为通用恢复能力。
- 所有310次0错误成功声明。历史低置信16400002仍失败、11100009已修复的事后回归同样保留，不计入新验收成绩。

成功、故意抓空失败和恢复的全景回放均由仿真渲染记录：

![V2受扰后自主完成](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-fragment-home-v2-candidate-01/disturbed-new_feedback-000/replay.gif)

![禁止重试时真实抓空失败](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-fragment-home-v2-candidate-01/miss-retries-0/replay.gif)

![重新定位后恢复](/home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp/outputs/goal-fragment-home-v2-candidate-01/miss-retries-2/replay.gif)

V1静态22000018的159帧真实RGB-D回放保存在`static-retained_v1-018/camera-replay.gif`，输入/输出/回放代码哈希在`camera_replay_manifest.json`，不是重建的全景。V2该例无动作，原初始图像为`static-new_feedback-018/observe-0.png`，没有伪造运动回放。

## 审计与冻结证据

310次，59对更新前匹配动作/观察前缀；221,102条20Hz状态、55,385同步5Hz RGB-D、1,251张精确决策图像；560次模型调用、418次真正参与动作的小策略调用、289个观察配对。148,500个连续最终评分状态复算，5,523,568物理步0禁止步骤；最高指令速度0.399985543rad/s。11份训练/验证数据、76资产、精确源代码、模型哈希及全历史种子账本均核对，无训练/开发/本轮测试交集。

审计是已记录决策、动作和物理评分谓词重构，未独立重跑整个物理轨迹；不能推导实机成绩。上文图像负样本实验另有逐像素物理重生成审计，范围不同。

- 正式输出：`outputs/goal-fragment-home-v2-candidate-01`，含`summary.json`、`audit.json`、`frozen_evaluation.json`、`prior_seed_ledger.json`、`source/`及冻结模型副本。
- [测试前约定](FRAGMENT_HOME_EVAL_PROTOCOL.md)，正式启动前副本`outputs/fragment-home-protocol-01`，协议SHA256 `88d1d66bfc4bd0c22fd7f970ffa670dd0205bc1926cc4265c4c34730ed2d0e1d`。
- [70次开发](FRAGMENT_HOME_DEVELOPMENT_REPORT.md)及全部历史失败候选保留。
- 冻结运行目录`versions/V2`，108文件完整复制、源码与正式试验逐文件一致；视觉SHA256 `71acc8fa4d5a5af40b8cb78ea7e0dade1181e505d84d514589ad30c2ee822636`，残差 `da11e50150cf2215d1942cba8b2e11731bf9a513d30a60944194c8888a0da9ec`。
- 冻结版本B区集成演示`outputs/v2-frozen-release-B`和新入口受扰C区演示`outputs/v2-verified-entry-C`均首次完成并通过单任务记录审计；分别608行/152帧、655行/164帧。这些只验证发布入口集成，不增加正式成功率。

## 精确复现及下一轮

```bash
cd /home/bonvoyage/Desktop/1seeed_Robotics/experiments/b601-vision-grasp
# 采用精确保存的正式源码再次审计历史试验：
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python versions/V2/audit_frozen.py --run goal-fragment-home-v2-candidate-01 --auditor multiview_audit.py
# 独立再验收示例。启动时自动排查全历史及训练/验证交集：
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python versions/V2/multiview_runner.py --name my-v2-repeat-evaluation --phase evaluate --episodes 30 --static-seed-start 26000000 --disturbed-seed-start 26100000 --negative-seed-start 26200000 --recovery-seed 26300000 --perception versions/V2/models/perception.pt --policy versions/V2/models/residual.pt --feedback-config versions/V2/feedback_config.json
```

若示例种子后来已被其他运行使用，选择新的未用范围；原正式参数保存在`config.json`，不能把旧种子重演当新验收。精确记录审计使用第一条命令。下一轮先诊断初始未检出时的观察能力，保留空场景无动作与有限恢复要求；每次改进都与最新V2比较，再冻结新至少30场景/块。全程纯仿真，不接ROS/CAN/SDK或实机。
