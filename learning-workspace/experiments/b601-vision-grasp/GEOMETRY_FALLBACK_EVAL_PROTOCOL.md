# 几何主路径与V2解码回退：正式对照约定

2026-10-22，正式试验前约定。须先完成70次新开发、全部记录审计及结果复核，通过后保存本文副本/哈希及开发证据哈希再启动。试验后不修改该轮候选、门限或挑选场景。

## 冻结候选和基线

- 候选视觉 `outputs/vision-geometry050-fallback-01/perception.pt`：SHA256 `e911a373325072eb9525031fa57f4270a6f8b6f853a5c6d370eb5ffd5d17bd37`，分割权重与V2相同。先几何工作区裁剪/0.50概率/25像素，只有无预测时回退到原V2解码0.60/25，并检查原工作区。每次预测记录`decoder_path`。
- 匹配1,443参数监督残差 `outputs/goal-policy-geometry050-fallback-01/residual.pt`：`d2de74cdbe37694cae299abf4b3b90e4dee117bba0a8c74ef658a7f4be5f80b6`，每轴最多8mm；固定300轮开发、选249轮。无在线调参，无测试监督。
- 基线为已采用的 **V2模型及feedback控制**，视觉SHA `71acc8fa4d5a5af40b8cb78ea7e0dade1181e505d84d514589ad30c2ee822636`，策略SHA `da11e50150cf2215d1942cba8b2e11731bf9a513d30a60944194c8888a0da9ec`。核对独立`versions/V2`清单SHA `1a1c4f3cb4461bafcbd2b6f7dd12bbe066219591cf1309939e8feb1b11062a51`及108文件；共享运动/控制源码须与冻结V2相同。不是用旧V1充当最新基线。
- 配置 `variants/initial-observation-01/configs/v2-feedback.json`：HOME等待1.2秒、成对观察间隔0.2秒、置信度0.75、覆盖率0.25–1.4、帧间位移≤4mm、最大更新80mm、原两姿态及HOME回退；最多两次重试。原两姿态组仅把HOME优先等待设为0。
- EGL渲染、20Hz动作和5Hz配对RGB-D、准确决策图、全部物理步禁止接触记录及最终500步原始评分，均固定源代码。真值只用于训练监督及独立评分，不能供视觉/动作决策使用。

## 场景与对照

计划静态种子28000000–28000029、受扰28100000–28100029；每块30场景，A/B/C各10。8个空桌28200000–28200007，强制抓空恢复28300000。启动前完整历史种子账本和所有视觉/残差训练/验证数据交集检查，若发生重叠则试验必须在任何物理执行前停止并重新约定。

每块五组：V2完整控制、候选原控制（无再观察）、候选只观察不更新、候选反馈更新、候选原两姿态反馈；共300任务，加8空桌和2次恢复，共310次。所有组相同初态、目标及扰动；只观察/反馈在首次更新前必须精确核对动作前缀一致。这些组不能单独证明残差的完整任务收益，因为未设置零残差的同模型组。

扰动使用已开发并冻结的Y向0.8N、0.5秒起持续0.06秒物理力，名义冲量0.048N·s；记录实际施加时间、步数和冲量。不修改运行中物体位姿、不绑定物体。0.3秒后的位移快照不保证物体已经稳定。

## 工程门槛与采用复核

1. 静态和受扰分别完整成功≥80%，且均不低于同场景V2；也不低于同候选原两姿态。完整成功要求视觉控制宣布完成、物理确实在区域内稳定释放、无禁止接触。
2. 8空桌均不动作。65mm强制抓空必须真实发生；不许重试时失败，允许最多两次时通过至少一次重试恢复。
3. 错误成功宣告为0；完整保留失败、物理成功但视觉拒绝及所有重试。动作≤0.4rad/s、物理步0禁止接触；数据/资产/模型/源码哈希、种子隔离、策略调用和实际模型动作、评分/前缀审计通过。
4. 同候选原两姿态对照中，反馈组静态平均仿真耗时不增加；单独报告与最新V2的完整/首次成功、累计重试和仿真耗时差异、逐场景得失及解码回退使用次数。部分候选场景额外录制回放，墙钟耗时不作为公平性能比较。
5. 门槛通过后仍需写明**相对V2的任务层面收益和代价**，才决定采用；图像均值改善或成功率达到80%不足以采用。同分且未出现任务收益时保留V2；平均仿真耗时小于0.5秒的差异仅作微小轨迹变化，不单独作为采用收益。有限样本的配对检验仅作探索性结果，不声称统计确证、残差独立收益或实机迁移成功。

已知16400002在回退版仍失败，作为开发回归保留，不纳入新的正式场景。旧的22000018/11100009修复也只算已知回归证据。失败不能从报告或分母中删除；若试验中断，保留全部原始记录及预留种子，不用剩余好场景替换。

## 复现入口

在`variants/initial-observation-01`执行：

```bash
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python multiview_runner.py \
  --name goal-geometry-fallback-v3-candidate-01 --phase evaluate --episodes 30 \
  --static-seed-start 28000000 --disturbed-seed-start 28100000 \
  --negative-seed-start 28200000 --recovery-seed 28300000 \
  --perception outputs/vision-geometry050-fallback-01/perception.pt \
  --policy outputs/goal-policy-geometry050-fallback-01/residual.pt \
  --feedback-config configs/v2-feedback.json \
  --protocol outputs/geometry-fallback-protocol-01/GEOMETRY_FALLBACK_EVAL_PROTOCOL.md
```

终止后用该轮保存的源码执行`multiview_audit.py --run goal-geometry-fallback-v3-candidate-01`；当前源码改变时用`audit_frozen.py`加载该轮源码。最后保留成功/失败/恢复回放、完整指标和限制。未经采用复核，`LATEST_VERIFIED.json`和用户入口仍指向V2。
