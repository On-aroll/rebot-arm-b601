# 下一候选：多姿态视觉训练

**开发诊断更新：** 旧多姿态数据新增部分使用GLFW，而原HOME部分和正式任务使用EGL。该轮及模型完整保留，图像指标的变化不能单独归因于姿态。统一EGL数据、`vision-multiview-egl-01`和匹配残差现已完成；在同一EGL验证集漏检47/546→29/546、空图0/94。详见 [渲染差异与修复](RENDER_BACKEND_FINDING.md)。[低覆盖物理诊断](LOW_COVERAGE_REPORT.md)的31张图像/23场景均完成任务，下一候选将检验覆盖率开发参数。

上一反馈候选190次测试已完成，静态和扰动下均为V1 28/30、两个候选26/30；质量拒绝造成退步，未采用。反馈降低了扰动下的重试次数，详见 [报告](GOAL_FEEDBACK_V2_REPORT.md)。本轮改善观察位的视觉模型。

## 数据

`observation_dataset.py` 生成图像训练数据：每张图像从一次新 reset 开始，仅在仿真时间0初始化机器人关节为HOME或两个观察姿态，然后渲染。观察姿态使用独立采样的名义锚点，未用物体真值选姿态。物体位姿仅由原有 reset 随机化；不执行运动、不产生任务成功记录。

这些图像不是物理执行示范，不计入20Hz动作轨迹或完整任务验收。原有任务轨迹仍由真实仿真运动生成。

- 新训练：256个场景，seeds6300000–6300255，每场景3个初始机器人姿态，共768张。
- 新验证：128个场景，seeds6400000–6400127，共384张。
- 合并原干净标签HOME数据后：训练1,792张（1,280个独立场景），验证640张（384个独立场景）。同一场景的不同姿态不会跨训练/验证划分。
- 训练/验证空图像分别258/94张，标签全部为空。对新图像共71,419个标签像素复核真实几何体积；保留完全遮挡的有目标图像（训练16、验证8），不挑选容易场景。
- 训练/验证种子不交叉，也不含当前960/990正式测试、空场景970或恢复980种子。

证据 `outputs/observation-images-01/data_and_baseline_audit.json` 已通过数据完整性核对。

## 原模型在新验证图像上的表现

| 图像组 | 正样本 | 漏检 | 空图误检 | 质量门限接受正样本 |
| --- | ---: | ---: | ---: | ---: |
| 原HOME验证 | 219 | 11 | 0 | 192 |
| 新HOME | 109 | 10 | 0 | 94 |
| 新观察位0 | 109 | 20 | 0 | 79 |
| 新观察位1 | 109 | 22 | 0 | 75 |

在两个观察位中，质量门限分别放过2个误差大于5mm的估计。门限不是可靠的误差上界；训练改进需同时查看漏检、误检、误差和分割覆盖，最终仍需新场景的完整任务验收。

## 当前训练与复现

从V1视觉权重开始，保留同一1,765参数网络，训练20 epoch；按既有验证规则（先误检数，再漏检数，再条件平均误差）选择epoch15。`outputs/vision-multiview-01` 已训练完成，尚未采用。

| 验证图像组 | 正样本 | 原模型漏检→新模型 | 质量接受原→新 | 接受后误差>5mm原→新 |
| --- | ---: | ---: | ---: | ---: |
| 原HOME | 219 | 11→6 | 192→206 | 0→0 |
| 新HOME | 109 | 10→6 | 94→98 | 0→0 |
| 观察位0 | 109 | 20→15 | 79→85 | 2→1 |
| 观察位1 | 109 | 22→18 | 75→81 | 2→3 |

总漏检63/546→45/546，94张空图误检均0，质量接受440→470；接受样本中>5mm误差仍各4例。新模型被检出501个正样本的平均误差1.674mm、P956.174mm。两个模型检出样本集合不同，不直接比较条件平均误差来宣称精度改善。证据为`data_and_candidate01_audit.json`。

匹配残差头 `outputs/goal-policy-multiview-01` 也已完成：1,443参数，1,426个可用训练图像，501个验证图像，300epoch中选292；同一501样本条件平均误差1.674→1.416mm，P956.174→5.030mm。仍需物理任务验证。

- 新视觉SHA256：`e334008016149d4c6ca06595fa7fef39cf22ce31135f8af1404ead9dcaf2777d`
- 新残差SHA256：`6db80a1db95d5613bca6f4ea5e27b348dfa78f93e69eb666283a45965eb0cebb`

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python observation_dataset.py --name my-observation-images --train-scenes 256 --validation-scenes 128 --train-seed 6300000 --validation-seed 6400000
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python observation_data_audit.py --run my-observation-images
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python vision_runner.py --name my-multiview-vision --samples 1792 --epochs 20 --initial outputs/vision-iteration-03/perception.pt --reuse-data outputs/my-observation-images --training-seed 20261022
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python observation_policy.py --name my-multiview-head --perception outputs/my-multiview-vision/perception.pt --dataset outputs/my-observation-images --epochs 300
```

`multiview-development-01` 使用新开发种子6500000/6600000各6场景，四组完整成功均6/6；额外8个空场景无运动，禁止/允许重试对照确实抓空并恢复；58次记录审计通过，25,000步最终评分已复算。小样本未用于采用，详见 [开发报告](MULTIVIEW_DEVELOPMENT_REPORT.md)。

正式 `goal-multiview-v2-candidate-01` 已按 [验收协议](MULTIVIEW_EVAL_PROTOCOL.md) 完成250次并审计通过。反馈静态29/30、扰动29/30，保留V1分别30/30、28/30；静态门槛失败未采用，见 [完整报告](GOAL_MULTIVIEW_V2_REPORT.md)。

当前下一候选开发为 `multiview-egl-development-02`：统一EGL模型/配套残差，覆盖率开发参数0.25，新静态8100000起6场景、扰动8200000起6场景、空场景8300000起8和恢复8400000两次。目标为58次开发运行及审计；尚未用于采用或正式独立成绩。

后续正式完整任务测试必须再次冻结、每块至少30个新场景。旧残差不能与新视觉混用，保留V1基线必须使用旧权重，不能把新模型的原控制组命名为V1。
