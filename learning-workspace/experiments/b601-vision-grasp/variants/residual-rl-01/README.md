# V3基础上的抓取残差强化学习

本支线使用Stable-Baselines3 2.7.1的SAC算法。冻结V3继续负责RGB-D视觉、观察位置、逆运动学、动作限幅、搬运、视觉验收和最多两次重试；新小模型只在闭爪、抬升期间给出有限的位置/夹爪行程修正。它不代表完整动作策略、重试决策或上一轮ACT模型已经强化学习成功。默认版本不变，全部为仿真，不含实机入口。

演员输入36维：V3从实际RGB-D估计的8维视觉特征、当前视觉有效标记、20维自身反馈/上次命令、A/B/C one-hot和上次4维修正。目标暂时看不见时保留上次有效视觉特征并明确置无效标记。无物体真值、摩擦/质量标签、接触真值、奖励、教师阶段或时钟输入。视觉网络固定；本轮没有从原始像素重新学习视觉。

每次抓取有18个决策步，最短周期0.2秒（原有速度限幅可能延长）。SAC输出XYZ及夹爪目标的残差，边界为±12/12/8/6mm，每步修正变化限为2/2/2/1.5mm。闭合/抬升的基础时序由明确程序给出，模型不选择整个抓放阶段。IK失败、视觉准备失败和禁止接触都保留为失败，不重新抽取场景掩盖。

奖励来自物理结果而非示范答案：抬高、双指稳定夹持加分；修正幅度和变化扣分；最终抬高80mm且双指夹持连续至少0.4秒加10，否则扣5，禁止接触或运行失败再扣10。该奖励仅训练抓取/抬升技能，**不能作为完整放置成功率**。正式任务仍需独立物理评分和视觉验收。

训练使用48000000–48000071，共72个种子，区域均衡；验证48100000–48100011，共12个种子；开发检查使用482开头，正式候选冻结后用49000000–49000029。沿用act-01的摩擦、质量、相机和动作延迟敏感性随机化，参数不是实机辨识值。参考配置：4个仿真进程，CPU上64×64小MLP，8192个环境交互步，学习率3e-4，batch128、gamma0.98、SAC熵系数auto_0.1；按验证技能奖励选择模型，不按正式测试挑选。

训练保存20Hz动作/物理评分、准备阶段的决策RGB-D、逐步实际策略输入、动作和奖励；不保存全程视频或全部5Hz原始训练RGB-D。训练时模型实际接收的36维输入已逐步完整保存。完整任务测试使用V3 Recorder的20Hz动作、5Hz RGB-D和物理评分，成功失败均保留。零修正混合控制器用于隔离新技能时序改变和RL修正本身的影响。

## 运行

从仓库根目录，沿用独立`.venv`；依赖见requirements-windows.lock。

```powershell
$py = '.\.venv\Scripts\python.exe'
$work = 'learning-workspace\experiments\b601-vision-grasp\variants\residual-rl-01'
& $py "$work\workflow.py" skill-test --name zero-repeat --episodes 12 --seed 48100000
& $py "$work\workflow.py" train --name sac-repeat --steps 8192 --workers 4
& $py "$work\workflow.py" skill-test --name learned-repeat --episodes 12 --seed 48100000 --checkpoint "$work\outputs\sac-repeat\best\best_model.zip"
& $py "$work\workflow.py" evaluate --name formal-repeat --episodes 30 --seed 49000000 --checkpoint "$work\outputs\sac-repeat\best\best_model.zip"
```

每次使用新输出名，原始失败与源码快照不覆盖。完整任务对照包含原V3、相同新技能时序但残差为零的控制器、SAC残差控制器；同一seed/区域/挑战，最多两次重试。静态/外力/首次抓偏65mm各10，区域A/B/C各10。抓偏在首次执行时生效，后续恢复主要仍由显式监督器完成，不能宣称SAC学会了重试策略。

上游来源与方法：
- https://github.com/DLR-RM/stable-baselines3/tree/v2.7.1
- https://stable-baselines3.readthedocs.io/en/v2.7.1/modules/sac.html
- https://residualrl.github.io/

阶段结果和实际作用以REPORT.md为准，不能以奖励曲线或模型导出替代任务验收。
