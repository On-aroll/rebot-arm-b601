# SAC 长训练 R2（纯仿真）

承接 `residual-rl-01`，保留冻结 V3 为默认。用户选择约一小时训练；本轮没有连接机械臂。

改动限制在抓取/抬升技能：相机估计与自身正运动学计算相对位置；先张开夹爪对准五步，再沿用原来的十八步闭合/抬升；SAC 输出位置和夹爪的小幅修正。观察、运输、放置、核查、最多两次重试仍由 V3 程序完成。

```powershell
# 仓库根目录，使用现有 Windows 环境
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/probe.py --output probe-align
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/train_long.py --seconds 3600 --output long-r2-hour
```

训练使用项目现有 MuJoCo、感知模型及 Stable-Baselines3 SAC 2.7.1。新增三维观测的初始网络权重为零，验证与上一轮演员输出完全一致；重新初始化优化器与回放池。

长训练包含由易到难的初始抓偏、摩擦/质量/相机/延迟变化、25%简单任务复习、逐段降低学习率，以及每约五分钟独立验证、保存检查点。模型选择优先保证简单任务，再比较困难任务成功数，不以训练奖励代替成功率。

`outputs/` 保留训练、验证、探测的动作/物理记录、模型及回放池。所有失败和验证另存实际演员 RGB-D；成功训练保留演员输入特征、动作、初始/复看 RGB-D，不保留每一步完整图像。新场景完整任务测试另行记录。

物理范围是尚未实测标定的敏感性测试。仿真夹爪位移不能直接当作板端电机角度。结果完成后见 REPORT.md。
