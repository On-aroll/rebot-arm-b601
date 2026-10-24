# SAC 长训练 R2（纯仿真）

承接 `residual-rl-01`，保留冻结 V3 为默认。用户选择约一小时训练；本轮没有连接机械臂。

新Windows电脑使用Python 3.12建立环境后，从 `requirements-windows.lock` 安装；PyTorch 2.7.1 / torchvision 0.22.1 CUDA12.8按[官方历史版本说明](https://pytorch.org/get-started/previous-versions/)获取。清单已将原本机wheel路径改为官方索引和同版本约束，没有升级训练环境。单独运行NumPy演员只需NumPy；完整仿真仍需本项目视觉和MuJoCo依赖。板端安排见 REAL_DATA.md。

```powershell
# 仅在尚未建立环境的新Windows电脑上执行
py -3.12 -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/requirements-windows.lock
```

改动限制在抓取/抬升技能：相机估计与自身正运动学计算相对位置；先张开夹爪对准五步，再沿用原来的十八步闭合/抬升；SAC 输出位置和夹爪的小幅修正。观察、运输、放置、核查、最多两次重试仍由 V3 程序完成。

```powershell
# 仓库根目录，使用现有 Windows 环境
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/probe.py --output probe-align
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/train_long.py --seconds 3600 --output long-r2-hour
```

训练使用项目现有 MuJoCo、感知模型及 Stable-Baselines3 SAC 2.7.1。新增三维观测的初始网络权重为零，验证与上一轮演员输出完全一致；重新初始化优化器与回放池。

长训练包含由易到难的初始抓偏、摩擦/质量/相机/延迟变化、25%简单任务复习、逐段降低学习率，以及每约五分钟独立验证、保存检查点。模型选择优先保证简单任务，再比较困难任务成功数，不以训练奖励代替成功率。

课程按墙钟时间切换，因此不同主机负载下交互总步数和切换步数会变化；已发布的实际源码快照、参数、数据和检查点用于逐项复核，不承诺重跑逐字节一致。

`outputs/` 保留训练、验证、探测的动作/物理记录、模型及回放池。所有失败和验证另存实际演员 RGB-D；成功训练保留演员输入特征、动作、初始/复看 RGB-D，不保留每一步完整图像。新场景完整任务测试另行记录。

物理范围是尚未实测标定的敏感性测试。仿真夹爪位移不能直接当作板端电机角度。结果完成后见 REPORT.md。

训练完成后：

```powershell
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/export.py
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/infer_numpy.py --directory learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/delivery --benchmark
# 新建输出目录运行宽范围36场/边界30场；参数、原始基线及分批索引见正式输出config.json
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/evaluate.py --output reproduce-broad --count 36
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/evaluate.py --suite boundary --output reproduce-boundary --count 30
```

交付包含完整SAC训练权重、TorchScript以及只需NumPy的确定性演员。NumPy版本只是同一网络的矩阵运算，需用实际记录逐项核对误差；不代表整个视觉抓放系统脱离PyTorch，也不代表板端已验收。

让模拟机械臂观察画面和自身反馈，完成一次指定区域任务（输出目录须为新目录）：

```powershell
& .\.venv\Scripts\python.exe learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/run_sim.py --goal B --seed 55000000 --output learning-workspace/experiments/b601-vision-grasp/variants/residual-rl-02/outputs/user-demo-B
```

此入口使用新残差演员和原V3任务程序；会保存画面、实际指令、评分和失败，并最多重试两次。它不含实机驱动。
