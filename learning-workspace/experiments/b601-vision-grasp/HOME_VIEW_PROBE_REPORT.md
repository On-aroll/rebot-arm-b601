# 固定HOME基座观察位开发：未采用

运行 `outputs/home-view-development-01`。在HOME自身关节姿态的基础上，将J1分别加1.2rad和减1.2rad；每次均从reset后进行真实物理关节运动，不移动相机。每个姿态采集间隔0.2秒的两帧RGB-D，所有姿态完整保留，没有按真值挑选观察位。

场景含旧测试现降为开发数据的9600008、9600014，以及新的开发7000000–7000005。共8场景×3姿态=24次物理观察。6个新开发种子与探测之前的历史种子账本无交叉，训练/验证数据不含这些种子。

三种姿态都在8/8场景检出并通过现有质量门限；校正后定位误差均值约1.033mm、最大2.117mm。对每个场景，两个转基座姿态的原始位置估计与HOME完全相同。没有观察到额外收益，**不加入任务控制器**；本探测不是完整抓取放置测试。

`home_view_audit.py`已通过：2,216条动作状态、560帧配对RGB-D、72帧决策RGB-D，55,008个物理步，0禁止接触；重算模型和门限，核对关节命令速度与图像索引。真值只在每次准确决策帧之后用于定位误差评分，并与附近20Hz物体状态交叉核对。

```bash
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python home_view_probe.py --name my-home-view-probe
MUJOCO_GL=egl OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 .venv/bin/python home_view_audit.py --run my-home-view-probe
```

后续开发转向验证“低掩码覆盖率是否真的导致抓取失败”，以及统一渲染后端；见 [渲染差异诊断](RENDER_BACKEND_FINDING.md)。
