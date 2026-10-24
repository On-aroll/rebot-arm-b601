# B601 动作分块仿真研究

默认已验证控制器继续为冻结 V3。实验结论、采用状态和精确成绩见 `REPORT.md`；模型并不因文件成功导出就具备实机抓取能力。

本目录直接导入 `versions/V3` 的物理模型、教师控制器和评分器，启动时核验原始108个文件。新功能仅在此目录，未改写冻结版本。`runtime.py` 增加接触/相机/响应随机化，`workflow.py` 管理示范与闭环，`chunk_policy.py` 是小型 ACT 风格 CVAE/Transformer，`infer.py` 只生成动作提议，`REAL_DATA.md` 说明实机回传和校正流程。

## 本机运行

以下从仓库根目录运行。Python 3.12，依赖见 `requirements-windows.lock`。本机独立环境在根目录 `.venv`，没有安装或启动 ROS/实机执行器。新设备先建立自己的环境，不复制 `.venv`。CUDA版PyTorch从官方 `https://download.pytorch.org/whl/cu128` 获取；锁文件适用于本轮Windows训练环境，板端须使用目标架构支持的运行时并核对输出。

```powershell
$py = '.\.venv\Scripts\python.exe'
$work = 'learning-workspace\experiments\b601-vision-grasp\variants\act-01'

# 新建一个V3原始物理基线回合；输出名必须未使用
& $py -u "$work\workflow.py" collect --name v3-repeat --episodes 1 --seed 43000000 --ideal --video

# 用本轮冻结源码和参数重采集；保留所有失败
& $py "$work\reproduce.py" --run demos-r1 --name act-repeat-demos-r1
& $py "$work\reproduce.py" --run validation-r1 --name act-repeat-validation
& $py "$work\reproduce.py" --run demos-r2-recovery --name act-repeat-recovery

# 精确核对原训练数据哈希，按对应源码/seed重新训练
& $py "$work\reproduce.py" --run model-r1 --name act-repeat-model-r1
& $py "$work\reproduce.py" --run model-r2-balanced --name act-repeat-model-r2
& $py "$work\reproduce.py" --run model-r3-clock --name act-repeat-model-r3

# 复跑正式对照的一个分组；另有 eval-r1-1/2 和 eval-r2-0/1/2
& $py "$work\reproduce.py" --run eval-r1-0 --name act-repeat-eval-r1-0

# 数据和评分复核
& $py "$work\audit.py" --training "$work\outputs\demos-r1" "$work\outputs\demos-r2-recovery" --validation "$work\outputs\validation-r1" --output "$work\outputs\audit-repeat.json"
& $py "$work\summarize.py" --round r1
& $py "$work\summarize.py" --round r2
```

每个原始 run 的 `source/`、`config.json` / `frozen_training.json`、数据/权重哈希都已保留。重训采用相同seed和配置；GPU库与渲染平台变化可能影响数值，不能承诺跨平台逐字节重现。不要用当前已修改的训练代码假冒首轮复现。

`outputs/` 保留所有本轮动作JSONL、5Hz RGB-D、独立评分、正例示范掩码、源快照和失败模型，受现有Git忽略规则保护。`delivery/` 中另存小型可搬运推理包、配置、报告和正式结果摘要；完整原始记录需另带走实验归档，不能只复制源码。

## 输入输出

原始输入为 RGB uint8、配准深度米，以及20维自身状态：六关节反馈、六关节速度、夹爪反馈/0.05、上次六关节命令、上次夹爪命令/0.05。输入区域为 A/B/C one-hot。后续候选额外使用由画面计算的红橙像素中心/深度/可见性；计时候选还使用本次目标/重试开始后的秒数。这些信息不含物体真值或教师阶段标签。

输出为10个未来目标，每步200ms，每个目标是六关节rad和单指模型行程m。推理脚本恢复归一化及相对动作。仿真运行器还限幅关节/夹爪每步变化，执行首步（实验可以显式设置分块执行步数）。启动视觉门槛、验收和最多两次重试由显式监督逻辑负责，不能称整个系统全部由模型学习。

离线推理使用交付包中的示例：

```powershell
& $py "$work\infer.py" --checkpoint "$work\delivery\research\policy.pt" --observation "$work\delivery\research\example_observation.npz" --goal A --elapsed-s 0 --output "$work\outputs\action-proposal.json"
& $py "$work\workflow.py" benchmark --checkpoint "$work\delivery\research\policy.pt"
```

参考流程与差异见 `PROTOCOL.md`。本实现使用单 RGB-D 相机与缩小网络，不是参考仓库双相机LeRobot ACT的原样复现，也不承诺checkpoint兼容。
