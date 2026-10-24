# 标准 ACT 的单条示范闭环门槛

默认继续使用冻结 V3。本目录复用 `act-01/runtime.py`、原始示范和独立物理评分器，直接调用上游 `ACTPolicy`，不再自实现模型。参考工程 `HJX-exoskeleton/lerobot-mujoco-rebot` 的源码为 `39177b3391ee090806acea3c8fc0fd0511af2547`；它的安装说明锁定 LeRobot `10b7b3532543b4adfb65760f02a49b4c537afde7`，本实验使用同一提交且检查源码无修改。

这一步是模仿学习诊断，不是强化学习，也不是新场景最终测试。仅训练原有 `demos-r1/episode-000`（目标 A）的164个有效帧。模型接收120×160单RGB画面、20维自身状态及3维目标区域one-hot；不提供教师阶段、时间、物体真值或未来图像。当前没有使用深度图，不能称完整复现参考工程的双相机配置。

采用上游默认ResNet18预训练骨干、512维Transformer、4层编码器、1层解码器及CVAE/KL=10。动作块25步，每次执行5步；5Hz绝对关节/夹爪目标与训练期一致。主学习率1e-4、骨干1e-5、batch=16、seed=20261024。ImageNet图像归一化，自身状态/动作由单条演示统计归一化。标准架构能否完成此任务仍以实际闭环为准。

教师和模型使用同一个 `command()` 限幅和0.2秒执行入口。固定场景重复10次，仅证明这一已知场景上的链路；无位置随机变化，不能把10次重复当作10个独立场景。诊断按演示长度运行后检查物体已释放、支撑稳定、区域正确且无禁止接触；这一步尚未接入最终系统的视觉验收和两次重试。正常动作全部来自模型，不使用脚本补完任务。失败结果和运行时源码分别保留在 `outputs/`。

## 复现

从仓库根目录执行；先恢复上一轮Release中的原始数据。沿用现有独立 `.venv`，完整依赖见本目录 `requirements-windows.lock`；匹配的CUDA版Torch/Torchvision从PyTorch官方源安装。无需安装参考工程中的相机/实机SDK或连接设备。

```powershell
git clone --filter=blob:none https://github.com/huggingface/lerobot.git .tools/lerobot-upstream
git -C .tools/lerobot-upstream checkout 10b7b3532543b4adfb65760f02a49b4c537afde7

$py = '.\.venv\Scripts\python.exe'
$gate = 'learning-workspace\experiments\b601-vision-grasp\variants\act-upstream-01\gate.py'
& $py $gate teacher --name teacher-repeat --repeats 10
& $py $gate train --name model-repeat --steps 2000
& $py $gate evaluate --name evaluation-repeat --repeats 10 --checkpoint 'learning-workspace\experiments\b601-vision-grasp\variants\act-upstream-01\outputs\model-repeat\checkpoint'
```

结果以 `outputs/<name>/summary.json` 为准，不以loss判定学会任务。教师入口和模型已知场景均10/10通过；±20mm探测4/4、±40mm探测1/4，详细限制和独立夹持/抬升审计见REPORT.md。`audit.py`重放已有命令并记录物体/接触真值，只用于评分。之后再扩展位置、区域、相机/动力学变化和来自模型实际访问状态的教师纠正，最终锁定至少30个新的未训练场景与V3配对比较。强化学习只考虑在可靠模仿起点上优化精度/恢复，不用它跳过当前基本闭环问题。

上游来源：
- https://github.com/HJX-exoskeleton/lerobot-mujoco-rebot/blob/39177b3391ee090806acea3c8fc0fd0511af2547/commands.txt
- https://github.com/huggingface/lerobot/tree/10b7b3532543b4adfb65760f02a49b4c537afde7/lerobot/common/policies/act

本阶段权重和130个新记录/源码的增量备份（8附件，约208MB，服务端SHA256已逐项核验）：
https://github.com/On-aroll/rebot-arm-b601/releases/tag/sim-act-upstream-gate-2026-10-24

恢复先验证SHA256SUMS，再按RESTORE.txt解压证据ZIP、合并四个模型分卷；模型合并哈希见BACKUP_MANIFEST.json。原始输入示范继续取自上一轮Release，不重复下载本轮不存在的训练数据。
