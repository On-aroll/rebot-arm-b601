# 未采用的研究模型：只做离线/影子推理

第二轮30个独立场景为0/30，默认控制器仍为V3。`STATUS.json` 是机器可读采用状态。此文件夹不导入MuJoCo或实机驱动，输出不是可直接执行的电机命令。

在兼容Python/PyTorch、NumPy和OpenCV的环境中，进入本文件夹：

```text
python infer.py --checkpoint policy.pt --observation example_observation.npz --goal A --output proposal.json
python benchmark_inference.py --checkpoint policy.pt --observation example_observation.npz --output my_cpu_benchmark.json
```

`proposal.json` 应与 `example_action.json` 的10×7动作数组一致。当前文件包已在训练主机独立运行验证；换板后需重新核对数值与资源，Windows锁文件不直接适用于ARM。

输入20维自身状态和图像的顺序见包根目录README。该候选另外从图像提取4维可见像素中心/深度/可见性，合计24维网络状态；不使用计时输入。输出是相对上次命令的动作，Python入口已还原为六关节rad加单指模型行程m。

`policy_cpu.ts` 为固定batch=1的CPU网络：输入标准化图像[1,4,60,80]、标准化状态[1,24]、目标one-hot[1,3]，输出标准化动作[1,10,7]。手动接入时按config.json反归一化，再加上上次请求的六关节命令和夹爪米制命令。TorchScript文件本身不包含图像预处理、限幅、视觉验收或重试监督逻辑。

真实J7仍是电机弧度。没有现场标定表时，不把输出第7维交给实机执行器。实机数据回传与拟合路径见包根目录REAL_DATA.md。
