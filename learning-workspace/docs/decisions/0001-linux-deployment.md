---
title: Kubuntu 原生系统部署基线
type: decision
status: accepted
date: 2026-10-18
---

# Kubuntu 原生系统部署基线

## 背景

用户希望将 Seeed B601-RS LeRobot 教程整合为一键部署包，随后明确目标为与当前电脑相同的原生 Kubuntu 24.04。当前环境识别为 Ubuntu 24.04 基础系统、KDE 桌面和 X11 会话，尚未提供机械臂的实际设备端口。

## 决策

- 以原生 Kubuntu 24.04 x86_64 为首要安装目标，兼容 Ubuntu / Kubuntu 22.04，采用项目独立 Python 3.12.12 环境和可重复执行的 Bash 安装器。
- 默认 CPU；CUDA 12.8 为可选安装方案，需要 NVIDIA 显卡及兼容驱动。
- 固定 Seeed DepthCameraSupport 源码提交 `fe078af31247a6144200abbfa3c2ce29d0062033`，附带未修改源码构建的 wheel。CPU/CUDA 各自使用带哈希的依赖锁文件。
- 首版支持 B601-RS + reBot 102 主臂 + USB RGB 摄像头的 ACT 流程，设备参数通过 JSON 配置。
- 软件安装与真实机械臂操作使用独立命令；安装仅运行合成数据上的训练和推理自检。
- 数据、模型和日志默认保存在本地；不自动上传 Hugging Face 或 W&B。

## 验证界限

软件链路已经在当前 Ubuntu 24.04 / KDE 原生系统的 CPU 环境验证。当前 PCI 设备包含 RTX 3050 Mobile，使用 nouveau 驱动，没有检测到 nvidia-smi；GPU 训练和机械臂实机效果仍待验证。详见 `VERIFICATION.md`。
