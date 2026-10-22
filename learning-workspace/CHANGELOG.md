# 更新记录

## 0.1.0 — 2026-10-18

- 首次提供 B601-RS / reBot 102 / USB RGB 摄像头的 LeRobot ACT 部署包。
- 固定 Seeed 源码版本、Python 3.12.12 和带哈希的 CPU / CUDA 依赖。
- 提供诊断、校准、遥操作、采集、训练、模型执行与离线软件自检。
- 添加 GitHub Release 安装器、版本目录和共享数据目录；SHA256 校验后安装，通过自检后切换 current。
- 增加 Ubuntu / Debian、Fedora、Arch 系系统依赖适配；实际验证为 Kubuntu 24.04 x86_64 CPU。
- 配置 GitHub Actions：Ubuntu 22.04 / 24.04 CPU 安装与 ACT 软件自检通过后发布附件。

发布包是联网安装入口，仍需下载 Python / PyTorch 等依赖；不附带真实抓取模型。CUDA、其他发行版和机械臂硬件效果尚未实测，详见 VERIFICATION.md 与 docs/SUPPORT.md。
