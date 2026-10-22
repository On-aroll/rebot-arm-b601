---
title: GitHub Release 与 Linux 分发边界
type: decision
status: accepted
date: 2026-10-18
---

# GitHub Release 与 Linux 分发边界

用户要求持续维护项目、上传 GitHub，并通过 Release 一键部署 Linux。

- 采用 VERSION + 标签管理版本，GitHub Actions 测试成功后发布固定归档和 SHA256SUMS。
- 保持小型联网包，Python 与 PyTorch 按锁文件下载，不随项目提交本机环境或真实任务数据。
- Release 下载器按版本安装独立环境，共享个人配置和数据，通过自检后切换入口。
- 当前二进制边界为 Linux x86_64 / glibc >= 2.28；扩展发行版包管理适配，同时区分“已适配”与“已实测”。不承诺任意 Linux、ARM 或 musl 可用。
- 继续固定已验证的 Seeed 上游提交；暂不切换最新 LeRobot 或新增其他机器人方案。
- 仓库目标和公开/私有状态由用户选择；不覆盖其他项目、不擅自公开现有私有仓库。
