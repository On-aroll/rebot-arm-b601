# 自遮挡视角探测：暂停封存

2026-10-22 用户要求暂停并提交 GitHub，未继续诊断、重跑或训练。

- 协议：`SELF_VISIBILITY_DEVELOPMENT_PROTOCOL.md`，SHA256 `17f96b571a15676f54b8a87c1ee1dfd06efaf6a5ec9386382da4313b16ad5e71`。
- 分支：`variants/self-visibility-01`，保留原始冻结源；运行：`outputs/self-visibility-known-development-01`。
- 6 个已知初态 × 原两视角/选择两视角，24 次独立物理探测完成，3957 动作行、996 RGB-D、120 决策图、98725 物理步。
- 禁止物理步骤与夹爪接触均 0；最大关节速度 0.399965734509 rad/s，物体 XY 位移最大约 1.16e-10 m。
- 只有 16400002、30000029 的 HOME 原质量门槛拒绝会触发重观察；原视角和新选择视角均没有可靠救回，机制门槛为 false。
- 完整审计退出 1：`self_visibility_probe.py` 的 `check_rays` 比较 `mj_ray` 与 `mj_multiRay` 时，geom ID/距离 1e-12 一致性断言失败。原因尚未调查，不能当作已确认的数值容差问题。
- 审计日志 `outputs/self-visibility-known-development-01.audit.console.log` 保留；没有审计成功文件，也没有做保存源重审。

**未通过机制门槛、未通过完整审计、未采用。默认仍 V3。** 暂停期间仅整理证据，不修补原冻结记录。
