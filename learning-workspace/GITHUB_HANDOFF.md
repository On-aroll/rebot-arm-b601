# 2026-10-28 当前 GitHub 入口

用户明确确认保持公开：[On-aroll/rebot-arm-b601](https://github.com/On-aroll/rebot-arm-b601)。本次增量在 main，新增高算力主机训练说明及同步核对记录；主线位于 `ReBot_Arm_DigitalTwin_RS/`，学习与视觉实验位于 `learning-workspace/`。先读根 `HANDOFF.md` 与 `handoff/updates/2026-10-28.md`。下方 10 月 22 日的提交、私有状态和封存说明是历史记录。

---

# 2026-10-22 用户暂停与 GitHub 换机交接

持续仿真实验已按用户要求暂停，不自动恢复训练或硬件操作。

统一私有仓库：https://github.com/On-aroll/rebot-arm-b601

本地封存目录：`/home/bonvoyage/Desktop/b601-github-handoff-20261022`。
已推送的源码提交：`eb37273`，分支 `main` / `handoff/2026-10-22`，标签 `handoff-2026-10-22`。

最终交接提交：`afc5758304400efa0ba35969bbe206d668e4f23e`，已核对远端 main 与 handoff 分支一致；发布源码/证据校验清单共 2071 文件，包含完整 Release 核验记录。封存标签仍固定源码提交 `eb37273`。

接手请读仓库根目录 `HANDOFF.md`。主线现在随仓库位于 `ReBot_Arm_DigitalTwin_RS/`，原本目录内容位于 `learning-workspace/`，视觉实验位于其中 `experiments/b601-vision-grasp/`。

源码、V2/V3、151 检查点、报告、实验摘要和同臂 215 份恢复历史已经进入 Git。94 个归档已逐文件核对；完整原始数据/录像和历史安装包共 192 个附件、13,876,360,180 字节，已于 2026-10-22 19:14（北京时间）全部上传并通过服务端 SHA256 对账。私有仓库和 Release 状态已核对，证明在仓库 `handoff/RELEASE_VERIFIED.json`；本机 `handoff/release-assets/` 保留全部附件。

完整原始数据下载：[handoff-2026-10-22 Release](https://github.com/On-aroll/rebot-arm-b601/releases/tag/handoff-2026-10-22)。Git clone 取得源码和恢复历史；全量 RGB-D/录像须从 Release 另行下载。

新设备先安装、恢复同臂历史、核对新鲜只读反馈；不要将旧 PID/姿态当当前状态。默认纯仿真 V3，没有 sim2real 执行验证。最新自遮挡探测机制门槛及射线审计均未通过，未采用。
