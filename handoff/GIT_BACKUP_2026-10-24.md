# 新主机Git身份与仿真备份

2026-10-24已核对登录账号为On-aroll。新主机原来没有设置Git的`user.name`/`user.email`，与GitHub已登录是两回事。已沿用仓库历史提交中的身份，仅设置当前仓库：

```text
git config --local user.name On-aroll
git config --local user.email On-aroll@users.noreply.github.com
```

用户明确要求按当前公开仓库直接上传，不按旧交接的私有要求阻止本轮备份，也不修改仓库可见性。

- 分支：`sim-act-work`，已推送初始成果提交`96d97eb`。
- 固定源码标签：`sim-act-2026-10-24`。
- Release：https://github.com/On-aroll/rebot-arm-b601/releases/tag/sim-act-2026-10-24
- Release为实验预发布，模型未采用、默认仍为V3。没有改写main或冻结版本。
- 20.8MB交付包和10.4GB原始归档已全部上传。1GiB分卷传输中断后，原始归档改为155个不超过64MiB的分卷；`BACKUP_MANIFEST.json`、`SHA256SUMS`和`RESTORE.txt`说明顺序、校验和恢复。
- 159个附件的大小与服务端SHA256均已逐项匹配本地文件。完整核验结果见`SIM_ACT_RELEASE_VERIFIED_2026-10-24.json`，其中保留下载地址、字节数、哈希与核验时间。模型仍是失败候选，备份完成不代表训练目标已实现。

每阶段及时提交、推送并核对远端。大附件采用现有`handoff/upload_assets.py`的上传和服务端SHA256核验流程；保留原始失败。新设备重新配置本机认证，不复制或提交凭据。

本轮已遇到Windows自动换行导致冻结哈希失配，因此分支新增`.gitattributes`保护冻结版本和交付包的原始字节。获取较早的固定标签时，克隆命令使用`git -c core.autocrlf=false clone ...`；不要改写历史证据以适应换行。

## 标准ACT最小闭环的增量备份

后续 `variants/act-upstream-01` 在已知单场景通过模型闭环10/10；较大位置变化仍失败，未作为V4采用。固定源码提交`09a69f3`、标签`sim-act-upstream-gate-2026-10-24`，报告说明全部限制。新增Release：

https://github.com/On-aroll/rebot-arm-b601/releases/tag/sim-act-upstream-gate-2026-10-24

仅新增8个附件、208,322,871字节：一份206MB模型分成四个64MiB以内二进制分卷，小型证据ZIP以及三个恢复/校验文件。没有重新上传旧10.4GB归档。模型合并SHA256为`38d670fc82c8c85ead0b26d9fb927070f9201da35d19ec7ece4c7878f846b243`。ZIP的130个新记录/源码逐文件复核通过，8个远端附件服务端SHA256全部与本地一致；回执见`SIM_ACT_UPSTREAM_GATE_VERIFIED_2026-10-24.json`。

## 首轮残差强化学习增量备份

用户后续明确授权开始强化学习。实验源码提交`927de0a`，交付记录标签`sim-sac-2026-10-24`；该标签包含后续生成的交付清单。新Release：

https://github.com/On-aroll/rebot-arm-b601/releases/tag/sim-sac-2026-10-24

本轮只备份`variants/residual-rl-01`新增内容。记录归档2,128,890,508字节，3754个文件已独立解压逐项核验，分为32个不超过64MiB的二进制分卷；另有490,090字节小交付ZIP和三个恢复/校验文件，共36附件。全部服务端SHA256与本地文件一致，回执`SIM_SAC_RELEASE_VERIFIED_2026-10-24.json`已保存。合并归档SHA256为`6603a6de5848b17aa19116a1614eff7c1ace90d7bb4a8715588eda74fd1d1217`。

原V3与新混合控制器30新场景均30/30，未证明任务收益；默认仍V3，不是V4或实机可执行版本。全部12个已完成训练失败保留；4个未完成尾回合的44步已从原重放缓冲恢复，原始尾部物理/画面缺失的限制明确写在报告中，未补造数据。之前的10.4GB ACT归档和206MB上游ACT模型未重复上传。
