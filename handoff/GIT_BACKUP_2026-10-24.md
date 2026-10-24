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
- 20.8MB交付包独立上传；10.4GB原始归档拆成10个不超过1GiB的分卷。`BACKUP_MANIFEST.json`、`SHA256SUMS`和`RESTORE.txt`说明顺序、校验和恢复。完整服务端核验结果另存`SIM_ACT_RELEASE_VERIFIED_2026-10-24.json`，该记录生成前不能宣称大附件全部完成。

每阶段及时提交、推送并核对远端。大附件采用现有`handoff/upload_assets.py`的上传和服务端SHA256核验流程；保留原始失败。新设备重新配置本机认证，不复制或提交凭据。
