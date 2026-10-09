# B1 验收证据

本目录只包含合成数据和离线测试结果，不包含现场参数、设备序列号或真实 CAN 日志。

- `report.json`：25 项 Mock 验收与六轴合成例程结果、配置哈希、首尾帧、轨迹峰值。
- `mock-tests.txt`：对应逐项测试日志。
- `synthetic-trace.csv`：346 帧六轴 q/qd/加速度/jerk，全部标为 synthetic-test-fixture。
- `config-tests.txt`：10 项软件配置与来源隔离测试。
- `frontend-tests.txt`：21 个文件、105 项前端测试。
- `ws-client-tests.txt`：新增客户端文件的 3 项测试；与上项合计 108 项。
- `build.txt`：生产构建结果与大包提示。

复现：从仓库根运行 `python3 scripts/verify_mit_mock.py --output docs/validation/batch-b1`。这只重建 Mock 结果，不启动服务、网关或 CAN。前端/配置测试需按 [B1 交接](../../B1交接_2026-10-09.md) 分别运行。

完整剩余风险见 B1 交接；软件与 Mock 验收不得转述成真实 MIT 参数备份、实机动态验收或物理停止证明。
