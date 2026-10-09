# P4 验收记录（2026-10-09；2026-10-10 最终回归）

| 范围 | 结果 / 证据 | 与实机的关系 |
|---|---|---|
| 官方 SDK 适配器/独立线程逻辑 | 24 项通过；[离线报告](offline-report.json)、[日志](offline-tests.txt) | Fake SDK / 虚拟时钟；真实电机运动 0 |
| 软件配置保护 | 同一离线报告内 11 项通过，总计 35 | 临时目录；覆盖冲突拒绝与不自动批准 |
| B1 六轴轨迹 | 原有 25 项全部通过、346 虚拟帧 | 输出 `outputs/mit-p4/b1/`；未更改 B1 物理能力结论 |
| MuJoCo 姿态同步 | 3 项通过，[日志](pose-sync-tests.txt) | 合成六轴反馈；含 FK、ctrl、重复同步、碰撞/过期拒绝 |
| HTTP 姿态同步 | [报告](pose-sync-http.json)，初始化误差 0 rad，旧计划作废 | 合成反馈，真实写入 0 |
| HTTP 写入保护 | `tests/verify_config_http.py http://127.0.0.1:8765` 通过 | 9 类实机操作及硬件点位请求均拒绝 |
| XYZ 规划及仿真 | [到点报告](point-http.json)，(200,150,200) mm，误差 0.56234 mm | 仿真实际反馈，不是实机到位 |
| 前端回归 | 23 文件 111 项通过，[日志](frontend-tests.txt) | 含旧 Studio 只读白名单保护 |
| 前端生产构建 | [日志](frontend-build.txt) | 通过，原有约 1.16 MB 包体警告保留 |
| 浏览器验收 | 桌面 1366×900、手机 390×844 已目视检查，手机展开详情 content=viewport=375 px | 顶部连接入口、三项增益/折叠/七轴/模型/XYZ 正常；未使能 |
| 真实 SDK 只读检查 | [原始记录](readonly-probe.json)，三轮六轴响应及模式成功，约149ms/轮 | 仅参数读取和 ping；不是动作/停止验收 |
| 历史只读姿态的模型检查 | [记录](recorded-pose-model-check.json) | J2略低于模型0°；其他轴在模型内、无模型碰撞；不是当前起点 |
| 10月9日浏览器真实同步尝试 | 16:21 读取7/7，六轴MIT，明确拒绝 J2 约−0.31° | 未裁剪或改零位，未初始化为错误姿态，无真实动作；不是10月10日实时状态 |
| 10月10日服务收尾核对 | 控制台和只读 HTTP 状态正常，最新前端资源已提供；软件配置 v46 保留后续编辑 | `hardware_allowed=false`，`auto_connect=false`；未连接 CAN |

复现离线部分：

```bash
python3 scripts/verify_mit_p4.py
python3 scripts/verify_mit_mock.py --output outputs/mit-p4/b1
~/.local/share/robotics/envs/rebot-sim/bin/python tests/test_pose_sync.py
~/.local/share/robotics/envs/rebot-sim/bin/python tests/verify_point_planner.py
```

HTTP 测试只对空闲的本机仿真服务运行，会改变仿真姿态和任务，但不会打开 CAN：

```bash
~/.local/share/robotics/envs/rebot-sim/bin/python tests/verify_pose_sync_http.py
python3 tests/verify_config_http.py http://127.0.0.1:8765
~/.local/share/robotics/envs/rebot-sim/bin/python tests/verify_point_http.py
```

真实只读探针 `scripts/rebot_readonly_probe.py` 是显式诊断工具，不属于离线测试，不在启动时运行。本轮按用户新要求执行了只读检查；不将上一轮 B1 的“0次CAN查询”历史结论改写成当前结论。

代码已实现 ≠ 虚拟测试通过 ≠ 实际设备运动/停止通过。真实多轴启动/保持、硬件看门狗、动态轴验收和网页 XYZ 实发仍未完成。详细剩余风险见 [P4交接](../../P4交接_2026-10-09.md)。
