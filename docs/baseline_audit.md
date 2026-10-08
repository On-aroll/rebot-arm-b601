# 批次 A 基线审计（2026-10-08）

依据用户提供的《Codex_B601_RS_MIT_Console_Task_v2.md》。本轮只进行源码、软件配置、Mock 与仿真开发，不授权 CAN 查询或真实写入。

## Git 基线与资料

- 开始分支 main，HEAD `d397d5e429265ffe49ec94b67ab6344a7a9a4634`，工作区干净。
- 工作分支 `codex/mit-console-protection`；保留原 outputs、环境、已调参数、浏览器映射。
- 已读 AGENTS、README、重启指南、当前进度、10-08 的映射/XYZ/电机配置记录和 10-07 控制台记录。
- 原说明的“三道前端闸门”、stop 演练例外、默认 PP 草稿、wrap 角度用于比较，均不能作为本轮 MIT 执行依据。当前原始多圈数据保留，不自动归一化到控制分支。
- 历史记录 J1–J6 为 MIT；历史 PP/CSP 寄存器值不是当前反馈，也不是 MIT 速度/加速度证明。没有把这些数字导入批准配置。夹爪实际模式不推断。

## 写入路径与边界

| 入口 | 实际路径 / 风险 | 本轮处理 |
|---|---|---|
| 原 MotorModuleCard | enable / disable / ensure_mode / robstride_write_param；待切模式默认 pos | 改为软件草稿、差异审核、历史；删除模式切换和真实下发 |
| useMotorGateway.write | selectMotor → WsGatewayClient.send → Rust 网关；前端 armed/drill 可变 | write/stopAll 一律拒绝；armed=false、drill=true；不再用 PP/CSP 守卫替代 MIT |
| useMotorGateway.stopAll | 逐轴 stop，绕过 drill，和读轮询争用连接目标 | 移除发帧和自动恢复轮询；界面明确软件停止本批不可用 |
| useMotorGateway mount | 自动连接并 set_target / read_param / close_bus | 默认断开；只有明确点击并确认后可启动只读轮询；本轮未点击 |
| useGatewayBridge.setTargetFor | 设置目标后写 0x7026 并 set_active_report | 上游通用页/机械臂页的 WsGatewayClient 默认不能连接；最终 send 操作白名单拒绝所有写入和 payload.op 覆盖 |
| useRobotArmOps | ensure_mode、写寄存器、轨迹 sendCmd | 通过上述旧客户端隔离；不删源代码，不允许设备连接 |
| lib/motorStudioOps / useMotorControl | enable/stop/disable、mit/pos_vel/vel、set_zero_position、寄存器与扫描操作 | 同一旧客户端隔离；禁止白名单外全部操作，不只列出的几个 op |
| third_page/simu/wsBridge | 独立仿真 WebSocket，sim_* 操作 | 未增加 CAN 适配器，不作为真实执行入口 |
| rebot_sim / rebot_point | HTTP /command、动作程序、XYZ；没有 CAN import 或硬件 transport | hardware=false，真实目标拒绝；新 /config/* 只写软件账本，没有解锁端点 |
| rebot_ros_bridge | 读取仿真状态发布 ROS | 保持只读，不增加控制订阅 |
| rebot_twin_sync / rebot_can_readstate / verify_gateway_readonly | set_target、参数查询或原生 CAN 请求；即使名字为 readonly 也会发帧 | 本轮不运行 |
| rebot_motor_probe | 可选 ensure_mode + send_mit；默认值和参数可能实际驱动 | 本轮不运行；未来必须单独现场审批 |
| rebot_can_tx_probe / ack_test / loopback 等 | 原生 socket.send 或接口操作 | 本轮不运行，不把“探针”视为无副作用 |
| restart_sim.sh | 仿真进程启动，不运行 CAN 网关 | 支持独立输出目录；验证实例不触及主服务任务 |

重要边界：现有独立 Rust `ws_gateway` 服务及命令行探针并未被改写或卸载。本次前端隔离不是系统级 CAN 防火墙；其他旧标签页/第三方客户端/本机进程仍可能访问原网关。本服务不创建真实执行通道，软件配置接口拒绝跨站和公网隧道来源。后续接入须将总线所有权收归受控执行器，不能只删除网页 disabled。

## 测试分类与现场副作用

允许：配置单元测试、jsdom 前端测试、Pinocchio/MuJoCo 到点/动作/写字测试、本地隔离 HTTP 服务、界面截图。
不运行：任何 can_*、motor_probe、twin_sync、verify_gateway_readonly；本轮不改 can0、不启停网关。
已有主仿真服务与旧浏览器属于外部现场状态；本次记录“主动发起的硬件操作为零”，不声称全系统总线无流量。

## 回退

本分支提交可逐个 `git revert <提交>` 回退源代码，重新构建、重启仿真。不要 reset --hard、clean、覆盖 outputs 或恢复出厂设置。回退至旧界面会恢复历史写入入口，不能将旧版作为已经验收的真实控制器。私有配置不入 Git，回退代码不改变电机寄存器；恢复软件账本的方法见批次 A 交接文档。
