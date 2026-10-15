# WsGatewayClient 只读兼容审查（2026-10-09）

批次 A 在 `connect()` 层拒绝旧客户端，导致旧版 Studio 的设备连接一起失效。本次将限制放回具体操作：**允许明确的只读连接和查询，全部真实写入仍拒绝**。本轮没有连接真实网关做验收。

## 变更与范围

- 所有 `WsGatewayClient` 实例都可以由用户手动连接，包括旧页面默认构造的实例。不再用 `readOnly` 构造开关区分新旧页面；改它也不能获得写权限。
- 精确白名单：`capabilities`、`set_target`、`close_bus`、`robstride_ping`、`robstride_read_param`、`get_register_u32`、`get_register_f32`。payload 不能覆盖 `op/req_id`，包括经过 `toJSON` 序列化后尝试覆盖。
- `useGatewayBridge.setTargetFor()` 仅改变网关查询目标；移除隐式写 `0x7026`、`set_active_report`、流式读取初始化。这里的 target 是网关路由目标，不是机械臂运动位置。
- mount/刷新不连接；断线不自动重连。旧 socket 的迟到消息/关闭事件不会污染新连接；断开和卸载拒绝未完成请求。读超时不自动重发。
- RS 刷新改为明确读 `0x7019/701A/701B/701C/7005`；在线检测读 `0x7019`。保留原始多圈位置，位置失败即未知/离线；可选字段失败也不沿用历史速度、模式。
- DM 恢复显式寄存器诊断与 `CTRL_MODE` 在线查询。寄存器读取不能证明实际位置/速度，诊断刷新将这些字段置为未知。
- `scan/verify/state_once/streams` 和未经审核的厂家操作仍拒绝；其他厂家的完整状态刷新尚未恢复。本次不声称恢复所有厂家的所有功能。旧页面可能仍显示写入按钮，但调用必定得到 `LOCKED`。

## 静态来源核对

安装包元数据为 `motorbridge 0.5.6`。本次只读取官方上游源码（工作区 Cargo 版本亦为 0.5.6），没有运行 SDK/网关：

- [router/mod.rs](https://github.com/motorbridge/motorbridge/blob/main/integrations/ws_gateway/src/router/mod.rs)：新会话 standby，流默认关闭，结束走 `disconnect(false)`。
- [connection.rs](https://github.com/motorbridge/motorbridge/blob/main/integrations/ws_gateway/src/router/handlers/connection.rs)：`capabilities` 返回元数据；`set_target` 替换会话路由；`close_bus` 走 `disconnect(false)`。
- [register.rs](https://github.com/motorbridge/motorbridge/blob/main/integrations/ws_gateway/src/router/handlers/register.rs)：读寄存器、RS ping 和读参数有独立分支，与写入分支分离。
- [session/connect.rs](https://github.com/motorbridge/motorbridge/blob/main/integrations/ws_gateway/src/session/connect.rs)：RS/DM 的 `disconnect(false)` 关闭总线句柄，不选择 shutdown 分支。

这是静态协议核对，不证明已安装二进制和当前上游逐字节一致，也不是新一次实机只读验收。以后更新网关需要重新核对上述约定。可重复测试使用内存 WebSocket 替身，覆盖手动连接、token、路由选择、全部允许操作、拒绝写入、协议覆盖、超时与旧连接隔离。

## 边界与剩余风险

前端白名单不是操作系统 CAN 防火墙。独立 Rust 网关、历史已加载页面、其他本机客户端/探针仍可能绕过此客户端。本项目 HTTP 与 B1 执行器均没有真实 transport，但尚未给原网关增加系统级只读权限模型。未来实机执行前，需要统一总线所有权与服务端能力授权，且读轮询不得与控制循环并行抢占总线。本次不启动、停止或重新配置任何硬件网关。
