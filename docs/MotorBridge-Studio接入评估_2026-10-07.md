# MotorBridge Studio 接入评估

核对日期：2026-10-07。用户指定入口：https://motorbridge.github.io/motorbridge-studio/

## 本次核对范围

- 在线首页实际加载，页面版本 v1.1.9，WebSocket disconnected，默认地址 ws://127.0.0.1:9002。
- 点击 reBot Arm 后浏览器读取连续超时，未完成三维界面的交互验证；不能据此认定网页本身损坏。
- 已下载官方源码到 external/motorbridge-studio，版本 v1.1.9，commit 23b6e62ab8173faee3928c827cca3b7f42ba2128。
- 未安装依赖、未启动网关、未连接或使能实机、未写电机参数。

## 已确认能力与边界

1. React + Vite 前端，通过 motorbridge 的 ws_gateway 访问电机；网页本身不直接访问 SocketCAN。
2. General 页面支持多厂商扫描和电机卡片，包含 RobStride。reBot Arm 有 damiao 和 robstride 配置，关节 ID 1–7。
3. ArmUrdfViewer 使用 Three.js 与 URDFLoader，支持模型、末端轨迹、导入导出与回放。RS 配置使用 arm03/reBot_Lite_RS_with_gripper，末端 gripper_end；应与本机 B601-RS 模型核对几何、零位、方向和夹爪映射。
4. RobotArmPage 构造 jointTargets 时读取 row.control.target。因此此处三维运动不能直接证明实机到达相同位置。需要独立真实反馈通道。
5. TrailManager 的 replayImportedTrail 只触发前端回放计数，没有在该路径下发实机轨迹。SequenceManager 另有逐轴示例动作，并可能扫描后使能；不能将示例当作已经验证的六轴书写执行器。
6. 独立 /simu 页面具备点位编辑、IK 可达性验证、执行顺序、轨迹显示和停止接口。其协议文档指定另一个 motorbridge-arm 后端，默认 ws://127.0.0.1:9011/ws。
7. 当前 /simu 默认 arm02 DM 模型，不能直接套给 RS 机械臂。跟随 backend joint_targets 也不自动等于跟随真实电机反馈。
8. 前端源码中未发现 gamepad/getGamepads 接入；手柄功能需补充或另行验证后端方案。

## 推荐接入方式

以 Studio 的本地版本作为后续主界面，在本项目内适配；官方托管页面可用于标准调试，但直接跳转不能加载本地定制任务功能。

- 前端：复用 Studio 三维场景、设备调试和轨迹视图，添加“国庆快乐”任务入口及实际反馈显示。
- 模型：核对并接入本项目已验证的 RS 模型、坐标定义与工具尖端。
- 后端：保留现有书写轨迹生成、仿真预检；接入任务执行与真实回读。普通电机网关和 /simu 网关是不同接口，不可混接。
- 数据：分别显示计划轨迹、仿真结果与真实反馈轨迹；真实姿态不可用目标值冒充。
- 控制：由统一执行层协调运动，避免 Studio 手动操作与任务程序同时下发；验证停止、失联和连续角度处理。

实施顺序：本地界面与 RS 模型 → 只读实机反馈 → 书写轨迹导入/预演 → 小范围实机控制验证 → 完整同步书写。首先空中书写，不涉及挤出打印。

## 参考

- https://github.com/motorbridge/motorbridge-studio
- external/motorbridge-studio/src/components/RobotArmPage.jsx
- external/motorbridge-studio/src/components/ArmUrdfViewer.jsx
- external/motorbridge-studio/src/components/robot-arm/managers/TrailManager.jsx
- external/motorbridge-studio/src/components/robot-arm/managers/SequenceManager.jsx
- external/motorbridge-studio/src/third_page/simu/WS_PROTOCOL.md

这里只完成了认识与评估，尚未迁移现有界面或接通控制链路。
