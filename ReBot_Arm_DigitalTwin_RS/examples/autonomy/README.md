# 指定点抓放：离线候选原型

先阅读 [调研与实施路线](../../docs/research/自主夹取与电弧焊调研_2026-10-19.md)。

新增工具：[离线视觉回放与 J7 采样说明](../../docs/离线视觉回放与J7采样_2026-10-19.md)。`replay_official_vision.py` 在独立视觉环境执行官方 OBB 几何算法，输入可用 `fixtures/synthetic-rgbd.json`；不会运行 YOLO 或控制机器人。`fetch_official_geometry.py` 可重新下载三个锁定并校验哈希的官方纯算法文件。其四项测试按该说明在独立环境运行，下面六项是原指定点候选原型测试。

这不是实机自动夹取按钮。脚本没有 ROS/CAN/SDK 调用，只生成独立格式的候选数据，现有动作程序导入器会拒绝该格式。

## 运行

在本工程根目录：

```bash
python3 -m unittest discover -s examples/autonomy -p 'test_fixed_pick.py' -v
python3 examples/autonomy/fixed_pick.py examples/autonomy/fixed-pick.example.json --output /tmp/b601-fixed-pick-proposal.json
```

输出文件必须不存在，避免覆盖已有结果/配置。相同输出路径再次运行会拒绝覆盖。

## 输入与输出

- 输入内部单位固定为 m/rad/s，姿态为与原 `TwinKinematics` 一致的 XYZ/RPY。
- 输入目标是原生 `gripper_end` 位姿；物体中心、两指中心或新工具尖端需要先通过已测变换换算。
- 示例来自模型关节 `[0.7,0.3,0.4,-0.1,0.2,0.2]` 的 FK，放置点做 5mm 的 X/Y 偏移；不是现场测得的抓取点。示例夹爪角度未经实物标定。
- 工作区只约束候选 TCP 的 XYZ，不表示整臂、工具、负载已检查。
- 接近/撤离沿基座 Z 采样，转姿只给端点；不保证实际关节插值产生直线。
- 输出含接近、闭合、夹持/附着验证、搬运、释放/分离验证和撤离。验证阶段必须由后续系统实现，脚本不会自动跳过。
- 现阶段没有 IK、环境碰撞、动力学、夹持传感、实际时序和焊机控制；不能从输出手工抽取若干位姿就当成合格运动程序。

## 本轮验证

6 项 unittest（含多组非法输入），以及示例全部 28 个 TCP 候选的原生运动学检查。证据保存在 `outputs/autonomy-research-20261018/`。运动学残差不是实际夹爪与物体的定位误差，也不是全路径可执行证明。

后续集成时复用主工程执行器、预览、租约、参数快照/恢复及现场许可，并先实现夹持/释放条件动作与完整场景校验。
