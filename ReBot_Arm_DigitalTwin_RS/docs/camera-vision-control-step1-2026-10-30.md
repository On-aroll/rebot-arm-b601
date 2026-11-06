# 相机视觉控制路线第一步（2026-10-30）

## 本轮目标

先把真实 Gemini 336L 彩色帧接入现有 ROS 控制台，并把仿真真值与真实相机观测区分开。当前只完成只读预览链，不向机械臂发送动作。

## 已核对

- `/dev/v4l/by-id/` 识别到 Orbbec Gemini 336L；UVC `video-index0` 支持 YUYV 和 MJPEG，采样为 640×480。`video-index2` 是 GREY/NV12，`video-index4` 是 Bayer/YV12，不能把它们当作已标定深度图。
- 本机没有安装 Orbbec ROS 驱动或 `pyorbbecsdk`。当前仅有已安装 OpenCV 的彩色 UVC 读取路径；首次采样亮度均值约 12，场景较暗，实际识别效果尚未通过。
- MuJoCo `/rebotarm_rs/mujoco/overhead_rgb/image_raw` 和 `/rebotarm_rs/mujoco/wrist_rgb/image_raw` 均有图像帧。`/rebotarm_rs/vision/color_blocks/detections` 来自 `mujoco/object_states`，消息标注 `rs_mujoco_object_states`，属于仿真真值，不是图像识别。
- 现有工作台 ROS 压缩相机入口使用 `/camera/color/image_raw/compressed`，与新桥接默认话题一致；视觉工作台现有算法/标定/预览路径仍需本机真实标定才能生成可靠 `base_link` 三维目标。

## 实现

- `scripts/orbbec_ros_camera_bridge.py` 从 Gemini 336L 稳定 `video-index0` 自动选择器读取彩色帧，以 15 Hz、JPEG quality 80 发布 `sensor_msgs/CompressedImage`，同时发布全零内参的 `CameraInfo`，明确表示设备内参尚未标定。
- `REBOTARM_START_ORBBEC_BRIDGE=true` 可让 `start_rs_sim.sh` 在仿真命名空间之外启动该只读相机源。相机进程使用系统 Python/OpenCV，避免主项目 NumPy 2 虚拟环境与 Ubuntu OpenCV ABI 冲突。
- 控制台把检测来源显示出来；真机视觉动作现在要求明确的 RGB-D 标定来源、已验证 calibration ID、`base_link` 三维坐标、置信度至少 0.8、图像年龄不超过 250 ms、图像与机器人反馈时差不超过 100 ms。仿真模式继续允许 MuJoCo 真值抓取。

## 未完成的闭环门槛

1. 需要检查真实画面亮度/遮挡并获得稳定彩色帧。
2. Gemini 深度流和 ROS 驱动尚未接入；RGB-only 检测只提供像素坐标，不能转换为机械臂目标。
3. 需要用标定板验证内参和固定相机到 `base_link` 外参，并记录标定误差与设备序列号。
4. 需要让真实视觉结果携带曝光/采集时间戳，并与机器人反馈时钟同步；仅发布时刻不等于曝光时刻。
5. 完成视觉目标进入现有路径预览、碰撞/重力检查和唯一 CSP 执行器的离线回归之后，才进入另一次现场授权的受控小步验收。

## 本机运行

```bash
REBOTARM_START_ORBBEC_BRIDGE=true ./scripts/start_rs_sim.sh
```

如果安全仿真已经运行，可在同一机器另开终端执行：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
/usr/bin/python3 scripts/orbbec_ros_camera_bridge.py
```

工作台在“实机观察画面”选择“ROS 压缩图像”，默认订阅 `/camera/color/image_raw/compressed`。本地浏览器可以连接相机画面；如果网页运行于隔离的远程浏览器容器，则该容器无法枚举本机 USB 设备或访问本机 ROSBridge。
