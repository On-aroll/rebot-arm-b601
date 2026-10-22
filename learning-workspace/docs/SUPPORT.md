# Linux 兼容范围

## 共同要求

Linux **x86_64 + glibc >= 2.28**、普通用户、Bash、网络访问。Release 下载器额外需要系统 Python 3.10+ 和 curl（只在下载入口用到）。不要求系统 Python 恰好为 3.12，安装器会创建独立的 Python 3.12.12。

| 系统 | 系统依赖适配 | 验证状态 |
| --- | --- | --- |
| Kubuntu / Ubuntu 24.04 | apt | 本机 CPU 软件安装、训练和推理已通过；全新系统的 apt 步骤未实测 |
| Ubuntu 22.04 | apt | 已配置 GitHub CI，等待仓库执行；本机未实测 |
| Debian 12 / 13 | apt | 已做包名适配，未实测 |
| Ubuntu / Debian 衍生版，如 Mint、Pop!_OS、KDE neon | 依据 ID_LIKE 使用 apt | 未实测；内核和厂商驱动另行检查 |
| Fedora | dnf | 已做包名适配，未实测；不自动添加第三方软件源 |
| Arch / Manjaro | pacman | 已做包名适配，未实测；先按发行版文档完成系统更新 |
| 其他 glibc x86_64 发行版 | 管理员预装依赖，使用 --skip-system | 试验性，安装末尾自检判断当前软件链路是否可用 |
| ARM / aarch64、Jetson、树莓派、32 位 Linux | 不支持 | 需要新的原生插件 wheel、依赖锁和测试 |
| Alpine / musl | 不支持 | 当前二进制依赖使用 glibc |

“包名适配”只说明安装器知道如何请求系统依赖。只有对应系统实际安装并通过自检后，才能记录为已验证。当前没有在这些发行版上安装显卡驱动或连接机械臂。

## 手工预装依赖

其他系统可由管理员提供：Python 3.10+、C/C++ 编译器、make、CA 根证书、FFmpeg、GLib、OpenGL/EGL、X11/XTest、xkbcommon-x11、libusb、usbutils、v4l-utils、can-utils、iproute/iproute2。

随后使用 `bash install.sh --skip-system`，或给 Release 安装器添加 `--skip-system`。此模式不修改系统包或用户组；管理员需要另行设置串口、摄像头权限。Arch 系串口通常使用 `uucp` 组，Debian/Fedora 通常使用 `dialout`，实际以设备权限为准。

## GPU 与硬件

- 默认 CPU。CUDA 可选包需要 NVIDIA 显卡和支持 CUDA 12.8 的驱动；安装器不会安装驱动。AMD/Intel GPU 尚无训练加速方案。
- 只有软件自检可以脱离设备运行。实体任务仍需要支持的主从臂、USB-CAN 驱动、摄像头、校准和示范数据。
- 键盘录制需要桌面终端。Wayland 下应实际验证快捷键；必要时使用已安装的 X11 桌面会话。
- Release 是小型联网部署包；安装后环境可达数 GB，训练数据和检查点另占空间。
