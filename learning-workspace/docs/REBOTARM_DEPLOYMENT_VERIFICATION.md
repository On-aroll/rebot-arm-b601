# ROS 一键部署包验证记录

日期：2026-10-18（Asia/Shanghai）。目标：Ubuntu / Kubuntu 24.04 x86_64、ROS 2 Jazzy、Python 3.12。

## 交付

- `dist/rebotarm-b601-rs-1.0.0-ubuntu24.04-x86_64.run`：204,687,765 字节，约 195.2 MiB。
- 同名 `.tar.gz`、`REBOTARM_SHA256SUMS`、同名 `.manifest.json` 和 `ReBotArm-部署说明.md`。
- source_id：`2dcf272fcd6ee27a28e9b50418557af21995a40e4825eeb62d291c5b09b51f1a`，800 个源码及资源文件的 SHA256。

## 实际检查

| 检查 | 结果 |
| --- | --- |
| 两个对话与现行工程对应 | 已确认，完整当前源码包含未提交文件 |
| `.run` / `.tar.gz` SHA256 | 两项均 OK |
| 真正 `.run --dry-run`：解包、校验、显示目标机计划 | 通过 |
| 损坏包拒绝、目标目录保护、失败保留 current / 配置 / 恢复记录 | 新增 5 项部署测试通过；含原包合计 15 项通过 |
| 新目标路径、新 Python 环境，原生依赖安装与 colcon | 7 个 ROS 包全部编译完成，约 18.3 秒 |
| 软件 doctor | Node WebSocket/fetch、ROS、锁定 SDK、FK、MuJoCo 场景及 182 步书写示例通过 |
| 网页回归 | 100 / 100 通过 |
| 控制器完整回归 | 首轮 322 通过、1 个时序用例在停止后等待 READONLY 超时；单独重跑该用例的 3 个参数场景，3 / 3 通过 |
| Fake ROS DOMAIN 82 | 25 / 25，通过程序、算法、暂停/停止、过期拒绝等链路 |
| 真正 MuJoCo + Fake，DOMAIN 83 | 物理仿真、两路场景相机和任务节点启动，收到 121 帧 MuJoCo 关节和 10 帧有效 twin 状态 |
| HTTP 实际服务 | 首页、启动台、脚本、Three.js、URDF、STL、书写程序共 7 个资源返回 200 |
| 活动版本升级保护 | 仿真运行时安装器拒绝更新，保留原版本 |
| 桌面 / 应用菜单入口 | 在隔离的临时用户 HOME 下生成，指向正确 current，无旧电脑路径 |

## 修复的部署问题

1. 默认系统 Node 18 无法提供当前网页所需的全局 WebSocket：随包固定 Node 22.23.2。
2. 原 CSP SDK 依赖开发机 Git checkout：打包单提交、无 remotes 的干净 SDK 仓库，保留原提交与工作树检查。
3. 源码固定层级推导在 colcon 安装目录下失效：改为向上发现目标工作空间，并给网关使用实际项目根目录。
4. 原 setuptools 84 不满足 colcon-core 的 `<80` 要求：锁定 79.0.1，重新编译通过。

## 限制和测试条件

本机为 Ubuntu 24.04 x86_64，已有 ROS / 编译系统包。验证时使用 `--skip-system`，没有执行 sudo apt 安装。由于本机未安装系统 ensurepip，验证用 uv 在最终绝对路径创建并 seed 一个全新的系统 Python 3.12 venv；之后依赖安装、编译、自检由正式安装器完成。正式安装器会先 apt 安装 python3-venv。

因此已验证“新路径、独立环境、完整源码”的重建，不等于已在另一台全新电脑验证系统包安装。安装器仍需要互联网和目标机的 sudo 授权。

首次 MuJoCo 探针误用了 reliable QoS，未收到 best-effort 关节帧；改用 ROS sensor-data QoS 后通过。这是验证探针的修正，产品代码未因此改变。首次控制器回归与仿真/ROS 验证有并行负载，不能据单次超时确定根因；原始失败与单独复验日志均保留。

本轮没有连接真实 CAN、准备或使能实机、运动电机、修改电机寄存器或恢复旧机械臂记录。真实 MIT、手柄/J7、相机接入、长时运行及原有跟随滞后/颤动仍保持各自现场待验收状态。

原始验证文件在 `.state/rebotarm-validation/`：`install-final.log`、`node-tests.log`、`controller-tests.log`、`controller-retry.log`、`fake-ros-tests.log`、`mujoco-start-final.json`、`web-smoke.json`、`live-upgrade-rejection.log`、`single-file-final.log`。
