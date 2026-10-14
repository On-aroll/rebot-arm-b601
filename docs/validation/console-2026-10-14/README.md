# 控制台与使用手册的软件交付验收

日期：2026-10-14。分支：`codex/mit-console-protection`。当前批次包括此前未提交的 ROS Fake 接入、启动器、七轴统一控制台、双模式 XYZ、初始状态检测，以及 [项目使用手册](../../项目使用手册.md)。已有 SDK/Mock/保护层作为原工程资产保留；没有合并 main。

## 本次复验

| 检查 | 结果 | 原始输出 |
| --- | --- | --- |
| 前端全套（虚拟 WebSocket/合成反馈） | 34 文件、188 项通过 | [frontend-tests.txt](frontend-tests.txt) |
| Python 离线单元检查 | 140 项通过，不打开真实 CAN | [offline-tests.txt](offline-tests.txt) |
| 生产前端构建 | 通过，保留原有大包提示 | [frontend-build.txt](frontend-build.txt) |
| MuJoCo XYZ 规划、执行、拒绝路径 | 通过；本次理想场景到位误差约 0.5596 mm | [point-planner.txt](point-planner.txt) |
| HTTP 路由、模型资源、路径穿越和只读端点 | 通过 | [routes.txt](routes.txt)、[JSON](studio_routing_verification.json) |
| 新启动脚本语法、Python 编译、文档本地链接、差异格式 | 通过 | [汇总](report.json) |

先前同日浏览器离线 XYZ `[200,150,200]` 和虚拟停止已验证，手册附截图。该理想 Fake FK 误差和本次 MuJoCo 误差来自不同执行器，不能混为真实精度；本次发布复验没有重新进行该浏览器运动，也没有实机指令。

从项目根目录复验：

```bash
# 使用已有项目 Python 环境，或替换成 .venv-sim/bin/python
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl ~/.local/share/robotics/envs/rebot-sim/bin/python -m unittest discover -s tests -p 'test_*.py'
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl ~/.local/share/robotics/envs/rebot-sim/bin/python tests/verify_point_planner.py
~/.local/share/robotics/envs/rebot-sim/bin/python tests/verify_studio_routing.py

cd external/motorbridge-studio
npm test
VITE_BASE_PATH=/studio/ npm run build
```

`verify_studio_routing.py` 需要已经运行的 8765 服务，但只读；单元检查和 PointPlanner 检查不需要运行服务。不要将 `verify_gateway_readonly.py` 或真实 trial 脚本混进离线复验。

## 当前物理边界

本次真实 CAN 请求、设备参数写入与真实运动均为 0；3/1/0、零位、限位和既有调定值未改。网页电源回执不能证明实际出力，初始检测不能自动建立物理零位、范围或承重能力。真实原位保持、小幅运动、停止、六轴 XYZ、通信时延、J7 和额外工具 TCP 尚待现场验证。新电脑仍需按手册安装系统 ROS、图形与 Python/Node 依赖，未宣称空白电脑或 Windows/macOS 全流程验收。

私有草稿、局部设备快照、启动日志、PID、依赖环境与生成的 `dist` 不纳入本次软件提交；网页源代码、重建清单、使用手册、筛选后的测试证据随仓库上传。
