# 批次 A 可复现验证

只在软件/仿真中运行；本次主动 CAN 连接、查询、写入、使能、运动均为 0。测试没有物理总线抓包，因此不声称其他进程也没有 CAN 流量。

在项目根目录，设置 `PY` 为仿真环境 Python、`NODE` 为已安装 Node 22.12+ 的路径：

```bash
python3 tests/test_config_protection.py
"$PY" tests/verify_point_planner.py
REBOT_OUTPUT_DIR=/tmp/rebot-batch-a-validation MUJOCO_GL=egl PYOPENGL_PLATFORM=egl "$PY" tests/verify_programs.py
REBOT_OUTPUT_DIR=/tmp/rebot-batch-a-validation MUJOCO_GL=egl PYOPENGL_PLATFORM=egl "$PY" tests/verify_writing.py
python3 tests/verify_config_http.py http://127.0.0.1:8765
REBOT_OUTPUT_DIR=/tmp/rebot-batch-a-validation python3 tests/verify_studio_routing.py http://127.0.0.1:8765
python3 scripts/setup_external.py --check
bash -n scripts/restart_sim.sh
python3 -m py_compile scripts/rebot_config.py scripts/rebot_config_restore.py scripts/rebot_sim.py scripts/rebot_point.py scripts/setup_external.py
cd external/motorbridge-studio
VITE_BASE_PATH=/studio/ "$NODE" node_modules/vite/bin/vite.js build
"$NODE" node_modules/vitest/vitest.mjs run
```

HTTP 测试只访问本地软件服务，不连接 CAN；服务需已启动。路由脚本会写报告，使用独立输出目录。配置单元测试使用临时目录。软件会话/审批/恢复的用例均使用模拟数据，不把示例增益升级为用户配置。

结果：后端 9 项通过、前端 21 文件/95 项通过、XYZ 误差 0.5596 mm、抓取和书写回归通过、HTTP 权限及路由通过。附日志为此次实际结果。ROS 实际节点、物理停止、实机动作、B1 伺服故障注入和全新电脑安装未执行。

浏览器人工交互通过 CUA 完成。测得三个视口 document scrollWidth 分别 1425/1351/375，均不超过设定的 1440/1366/390。桌面 XYZ/3D 并列，七卡存在，展开第二轴后第一轴折叠；临时服务草稿刷新保留。截图位于 ../../screenshots。旧界面没有启动采截图，防止其自动 CAN 查询。
