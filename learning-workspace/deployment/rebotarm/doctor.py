#!/usr/bin/env python3
"""Software-only checks: no CAN socket, physical driver or motor object."""
import json
from pathlib import Path
import importlib
import subprocess

import numpy as np
import mujoco
from ament_index_python.packages import get_package_share_directory
from rebotarmcontroller.twin_kinematics import TwinKinematics
from rebotarmcontroller.csp_runtime.rebot_seeed_transport import official_module

root = Path(__file__).resolve().parents[1]
for module in ['rclpy', 'sensor_msgs.msg', 'rebotarm_msgs.msg', 'motorbridge', 'ruckig', 'fastmcp', 'pinocchio']:
    importlib.import_module(module)
    print('PASS import', module)
for package in ['rebotarmcontroller', 'rebotarm_bringup', 'rebotarm_mujoco_rs', 'rosbridge_server', 'rebotarm_moveit_config']:
    get_package_share_directory(package)
    print('PASS ROS package', package)
sdk = official_module()  # verifies the locked commit; importing opens no CAN
print('PASS pinned SDK', sdk.__file__)
kinematics = TwinKinematics()
assert np.isfinite(kinematics.pose([0] * 6)).all()
scene = Path(get_package_share_directory('rebotarm_mujoco_rs')) / 'models/rs_grasp_scene.xml'
model = mujoco.MjModel.from_xml_path(str(scene))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
print('PASS FK / MuJoCo model', model.nq, model.nv)
program = json.loads((root / 'examples/writing/guoqing_kuaile.json').read_text())
assert program['robot_model'] == 'b601_rs' and program['steps']
subprocess.run(['git', '-C', str(root / 'rebotarm_ros2_RS/third_party/reBotArm_control_py_csp'),
                'diff', '--exit-code'], check=True)
print('PASS 国庆快乐示例', len(program['steps']), 'steps')
print('PASS 软件自检完成；未连接机械臂。')
