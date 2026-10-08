"""Verify repaired camera through actual ROS messages, with DISPLAY absent."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import String

ROOT=Path(__file__).resolve().parents[1]
model=ROOT/'external/reBotArmController_ROS2/src/rebotarm_mujoco_rs/models/rs_grasp_scene.xml'
env=os.environ.copy()
for key in ('DISPLAY','MUJOCO_GL','PYOPENGL_PLATFORM'): env.pop(key,None)
rclpy.init()
node=rclpy.create_node('camera_repair_verification')
frames=[];infos=[]
node.create_subscription(Image,'/camera_test/mujoco/overhead_rgb/image_raw',lambda m:frames.append((time.monotonic(),m)),qos_profile_sensor_data)
node.create_subscription(CameraInfo,'/camera_test/mujoco/overhead_rgb/camera_info',infos.append,qos_profile_sensor_data)
objpub=node.create_publisher(String,'/camera_test/mujoco/object_states',10)
jointpub=node.create_publisher(JointState,'/camera_test/mujoco/joint_states',qos_profile_sensor_data)
log=(ROOT/'outputs/ros_camera_test.log').open('w')
p=subprocess.Popen(['ros2','run','rebotarm_mujoco_rs','rs_scene_camera','--ros-args','-p','arm_namespace:=camera_test','-p',f'model_path:={model}'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    deadline=time.monotonic()+30
    while len(frames)<12 and time.monotonic()<deadline and p.poll() is None:
        rclpy.spin_once(node,timeout_sec=.1)
    assert len(frames)>=12,'No camera frames; see ros_camera_test.log'
    before=np.frombuffer(frames[-1][1].data,dtype=np.uint8).copy()
    count=len(frames)
    # Move red object in the scene state input; a live image must change.
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        objpub.publish(String(data=json.dumps(dict(objects=[dict(name='red_cube',position=[.40,-.09,.1225],quaternion=[1,0,0,0])]))))
        jointpub.publish(JointState(name=[f'joint{i}' for i in range(1,7)],position=[0.]*6))
        rclpy.spin_once(node,timeout_sec=.1)
    after=np.frombuffer(frames[-1][1].data,dtype=np.uint8)
    delta=float(np.mean(np.abs(after.astype(float)-before)))
    image=frames[-1][1]
    assert image.encoding=='rgb8' and image.width==320 and image.height==240
    assert infos and infos[-1].k[0]>0
    stamps={(m.header.stamp.sec,m.header.stamp.nanosec) for _,m in frames}
    assert (infos[-1].header.stamp.sec,infos[-1].header.stamp.nanosec) in stamps
    assert delta>.05 and len(frames)>count+3,(delta,len(frames))
    # PPM needs no extra image dependency in the system ROS Python environment.
    (ROOT/'outputs/ros_camera_verified.ppm').write_bytes(b'P6\n320 240\n255\n'+bytes(image.data))
    report=dict(success=True,display_unset=True,backend='egl',frames=len(frames),
                width=image.width,height=image.height,encoding=image.encoding,
                observed_hz=(len(frames)-1)/(frames[-1][0]-frames[0][0]),
                changed_object_image_mean_difference=delta,camera_info_received=len(infos))
    (ROOT/'outputs/ros_camera_verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
finally:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    log.close();node.destroy_node();rclpy.shutdown()
