"""Verify one physics source feeds ROS actual/target states and matching TF.
Also prove stale/disconnected samples are not re-published as live measurements.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from rebot_client import RebotClient
from rebot_ros_bridge import FeedbackBridge

client=RebotClient()
state=client.state()
assert not state['job'] or state['job']['status'] not in ('running','paused')
client.control('reset')
rclpy.init()
node=rclpy.create_node('same_sim_bridge_verification')
actual=[];target=[];health=[]
node.create_subscription(JointState,'/rebot/sim/joint_states',actual.append,qos_profile_sensor_data)
node.create_subscription(JointState,'/rebot/sim/target_joint_states',target.append,qos_profile_sensor_data)
node.create_subscription(String,'/rebot/sim/status',lambda x:health.append(json.loads(x.data)),10)
buffer=Buffer();listener=TransformListener(buffer,node)
log=(ROOT/'outputs/ros_bridge_test.log').open('w')
p=subprocess.Popen(['ros2','launch',str(ROOT/'scripts/same_sim_ros.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
probe=None;server=None
try:
    deadline=time.monotonic()+20
    while len(actual)<10 and time.monotonic()<deadline and p.poll() is None:
        rclpy.spin_once(node,timeout_sec=.1)
    assert len(actual)>=10,'No actual-state ROS messages; inspect ros_bridge_test.log'
    q=client.state()['q_deg'];q_move=q.copy();q_move[0]+=12
    job=client.submit(dict(version=1,name='ROS 同源状态验证',steps=[
        dict(op='move_joints',q_deg=q_move,seconds=2),
        dict(op='wait',seconds=.3),
        dict(op='move_joints',q_deg=q,seconds=2),
        dict(op='wait',seconds=.3)]))
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if client.job(job['id'])['status'] not in ('running','paused'):break
    assert client.job(job['id'])['status']=='succeeded'
    deadline=time.monotonic()+.6
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.03)
    state=client.state()
    assert list(actual[-1].name)==[f'joint{i}' for i in range(1,7)]+['gripper_joint1','gripper_joint2']
    error=max(abs(a-b) for a,b in zip(actual[-1].position[:6],state['q_rad']))
    assert error<1e-4,error
    targets={(m.header.stamp.sec,m.header.stamp.nanosec):m for m in target}
    gaps=[]
    for msg in actual:
        desired=targets.get((msg.header.stamp.sec,msg.header.stamp.nanosec))
        if desired:gaps.append(max(abs(a-b) for a,b in zip(msg.position[:6],desired.position[:6])))
    assert gaps and max(gaps)>1e-4,'Actual and target streams appear to be identical commands'
    assert max(m.position[0] for m in actual)-min(m.position[0] for m in actual)>.15
    pose=buffer.lookup_transform('rebot_sim/base_link','rebot_sim/gripper_end',Time())
    pos=pose.transform.translation
    fk_error=math.dist([pos.x,pos.y,pos.z],state['xyz'])
    assert fk_error<1e-4,fk_error
    assert health[-1]['connected'] and not health[-1]['hardware_connected']

    # A controlled local source checks stale/disconnected behavior without
    # stopping the user's simulator or generating any robot commands.
    payload=state.copy()
    class Source(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            data=json.dumps(payload).encode();self.send_response(200);self.end_headers();self.wfile.write(data)
    server=ThreadingHTTPServer(('127.0.0.1',0),Source)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    probe=FeedbackBridge(f'http://127.0.0.1:{server.server_port}',prefix='/probe/sim')
    received=[]
    node.create_subscription(JointState,'/probe/sim/joint_states',received.append,qos_profile_sensor_data)
    deadline=time.monotonic()+1
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    payload['feedback_time_unix_s']=time.time();probe.poll()
    deadline=time.monotonic()+.3
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    assert received and probe.was_healthy
    count=len(received)
    payload['sequence']+=1;payload['feedback_time_unix_s']=time.time()-2
    probe.poll();assert probe.was_healthy is False
    deadline=time.monotonic()+.3
    while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
    assert len(received)==count,'Stale sample was published'
    server.shutdown();server.server_close();server=None
    probe.poll();assert probe.was_healthy is False
    report=dict(success=True,actual_samples=len(actual),target_samples=len(target),
                actual_http_max_error_rad=error,actual_target_max_difference_rad=max(gaps),
                tf_tcp_error_m=fk_error,stale_feedback_rejected=True,disconnect_reported=True,
                same_physics_source=True,hardware_connected=False)
    (ROOT/'outputs/ros_bridge_verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))
finally:
    if server:server.shutdown();server.server_close()
    if probe:probe.destroy_node()
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    log.close();node.destroy_node();rclpy.shutdown()
