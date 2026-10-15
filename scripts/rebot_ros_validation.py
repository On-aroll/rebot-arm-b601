#!/usr/bin/env python3
"""Isolated ROS 2 integration, upstream FakeRsDriver + ArmActions. NO CAN.

Fixed localhost-only domain/namespace, not configurable as a real motor endpoint.
Fake kinematics do not certify MIT gains, dynamics, timing or physical stops.
"""
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ['ROS_DOMAIN_ID']='43'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rebotarmcontroller.fake_rs_driver import FakeRsDriver, _JOINT_LIMITS
from rebotarmcontroller.ros_actions import ArmActions

NS='rebotarm_validation'

class FakeHardware:
    def __init__(self,node):
        self.node=node; self._cmd_lock=node._leader_lock
        self.joint_names=node.joint_names; self.teleop_owned=False; self.cancelled=False
    @property
    def state_machine(self): return self.node.state_machine
    def set_state_machine(self,value): self.node.state_machine=value
    def get_joint_positions(self):
        with self._cmd_lock: return np.array(self.node.positions)
    def get_joint_velocities(self):
        with self._cmd_lock: return np.array(self.node.velocities)
    def begin_trajectory_stream(self):
        with self._cmd_lock:
            if not self.node.enabled or self.cancelled: raise ValueError('虚拟电机未使能或任务已取消')
            self.node.state_machine='TRAJ_RUNNING'
    def update_trajectory_reference(self,q,v):
        with self._cmd_lock:
            if self.cancelled or not self.node.enabled: raise ValueError('任务已停止；拒绝继续旧目标')
            if len(q)!=6 or not all(math.isfinite(float(x)) and lo<=x<=hi for x,(lo,hi) in zip(q,_JOINT_LIMITS)):
                raise ValueError('目标越界或非有限值，未静默裁剪')
            self.node.targets=list(map(float,q))
    def hold_current_position(self):
        with self._cmd_lock: self.node.targets=list(self.node.positions)

class ValidationDriver(FakeRsDriver):
    def __init__(self):
        super().__init__()
        self.reentrant_group=ReentrantCallbackGroup()
        self.publish_arm_status=self.publish_status
        self.hardware=FakeHardware(self)
        # Reuse only the upstream trajectory action; no pose/gripper/control driver.
        self.actions=ArmActions.__new__(ArmActions)
        self.actions._node=self; self.actions._hardware=self.hardware; self.actions._namespace=NS
        self.action_server=ActionServer(self,FollowJointTrajectory,f'/{NS}/follow_joint_trajectory',
            execute_callback=self.actions.execute_follow_joint_trajectory,
            goal_callback=self.goal_check,cancel_callback=self.actions.cancel_callback,
            callback_group=self.reentrant_group)
        self.client=ActionClient(self,FollowJointTrajectory,f'/{NS}/follow_joint_trajectory',callback_group=self.reentrant_group)
        self.job=dict(status='idle'); self.action_handle=None; self.job_lock=threading.RLock(); self.last_feedback=0.
        self.tick_intervals=[]; self.previous_tick=None
    def goal_check(self,goal):
        if not self.enabled or self.hardware.cancelled: return GoalResponse.REJECT
        if list(goal.trajectory.joint_names)!=self.joint_names: return GoalResponse.REJECT
        return self.actions.arm_goal_callback(goal)
    def _disable(self,request,response):
        # Upstream Fake disable teleports home; integration deliberately holds.
        with self._leader_lock:
            self.targets=list(self.positions); self.enabled=False
            self.state_machine='IDLE'
        response.success=True; response.message='virtual disabled without homing'
        return response
    def _tick(self):
        now=time.monotonic()
        if self.previous_tick is not None: self.tick_intervals.append(now-self.previous_tick)
        self.tick_intervals=self.tick_intervals[-1000:]; self.previous_tick=now
        super()._tick(); self.last_feedback=time.time()
    def status(self):
        with self._leader_lock, self.job_lock:
            return dict(backend='ros2-fake',namespace=NS,domain=43,online=True,fresh=time.time()-self.last_feedback<1,
                enabled=self.enabled,q_rad=list(self.positions),qd_rad_s=list(self.velocities),
                feedback_time=self.last_feedback,job=dict(self.job),hardware=False,physical_acceptance=False,
                upstream='Yang-Ci/ReBot_Arm_DigitalTwin_RS@aa807cf2affe85b13b86221159871aae7742c01b',
                scheduling=dict(samples=len(self.tick_intervals),mean_s=float(np.mean(self.tick_intervals)) if self.tick_intervals else None,max_s=max(self.tick_intervals,default=None)))
    def stop(self):
        self.hardware.cancelled=True; self.hardware.hold_current_position()
        with self.job_lock:
            if self.action_handle: self.action_handle.cancel_goal_async()
            if self.job['status'] in ('starting','running'): self.job['status']='cancelling'
        return self.status()
    def execute(self,body):
        start=body.get('start'); target=body.get('target'); seconds=body.get('seconds')
        for q in (start,target):
            if not isinstance(q,list) or len(q)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) or not lo<=v<=hi for v,(lo,hi) in zip(q,_JOINT_LIMITS)):
                raise ValueError('需要模型范围内六轴有限目标')
        if type(seconds) not in (int,float) or not math.isfinite(seconds) or not 1<=seconds<=60: raise ValueError('轨迹时间无效')
        with self._leader_lock,self.job_lock:
            if not self.enabled or time.time()-self.last_feedback>1: raise ValueError('请先使能虚拟电机并取得新鲜反馈')
            if self.job['status'] in ('starting','running','cancelling'): raise ValueError('已有 ROS 任务')
            if max(abs(a-b) for a,b in zip(start,self.positions))>math.radians(.5): raise ValueError('ROS 起点已改变，请先同步仿真并重新预览')
            self.hardware.cancelled=False; self.job=dict(status='starting',target=target)
        goal=FollowJointTrajectory.Goal(); goal.trajectory.joint_names=self.joint_names
        for q,t in ((start,0.),(target,float(seconds))):
            p=JointTrajectoryPoint();p.positions=list(map(float,q));p.time_from_start.sec=int(t);p.time_from_start.nanosec=int((t-int(t))*1e9);goal.trajectory.points.append(p)
        future=self.client.send_goal_async(goal)
        def accepted(f):
            try:
                handle=f.result()
                with self.job_lock:
                    if not handle.accepted: self.job.update(status='failed',error='ROS Action 拒绝目标'); return
                    self.action_handle=handle
                    if self.hardware.cancelled: self.action_handle.cancel_goal_async()
                    else: self.job['status']='running'
                handle.get_result_async().add_done_callback(done)
            except Exception as e:
                with self.job_lock: self.job.update(status='failed',error=str(e))
        def done(f):
            try:
                result=f.result(); actual=self.hardware.get_joint_positions()
                error=float(np.max(np.abs(actual-np.array(target))))
                with self.job_lock:
                    self.job.update(status='cancelled' if self.hardware.cancelled or result.status==5 else 'succeeded' if result.status==4 and error<.003 else 'failed',
                        error=result.result.error_string,max_error_rad=error,ros_status=result.status)
                    self.action_handle=None
            except Exception as e:
                with self.job_lock: self.job.update(status='failed',error=str(e))
        future.add_done_callback(accepted)
        return self.status()


def main():
    rclpy.init(args=['--ros-args','-p','arm_namespace:='+NS,'-p','start_enabled:=false','-p','max_joint_speed:=0.3'])
    node=ValidationDriver(); executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    threading.Thread(target=executor.spin,daemon=True).start()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def reply(self,data,code=200):
            b=json.dumps(data,allow_nan=False).encode();self.send_response(code);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
        def do_GET(self): self.reply(node.status()) if self.path=='/state' else self.reply({'error':'not found'},404)
        def do_POST(self):
            try:
                if self.client_address[0]!='127.0.0.1' or self.headers.get('Origin') not in (None,'http://127.0.0.1:8767'): raise ValueError('local bridge only')
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<16000: raise ValueError('invalid payload')
                b=json.loads(self.rfile.read(n));action=b.get('action')
                if action=='execute': return self.reply(node.execute(b))
                if action=='stop': return self.reply(node.stop())
                if action=='enable':
                    with node._leader_lock: node.targets=list(node.positions);node.enabled=True
                elif action=='disable':
                    node.stop()
                    with node._leader_lock: node.enabled=False
                elif action!='connect': raise ValueError('unknown action')
                self.reply(node.status())
            except Exception as e:self.reply(dict(error=str(e)),400)
    server=ThreadingHTTPServer(('127.0.0.1',8767),Handler)
    try: server.serve_forever()
    finally:node.stop();server.server_close();executor.shutdown();node.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
