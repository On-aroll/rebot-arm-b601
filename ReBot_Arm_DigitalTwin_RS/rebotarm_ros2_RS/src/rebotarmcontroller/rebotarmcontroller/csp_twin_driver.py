"""ROS front end for the bounded native-CSP hardware twin."""
import os
import faulthandler
import signal
import threading
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from rebotarm_msgs.msg import ArmStatus
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

from .csp_hardware import CspHardware
from .twin_control import TwinControl
from .twin_kinematics import TwinKinematics
from .csp_runtime.gateway_lease import GatewayLease
from .csp_runtime.rebot_native_xyz import TORQUE, gravity_limits_for_mode


class CheckedKinematics(TwinKinematics):
    def __init__(self):
        super().__init__(position_tolerance=.00015,rotation_tolerance=.001,start_tolerance=np.deg2rad(1))
        self.lower[0], self.upper[0] = -np.deg2rad(150),np.deg2rad(150)
        # Preview and CAN references must not wait for each other's model data.
        # Models are immutable; Pinocchio/MuJoCo scratch data belongs to a thread.
        self.path_data = threading.local()
        scene = Path(get_package_share_directory("rebotarm_mujoco_rs"))/"models/rs_grasp_scene.xml"
        self.collision_model = mujoco.MjModel.from_xml_path(str(scene))
        self.qadr = [self.collision_model.jnt_qposadr[self.collision_model.joint(f"joint{i}").id] for i in range(1,7)]
        base = self.collision_model.body("base_link").id
        self.robot_bodies = {base}
        for body in range(base+1,self.collision_model.nbody):
            if self.collision_model.body_parentid[body] in self.robot_bodies:
                self.robot_bodies.add(body)

    def check_path(self, start, goal, *, gravity_limits=TORQUE):
        if not hasattr(self.path_data, 'gravity'):
            self.path_data.gravity = self.model.createData()
            self.path_data.collision = mujoco.MjData(self.collision_model)
        self._check_path(start,goal,gravity_limits)

    def check_support_recovery_path(self, start, goal, plan, stage, limits):
        self.check_path(start, goal, gravity_limits=limits)

    def _check_path(self, start, goal, gravity_limits=TORQUE):
        # Keep a <=1 degree sampling interval for long full-range previews.
        # Runtime targets are also checked each cycle; this is a sampled self
        # collision check, not a continuous/environment collision planner.
        steps = max(7,int(np.ceil(np.max(np.abs(np.asarray(goal)-start))/np.deg2rad(1)))+1)
        for blend in np.linspace(0,1,steps):
            q = np.asarray(start)+(np.asarray(goal)-start)*blend
            full = np.zeros(self.model.nq);full[self.indices] = q
            gravity = pin.computeGeneralizedGravity(self.model,self.path_data.gravity,full)[self.vel_indices]
            if not np.all(np.isfinite(gravity)):
                raise ValueError('模型重力计算无效，未发送运动')
            exceeded = np.flatnonzero(np.abs(gravity) > gravity_limits)
            if len(exceeded):
                axis = int(exceeded[0])
                raise ValueError(f'模型重力超过本次 CSP 力矩包络：J{axis+1} 估算 {abs(gravity[axis]):.2f} N·m > '
                    f'允许 {gravity_limits[axis]:.2f} N·m（路径约 {blend*100:.0f}%）。请收拢姿态或调整目标。')
            self.path_data.collision.qpos[self.qadr] = q
            mujoco.mj_forward(self.collision_model,self.path_data.collision)
            for contact in self.path_data.collision.contact:
                bodies = self.collision_model.geom_bodyid[[contact.geom1,contact.geom2]]
                if contact.dist < -1e-5 and all(int(b) in self.robot_bodies for b in bodies):
                    raise ValueError("模型检测到机械臂自身碰撞，未发送该目标")


class CspTwinDriver(Node):
    def __init__(self):
        super().__init__("rebotarm_csp_twin")
        self.hardware = None
        self.kinematics = CheckedKinematics()
        self.lease = GatewayLease(Path(os.environ.get("REBOTARM_GATEWAY_ROOT","/home/bonvoyage/Desktop/seeed_Robotics")))
        self.lease.acquire()
        try:
            output = Path(os.environ.get("REBOTARM_CSP_OUTPUT",str(Path.cwd()/"outputs/twin-console/hardware")))
            self.hardware = CspHardware(list(zip(self.kinematics.lower,self.kinematics.upper)),output=output,
                path_check=lambda start,goal:self.kinematics.check_path(start,goal,
                    gravity_limits=gravity_limits_for_mode(self.hardware.config.version)),
                recovery_path_check=self.kinematics.check_support_recovery_path)
            self.hardware.connect()
            self.twin = TwinControl(self,self.hardware,"rebotarm",kinematics=self.kinematics)
            self.joint_pub = self.create_publisher(JointState,"/rebotarm/joint_states",10)
            self.status_pub = self.create_publisher(ArmStatus,"/rebotarm/arm_status",10)
            self.enable_service = self.create_service(Trigger,"/rebotarm/enable",self.prepare)
            self.disable_service = self.create_service(Trigger,"/rebotarm/disable",self.disable)
            self.pub_timer = self.create_timer(.05,self.publish)
            self.get_logger().info("CSP twin ready: read-only; no motor enabled")
        except Exception:
            if self.hardware:
                self.hardware.close()
            self.lease.release()
            raise

    def prepare(self, _req, response):
        try:
            self.hardware.prepare()
            name = "完整" if self.hardware.control_mode == "full" else "小幅"
            response.success,response.message = True,f"{name}控制已准备；尚未使能。接管后可启动电机保持，或直接发起运动。"
        except Exception as exc:
            response.success,response.message = False,str(exc)
        self.publish()
        return response

    def disable(self, _req, response):
        self.twin.stop(None,Trigger.Response())
        self.hardware.disable()
        response.success,response.message = True,"停止/失能请求已提交，等待状态和参数恢复核对"
        return response

    def publish(self):
        q,v,_ = self.hardware.get_joint_state()
        stamp = self.get_clock().now().to_msg()
        joint = JointState();joint.header.stamp=stamp
        joint.name=[f"joint{i}" for i in range(1,7)]
        joint.position=q.tolist();joint.velocity=v.tolist()
        self.joint_pub.publish(joint)
        status=ArmStatus();status.header.stamp=stamp
        status.mode="native_csp_twin"
        # enabled denotes software preparation here; control_loop_active and
        # twin.hardware_phase explicitly distinguish actual drive output.
        status.enabled=self.hardware.armed
        status.control_loop_active=self.hardware.motion_ready
        status.state_machine=self.hardware.phase
        status.joint_names=joint.name
        status.error_codes=[self.hardware.error] if self.hardware.error else []
        self.status_pub.publish(status)

    def close(self):
        self.twin.shutdown()
        self.hardware.close()
        self.lease.release()


def main():
    # Keep the ROS context alive until CAN has stopped and restored its limits.
    # rclpy's default SIGTERM handler shuts the context down before cleanup.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    exiting=threading.Event()
    def terminate(_signal, _frame):
        exiting.set()
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGINT, terminate)
    node=None
    executor=None
    stack_signal=False
    try:
        node=CspTwinDriver()
        if hasattr(signal,'SIGUSR1'):
            faulthandler.register(signal.SIGUSR1,all_threads=True)
            stack_signal=True
            node.hardware.stack_dump_signal=int(signal.SIGUSR1)
        executor=MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        while not exiting.is_set() and rclpy.ok():
            executor.spin_once(timeout_sec=.1)
            # Jazzy's ready-callback loop can repeatedly reacquire the GIL,
            # starving the CAN IO thread even while it is returning from IO.
            # Yield between ROS dispatches; retain all per-IO checks/deadlines.
            exiting.wait(.001)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if stack_signal:faulthandler.unregister(signal.SIGUSR1)
        if node:
            node.close()
            if executor:
                executor.shutdown(timeout_sec=2)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
