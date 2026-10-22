"""One execution/feedback path for point goals, keyboard and gamepad.

Uses the driver's existing exclusive teleop lease and motor-side watchdog.
Enable requests go through the guarded hardware worker. Command input is volatile, depth one.
"""
from __future__ import annotations

import json
import math
import secrets
import threading
import time
from pathlib import Path

import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .leader_policy import finite_vector
from .twin_kinematics import TwinKinematics
from .twin_tuning import MotionSettings
from .twin_jog import JogRamp
from .twin_motion import PROFILES, RuckigVelocity, point_duration, point_sample
from .twin_extensions import TwinExtensions
from .csp_runtime.rebot_native_xyz import FeedbackTimeout


class TwinControl:
    HEARTBEAT_TIMEOUT = 0.7
    JOG_TIMEOUT = 0.25
    MAX_SPEED = 0.3

    def __init__(self, node, hardware, namespace, *, kinematics=None, clock=time.monotonic, simulation=False):
        self.node, self.hw, self.namespace = node, hardware, namespace
        self.kin = kinematics or TwinKinematics()
        self.clock = clock
        self.lock = threading.RLock()
        self.session = ""
        self.mode = "IDLE"
        self.message = "连接后接管控制；默认不使能实机"
        self.plan = None
        self.goal = None
        self.reference = None
        self.sequence = 0
        self.input_seq = -1
        self.last_heartbeat = 0.0
        self.last_jog = 0.0
        self.last_jog_input = 0.0
        self.jog_ramp = JogRamp()
        self.ruckig_jog = RuckigVelocity()
        self.motion_profile = 'ruckig'
        self.motion_settings = MotionSettings()
        self.last_input_stamp = -1
        # Jog input is intentionally a latest-value lane.  ROS/rosbridge can
        # deliver reliable messages in bursts; replaying every queued sample
        # makes manual control feel like stop-and-go motion.  The 20 ms
        # dispatcher consumes at most one sample and older samples are dropped.
        self._pending_jog = None
        self._pending_jog_at = 0.0
        self._jog_dropped = 0
        self._next_state_at = self.clock()
        self.simulation = simulation
        self.mujoco_feedback = None
        self.mujoco_velocity = None
        self.mujoco_feedback_at = 0.0
        self.last_result = None
        self.reset_status = None
        self.reset_stamp_ms = 0.
        self.extensions = TwinExtensions(self)
        self.pub = node.create_publisher(String, f"/{namespace}/twin/state", 1)
        self.events = node.create_publisher(String, f"/{namespace}/twin/events", 10)
        self.reference_pub = node.create_publisher(JointState, f"/{namespace}/twin/reference", 1)
        group = ReentrantCallbackGroup()
        self.acquire_service = node.create_service(Trigger, f"/{namespace}/twin/acquire", self.acquire, callback_group=group)
        self.stop_service = node.create_service(Trigger, f"/{namespace}/twin/stop", self.stop, callback_group=group)
        self.suspend_service = node.create_service(Trigger, f"/{namespace}/twin/suspend", self.suspend, callback_group=group)
        self.reset_service = node.create_service(Trigger, f"/{namespace}/twin/reset", self.reset, callback_group=group)
        if hasattr(self.hw,'request_register_audit'):
            self.register_audit_service = node.create_service(Trigger, f"/{namespace}/twin/audit_registers", self.audit_registers, callback_group=group)
        if hasattr(self.hw,'seal_readonly_shutdown'):
            self.close_readonly_service = node.create_service(Trigger, f"/{namespace}/twin/close_readonly", self.close_readonly, callback_group=group)
        self.command_subscription = node.create_subscription(
            String, f"/{namespace}/twin/command", self.command,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE), callback_group=group,
        )
        self.stop_notice_subscription = node.create_subscription(
            String,f"/{namespace}/twin/stop_notices",self.stop_notice,10,callback_group=group)
        # Run the shared control dispatcher at the same 20 ms cadence used by
        # full-mode CAN reference generation. Hardware IO remains owned by its
        # dedicated CAN thread; this timer only consumes input and publishes
        # state, so it cannot create a second bus writer.
        self.timer = node.create_timer(0.02, self.tick, callback_group=group)
        if simulation:
            self.mujoco_subscription = node.create_subscription(
                JointState, f"/{namespace}/mujoco/joint_states", self._mujoco_state,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), callback_group=group,
            )

    def _mujoco_state(self, message):
        values = dict(zip(message.name, message.position))
        try:
            joints = finite_vector([values[f"joint{i}"] for i in range(1, 7)], 6)
        except (KeyError, ValueError):
            return
        with self.lock:
            self.mujoco_feedback, self.mujoco_feedback_at = joints, self.clock()
            try:
                velocities = dict(zip(message.name, message.velocity))
                self.mujoco_velocity = finite_vector([velocities[f'joint{i}'] for i in range(1, 7)], 6)
            except (KeyError, ValueError):
                self.mujoco_velocity = None

    def stop_notice(self, message):
        try:
            notice = json.loads(message.data)
            if isinstance(notice,dict) and hasattr(self.hw,"note_stop_request"):
                self.hw.note_stop_request(notice.get("session"),notice.get("reason"),notice.get("stamp_ms"))
        except (ValueError,TypeError):
            pass  # Invalid diagnostics have no control effect.

    def _using_mujoco(self):
        return self.mujoco_feedback is not None and self.clock() - self.mujoco_feedback_at < 0.3

    def _motion_profile(self):
        if getattr(self.hw,'control_mode',None) == 'small':return 'quintic'
        return getattr(self.hw,'motion_profile',self.motion_profile)

    def _motion_limits(self):
        if hasattr(self.hw, 'motion_limits'):
            return self.hw.motion_limits()
        acceleration = min(a.a_max for a in self.hw.config.axes) if hasattr(self.hw, 'config') else .5
        jerk = min(getattr(a, 'jerk_max', 30.) for a in self.hw.config.axes) if hasattr(self.hw, 'config') else 30.
        return self.motion_settings.limits(acceleration, jerk, small=getattr(self.hw, 'control_mode', None) == 'small')

    def _motion_state(self):
        settings = getattr(self.hw, 'motion_settings', self.motion_settings)
        return dict(selected=self._motion_profile(), available=list(PROFILES), profiles=settings.profiles,
                    effective_acceleration=self._motion_limits()[0], effective_jerk=self._motion_limits()[1],
                    driver=self.hw.drive_settings.state() if hasattr(self.hw,'drive_settings') else None,
                    backend=self.hw.drive_settings.selected.upper() if hasattr(self.hw,'drive_settings') else 'simulation')

    def _positions(self):
        if self.mujoco_feedback is not None:
            return self.mujoco_feedback[:]
        return finite_vector(self.hw.get_joint_state(request_feedback=False)[0], 6)

    def _fresh(self):
        if self.simulation and self.mujoco_feedback is not None and not self._using_mujoco():
            raise RuntimeError("MuJoCo 反馈已过期，停止控制")
        refreshed = getattr(self.hw, "_feedback_refreshed_at", self.clock())
        if self.clock() - refreshed > getattr(self.hw,"feedback_timeout",.5):
            raise FeedbackTimeout("实机反馈已过期，停止控制")

    def acquire(self, _request, response):
        with self.lock:
            try:
                self._fresh()
                if self.session:
                    raise RuntimeError("已有页面接管控制，请先停止并释放")
                self.session = self.hw.teleop_acquire()
                self.hw.teleop_pause(self.session)
                self.last_heartbeat = self.clock()
                self.input_seq, self.last_input_stamp = -1, -1
                self.plan = self.goal = None
                self._pending_jog = None
                self.jog_ramp.reset()
                self.ruckig_jog.reset()
                self.last_result = None
                self.mode, self.message = "HOLD", ("已重新接管保持状态；旧动作已取消，请提交新动作" if getattr(self.hw,'holding',None) else "已接管控制权，电机尚未使能；可启动电机保持或发起运动" if hasattr(self.hw,"motion_ready") else "已接管：保持当前位置")
                response.success = True
                response.message = json.dumps({"session": self.session, "namespace": self.namespace})
            except Exception as exc:
                response.success, response.message = False, str(exc)
        return response

    def stop(self, _request, response):
        # A stop is deliberately available to every connected observer.
        with self.lock:
            self._release("停止服务请求：停止并失能，等待停稳和参数恢复" if hasattr(self.hw,"motion_ready")
                else "已停止并释放控制；保持当前位置")
            if hasattr(self.hw,"motion_ready"):
                self.hw.disable()
            response.success, response.message = True, self.message
        return response

    def _release(self, reason):
        if getattr(self, 'extensions', None):self.extensions.stop(reason)
        if self.session:
            try:
                self.hw.teleop_stop(self.session, reason)
            except RuntimeError:
                pass  # The driver's independent watchdog may have released first.
        self.session = ""
        self.goal = self.plan = None
        self._pending_jog = None
        self.reference = None
        self.jog_ramp.reset()
        self.ruckig_jog.reset()
        self.mode, self.message = "IDLE", reason

    def suspend(self, _request, response):
        with self.lock:
            self._suspend('网页请求暂停并保持；旧动作与控制权已撤销')
            response.success, response.message = True, self.message
        return response

    def close_readonly(self, _request, response):
        with self.lock:
            try:
                self.hw.seal_readonly_shutdown()
                response.success,response.message = True,'只读退出已锁定；禁止重新准备或接管'
            except Exception as exc:
                response.success,response.message = False,str(exc)
            return response

    def audit_registers(self, _request, response):
        with self.lock:
            try:
                if self.session or self.goal or self.reference is not None:
                    raise RuntimeError('请先释放控制，寄存器核查不停止或接管已有动作')
                status = self.hw.request_register_audit()
                response.success,response.message = True,json.dumps(status,ensure_ascii=False)
            except Exception as exc:
                response.success,response.message = False,str(exc)
            return response

    def _suspend(self, reason):
        if hasattr(self.hw,'suspend') and self.hw.suspend(reason):
            self.extensions.stop(str(reason),
                preserve_support_recovery=bool(getattr(self.hw,'support_recovery',None)))
            self.session = ''
            self.goal = self.plan = self.reference = None
            self._pending_jog = None
            self.jog_ramp.reset()
            self.ruckig_jog.reset()
            self.mode, self.message = 'HOLD', str(reason)
        else:
            self._release(str(reason))

    def reset(self, _request, response):
        # Always available as a stop/release; it never acquires a motion lease.
        with self.lock:
            self._release("正在重置控制状态")
            self.last_result = None
            self.sequence = 0
            self.input_seq = self.last_input_stamp = -1
            self.last_heartbeat = self.last_jog = self.last_jog_input = 0.
            self._pending_jog = None
            self._pending_jog_at = 0.0
            self._jog_dropped = 0
            self.reset_stamp_ms = time.time()*1000
            try:
                if hasattr(self.hw,"request_reset"):
                    self.reset_status = self.hw.request_reset()
                elif self.simulation:
                    # The fake driver holds its measured pose on release.
                    self.reset_status = dict(id=secrets.token_hex(8),status="succeeded",
                        message="仿真控制已重置：旧目标已清空，保持反馈姿态；请重新接管。")
                else:
                    raise RuntimeError("此实机驱动不支持重置；控制已释放，请使用 CSP 驱动")
                self.message = self.reset_status["message"]
                response.success, response.message = True, json.dumps(self.reset_status,ensure_ascii=False)
            except Exception as exc:
                self.message = str(exc)
                response.success, response.message = False, str(exc)
        return response

    def _hold(self, reason, *, preserve_program=False):
        self._pending_jog = None
        if not preserve_program:
            self.extensions.on_hold(reason)
        if self.session:
            self.hw.teleop_pause(self.session)
        self.goal = None
        self.reference = None
        self.jog_ramp.reset()
        self.ruckig_jog.reset()
        self.mode, self.message = "HOLD", reason

    def _send(self, joints, speed, *, trajectory=None, velocity=None, sampled_at=None):
        if self.mode == "HOLD":
            self.hw.teleop_resume(self.session)
        self.sequence += 1
        if trajectory is not None and hasattr(self.hw,"teleop_point"):
            self.hw.teleop_point(self.session,self.sequence,self.clock(),trajectory,speed)
        elif velocity is not None and hasattr(self.hw,"teleop_stream"):
            self.hw.teleop_stream(self.session,self.sequence,self.clock() if sampled_at is None else sampled_at,joints,velocity,speed)
        else:
            self.hw.teleop_send(self.session, self.sequence, self.clock() if sampled_at is None else sampled_at, joints, None, speed)
        self.reference = list(joints)
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = [f"joint{i}" for i in range(1, 7)]
        msg.position = list(joints)
        self.reference_pub.publish(msg)

    def command(self, message):
        request = {}
        with self.lock:
            try:
                if len(message.data) > 131072:
                    raise ValueError("控制消息过长")
                request = json.loads(message.data)
                if not isinstance(request, dict):
                    raise ValueError("控制消息必须是对象")
                operation = request.get("op")
                if getattr(self.hw,"reset_pending",False):
                    raise RuntimeError("正在重置，旧控制请求已作废")
                if self.reset_stamp_ms and float(request.get("stamp_ms",0)) < self.reset_stamp_ms:
                    raise RuntimeError("重置前的旧请求已作废，请重新操作")
                if operation == 'reviewed_mode_restore':
                    if self.session or self.goal or self.reference is not None or not hasattr(self.hw,'request_mode_restore'):
                        raise RuntimeError('已评审模式恢复须在无控制者、无目标的实机只读状态提交')
                    status = self.hw.request_mode_restore(request)
                    self.extensions.stop('已提交六轴模式维护，旧动作已作废')
                    self.plan = self.goal = self.reference = None
                    self._pending_jog = None
                    self.reset_stamp_ms = time.time()*1000
                    self.mode,self.message = 'IDLE',status['message']
                    self._event(request,True,self.message,reset_status=status)
                    return
                if operation == 'drive_config':
                    if self.session or self.hw.teleop_owned or getattr(self.hw,'armed',False):
                        raise RuntimeError('请先停止并释放控制后再更改驱动模式')
                    self.extensions._idle()
                    if not hasattr(self.hw,'set_drive_config'):
                        raise ValueError('当前仿真驱动不模拟MIT电机动力学；MIT模式仅由真实执行器提供')
                    self.hw.set_drive_config(request.get('driver'), request.get('parameters'))
                    self.extensions.stop('驱动模式或MIT参数改变，请重新检查')
                    self.plan = self.goal = self.reference = None
                    self.jog_ramp.reset();self.ruckig_jog.reset()
                    self.message = '驱动配置已生效，尚未使能；请重新检查路径'
                    self._event(request,True,self.message,motion_settings=self._motion_state())
                    return
                if operation == 'motion_profile':
                    profile = request.get('profile')
                    if profile not in PROFILES:raise ValueError('未知运动方案')
                    if self.session or self.hw.teleop_owned or getattr(self.hw,'armed',False):
                        raise RuntimeError('请先停止并释放控制后再切换运动方案')
                    self.extensions._idle()
                    if hasattr(self.hw,'set_motion_profile'):self.hw.set_motion_profile(profile, request.get('parameters'))
                    else:self.motion_settings.apply(profile, request.get('parameters'))
                    self.motion_profile = profile
                    self.extensions.stop('运动方案已切换，请重新检查路径')
                    self.plan = self.goal = self.reference = None
                    self.jog_ramp.reset();self.ruckig_jog.reset()
                    self.message = '运动方案已切换，尚未使能或运动'
                    self._event(request,True,self.message,motion_profile=profile,motion_settings=self._motion_state())
                    return
                if operation == "control_mode":
                    if self.session or not hasattr(self.hw,"set_control_mode"):
                        raise RuntimeError("请先停止并释放实机控制后再切换范围")
                    self.hw.set_control_mode(request.get("mode"))
                    self.extensions.stop("控制范围改变，请重新预览")
                    self.plan = self.goal = self.reference = None
                    self.message = "已切换控制范围；尚未准备或使能，请重新预览"
                    self._event(request,True,self.message,control_settings=self.hw.control_settings())
                    return
                if operation == "preview":
                    self._preview(request)
                    return
                if operation == 'suspend' and self.session and request.get('session') == self.session:
                    self._suspend('通信/页面暂停请求，冻结原位置目标')
                    self._event(request,True,self.message)
                    return
                if self.extensions.readonly(request):
                    return
                if not self.session or request.get("session") != self.session:
                    raise RuntimeError("控制权已过期，请重新接管")
                # Wall-clock age is checked against the ROS host, not trusted as
                # the watchdog clock. Browser and ROS are on this local host.
                stamp = float(request.get("stamp_ms", 0))
                if not math.isfinite(stamp) or abs(time.time() * 1000 - stamp) > 500:
                    raise RuntimeError("控制消息过期或浏览器与 ROS 主机时钟不同步")
                seq = int(request.get("seq", -1))
                if seq <= self.input_seq or stamp < self.last_input_stamp:
                    raise RuntimeError("忽略重复或乱序控制消息")
                self.input_seq, self.last_input_stamp = seq, stamp
                self._fresh()
                self.last_heartbeat = self.clock()
                self.hw.teleop_heartbeat(self.session)
                if operation == "heartbeat":
                    return
                if operation != "jog":
                    # A newer explicit action supersedes any undispatched input.
                    self._pending_jog = None
                extension_result = self.extensions.command(request)
                if extension_result == 'pending':
                    return
                if extension_result:
                    pass
                elif operation == "hold":
                    self._hold("已松开控制：保持当前位置")
                elif operation == "release":
                    reason = request.get("reason")
                    self._release(reason[:160] if isinstance(reason,str) and reason else "已停止并释放控制（网页释放请求）")
                elif operation == "goal":
                    self.extensions._idle()
                    self._start_goal(request)
                elif operation == "power_hold":
                    self.extensions._idle()
                    if not hasattr(self.hw,"motion_ready") or self.hw.phase != "READONLY":
                        raise RuntimeError("只有已接管的实机只读会话可以启动电机保持")
                    if self.mode in ("GOAL","JOG"):
                        raise RuntimeError("请先暂停运动再启动保持")
                    self.hw.teleop_resume(self.session)
                    self.plan = self.goal = self.reference = None
                    self.mode,self.message = "HOLD","已请求从实测姿态使能保持；请观察电机状态"
                elif operation == "jog":
                    self._validate_jog(request)
                    # Keep the command callback cheap and coalesce bursts.
                    # Stop/hold operations remain synchronous and therefore
                    # always take precedence over a pending jog sample.
                    if self._pending_jog is not None:
                        self._jog_dropped += 1
                    self._pending_jog = dict(request)
                    self._pending_jog_at = self.clock()
                    self.message = "手动输入已接收，按控制周期执行"
                else:
                    raise ValueError("未知控制操作")
                self._event(request, True, self.message)
            except Exception as exc:
                # Malformed/foreign traffic cannot stop the current owner.
                if isinstance(request, dict) and self.session and request.get("session") == self.session:
                    try:
                        self._hold(str(exc))
                    except RuntimeError:
                        self._release(str(exc))
                self._event(request if isinstance(request, dict) else {}, False, str(exc),diagnostic=getattr(exc,"diagnostic",None))

    def _preview(self, request):
        self.extensions._idle()
        if self.session and request.get("session") != self.session:
            raise RuntimeError("已有页面接管控制，其他页面仅可观察")
        if self.mode in ("GOAL", "JOG"):
            raise RuntimeError("请先保持当前位置，再计算预览")
        self._fresh()
        self.plan = None
        pose = finite_vector(request.get("pose", []), 6)
        self.last_result = None
        if np.linalg.norm(pose[:3]) > 0.8:
            raise ValueError("目标超出 RS 工作空间参考范围（坐标单位为米）")
        observed = self._positions()
        # An enabled hold has a frozen reference. Encoder settling must not
        # change the origin of the next trajectory or trigger the 0.1° first
        # enable check on every subsequent point.
        seed = finite_vector(self.hw.command_reference,6) if getattr(self.hw,"motion_ready",False) else observed
        orientation = bool(request.get("orientation", True))
        check=getattr(self.hw,'validate_preview',None)
        joints, residual = self.kin.solve(pose, seed, orientation,path_check=check)
        # Smooth interpolation between these endpoints stays inside each
        # joint's interval. This is NOT a collision/obstacle check.
        for value, (lo, hi) in zip(joints, self.hw.teleop_limits):
            if not lo <= value <= hi:
                raise ValueError("IK 结果超出驱动关节限位")
        speed = float(request.get("speed", 0.15))
        minimum, maximum = getattr(self.hw,"min_twin_speed",.05), getattr(self.hw,"max_twin_speed",self.MAX_SPEED)
        if not math.isfinite(speed) or not minimum <= speed <= maximum:
            raise ValueError(f"关节速度须为 {minimum:g}–{maximum:g} rad/s")
        duration = self._goal_duration(seed,joints,speed)
        self._check_goal_budget(duration)
        self.plan = {
            "id": secrets.token_hex(12), "pose": pose, "orientation": orientation,
            "start": seed, "observed_start":observed, "joints": joints, "speed": speed, "duration": duration,
            "solved_pose": self.kin.pose(joints),
            "at": self.clock(), "residual": residual,
        }
        self._event(request, True, "计算完成：目标虚影仅作预览", plan=self.plan)

    def _goal_duration(self, start, joints, speed):
        delta = max(abs(a-b) for a,b in zip(start,joints))
        acceleration, jerk = self._motion_limits()
        minimum = max(1.0,1.875*delta/speed,math.sqrt(5.78*delta/acceleration),(60*delta/jerk)**(1/3))
        return point_duration(start,joints,speed,acceleration,jerk,minimum) if self._motion_profile() == 'ruckig' else minimum

    def _check_goal_budget(self, duration):
        if hasattr(self.hw,"control_settings"):
            settings = self.hw.control_settings()
            budget = settings["remaining_s"] if settings["remaining_s"] is not None else settings["session_limit_s"]
            if budget is not None and duration + self.hw.goal_settle_timeout + 2 > budget:
                raise ValueError("轨迹超过本次控制剩余时长，请提高许可范围内的速度或停止后重新接管")

    def _start_goal(self, request):
        if self.mode in ("GOAL", "JOG"):
            raise RuntimeError("已有运动，请先停止/保持")
        if not self.plan or request.get("plan_id") != self.plan["id"] or self.clock() - self.plan["at"] > 30:
            raise ValueError("预览已过期，请重新计算")
        if max(abs(a - b) for a, b in zip(self._positions(), self.plan.get("observed_start",self.plan["start"]))) > 0.03:
            raise ValueError("机械臂位置已改变，请重新预览")
        self._check_goal_budget(self.plan["duration"])
        self.hw.teleop_resume(self.session)
        self.goal = {**self.plan, "began": self.clock(), "align_start": hasattr(self.hw,"command_reference")}
        self.last_result = None
        self.plan = None
        self.mode, self.message = "GOAL", "正在执行预览轨迹"

    def _validate_jog(self, request):
        if self.extensions.controller.active:
            raise RuntimeError('控制算法执行中，请先停止算法再使用手动控制')
        if self.extensions.run and self.extensions.run['status'] == 'running':
            raise RuntimeError('程序执行中，请先暂停再使用手动控制')
        if self.mode == "GOAL":
            raise RuntimeError("到点运动中，请先保持再使用手动控制")
        twist = finite_vector(request.get("twist", []), 6)
        if np.linalg.norm(twist[:3]) > 0.041 or np.linalg.norm(twist[3:]) > 0.36:
            raise ValueError("遥操作速度超过 40 mm/s 或 20°/s")
        speed = float(request.get("speed", 0.15))
        minimum, maximum = getattr(self.hw,"min_twin_speed",.05), getattr(self.hw,"max_twin_speed",self.MAX_SPEED)
        if not math.isfinite(speed) or not minimum <= speed <= maximum:
            raise ValueError(f"关节速度须为 {minimum:g}–{maximum:g} rad/s")
        grip = float(request.get('gripper_velocity', 0.))
        if not math.isfinite(grip):raise ValueError('夹爪速度需要有限数值')
        if grip and not hasattr(self.hw, 'gripper_jog'):raise ValueError('当前驱动不支持夹爪手动控制')
        if grip and hasattr(self.hw,'validate_gripper_jog'):self.hw.validate_gripper_jog(self.session,grip)
        return twist, speed, grip

    def _jog(self, request, *, sampled_at=None):
        twist, speed, grip = self._validate_jog(request)
        received_at = self.clock() if sampled_at is None else sampled_at
        if self.mode == "JOG" and self.clock()-self.last_jog_input >= self.JOG_TIMEOUT:
            self._hold("旧手动输入已过期，清空平滑状态")
        neutral = np.linalg.norm(twist) < 1e-8
        if neutral and not grip and (self.mode != "JOG" or self.reference is None):
            self._hold("已松开控制：保持当前位置")
            return
        if hasattr(self.hw,"motion_ready") and not self.hw.motion_ready:
            self.hw.teleop_resume(self.session)
            self.last_jog = self.clock()
            self.last_jog_input = received_at
            self.mode, self.message = "JOG", "正在从实测姿态准备 CSP；松键会取消启动"
            return
        if hasattr(self.hw, 'gripper_jog') and (grip or 'gripper_velocity' in request):
            self.hw.gripper_jog(self.session, grip)
        now = self.clock()
        dt = min(.05,max(0.,now-self.last_jog)) if self.mode == "JOG" else .05
        if self.mode != "JOG":
            self.jog_ramp.reset()
            self.ruckig_jog.reset()
        seed = self._positions()
        if getattr(self.hw,"integrate_jog_reference",False):
            if getattr(self.hw,'continuous_jog_reference',False):
                seed = finite_vector(self.hw.command_reference,6)
            elif self.mode == "JOG" and self.reference is not None and now-self.last_jog < self.JOG_TIMEOUT:
                seed = self.reference
            elif hasattr(self.hw,"command_reference"):
                seed = finite_vector(self.hw.command_reference,6)
        if dt <= 0:
            return
        fraction=getattr(self.hw,'manual_speed_fraction',.8)
        options = dict(lookahead=dt,max_step=fraction*speed*dt)
        if request.get('orientation') is False:options['orientation'] = False
        requested = seed if neutral else self.kin.jog(seed,twist,**options)
        acceleration, jerk = self._motion_limits()
        if self._motion_profile() == 'ruckig':
            if getattr(self.hw,'continuous_jog_reference',False):
                # The CAN owner applies Ruckig at each reference opportunity.
                # Avoid a second ROS ramp and do not cut its braking short.
                target = requested;velocity = (np.asarray(requested)-seed)/dt
                ramp = self.hw.stream_ruckig
            else:
                target = self.ruckig_jog.step(seed,(np.asarray(requested)-seed)/dt,dt,speed,acceleration,
                    self.kin.lower,self.kin.upper,jerk=jerk,speed_fraction=fraction)
                ramp = self.ruckig_jog;velocity = ramp.velocity
        else:
            target = self.jog_ramp.step(seed,(np.asarray(requested)-seed)/dt,dt,speed,acceleration,self.kin.lower,self.kin.upper,speed_fraction=fraction)
            ramp = self.jog_ramp;velocity = ramp.velocity
        if neutral and not grip and np.max(np.abs(ramp.velocity)) < 1e-8 and np.max(np.abs(getattr(ramp,'acceleration',0))) < 1e-8:
            self._hold("已平滑减速：保持当前位置")
            return
        self.last_result = None
        self._send(target, speed,velocity=np.asarray(velocity).tolist(),sampled_at=received_at)
        self.last_jog = now
        self.last_jog_input = received_at
        self.plan = None
        self.mode, self.message = "JOG", "正在平滑减速（继续按住空格 / RB）" if neutral else "平滑手动控制中（基座坐标系）"

    def tick(self):
        with self.lock:
            try:
                self.hw.teleop_watchdog()
                hold_requested = getattr(self.hw,'hold_requested',None)
                if getattr(self.hw,'holding',None) or hold_requested is not None and hold_requested.is_set():
                    if self.session and not getattr(self.hw,'hold_takeover',False):
                        reason = ((getattr(self.hw,'holding',None) or {}).get('reason') or
                                  getattr(self.hw,'stop_reason','') or getattr(self.hw,'error','') or
                                  '通信异常暂停，等待重新接管')
                        if getattr(self.hw,'hold_recoverable',False):reason = FeedbackTimeout(reason)
                        self._suspend(reason)
                if self.session:
                    self._fresh()
                    if not self.hw.teleop_owned or self.clock() - self.last_heartbeat > self.HEARTBEAT_TIMEOUT:
                        self._suspend("输入/连接已超时：冻结原位置目标，等待重新接管")
                    elif not self.extensions.check_grasp():
                        pass
                    elif self._pending_jog is not None:
                        request = self._pending_jog
                        queued_at = self._pending_jog_at
                        self._pending_jog = None
                        if self.clock() - queued_at > self.JOG_TIMEOUT:
                            self._hold("手动输入在队列中已过期：保持当前位置")
                        else:
                            try:
                                self._jog(request, sampled_at=queued_at)
                            except Exception as exc:
                                self._hold(str(exc))
                                self._event(request, False, str(exc))
                    elif self.mode == "JOG" and self.clock() - self.last_jog_input > self.JOG_TIMEOUT:
                        self._hold("按键或手柄输入超时：保持当前位置")
                    elif self.mode == "GOAL":
                        self._advance_goal()
                    if self.session:
                        self.extensions.tick()
                hardware_error = getattr(self.hw,"error","")
                if hardware_error and not self.session:
                    self.message = hardware_error
                # Control/watchdogs remain 50 Hz. The browser consumes state at
                # 20 Hz; redundant model/JSON work competes with the CAN owner.
                now = self.clock()
                if now >= self._next_state_at:
                    self._publish_state()
                    self._next_state_at += .05 * max(1, math.floor((self.clock()-self._next_state_at)/.05)+1)
            except Exception as exc:
                self._suspend(exc)
                self.node.get_logger().warn(f"Twin control stopped: {exc}")

    def _advance_goal(self):
        goal = self.goal
        if hasattr(self.hw,"motion_ready") and not self.hw.motion_ready:
            goal["began"] = self.clock()
            self.message = "正在从实测姿态准备 CSP；停止可取消启动"
            return
        if goal.get("align_start"):
            # A preview's encoder sample predates the per-drive preloads.
            # Start at the frozen commanded hold, so stationary encoder noise
            # cannot ask the first segment to step outward across a boundary.
            start = finite_vector(self.hw.command_reference,6)
            if max(abs(a-b) for a,b in zip(start,goal["start"])) > self.hw.joint_arrival_tolerance:
                raise ValueError("使能保持起点与预览相差超过 0.1°，请重新预览")
            self.hw.validate_preview(start,goal["joints"],
                                    support_recovery=goal.get('support_recovery') is True)
            duration = self._goal_duration(start,goal["joints"],goal["speed"])
            self._check_goal_budget(duration)
            goal.update(start=start,duration=duration,began=self.clock(),align_start=False)
        elapsed = self.clock() - goal["began"]
        self.message = "正在执行预览轨迹"
        t = min(1.0, elapsed / goal["duration"])
        acceleration, jerk = self._motion_limits()
        reference = point_sample(self._motion_profile(),goal['start'],goal['joints'],goal['speed'],
            acceleration,jerk,goal['duration'],elapsed).tolist()
        actual = self._positions()
        # CAN references use capped virtual time after a delayed SDK reply.
        # Compare against the reference actually sent, not a wall-clock future.
        tracking_reference = self.hw.command_reference if getattr(self.hw,'continuous_jog_reference',False) else reference
        if max(abs(a-b) for a, b in zip(tracking_reference, actual)) > 0.2:
            self.extensions.goal_finished(False, '跟踪偏差超过 0.2 rad')
            self._hold("跟踪偏差超过 0.2 rad，已保持；检查实机与模型", preserve_program=True)
            return
        self._send(reference, goal["speed"],trajectory=goal)
        error = self.kin.error(actual, goal["pose"], goal["orientation"])
        self.last_result = {
            "position_error_mm": float(np.linalg.norm(error[:3])*1000),
            "rotation_error_deg": float(np.linalg.norm(error[3:])*180/math.pi),
            "orientation_constrained": goal["orientation"],
        }
        support_tolerance = goal.get('support_tolerance_rad')
        if support_tolerance is not None:
            # Support return is defined by J2–J6 motor angles, independent of
            # TCP error. J1 is held at the start angle and J7 is never commanded.
            joint_errors = [abs(a-b) for a,b in zip(actual[1:],goal['joints'][1:])]
            arrived = max(joint_errors) <= support_tolerance + 1e-12
            self.last_result.update(support_joint_errors_deg=[math.degrees(e) for e in joint_errors],
                                    support_tolerance_deg=math.degrees(support_tolerance))
            arrived_message = f'已到达支撑姿态（J2–J6 每轴偏差 ≤ {math.degrees(support_tolerance):g}°）'
        else:
            joint_arrived = max(abs(a-b) for a,b in zip(actual,goal["joints"])) <= getattr(self.hw,"joint_arrival_tolerance",float("inf"))
            arrived = joint_arrived and np.linalg.norm(error[:3]) <= 0.002 and np.linalg.norm(error[3:]) <= 0.035
            arrived_message = "已到达目标（位置误差 ≤ 2 mm；受约束姿态 ≤ 2°，基于反馈与 URDF）"
        if t >= 1 and arrived:
            self._hold(arrived_message, preserve_program=True)
            self.extensions.goal_finished(True, self.message)
        elif elapsed > goal["duration"] + getattr(self.hw,"goal_settle_timeout",5):
            self.extensions.goal_finished(False, '到点等待超时，程序已中断')
            self._hold("到点等待超时；未确认到达，已保持当前位置", preserve_program=True)

    def _publish_state(self):
        joints = self._positions()
        error = self.kin.error(joints, self.goal["pose"], self.goal["orientation"]) if self.goal else None
        feedback_at = self.mujoco_feedback_at if self.mujoco_feedback is not None else getattr(self.hw, "_feedback_refreshed_at", self.clock())
        age_ms = max(0, (self.clock()-feedback_at)*1000)
        ramp = (getattr(self.hw,'stream_ruckig',self.ruckig_jog) if self._motion_profile() == 'ruckig' else self.jog_ramp)
        self.pub.publish(String(data=json.dumps({
            "namespace": self.namespace, "mode": self.mode, "owned": bool(self.session),
            "input_seq":self.input_seq,
            "feedback_source": getattr(self.hw,"feedback_source",f"/{self.namespace}/mujoco/joint_states" if self.mujoco_feedback is not None else f"/{self.namespace}/joint_states"),
            "feedback_valid": getattr(self.hw,'phase',None) != 'FEEDBACK_WAIT' and age_ms < (300 if self.mujoco_feedback is not None else getattr(self.hw,"feedback_timeout",.5)*1000),
            "message": self.message, "actual": joints, "pose": self.kin.pose(joints),
            "reference": self.reference, "goal": self.goal["joints"] if self.goal else None,
            "position_error_mm": float(np.linalg.norm(error[:3])*1000) if error is not None else None,
            "rotation_error_deg": float(np.linalg.norm(error[3:])*180/math.pi) if error is not None else None,
            "joint_error_deg": max(abs(a-b) for a,b in zip(joints, self.reference))*180/math.pi if self.reference else 0,
            "last_result": self.last_result,
            "control_backend": "csp" if hasattr(self.hw,"motion_ready") else "driver",
            "hardware_phase": getattr(self.hw,"phase",None),
            "communication_hold": getattr(self.hw,"holding",None),
            "control_prepared": getattr(self.hw,"armed",None),
            "cleanup_ok": False if getattr(self.hw,"recovery_required",None) else (getattr(self.hw,"last_report",None) or {}).get("cleanup_ok"),
            "recovery_required": getattr(self.hw,"recovery_required",None),
            "reset_status": getattr(self.hw,"reset_status",None) or self.reset_status,
            "register_audit_status": getattr(self.hw,"register_audit_status",None),
            "drive_modes": {str(i):s.mode for i,s in getattr(self.hw,"samples",{}).items()},
            "motor_parameters": {str(i+1):dict(lower=float(self.kin.lower[i]),upper=float(self.kin.upper[i])) for i in range(6)},
            "control_settings": self.hw.control_settings() if hasattr(self.hw,"control_settings") else None,
            "motion_profile": self._motion_state(),
            "jog_smoothing": {"enabled":True,"algorithm":self._motion_profile(),"velocity_rad_s":ramp.velocity.tolist(),
                              "acceleration_rad_s2":getattr(ramp,'acceleration',np.zeros(6)).tolist(),"max_step_s":.05},
            "jog_input": {"dispatch_period_ms":20,"policy":"latest_only","pending":self._pending_jog is not None,
                          "dropped":self._jog_dropped},
            "feedback_age_ms": age_ms,
            "stamp_ms": time.time()*1000,
            "extensions": self.extensions.state(),
        }, allow_nan=False)))

    def _event(self, request, ok, message, **extra):
        self.events.publish(String(data=json.dumps({"id": request.get("id"), "ok": ok, "message": message, **extra}, allow_nan=False)))
        if not ok and getattr(self.hw,'output',None):
            # Persist reasons missing from motor-session logs, without tokens.
            record=dict(stamp_ms=time.time()*1000,op=request.get('op'),message=message,
                diagnostic=extra.get('diagnostic'),pose=request.get('pose'),orientation=request.get('orientation'),
                speed=request.get('speed'),actual=self._positions(),reference=self.reference)
            try:
                with (Path(self.hw.output)/'control-rejections.jsonl').open('a') as output:
                    output.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
            except (OSError, TypeError, ValueError):
                pass  # A diagnostic write must never affect stop/control.

    def shutdown(self):
        with self.lock:
            self._release("控制器已关闭")
