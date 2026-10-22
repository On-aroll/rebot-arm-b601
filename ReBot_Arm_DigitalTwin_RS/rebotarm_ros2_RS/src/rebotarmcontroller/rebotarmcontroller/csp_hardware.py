"""Bounded CSP follower for the web twin. One thread owns every CAN operation.

Connection is read-only. Explicit power-hold or motion starts a guarded session;
the drive PID is preserved and temporary operating limits are restored after
disable/status/stationary confirmation. No upstream group lifecycle is called.
"""
from dataclasses import asdict
from collections import deque
import gc
import json
import math
import os
from pathlib import Path
import secrets
import threading
import time

import numpy as np

from .leader_policy import finite_vector
from .csp_runtime.rebot_mit_hardware import Envelope, OnsitePermit
from .csp_runtime.rebot_native_xyz import AXES, MODE, LIVE_FULL_MODE, OFFICIAL_FULL_MODES, FeedbackTimeout, NativeXYZTransport, NativeXYZExecutor, proposal, gravity_limits_for_mode, runtime_for_mode
from .csp_runtime.support_recovery import (SupportRecovery, START_MATCH_RAD, build_support_recovery,
                                           gravity_limits_for_recovery_stage)
from .csp_official_limits import SPEED_MAX, SOURCES
from .csp_gripper import CspGripper
from .csp_reset import reset_source, validate_snapshot, check_current_registers, RUNTIME_KEYS, PROTECTED
from .csp_recovery_review import validate_review, read_reviewed_registers
from .csp_hold import validate_takeover, wait_for_takeover
from .twin_tuning import MotionSettings
from .twin_drive import DriveSettings
from .twin_jog import JogRamp
from .twin_motion import PROFILES, RuckigVelocity, point_sample


class CspHardware:
    min_twin_speed = 0.005
    max_twin_speed = 0.02
    max_twin_delta = math.radians(0.5)
    start_tolerance = math.radians(1)
    joint_arrival_tolerance = math.radians(0.1)
    feedback_timeout = .6
    goal_settle_timeout = 10
    integrate_jog_reference = True
    feedback_source = "RobStride mechPos / 0x7019 · CSP"

    def __init__(self, limits, *, transport=None, output=None, clock=time.monotonic, path_check=None,
                 recovery_path_check=None):
        self.clock = clock
        self.teleop_limits = [tuple(pair) for pair in limits]
        self._configure_mode("small")
        self.bus = transport or NativeXYZTransport()
        self.output = Path(output) if output else None
        self.path_check = path_check
        self.recovery_path_check = recovery_path_check
        self.support_recovery = None
        self.lock = threading.RLock()
        self.closing = threading.Event()
        self.shutdown_sealed = False
        self.cancel_motion = threading.Event()
        self.hold_requested = threading.Event()
        self.hold_recoverable = False
        self.hold_takeover = False
        self.holding = None
        self.worker = None
        self._feedback_refreshed_at = 0.
        self._q = np.zeros(6)
        self._v = np.zeros(6)
        self.samples = {}
        self.session = ""
        self.armed = False
        self.phase = "CONNECTING"
        self.error = ""
        self.last_report = None
        self.last_heartbeat = 0.
        self.last_sample = 0.
        self.last_seq = -1
        self.input_seq = 0
        self.target = None
        self.point_reference = None
        self.stream_velocity = None
        self.stream_ramp = JogRamp()
        self.stream_ruckig = RuckigVelocity()
        self.motion_settings = MotionSettings(self.output/'motion-settings.json' if self.output else None)
        self.motion_profile = self.motion_settings.selected
        self.drive_settings = DriveSettings(self.output/'drive-settings.json' if self.output else None)
        self.origin = None
        self.command_reference = None
        self.stop_reason = ""
        self.stop_request = None
        self.stopped_session = ""
        self.stopped_at = 0.
        self.speed = .01
        self.command_period_s = None
        self.poll_duration_s = None
        self.start_requested = False
        self.executor = None
        self.paused = True
        self.recovery_required = self._unrestored_session()
        self.reset_status = None
        self.reset_pending = False
        self._mode_restore_review = None
        self.mode_restore_cancelled = threading.Event()
        self.register_audit_status = None
        self._register_audit = None
        self.gripper = CspGripper(self)
        self.host_timings = deque(maxlen=32)
        self._gc_started = {}

    def _record_timing(self, stage, began, cpu):
        elapsed=time.perf_counter()-began
        if elapsed>=.02:
            self.host_timings.append(dict(stage=stage,at=self.clock(),elapsed_ms=round(elapsed*1000,3),
                cpu_ms=round((time.thread_time()-cpu)*1000,3),thread=threading.current_thread().name))

    def _timed(self, stage, action, *args):
        began,cpu=time.perf_counter(),time.thread_time()
        try:return action(*args)
        finally:self._record_timing(stage,began,cpu)

    def _gc_timing(self, phase, info):
        generation=info['generation']
        if phase=='start':self._gc_started[generation]=(time.perf_counter(),time.thread_time())
        elif generation in self._gc_started:
            self._record_timing(f'gc-{generation}',*self._gc_started.pop(generation))

    def _unrestored_session(self):
        # A preflight refusal has no device writes and cannot certify recovery
        # of an earlier session. Preserve that session across process restarts.
        if self.output:
            for path in sorted(self.output.glob("csp-session-*.json"),reverse=True):
                try:
                    report = json.loads(path.read_text())
                except (OSError,ValueError):
                    return dict(report=str(path),reason="历史清理报告不可读，需核查")
                changed = bool(report.get("write_intent") or report.get("register_log") or (report.get("stop") or {}).get("attempted_axes"))
                if changed:
                    if report.get("cleanup_ok") and (report.get("restoration") or {}).get("ok"):
                        return None
                    return dict(report=str(path),reason=report.get("reason"),
                                cleanup_error=report.get("cleanup_error"))
        return None

    def _recovery_message(self):
        detail = self.recovery_required or {}
        return ("上次停止/参数恢复未确认，已锁定实机使能；仅可观察。"
                f"原因：{detail.get('cleanup_error') or detail.get('reason') or '运行模式需核查'}。"
                "请先核查异常声与机械状态，再核对停稳和原始参数快照；不会自动切模式或恢复旧目标。")

    def _readonly_phase(self):
        unexpected = {f"J{i}":s.mode for i,s in self.samples.items() if s.mode != 0}
        if not self.recovery_required and unexpected:
            self.recovery_required = dict(reason=f"当前运行模式 {unexpected}，初始检查要求模式 0；模式值不代表使能状态")
        if self.recovery_required:
            self.phase,self.error = "RECOVERY",self._recovery_message()
        else:
            self.phase = "READONLY"
        if self.reset_pending:
            self.phase = "RESETTING"

    def _require_recovered(self):
        if self._register_audit:
            raise RuntimeError('正在只读核查寄存器，请等待完成后再准备或接管')
        if self.gripper.state()['recovery_required']:
            raise RuntimeError('J7上次停止/参数恢复未确认，请重置核查')
        if self.reset_pending:
            raise RuntimeError("正在重置，旧控制请求已作废，请等待完成")
        if self.recovery_required:
            raise RuntimeError(self._recovery_message())

    def _configure_mode(self, mode):
        if mode not in ("small", "full"):
            raise ValueError("控制模式仅支持 small / full")
        raw, public = proposal(self.teleop_limits,mode=LIVE_FULL_MODE if mode == "full" else MODE)
        self.config = Envelope.parse(raw)
        self.control_mode = mode
        self.max_twin_speed = min(public["speed_max_rad_s"],min(getattr(self,'device_speed_limits',{}).values(),default=SPEED_MAX))
        self.max_twin_delta = None if mode == "full" else math.radians(.5)
        self.manual_speed_fraction = 1. if mode == 'full' else .8
        self.continuous_jog_reference = mode == 'full'
        # The envelope retains its finite standalone-trajectory policy and
        # fingerprint, including old recovery snapshots. The interactive full
        # session instead uses a renewable live permit with no total time cap.
        self.session_seconds = None if mode == "full" else self.config.policy.task_timeout

    def set_control_mode(self, mode):
        # A mode change is configuration only. It never enables a drive and
        # cannot widen an existing owner's live execution envelope.
        with self.lock:
            self._require_recovered()
            if self.phase != "READONLY" or self.armed or self.session:
                raise RuntimeError("请先停止并释放，等待只读状态后再切换控制范围")
            if self.last_report and not self.last_report.get("cleanup_ok"):
                raise RuntimeError("停止清理未确认，禁止切换控制范围")
            self._configure_mode(mode)
            if mode == 'small':
                self.drive_settings.selected = self.bus.drive_backend = 'csp'
                self.feedback_source = 'RobStride mechPos / 0x7019 · CSP'
            self.error = ""

    def set_drive_config(self, selected, parameters=None):
        with self.lock:
            self._require_recovered()
            if self.phase != 'READONLY' or self.armed or self.session:
                raise RuntimeError('驱动模式与MIT增益须在只读、未准备、无控制者时修改')
            if self.control_mode != 'full':
                raise RuntimeError('请先选择完整控制范围；小幅试验保持原CSP方案')
            if selected == 'mit' and self.output is None and self.bus.factory is None:
                raise RuntimeError('真实MIT执行需要持久恢复日志目录，不能仅依赖模式0判断停止')
            if self.last_report and not self.last_report.get('cleanup_ok'):
                raise RuntimeError('停止清理未确认，禁止更改驱动模式')
            self.drive_settings.apply(selected, parameters)
            self.bus.drive_backend = selected
            self.bus.mit_parameters = self.drive_settings.parameters
            self.feedback_source = f'RobStride mechPos / 0x7019 · {selected.upper()}'
            self.point_reference = self.stream_velocity = self.target = None
            self.stream_ruckig.reset();self.stream_ramp.reset()

    def set_motion_profile(self, profile, parameters=None):
        with self.lock:
            self._require_recovered()
            if profile not in PROFILES:
                raise ValueError('运动方案仅支持 ruckig / quintic')
            if self.phase != 'READONLY' or self.armed or self.session:
                raise RuntimeError('请先停止并释放，等待只读状态后再切换运动方案')
            if self.last_report and not self.last_report.get('cleanup_ok'):
                raise RuntimeError('停止清理未确认，禁止切换运动方案')
            self.motion_settings.apply(profile, parameters)
            self.motion_profile = profile
            self.stream_ruckig.reset();self.stream_ramp.reset()
            self.point_reference = self.stream_velocity = self.target = None

    def motion_limits(self):
        return self.motion_settings.limits(min(a.a_max for a in self.config.axes),
            min(a.jerk_max for a in self.config.axes), small=self.control_mode == 'small')

    def control_settings(self):
        with self.lock:
            remaining = None
            now = self.clock()
            io = list(getattr(self.bus, 'poll_io', ()))
            gaps = [max(0., b['requested_at']-a['received_at'])*1000 for a,b in zip(io,io[1:])]
            motors = {}
            for i in AXES:
                sample = self.samples.get(i)
                original = getattr(self.bus.drives.get(i), 'original', {})
                motors[str(i)] = dict(
                    current_a=sample.iq_raw_a if sample else None,
                    position_age_ms=max(0.,now-sample.requested_at)*1000 if sample else None,
                    following_error_deg=math.degrees(self.command_reference[i-1]-sample.q) if sample and self.command_reference is not None else None,
                    csp_gains={hex(index):original[index] for index in (0x701E,0x701F,0x7020,0x702B) if index in original})
            if self.executor and self.executor.deadline is not None:
                remaining = max(0.,self.executor.deadline-self.clock())
            nominal_gravity=gravity_limits_for_mode(self.config.version)
            recovery_status=None
            if self.support_recovery:
                stage=self.support_recovery.stage
                recovery_status=dict(stage=stage+1,stages=len(self.support_recovery.waypoints),
                    active=self.support_recovery.active,
                    gravity_limits_nm=gravity_limits_for_recovery_stage(stage,nominal_gravity).tolist(),
                    exception='仅第1段J2/J3模型重力预检上限15 N·m；普通动作及后续段仍为额定11/5 N·m')
            return dict(mode=self.control_mode,min_speed_rad_s=self.min_twin_speed,
                drive=self.drive_settings.state(),
                motion_profile=self.motion_profile if self.control_mode == 'full' else 'quintic',
                profile_version=self.config.version,measured_speed_guard_rad_s=self.config.axes[0].v_max,
                acceleration_rad_s2=self.motion_limits()[0],
                acceleration_envelope_rad_s2=self.config.axes[0].a_max,
                jerk_rad_s3=self.motion_limits()[1],
                jerk_envelope_rad_s3=self.config.axes[0].jerk_max,
                manual_speed_fraction=self.manual_speed_fraction,
                official_speed_max_rad_s=SPEED_MAX if self.control_mode == 'full' else None,
                device_speed_limits_rad_s=getattr(self,'device_speed_limits',{}),
                motor_diagnostics=motors,
                poll_timing=dict(window=len(io),max_reply_ms=max((row['duration_ms'] for row in io),default=None),
                    max_between_queries_ms=max(gaps,default=None)),
                host_timings=list(self.host_timings),
                stack_dump_signal=getattr(self,'stack_dump_signal',None),
                torque_limits_nm=[runtime_for_mode(self.config.version)[i][0x700B] for i in AXES],
                current_limits_a=[runtime_for_mode(self.config.version)[i][0x7018] for i in AXES],
                gravity_limits_nm=list(nominal_gravity),support_recovery=recovery_status,
                limit_sources=SOURCES if self.control_mode == 'full' else {},
                reference_period_target_ms=20 if self.control_mode == 'full' else 90,
                command_timeout_ms=round(self.config.policy.command_age*1000),
                command_period_ms=None if self.phase != 'ACTIVE' or self.command_period_s is None else round(self.command_period_s*1000,1),
                poll_duration_ms=None if self.poll_duration_s is None else round(self.poll_duration_s*1000,1),
                feedback_schedule='ACTIVE full: oldest field first, mode/current/position every cycle; otherwise two oldest diagnostic pairs; each field <=600 ms',
                max_speed_rad_s=self.max_twin_speed,max_axis_delta_deg=.5 if self.control_mode == "small" else None,
                session_limit_s=self.session_seconds,remaining_s=remaining,continuous=self.session_seconds is None,
                joint_limits_rad=self.teleop_limits,tracking_limit_deg=math.degrees(self.config.axes[0].error_max))

    @property
    def enabled(self):
        return self.armed

    @property
    def teleop_owned(self):
        return bool(self.session)

    @property
    def motion_ready(self):
        return self.phase == "ACTIVE"

    def connect(self):
        if self.bus.factory is None:gc.callbacks.append(self._gc_timing)
        self.worker = threading.Thread(target=self._worker, name="csp-can-owner", daemon=False)
        self.worker.start()
        deadline = self.clock() + 12
        while self.phase == "CONNECTING" and self.clock() < deadline:
            time.sleep(.02)
        if self.phase not in ("READONLY","RECOVERY"):
            raise RuntimeError(self.error or "CSP 只读连接超时")

    def prepare(self):
        with self.lock:
            if self.shutdown_sealed:raise RuntimeError('项目正在结束，禁止重新准备')
            self._require_recovered()
            if self.phase != "READONLY" or self.clock()-self._feedback_refreshed_at > self.feedback_timeout:
                raise RuntimeError("等待新鲜只读反馈和上次停止清理完成")
            if self.last_report and not self.last_report.get("cleanup_ok"):
                raise RuntimeError("上次停止/参数恢复未确认，禁止重新准备")
            self.armed = True
            self.error = ""

    def disable(self):
        with self.lock:
            if self._mode_restore_review:
                self.mode_restore_cancelled.set()
            self._cancel_register_audit('停止/失能请求已取消只读核查')
            self.gripper.authorized = False
            if self.session:
                self.stopped_session,self.stopped_at = self.session,self.clock()
                self.stop_reason = "失能/服务停止请求"
            self.armed = False
            self.session = ""
            self.cancel_motion.set()
            self.start_requested = False

    def request_reset(self, *, _mode_review=None):
        with self.lock:
            if self.reset_pending:
                return dict(self.reset_status)
            self._cancel_register_audit('重置请求已取消只读核查')
            self.disable()
            self.target = None
            self.paused = True
            if not self.worker or not self.worker.is_alive() or self.closing.is_set() or self.phase == "FAULT":
                raise RuntimeError("CAN 工作线程未运行；已清除控制请求，请重新连接只读反馈")
            self.reset_status = dict(id=secrets.token_hex(8), status="queued", message="已取消旧目标，等待失能并重置")
            self._mode_restore_review = _mode_review
            self.mode_restore_cancelled.clear()
            if _mode_review:
                self.reset_status.update(kind='reviewed-mode-only',message='已排队：按已确认历史基准仅恢复六轴运行模式')
            self.reset_pending = True
            return dict(self.reset_status)

    def request_mode_restore(self, request):
        with self.lock:
            self._check_register_audit_idle()
            if self._register_audit or self.gripper.state()['recovery_required']:
                raise RuntimeError('须等待只读诊断完成，且J7无待恢复状态')
            # Copy the exact request: a caller cannot change confirmations or
            # source hashes while the CAN owner is waiting for its next cycle.
            review = json.loads(json.dumps(request,allow_nan=False))
            validate_review(self,review)
            return self.request_reset(_mode_review=review)

    def _check_register_audit_idle(self):
        if (self.shutdown_sealed or self.closing.is_set() or self.reset_pending
                or self.phase not in ('READONLY', 'RECOVERY') or self.armed
                or self.session or self.bus.active or self.start_requested
                or self.target is not None or self.command_reference is not None):
            raise RuntimeError('寄存器核查仅允许只读/恢复锁、未准备、无控制者及无运动请求状态')
        g = self.gripper
        if g.authorized or g.pending or g.pending_grasp or g.manual or g.view and g.view.active:
            raise RuntimeError('J7仍有控制或出力请求，不能开始寄存器核查')
        if not 0 <= self.clock()-self._feedback_refreshed_at < self.feedback_timeout:
            raise RuntimeError('寄存器核查需要新鲜实机反馈')

    def request_register_audit(self):
        """Queue reads only. Never create drive views, original snapshots or permits."""
        with self.lock:
            self._check_register_audit_idle()
            if not self.worker or not self.worker.is_alive():
                raise RuntimeError('唯一CAN工作线程未运行')
            if self._register_audit:
                return dict(self.register_audit_status)
            self.register_audit_status = dict(id=secrets.token_hex(8), status='queued',
                completed=0, total=48, message='已排队，只读核查J1–J6当前寄存器',
                hardware_writes=0, recovery_authorized=False)
            self._register_audit = dict(schema='rebot-register-audit-v1', started_at=time.time(),
                deadline=self.clock()+15, recovery_required=dict(self.recovery_required or {}),
                observed_registers={str(i):{} for i in AXES}, samples=[], hardware_writes=0,
                pre_session_snapshot=False, identity_verified=False, recovery_authorized=False)
            return dict(self.register_audit_status)

    def _cancel_register_audit(self, message):
        if self._register_audit:
            self.register_audit_status = {**self.register_audit_status, 'status':'cancelled', 'message':message}
            self._register_audit = None

    def _register_audit_step(self):
        # Called only by the CAN owner after a normal feedback sweep. At most
        # one extra 50 ms read per cycle; no reset/disable/write on this path.
        with self.lock:
            audit = self._register_audit
            if audit is None:return
            error = None
            try:
                self._check_register_audit_idle()
                if self.clock() > audit['deadline']:
                    raise TimeoutError('寄存器只读核查超过15秒，未自动重试')
                keys = (*RUNTIME_KEYS, *PROTECTED)
                done = len(audit['samples'])
                axis, index = AXES[done//len(keys)], keys[done%len(keys)]
                self.bus.ownership.check()
                motor = self.bus.motors[axis]
                began = self.clock()
                value = (motor.robstride_get_param_i8(index,50) if index == 0x7005 else
                         motor.robstride_get_param_f32_host_id(index,0xFD,50))
                self.bus.ownership.check()
                self._check_register_audit_idle()
                if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
                    raise ValueError(f'J{axis} {index:#x} 读数无效')
                if index == 0x7005 and type(value) is not int:
                    raise ValueError(f'J{axis} 模式读数不是整数')
                audit['observed_registers'][str(axis)][hex(index)] = value
                audit['samples'].append(dict(axis=axis,index=hex(index),value=value,
                    requested_at=began,received_at=self.clock(),received_wall_time=time.time()))
                self.register_audit_status = {**self.register_audit_status,
                    'status':'running', 'completed':done+1, 'message':f'已只读核查 {done+1}/48 项'}
                if done+1 < 48:return
            except Exception as exc:
                error = str(exc)
            audit.pop('deadline')
            audit.update(finished_at=time.time(), complete=error is None, error=error)
            path = None
            if self.output:
                try:
                    path = self.output/f"register-audit-{time.time_ns()}-{self.register_audit_status['id']}.json"
                    with path.open('x') as out:
                        json.dump(audit,out,ensure_ascii=False,indent=2,allow_nan=False)
                except (OSError,TypeError,ValueError) as exc:
                    error = f'{error + "; " if error else ""}诊断报告保存失败：{exc}'
                    path = None
            self.register_audit_status = {**self.register_audit_status,
                'status':'failed' if error else 'succeeded', 'report':str(path) if path else None,
                'message':error or '当前寄存器只读核查完成；未恢复参数、未解除恢复锁'}
            self._register_audit = None

    def get_joint_state(self, request_feedback=False):
        # ROS publishers never compete with the CAN owner for parameter reads.
        with self.lock:
            return self._q.copy(), self._v.copy(), np.zeros(6)

    def validate_preview(self, start, target, *, support_recovery=False):
        if self.support_recovery:
            if not support_recovery:
                raise ValueError('正在执行支撑恢复，只接受已预览的恢复路线')
            self.support_recovery.preview_segment(self.support_recovery.stage,start,target)
            if self.support_recovery.stage==0:self._check_recovery_path(start,target,self.support_recovery,0)
            elif self.path_check:self.path_check(start,target)
            return
        if support_recovery:
            raise ValueError('支撑恢复许可尚未建立')
        if self.max_twin_delta is not None and max(abs(a-b) for a,b in zip(start,target)) > self.max_twin_delta + 1e-9:
            raise ValueError("实机每次接管仅允许各轴偏移 0.5°；请缩小坐标或姿态变化")
        for q,(lo,hi) in zip(target,self.teleop_limits):
            if not lo <= q <= hi:
                raise ValueError("预览目标须在名义关节限位内")
        if self.path_check:
            self.path_check(start,target)

    def plan_support_recovery(self,start,target):
        self._require_support_recovery_backend()
        plan=build_support_recovery(start,target,self.teleop_limits)
        policy=SupportRecovery(plan,self.teleop_limits)
        for stage,goal in enumerate(policy.waypoints):
            begin=policy.start if stage==0 else policy.waypoints[stage-1]
            self.validate_support_recovery_preview(begin,goal,plan,stage)
        return plan

    def validate_support_recovery_preview(self,start,target,plan,stage):
        self._require_support_recovery_backend()
        candidate=SupportRecovery(plan,self.teleop_limits)
        policy=self.support_recovery
        if policy is None:
            policy=candidate
        elif (np.max(np.abs(policy.start-candidate.start))>1e-9 or
              np.max(np.abs(policy.target-candidate.target))>1e-9 or
              any(np.max(np.abs(a-b))>1e-9 for a,b in zip(policy.waypoints,candidate.waypoints))):
            raise ValueError('支撑恢复预览与当前活动路线不一致')
        policy.preview_segment(stage,start,target)
        if stage==0:self._check_recovery_path(start,target,policy,stage)
        elif self.path_check:self.path_check(start,target)

    def begin_support_recovery(self,token,plan):
        with self.lock:
            self._require(token)
            self._require_support_recovery_backend()
            if self.support_recovery:raise RuntimeError('已有支撑恢复许可，须先停止并重新预览')
            if self.control_mode!='full' or self.phase!='READONLY' or self.motion_ready or self.bus.active:
                raise RuntimeError('支撑恢复仅能从完整控制只读、未使能状态启动')
            now=self.clock()
            if set(self.samples)!=set(AXES):raise RuntimeError('支撑恢复需要完整六轴新鲜反馈')
            for i,sample in self.samples.items():
                if sample.mode!=0 or not 0<=now-sample.requested_at<=self.feedback_timeout or abs(sample.qd)>.01:
                    raise RuntimeError(f'J{i} 必须是模式0、新鲜且静止的起点')
            policy=SupportRecovery(plan,self.teleop_limits)
            q=self._q.copy()
            if not np.all(np.isfinite(q)) or np.max(np.abs(q-policy.start))>START_MATCH_RAD:
                raise RuntimeError('实测支撑恢复起点与只读预览相差超过 0.1°，请重新预览')
            # Keep the previewed route; recheck from fresh feedback within the
            # existing 0.1-degree start tolerance, including encoder noise on J1.
            canonical=policy.payload()
            for stage,goal in enumerate(policy.waypoints):
                begin=q if stage==0 else policy.waypoints[stage-1]
                self.validate_support_recovery_preview(begin,goal,canonical,stage)
            self.support_recovery=policy
            self.bus.support_recovery=self.support_recovery
            return self.support_recovery.payload()

    def _check_recovery_path(self,start,target,recovery,stage):
        if self.recovery_path_check is None:raise ValueError('驱动没有加载支撑恢复专用路径校验')
        nominal_limits=gravity_limits_for_mode(self.config.version)
        recovery_limits=gravity_limits_for_recovery_stage(stage,nominal_limits)
        self.recovery_path_check(start,target,recovery.payload(),stage,recovery_limits)

    def _require_support_recovery_backend(self):
        if (self.control_mode!='full' or self.config.version not in OFFICIAL_FULL_MODES or
                getattr(self.bus,'drive_backend','csp')!='csp'):
            raise RuntimeError('支撑恢复仅允许完整控制的 CSP 后端；MIT/小幅方案不开放恢复力矩例外')

    def complete_support_recovery_segment(self,arrival_tolerance_rad=START_MATCH_RAD):
        with self.lock:
            if not self.support_recovery:raise RuntimeError('支撑恢复许可已失效')
            done=self.support_recovery.complete_segment(self._q.copy(),arrival_tolerance_rad)
            if done:self._clear_support_recovery()
            return done

    def _clear_support_recovery(self):
        self.support_recovery=None
        self.bus.support_recovery=None
        for drive in getattr(self.bus,'drives',{}).values():drive.support_recovery=None
        if self.executor:self.executor.support_recovery=None

    def teleop_acquire(self):
        with self.lock:
            if self.shutdown_sealed:raise RuntimeError('项目正在结束，禁止重新接管')
            self._require_recovered()
            if self.phase == 'HOLDING' and not self.session:
                validate_takeover(self)
                self.session = secrets.token_hex(16)
                self.last_heartbeat = self.clock()
                self.last_seq = -1
                self.hold_takeover = True
                return self.session
            if not self.armed or self.phase != "READONLY" or self.session:
                raise RuntimeError("请先准备控制，等待上次停止完成后再接管")
            if self.clock()-self._feedback_refreshed_at > self.feedback_timeout:
                raise RuntimeError("实机反馈已过期")
            self.origin = self._q.copy()
            self.gripper.authorized = False
            self.command_reference = None
            self.stop_reason = ""
            self.stop_request = None
            self.stopped_session = ""
            self.session = secrets.token_hex(16)
            self.last_heartbeat = self.clock()
            self.last_seq = -1
            self.target = None
            self.paused = True
            self.point_reference = self.stream_velocity = None
            self.stream_ramp.reset();self.stream_ruckig.reset()
            self.cancel_motion.clear()
            return self.session

    def _require(self, token):
        if not self.session or token != self.session:
            raise RuntimeError("CSP 控制租约已失效")

    def teleop_heartbeat(self, token):
        with self.lock:
            self._require(token)
            self.last_heartbeat = self.clock()
            self.input_seq += 1
            if self.executor:
                self.executor.heartbeat(self.executor.session, self.input_seq, True)

    def teleop_pause(self, token):
        with self.lock:
            self._require(token)
            self.gripper.pause()
            self.paused = True
            self.target = None
            self.point_reference = self.stream_velocity = None
            self.stream_ramp.reset();self.stream_ruckig.reset()
            # Releasing input during mode preparation must prevent later enable.
            if self.phase == "STARTING":
                self.cancel_motion.set()
            self.start_requested = False

    def teleop_resume(self, token):
        with self.lock:
            self._require(token)
            if self.cancel_motion.is_set():
                raise RuntimeError("本次启动已停止，请释放后重新接管")
            self.paused = False
            self.last_sample = self.clock()
            self.start_requested = True

    def teleop_send(self, token, seq, sampled_at, targets, gripper, speed):
        with self.lock:
            if self.support_recovery:raise ValueError('支撑恢复期间禁止普通手动目标')
            self._validate_input(token,seq,sampled_at,speed)
            target = np.asarray(finite_vector(targets,6))
            self._check_target(target)
            self.target = target
            self.point_reference = self.stream_velocity = None
            self.speed = float(speed)
            self.last_seq = seq
            self.last_sample = self.clock()

    def _validate_input(self,token,seq,sampled_at,speed):
        self._require(token)
        if seq <= self.last_seq or not -.1 <= self.clock()-sampled_at <= .25:
            raise ValueError('CSP 丢弃过期或乱序目标')
        if not math.isfinite(speed) or not self.min_twin_speed <= speed <= self.max_twin_speed:
            raise ValueError(f'当前模式关节速度须为 {self.min_twin_speed:g}–{self.max_twin_speed:g} rad/s')

    def teleop_point(self, token, seq, sampled_at, trajectory, speed):
        # The ROS timer refreshes one immutable analytic curve; the CAN owner
        # samples it between parameter replies instead of replaying 20 Hz steps.
        with self.lock:
            self._validate_input(token,seq,sampled_at,speed)
            start=finite_vector(trajectory['start'],6)
            goal=finite_vector(trajectory['joints'],6)
            is_recovery=trajectory.get('support_recovery') is True
            if self.support_recovery:
                if not is_recovery:raise ValueError('支撑恢复期间只接受已预览的支撑程序目标')
                self.support_recovery.begin_segment(start,goal)
            elif is_recovery:raise ValueError('支撑恢复许可尚未建立')
            duration=float(trajectory['duration']);began=float(trajectory['began'])
            if not math.isfinite(duration) or duration <= 0 or not math.isfinite(began):
                raise ValueError('坐标轨迹时间无效')
            delta=max(abs(a-b) for a,b in zip(start,goal))
            required=max(1.875*delta/speed,math.sqrt(5.78*delta/self.motion_limits()[0]),
                (60*delta/self.motion_limits()[1])**(1/3))
            if duration+1e-9<required: raise ValueError('坐标轨迹时间不满足速度/加速度/jerk上限')
            previous=self.point_reference
            changed = previous is None or began!=previous['began'] or duration!=previous['duration'] or not np.array_equal(goal,previous['goal']) or not np.array_equal(start,previous['start'])
            if changed:
                self._check_target(goal)
            self.target=np.asarray(goal);self.stream_velocity=None
            self.speed=float(speed);self.last_seq=seq;self.last_sample=self.clock()
            if changed:
                self.point_reference=dict(start=np.asarray(start),goal=np.asarray(goal),duration=duration,began=began,elapsed=0.)

    def teleop_stream(self, token, seq, sampled_at, targets, velocity, speed):
        velocity=np.asarray(finite_vector(velocity,6))
        target=np.asarray(finite_vector(targets,6))
        if np.max(np.abs(velocity)) > speed+1e-9: raise ValueError('手动参考速度超出输入上限')
        with self.lock:
            if self.support_recovery:raise ValueError('支撑恢复期间禁止普通手动速度输入')
            self._validate_input(token,seq,sampled_at,speed)
            self._check_target(target,check_path=False)
            start=self._q.copy()
        # Model scratch data is thread-local. Do not block CAN feedback and
        # reference maintenance while the ROS dispatcher checks a new input.
        if self.path_check:self.path_check(start,target)
        with self.lock:
            # A stop, expiry or inward-boundary recovery may happen while the
            # model runs. Revalidate before accepting anything; the CAN owner
            # still checks the complete actual path before every send.
            self._validate_input(token,seq,sampled_at,speed)
            self._check_target(target,check_path=False)
            continuing=self.stream_velocity is not None
            self.target=target;self.point_reference=None
            self.speed=float(speed);self.last_seq=seq
            # Dispatcher waiting time counts toward the manual sample's TTL.
            self.last_sample=min(self.clock(),sampled_at)
            if not continuing:
                self.stream_ramp.reset();self.stream_ruckig.reset()
            self.stream_velocity=velocity

    def _desired_reference(self, now, held, dt=None):
        if self.target is None or self.paused or now-self.last_sample >= .25:
            self.stream_ramp.reset()
            self.stream_ruckig.reset()
            return held.copy()
        if self.point_reference:
            p=self.point_reference
            if self.control_mode == 'full' and self.motion_profile == 'ruckig':
                # Virtual trajectory time advances by at most 50 ms after an
                # SDK stall. It never jumps forward to the old wall-clock point.
                p['elapsed'] = p.get('elapsed',0.)+min(.05,max(0.,dt or 0.))
                try:
                    return point_sample('ruckig',p['start'],p['goal'],self.speed,
                        *self.motion_limits(),
                        p['duration'],p['elapsed'])
                except RuntimeError as exc:
                    raise TimeoutError(str(exc)) from exc
            t=np.clip((now-p['began'])/p['duration'],0.,1.)
            return p['start']+(p['goal']-p['start'])*t**3*(10-15*t+6*t*t)
        if self.stream_velocity is not None:
            if self.continuous_jog_reference and dt is not None:
                # Integrate fresh velocity on the CAN cadence, independent of
                # browser packet spacing. Never catch up a blocked interval.
                lo,hi=np.asarray(self.teleop_limits).T
                if self.motion_profile == 'ruckig':
                    try:
                        return self.stream_ruckig.step(held,self.stream_velocity,dt,self.speed,
                            self.motion_limits()[0],lo,hi,
                            jerk=self.motion_limits()[1])
                    except RuntimeError as exc:
                        raise TimeoutError(str(exc)) from exc
                return self.stream_ramp.step(held,self.stream_velocity,dt,self.speed,
                    self.motion_limits()[0],lo,hi,speed_fraction=1.)
            else:
                # Legacy small mode retains its one-interval prediction.
                predicted=self.target+self.stream_velocity*min(.05,max(0.,now-self.last_sample))
            lo,hi=np.asarray(self.teleop_limits).T
            return np.clip(predicted,np.minimum(held,lo),np.maximum(held,hi))
        return self.target.copy()

    def _check_target(self, target, *, check_path=True):
        if self.support_recovery:
            if not self.support_recovery.allows_reference(target):
                raise ValueError('目标偏离已预览的支撑恢复路线')
            if check_path:
                if self.support_recovery.active:
                    if self.support_recovery.stage==0:self._check_recovery_path(self._q,target,self.support_recovery,0)
                    elif self.path_check:self.path_check(self._q,target)
            return
        if self.origin is None or (self.max_twin_delta is not None and np.max(np.abs(target-self.origin)) > self.max_twin_delta+1e-9):
            raise ValueError("已到本次接管 0.5° 范围，请松开并释放控制")
        for i,(q,(lo,hi)) in enumerate(zip(target,self.teleop_limits)):
            start = self.origin[i]
            drive = self.bus.drives.get(i+1)
            if drive and i+1 in self.bus.active and drive.last_boundary_target is not None:
                # The first enabled reference is the fresh measured preload,
                # not the slightly older encoder sample taken at acquisition.
                # Later references may only recover inward from that fixed
                # command; encoder noise never moves this boundary outward.
                start = drive.last_boundary_target
            if lo <= q <= hi:
                continue
            # Preserve measured first pose in the already-confirmed 1° start
            # tolerance; no offset/limit write and no movement farther outward.
            recovered = i+1 in self.bus.boundary_recovered if drive and i+1 in self.bus.active else False
            if not recovered and lo-self.start_tolerance <= start < lo and start-1e-9 <= q < lo:
                continue
            if not recovered and hi < start <= hi+self.start_tolerance and hi < q <= start+1e-9:
                continue
            raise ValueError(f"J{i+1} CSP 目标 {math.degrees(q):.4f}° 越过关节限位或起点恢复方向；"
                f"参考 {math.degrees(start):.4f}°，名义范围 {math.degrees(lo):.2f}–{math.degrees(hi):.2f}°")
        if check_path and self.path_check:
            self.path_check(self._q,target)

    def teleop_stop(self, token, reason="stopped"):
        with self.lock:
            self._require(token)
            self.stopped_session,self.stopped_at = token,self.clock()
            self.stop_reason = str(reason)[:160]
            self.session = ""
            self.cancel_motion.set()
            self.start_requested = False
            self.target = None
            self._clear_support_recovery()

    def suspend(self, reason):
        """Revoke motion permission without withdrawing gravity support."""
        with self.lock:
            if self.control_mode == 'full' and self.phase in ('ACTIVE','HOLDING'):
                if self.phase == 'ACTIVE':
                    self.stop_reason = str(reason)[:160]
                    self.hold_recoverable = isinstance(reason, FeedbackTimeout)
                    self.hold_requested.set()
                return True
            # No full active drive hold exists during a startup refusal.
            if self.session:
                self.teleop_stop(self.session, str(reason))
            return False

    def note_stop_request(self, token, reason, stamp_ms):
        # Diagnostic only: never enables, stops, renews or acquires anything.
        # A separate ROS queue preserves the notice when the priority stop
        # service arrives before the depth-one release command.
        with self.lock:
            valid_token = token and (token == self.session or
                token == self.stopped_session and 0 <= self.clock()-self.stopped_at <= 2.)
            if not valid_token or not isinstance(reason,str) or not reason or len(reason)>160:
                return
            if type(stamp_ms) not in (int,float) or not math.isfinite(stamp_ms) or abs(time.time()*1000-stamp_ms)>2000:
                return
            self.stop_request = dict(reason=reason,stamp_ms=stamp_ms,received_at=self.clock(),source="browser notice; diagnostic only")

    def teleop_watchdog(self):
        with self.lock:
            if self.session and self.clock()-self.last_heartbeat > .7:
                self.suspend("网页心跳超过700 ms，锁定暂停")

    def _cache(self, samples):
        with self.lock:
            self.samples.update(samples)
            if all(i in self.samples for i in AXES):
                self._q = np.array([self.samples[i].q for i in AXES])
                self._v = np.array([self.samples[i].qd for i in AXES])
                self._feedback_refreshed_at = min(self.samples[i].requested_at for i in AXES)

    def _read(self, check=lambda:None, *, axes=AXES):
        samples = self.bus.read(axes,self.config.policy,check,on_sample=self._cache)
        self._cache(samples)
        return samples

    def _read_arm_cycle(self, check=lambda:None, *, on_io=None):
        # Stop/reset deliberately clears the velocity window. Rebuild it with
        # the original full read before resuming interleaved read-only polling.
        if any(i not in self.bus.previous_samples for i in AXES):
            samples = self._read(check)
        elif self.control_mode == 'full' and self.phase == 'ACTIVE':
            # The staggered two-pair schedule aged diagnostics over three
            # cycles. A ~300 ms SDK reply then exhausted their 600 ms window.
            # Read each axis's mode/current/position together every cycle;
            # still yield the bounded motion reference after EVERY reply.
            samples = self.bus.read_cycle(self.config.policy,AXES,check,
                on_sample=self._cache,on_io=on_io,interleaved=True)
        else:
            def diagnostic_time(i):
                sample = self.bus.latest_samples[i]
                return min(sample.mode_requested_at if sample.mode_requested_at is not None else sample.requested_at,
                    sample.diagnostics_requested_at if sample.diagnostics_requested_at is not None else sample.requested_at)
            # Two axes per cycle leave room for an SDK retry within the SAME
            # 600 ms limit. Choose actual oldest fields, not a fixed phase index.
            axes = sorted(AXES,key=diagnostic_time)[:2]
            samples = self.bus.read_cycle(self.config.policy,axes,check,on_sample=self._cache,on_io=on_io)
        self._cache(samples)
        return samples

    def _read_cycle(self, check=lambda:None, *, on_io=None):
        began = self.clock()
        stepping_gripper = False
        def gripper_check():
            check()
            if self.phase == 'ACTIVE' and on_io and self.gripper.preparing and not self.gripper.reading:
                # J7 startup contains a snapshot and several verified
                # writes. Keep six-axis telemetry fresh between those
                # operations on this same owner, within the old deadlines.
                oldest = min(stamp for i in AXES for stamp in (
                    self.samples[i].requested_at,self.samples[i].mode_requested_at,
                    self.samples[i].diagnostics_requested_at) if stamp is not None)
                if self.clock()-oldest >= .2:
                    # A full sweep can itself exceed 200 ms. Repeating it
                    # at every J7 check starves J7; service three oldest
                    # fields here and retain each field's real timestamp.
                    samples=self.bus.read_cycle(self.config.policy,AXES,check,
                        on_sample=self._cache,on_io=on_io,interleaved=True,max_fields=3)
                    self._cache(samples)
            if on_io:self._timed('reference-advance',on_io)
        def step_gripper():
            nonlocal stepping_gripper
            if stepping_gripper:return
            stepping_gripper = True
            try:
                g = self.gripper
                if self.phase == 'ACTIVE' and on_io and g.sample is not None and not g.request_probe and (
                        g.view and g.view.active or not (g.pending or g.pending_grasp or g.manual)):
                    oldest = min(stamp for i in AXES for stamp in (
                        self.samples[i].requested_at,self.samples[i].mode_requested_at,
                        self.samples[i].diagnostics_requested_at) if stamp is not None)
                    # Compare individual J7 fields too, including an inactive
                    # probed gripper. Never block older arm fields behind a
                    # three-query group after a delayed J7 reply.
                    g.step(gripper_check,poll=g.feedback_time()<=oldest,one_field=True)
                    return
                self.gripper.step(gripper_check)
            finally:stepping_gripper = False
        def arm_io():
            self._timed('reference-advance',on_io)
            # A complete six-axis sweep can take 300–500 ms. Service J7
            # between replies so its feedback and bounded reference updates
            # do not depend on that sweep. Nested J7 checks retain the plain
            # arm-reference callback, never recursively entering this step.
            if self.phase == 'ACTIVE':step_gripper()
        self._read_arm_cycle(check,on_io=arm_io if on_io and self.phase=='ACTIVE' else on_io)
        if self.phase == 'HOLDING':self.gripper.observe_hold(check)
        else:step_gripper()
        self.poll_duration_s = self.clock()-began
        return dict(self.samples)

    def _worker(self):
        try:
            self.bus.open_readonly()
            self._read()
            self.device_speed_limits = {}
            for i in AXES:
                value=self.bus.motors[i].robstride_get_param_f32_host_id(0x7017,0xFD,50)
                if not math.isfinite(value) or value <= 0: raise ValueError(f'J{i} 设备限速无效')
                self.device_speed_limits[i]=value
            self._configure_mode(self.control_mode)
            self._read()
            self._readonly_phase()
            while not self.closing.is_set():
                if self.reset_pending:
                    self._reset_control()
                elif self.start_requested and self.session:
                    self._motion_session()
                else:
                    began = self.clock()
                    try:
                        self._read_cycle()
                        self._register_audit_step()
                    except TimeoutError as exc:
                        # A lost read while no drive is active must not kill
                        # the only CAN observer forever. Rebuild the velocity
                        # window with two fresh sweeps; revoke any preparation
                        # or lease and retain every prior recovery lock.
                        if self.bus.active or self.gripper.view and self.gripper.view.active:
                            raise
                        with self.lock:
                            self._cancel_register_audit('反馈中断，已取消只读核查；未自动重试')
                            self.phase='FEEDBACK_WAIT'
                            self.error=f'只读反馈恢复中：{exc}；禁止使能，恢复后需重新准备与接管。'
                            self.armed=False;self.session='';self.start_requested=False
                            self.paused=True
                            self.target=self.point_reference=self.stream_velocity=self.command_reference=None
                            self.stream_ramp.reset();self.stream_ruckig.reset()
                            self.gripper.authorized=False;self.gripper.pause()
                        self.bus.previous_samples.clear()
                        self.closing.wait(.1)
                        continue
                    if self.phase=='FEEDBACK_WAIT':
                        with self.lock:
                            self.error='只读反馈已恢复；未使能，旧控制已作废，请重新准备与接管。'
                            self._readonly_phase()
                    self.teleop_watchdog()
                    self.closing.wait(max(0.,.09-(self.clock()-began)))
        except Exception as exc:
            self.error = str(exc)
            self.armed = False
            self.phase = "FAULT"
            self.session = ""
            if self.reset_pending:
                self.reset_pending = False
                self.reset_status = {**self.reset_status,"status":"failed","message":f"控制线程异常，重置未完成：{exc}；请重连只读反馈"}
        finally:
            with self.lock:
                self._cancel_register_audit('CAN工作线程结束，只读核查未完成')
            if self.bus.active:
                self.bus.stop_supported()
            try:
                self.gripper.finish()
            except Exception as exc:
                self.error = f'J7恢复未完成：{exc}'
            self.bus.close()

    def probe_gripper(self):
        with self.lock:
            self.gripper.request_probe = True

    def gripper_state(self):
        with self.lock:
            return self.gripper.state()

    def authorize_gripper(self, token, enabled):
        with self.lock:
            self._require(token)
            if enabled and (self.control_mode != 'full' or self.gripper.state()['recovery_required'] or not self.gripper.state()['fresh']):
                raise ValueError('请选完整控制，并先检查J7新鲜反馈及恢复状态')
            if enabled and self.gripper.sample.mode != (5 if self.gripper.view and self.gripper.view.active else 0):
                raise ValueError('J7运行模式与已知控制状态不符，不能授权')
            self.gripper.authorized = enabled
            if not enabled:self.gripper.pause()

    def gripper_target(self, token, position, speed):
        from .twin_program import number
        with self.lock:
            self._require(token)
            if not self.gripper.authorized or not self.motion_ready:
                raise ValueError('请先使能机械臂保持，并单独授权J7')
            if not self.gripper.state()['fresh'] or self.gripper.state()['recovery_required']:
                raise ValueError('J7反馈过期或参数恢复待核查')
            self.gripper.pending = (number(position, '夹爪角度', 0, 5),
                                    number(speed, '夹爪速度', .005, self.gripper.speed_max))
            self.gripper.pending_grasp = None
            self.gripper.grasp = None
            self.gripper.manual = None
            self.gripper.arrived = False

    def gripper_grasp(self, token, profile, speed):
        from .twin_grasp import GraspMonitor, MAX_GRASP_SPEED, validate_profile
        from .twin_program import number
        import secrets
        profile = validate_profile(profile)
        with self.lock:
            self._require(token)
            state = self.gripper.state()
            if not self.gripper.authorized or not self.motion_ready or not state['fresh'] or state['recovery_required']:
                raise ValueError('夹持需要已使能保持、有效J7授权及新鲜反馈')
            speed = number(speed, '接触检测闭合速度', .005, min(MAX_GRASP_SPEED, self.gripper.speed_max))
            if speed <= profile.get('stall_velocity', 0)*2:
                raise ValueError('闭合速度须大于停滞阈值的两倍')
            if abs(state['velocity']) > .01:raise ValueError('J7仍在运动，不能开始夹持判定')
            job_id = secrets.token_hex(12)
            monitor = GraspMonitor(profile, state['position'], self.clock(), job_id)
            self.gripper.pause()
            self.gripper.grasp = monitor
            self.gripper.pending_grasp = dict(id=job_id, profile=monitor.profile, speed=speed)
            return job_id

    def validate_gripper_jog(self, token, velocity):
        with self.lock:
            self._require(token)
            if self.control_mode != 'full' or not self.gripper.state()['fresh'] or self.gripper.state()['recovery_required']:
                raise ValueError('夹爪需要完整控制和新鲜J7反馈，恢复状态须正常')
            if not math.isfinite(velocity) or abs(velocity) > self.gripper.speed_max:
                raise ValueError(f'J7速度超过当前设备上限 {self.gripper.speed_max:g} rad/s')

    def gripper_jog(self, token, velocity):
        with self.lock:
            self._require(token)
            if not velocity:
                if self.gripper.manual:self.gripper.pause()
                return
            self.validate_gripper_jog(token, velocity)
            if not self.motion_ready:raise ValueError('先使能机械臂保持')
            if not self.gripper.authorized:self.authorize_gripper(token, True)
            self.gripper.pending = self.gripper.curve = None
            self.gripper.pending_grasp = self.gripper.grasp = None
            self.gripper.manual = dict(velocity=velocity, at=self.clock())

    def _reset_control(self):
        review = self._mode_restore_review
        self.phase = "RESETTING"
        self.reset_status = {**self.reset_status,"status":"running","message":"正在失能、核对连续停稳和原参数"}
        report = dict(backend="csp-control-reset-v1", started_at=time.time(), cleanup_ok=False,
                      source_report=(self.recovery_required or {}).get("report"),
                      snapshot_required=bool(self.recovery_required), register_log=[])
        if review:
            # Deliberately not an ordinary reset lineage or a pre-session
            # snapshot. A failed attempt cannot authorize a generic reset retry.
            report.update(backend='csp-reviewed-mode-restore-v1',restore_scope=['0x7005'],
                          original_incident_source_known=False)
        path = self.output/f"csp-session-{time.time_ns()}.json" if self.output else None

        def save():
            if path:
                path.parent.mkdir(parents=True,exist_ok=True)
                temporary = path.with_suffix(".tmp")
                if review:
                    with temporary.open('w') as stream:
                        json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
                        stream.flush();os.fsync(stream.fileno())
                else:
                    temporary.write_text(json.dumps(report,ensure_ascii=False,indent=2))
                temporary.replace(path)
                if review:
                    directory = os.open(path.parent,os.O_RDONLY)
                    try:os.fsync(directory)
                    finally:os.close(directory)

        def check():
            if review and self.mode_restore_cancelled.is_set():
                raise InterruptedError('停止请求已取消模式恢复；保留锁定')
            if self.closing.is_set():
                raise InterruptedError("服务正在停止，保留恢复锁定")
            if self.clock()-self._feedback_refreshed_at > self.feedback_timeout:
                raise TimeoutError("重置时反馈过期，保留恢复锁定")

        def restore_check():
            # A six-axis register scan can take longer than one encoder age
            # window. Interleave feedback on the same owner, never another bus.
            if self.clock()-self._feedback_refreshed_at > .2:
                self._read()
            check()
            if any(abs(s.qd) > .01 for s in self.samples.values()):
                raise RuntimeError("参数核对/恢复期间位置变化，停止恢复并保留锁定")

        try:
            if review:
                # Review is checked again on the owner before any disable/write.
                # J7 is outside this maintenance scope, including recovery.
                with self.lock:
                    if self.gripper.state()['recovery_required'] or self.gripper.view and self.gripper.view.active:
                        raise RuntimeError('J7状态改变，不能执行六轴模式恢复')
            else:
                self.gripper.finish(recover=True)
            source_error = None
            originals = None
            version = MODE
            try:
                if review:
                    source,originals,version,evidence = validate_review(self,review)
                    report.update(reviewed_source=evidence,reference_snapshot=source['register_snapshot'])
                else:
                    source, source_path, digest = reset_source(self)
                    report.update(source_report=source_path,source_sha256=digest)
                if source and not review:
                    originals,version = validate_snapshot(source,self.teleop_limits)
                    report["register_snapshot"] = source["register_snapshot"]
                    self._reset_original = source
                report["snapshot_required"] = bool(source)
            except Exception as exc:
                source_error = exc
            if review and source_error:
                raise source_error
            self.bus.reset_views(originals,mode=version)
            # Persist the recovery lineage before any hardware operation.
            report["stop"] = dict(attempted_axes=list(AXES))
            if review:report['write_intent'] = True
            journal_error = None
            try:
                save()
            except OSError as exc:
                journal_error = exc
            if review and journal_error:
                raise RuntimeError(f'模式恢复意图未持久保存；未发送失能或参数写入：{journal_error}')
            if review:check()
            report["stop"] = self.bus.disable_all_for_reset()
            if report["stop"]["errors"]:
                raise RuntimeError("六轴失能状态未全部确认")
            quiet = None
            began = self.clock()
            observations = report["stationary_observations"] = []
            self.bus.previous_samples.clear()
            while True:
                self._read()
                check()
                now = self.clock()
                observations.append({str(i):asdict(s) for i,s in self.samples.items()})
                valid = len(self.samples) == 6 and all(
                    0 <= now-s.requested_at <= self.feedback_timeout and abs(s.qd) <= .01 for s in self.samples.values())
                quiet = (now if quiet is None else quiet) if valid else None
                if quiet is not None and now-quiet >= .3:
                    break
                if now-began > 2.:
                    raise RuntimeError("未确认连续停稳，保留低限制；请检查机械状态")
                self.closing.wait(.015)
            if source_error:
                raise source_error
            if journal_error:
                raise RuntimeError(f"无法保存重置记录，已失能；未恢复参数：{journal_error}")
            if originals:
                report["observed_registers"] = (read_reviewed_registers(self.bus,originals,restore_check) if review else
                                                check_current_registers(self.bus,originals,restore_check))
                # Refresh after register reads and check stability again before
                # allowing restore-only writes. Never load an old position.
                self._read();check()
                if any(abs(s.qd) > .01 for s in self.samples.values()):
                    raise RuntimeError("参数核对期间位置变化，未恢复")
                for axis,drive in self.bus.drives.items():
                    drive.changed = ({0x7005} if report['observed_registers'][str(axis)]['0x7005'] != 0 else set()) if review else set(RUNTIME_KEYS)
                report["restoration"] = self.bus.restore(restore_check)
                if not report["restoration"]["ok"]:
                    raise RuntimeError("临时参数未全部恢复，保留锁定")
                if review:
                    report['verified_registers'] = read_reviewed_registers(self.bus,originals,restore_check,require_zero=True)
            else:
                if any(s.mode != 0 for s in self.samples.values()):
                    raise RuntimeError("当前模式非 0，缺少事故前完整参数快照；未覆盖默认值")
                report["restoration"] = dict(ok=True,restored={},old_target_restored=False)
            self.bus.previous_samples.clear()
            self._read()
            if any(s.mode != 0 for s in self.samples.values()):
                raise RuntimeError("重置后模式读回不符，保留锁定")
            report["cleanup_ok"] = True
        except Exception as exc:
            report["cleanup_error"] = str(exc)
        finally:
            report.update(register_log=self.bus.register_log,finished_at=time.time())
            try:
                save()
            except OSError as exc:
                report.update(cleanup_ok=False,cleanup_error=f"恢复报告保存失败：{exc}")
                if review:
                    # A directory fsync can fail AFTER the successful-looking
                    # file was replaced. Withdraw that result on disk as well
                    # as in memory so a reconnect retains the recovery lock.
                    try:save()
                    except OSError:pass
            with self.lock:
                ok = report["cleanup_ok"]
                self.last_report = report
                self.recovery_required = None if ok else dict(report=str(path) if path else None,
                    reason="重置检查未通过",cleanup_error=report.get("cleanup_error"))
                self.session = ""
                self.armed = self.start_requested = False
                self.target = self.origin = self.executor = None
                self.last_seq = -1
                self.input_seq = 0
                self.reset_pending = False
                self._mode_restore_review = None
                self.error = ""
                self._readonly_phase()
                message = ("控制已重置：已失能，旧目标已清空，参数核对完成；未使能、未回零。"
                    if ok else f"控制输入已清空；实机恢复未通过：{report.get('cleanup_error')}。保持锁定。")
                if ok and review:
                    message = '六轴模式已恢复为0，其余42项读回一致；已回到只读，未使能或运动'
                self.reset_status = {**self.reset_status,"status":"succeeded" if ok else "failed","message":message}
                if ok:
                    self._reset_original = None

    def _motion_session(self):
        self.phase = "STARTING"
        self.bus.support_recovery=self.support_recovery
        executor = NativeXYZExecutor(self.config,self.bus,fk=lambda q:[0.,0.,0.],clock=self.clock)
        # Full sessions continue until released. Retain bounded diagnostics
        # rather than growing per-frame in-memory logs for the whole session.
        if self.control_mode == "full":
            executor.frames = deque(maxlen=3000)
        with self.lock:
            self.input_seq = 0
            executor.heartbeat(executor.session,0,True)
            self.executor = executor
        permit = OnsitePermit(self.config.fingerprint,AXES,self.clock()+(self.session_seconds+20 if self.session_seconds is not None else 2.),
            f"3002 本机用户接管 {self.control_mode} 控制；现场方向/零位/限位/断电已确认",AXES,True,True,
            coordinated_start_checked=True)
        deadline = None
        hold = None
        enabled = []
        report = {"started_at":time.time(),"origin":self.origin.tolist(),"backend":"csp-twin-v1",
                  "control_settings":self.control_settings(),"config_fingerprint":self.config.fingerprint,
                  "log_window":"last 3000 frames / 5000 register operations per axis" if self.control_mode == "full" else "complete"}

        def check():
            if self.hold_requested.is_set():
                raise (FeedbackTimeout if self.hold_recoverable else TimeoutError)(self.stop_reason or '通信异常，锁定暂停')
            if self.closing.is_set() or self.cancel_motion.is_set() or not self.session:
                raise InterruptedError(self.stop_reason or "已释放/停止控制")
            if self.clock()-self.last_heartbeat > .7:
                raise TimeoutError("网页心跳超时，停止实机")
            # Check expiration before renewal. A stalled/expired session is
            # never revived by a late heartbeat, and every write still has its
            # independent profile command and 600 ms feedback deadline.
            executor.check(permit)
            if self.session_seconds is None and 0 <= self.clock()-self._feedback_refreshed_at <= self.feedback_timeout:
                permit.expires_at = self.clock()+2.
            # Six serial encoder/mode/current reads normally take ~200 ms.
            # Refresh the same bounded reference between replies so a retry
            # cannot use the entire command window. send() still checks
            # the original deadline, heartbeat and <=600 ms feedback age.
            if self.phase in ("STARTING","ACTIVE") and hold is not None and enabled and executor.last_axis_command:
                age = max(self.clock()-t for t in executor.last_axis_command.values())
                if age > .12:
                    executor.send(hold.tolist(),dict(self.samples),permit,"hold",tuple(enabled))
                    executor.frames[-1]["refresh"] = True
            executor.check(permit)

        try:
            permit.validate(self.config,self.clock())
            samples = self._read(check)
            executor.validate(samples,quiet=True)
            if np.max(np.abs(self._q-self.origin)) > self.joint_arrival_tolerance:
                raise ValueError("实机起点改变，请释放后重新接管")
            # Explicit motor hold must pass the same model/path envelope as a
            # point goal, before any runtime write or drive enable.
            # validate() checks the fresh measured pose against the existing
            # 1° start margin, and the check above limits drift to 0.1°.
            # Check the intended held reference separately: two stationary
            # encoder reads need not be bit-identical at a nominal boundary.
            # Each drive still preloads its freshest position before enable.
            self._check_target(self.origin)
            self.bus.authorize(self.config,permit)
            if self.control_mode == "full":
                for drive in self.bus.drives.values():
                    drive.register_log = deque(maxlen=5000)
            for drive in self.bus.drives.values():
                check();drive.quiesce();check()
            for drive in self.bus.drives.values():
                drive.snapshot_registers(check)
                self._read(check)
            if self.output:
                # Both CSP and MIT need the actual pre-write snapshot even
                # if the process never reaches HOLDING or normal cleanup.
                self.output.mkdir(parents=True,exist_ok=True)
                path=self.output/f'csp-session-{time.time_ns()}.json'
                snapshot=dict(report,write_intent=True,cleanup_ok=False,
                    reason=f'{self.drive_settings.selected.upper()}启动写入意图；完整停止与恢复尚未确认',
                    register_log=self.bus.register_log,
                    register_snapshot={str(i):{hex(k):v for k,v in d.original.items()} for i,d in self.bus.drives.items()})
                temporary=path.with_suffix('.tmp')
                with temporary.open('w') as stream:
                    json.dump(snapshot,stream,ensure_ascii=False,indent=2)
                    stream.flush();os.fsync(stream.fileno())
                temporary.replace(path)
                directory_fd=os.open(self.output,os.O_RDONLY|os.O_DIRECTORY)
                try:os.fsync(directory_fd)
                finally:os.close(directory_fd)
                report['write_intent_report']=str(path)
            if self.control_mode == 'full' and min(d.runtime[0x7017] for d in self.bus.drives.values())+1e-6 < self.max_twin_speed:
                raise ValueError('设备限速在连接后降低，请重连只读反馈以更新页面生效上限；未提高设备限速')
            for i,drive in self.bus.drives.items():
                for index in (0x7005,0x700B,0x7018,0x7017):
                    drive.write_register(index,drive.runtime[index],check,phase="prepare")
                self._read(check)
            hold = self.origin.copy()
            for i,drive in self.bus.drives.items():
                samples = self._read(check)
                # Refresh the next drive after the full scan so other replies
                # do not age its preload. Preserve scan order/velocity windows.
                samples.update(self._read(check,axes=(i,)))
                executor.validate(samples,csp=True,quiet=True)
                if any(abs(samples[j].q-self.origin[j-1]) > self.joint_arrival_tolerance for j in AXES if j not in enabled):
                    raise ValueError("未使能轴起点变化，禁止追赶旧姿态")
                if enabled:
                    executor.send(hold.tolist(),samples,permit,"hold",tuple(enabled))
                hold[i-1] = samples[i].q
                drive.write_register(0x7016,float(hold[i-1]),check,phase="preload-current")
                first_age = self.clock()-samples[i].requested_at
                if first_age > self.config.policy.command_age:
                    raise TimeoutError(f"J{i} 首目标反馈过期（{first_age*1000:.0f} ms > "
                        f"{self.config.policy.command_age*1000:.0f} ms），该轴未使能；停止并清理本次启动")
                if "enable_attempted_at" not in report:
                    report["enable_attempted_at"] = self.clock()
                    if self.session_seconds is not None:
                        deadline = self.clock()+self.session_seconds
                        executor.deadline = deadline
                executor.last_axis_command[i] = self.clock()
                drive.enable(i);check();enabled.append(i)
                executor.send(hold.tolist(),samples,permit,"hold",tuple(enabled))
            with self.lock:
                self.command_reference = tuple(hold)
                report["enabled_start"] = list(hold)
                self.phase = "ACTIVE"
            last = self.clock()
            next_reference = last+.02
            self.last_sample = last
            self.command_period_s = None
            def advance_reference(force=False):
                nonlocal last,hold,next_reference
                now=self.clock()
                if not force and now < next_reference: return
                check()
                with self.lock:
                    try:
                        wanted=np.asarray(self._desired_reference(now,hold,now-last),dtype=float)
                        if wanted.shape != (6,) or not np.all(np.isfinite(wanted)):
                            raise ValueError('参考必须是六个有限数值')
                    except (TypeError,ValueError,ArithmeticError) as exc:
                        # Only pure reference generation is inside this gate.
                        # Drive reads, limits and electrical protection retain
                        # their existing stop/disable policy.
                        raise TimeoutError(f'运动参考计算异常：{exc}；保留最后目标，需明确重新接管') from exc
                    speed=self.speed
                self._timed('reference-path',self._check_target,wanted)
                # A late SDK reply must not produce a large catch-up jump.
                step=min(.05,now-last)*speed
                next_hold=hold+np.clip(wanted-hold,-step,step)
                sent_at=self._timed('reference-send',executor.send,next_hold.tolist(),dict(self.samples),permit,'trajectory')
                with self.lock:
                    if self.support_recovery:
                        self.support_recovery.advance_reference(next_hold)
                hold=next_hold
                with self.lock:
                    self.command_reference=tuple(hold)
                    self.command_period_s=sent_at-last
                last=sent_at
                # Keep a 20 ms phase despite reply jitter. Skip missed slots;
                # include computation/send time, not only the initial clock.
                # A preempted calculation must never trigger an immediate
                # catch-up frame with a fictitious elapsed velocity budget.
                finished=self.clock()
                next_reference += .02*max(1,math.floor((finished-next_reference)/.02)+1)
            while True:
                try:
                    began = self.clock()
                    interleaved=self.control_mode=='full'
                    samples = self._read_cycle(check,on_io=advance_reference if interleaved else None)
                    if interleaved:
                        advance_reference()
                        if self.closing.wait(.001): raise InterruptedError('服务停止')
                        continue
                    now = self.clock()
                    with self.lock:
                        wanted = self.target.copy() if self.target is not None and not self.paused and now-self.last_sample < .25 else hold.copy()
                        speed = self.speed
                    self._check_target(wanted)
                    step = min(.25,now-last)*speed
                    hold += np.clip(wanted-hold,-step,step)
                    executor.send(hold.tolist(),samples,permit,"trajectory")
                    with self.lock:
                        self.command_reference = tuple(hold)
                        self.command_period_s = now-last
                    last = now
                    if self.closing.wait(max(0.,.09-(self.clock()-began))):
                        raise InterruptedError("服务停止")
                except TimeoutError as exc:
                    if self.control_mode != 'full':
                        raise
                    report['last_communication_incident']=dict(at=self.clock(),reason=str(exc),
                        host_timings=list(self.host_timings),poll_io=list(self.bus.poll_io))
                    hold = wait_for_takeover(self,executor,permit,report,exc)
                    last = self.clock()
                    next_reference = last+.02
        except Exception as exc:
            report["reason"] = str(exc)
            self.error = "" if isinstance(exc,InterruptedError) else str(exc)
        finally:
            self.phase = "STOPPING"
            report["last_samples"] = {str(i):asdict(s) for i,s in self.samples.items()}
            report["stop_requested_at"] = self.clock()
            report["feedback_age_ms_at_stop"] = {str(i):round((self.clock()-s.requested_at)*1000,3) for i,s in self.samples.items()}
            try:
                if self.bus.active:
                    stop = self.bus.stop_supported()
                    report["stop"] = stop
                    try:
                        self.gripper.finish()
                    finally:
                        self.bus.previous_samples.clear()
                    if stop["errors"]:
                        raise RuntimeError("失能状态未全部确认")
                    quiet = None
                    began = self.clock()
                    observations = []
                    report["stationary_observations"] = observations
                    while True:
                        samples = self._read()
                        observations.append({str(i):asdict(s) for i,s in samples.items()})
                        now = self.clock()
                        valid = all(0 <= now-s.requested_at <= .6 and abs(s.qd) <= .01 for s in samples.values())
                        quiet = (now if quiet is None else quiet) if valid else None
                        if quiet is not None and now-quiet >= .3:
                            break
                        if now-began > 1.5:
                            raise RuntimeError("失能后未确认连续停稳，保留低限制")
                    report["restoration"] = self.bus.restore()
                    if not report["restoration"]["ok"]:
                        raise RuntimeError("临时参数未全部恢复")
                    # Register restoration can exceed the velocity window;
                    # require two new sweeps, as on initial read-only connect.
                    self.bus.previous_samples.clear()
                    self._read()  # publish the restored mode, not the old CSP sample
                report["cleanup_ok"] = True
            except Exception as exc:
                report["cleanup_ok"] = False
                report["cleanup_error"] = str(exc)
                self.error = str(exc)
            report["frames"] = list(executor.frames)
            report["browser_stop_request"] = self.stop_request
            report["register_log"] = self.bus.register_log
            report["register_snapshot"] = {str(i):{hex(k):v for k,v in d.original.items()} for i,d in self.bus.drives.items()}
            report["finished_at"] = time.time()
            self.bus.previous_samples.clear()
            self.last_report = report
            if not report.get("cleanup_ok"):
                self.recovery_required = dict(reason=report.get("reason"),cleanup_error=report.get("cleanup_error"))
            if self.output:
                self.output.mkdir(parents=True,exist_ok=True)
                path = self.output/f"csp-session-{time.time_ns()}.json"
                path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
                if self.recovery_required:
                    self.recovery_required["report"] = str(path)
            with self.lock:
                self.session = ""
                self.armed = False
                self.start_requested = False
                self.executor = None
                self.holding = None
                self.hold_requested.clear()
                self.target = None
                self.command_reference = None
                self._readonly_phase()

    def seal_readonly_shutdown(self):
        """Atomic exit gate. Never disables an active/holding arm."""
        with self.lock:
            self._require_recovered()
            if self.phase != 'READONLY' or self.armed or self.session or self.bus.active:
                raise RuntimeError('机械臂仍在控制、准备或保持中；请先支撑并明确失能，等待只读状态')
            if self.clock()-self._feedback_refreshed_at > self.feedback_timeout:
                raise RuntimeError('实机反馈未确认，不能结束项目')
            if self.gripper.view and self.gripper.view.active:
                raise RuntimeError('J7仍在出力，请先支撑并失能')
            if self.last_report and not self.last_report.get('cleanup_ok'):
                raise RuntimeError('上次清理和参数恢复未确认，不能结束项目')
            self.shutdown_sealed = True

    def close(self):
        self.disable()
        self.closing.set()
        if self.worker:
            self.worker.join(timeout=15)
            if self.worker.is_alive():
                raise RuntimeError("CAN 清理尚未结束；不强制终止控制线程")
        if self._gc_timing in gc.callbacks:gc.callbacks.remove(self._gc_timing)
