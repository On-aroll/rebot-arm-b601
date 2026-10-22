"""Latched native-position hold after communication loss, on the CAN owner.

No target, mode, gain, enable or disable writes are made while held. The drive
keeps its existing loc_ref. Missing telemetry is never evidence of support.
"""
import json
import math
import time
from dataclasses import asdict

import numpy as np

from .csp_runtime.rebot_native_xyz import AXES, FeedbackTimeout


def validate_takeover(hw):
    """Read-only validation; called again on the owner before accepting takeover."""
    if not hw.holding or not hw.holding.get('feedback_confirmed'):
        raise RuntimeError('通信异常保持中；等待完整新鲜反馈，尚不能重新接管')
    hw.executor.validate(dict(hw.samples), csp=True, quiet=True)
    if any(abs(hw.command_reference[i-1]-hw.samples[i].q) > hw.config.axes[i-1].error_max for i in AXES):
        raise RuntimeError('保持位置偏差超过原跟踪限制，不能重新接管')


def wait_for_takeover(hw, executor, permit, report, reason):
    """Remain powered but motion-inhibited until explicit takeover or disable.

This applies only after all six axes reached ACTIVE in full control. Startup
refusals and electrical/mechanical protection still use the existing cleanup.
"""
    with hw.lock:
        hw.stopped_session, hw.stopped_at = hw.session, hw.clock()
        hw.session = ''
        hw.start_requested = False
        hw.paused = True
        hw.target = hw.point_reference = hw.stream_velocity = None
        hw.stream_ramp.reset();hw.stream_ruckig.reset()
        hw.gripper.pause()
        hw.gripper.authorized = bool(hw.gripper.view and hw.gripper.view.active)
        hw.hold_requested.clear()
        hw.hold_takeover = False
        hw.phase = 'HOLDING'
        hw.holding = dict(reason=str(reason), since=time.time(), feedback_confirmed=False,
                          target_write_inhibited=True, axes=list(AXES),
                          automatic_manual_recovery=isinstance(reason,FeedbackTimeout))
        hw.error = f'通信异常：{reason}；已冻结电机原位置目标，反馈未确认，禁止运动。'
        # The most recent attempted write may have reached the drive even if
        # its post-send deadline failed. Read loc_ref to resolve that ambiguity.
        candidates = {i:[executor.command_targets[i]] for i in AXES}
        seen = set()
        for entry in reversed(hw.bus.register_log):
            if entry['index'] in ('0x7016','mit') and entry['phase'] in ('hold','trajectory'):
                axis = entry['axis']
                if axis not in seen:candidates[axis].append(entry['value']);seen.add(axis)
    now=hw.clock()
    report.setdefault('communication_holds', []).append(dict(hw.holding,
        feedback_at_hold={str(i):asdict(s) for i,s in hw.samples.items()},
        field_age_ms={str(i):{key:round((now-getattr(s,key))*1000,3)
            for key in ('requested_at','mode_requested_at','diagnostics_requested_at')
            if getattr(s,key) is not None} for i,s in hw.samples.items()},
        recent_poll_io=list(getattr(hw.bus,'poll_io',()))))
    # Preserve restoration lineage if this process is interrupted while holding.
    if hw.output:
        path = hw.output / f'csp-session-{time.time_ns()}.json'
        snapshot = dict(report, cleanup_ok=False, reason=str(reason),
                        holding=dict(hw.holding), register_log=hw.bus.register_log,
                        register_snapshot={str(i):{hex(k):v for k,v in d.original.items()} for i,d in hw.bus.drives.items()})
        try:
            hw.output.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix('.tmp')
            temporary.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2))
            temporary.replace(path)
        except OSError as exc:
            hw.holding['journal_error'] = str(exc)  # logging failure must not drop the arm

    def interrupted():
        if hw.closing.is_set() or hw.cancel_motion.is_set():
            raise InterruptedError(hw.stop_reason or '明确失能/服务停止请求')
        hw.bus.ownership.check()

    # Start a new velocity window; never differentiate across a telemetry gap.
    hw.bus.previous_samples.clear()
    references = None
    while True:
        interrupted()
        try:
            hw._read(interrupted)
            hw.gripper.observe_hold(interrupted)
            executor.validate(dict(hw.samples), csp=True)
            if references is None:
                if getattr(hw.bus,'drive_backend','csp') == 'mit':
                    frames = {i:hw.bus.drives[i].mit_last for i in AXES}
                    if any(not f or f['status'] != 'host-send-complete' for f in frames.values()):
                        raise TimeoutError('MIT末帧发送结果不确定，保持锁定；不能用CSP位置寄存器确认MIT目标')
                    observed = {i:frames[i]['value'] for i in AXES}
                    # MIT has no loc_ref readback. Retained-target hold is a
                    # host inference, explicitly reported rather than an ACK.
                    hw.holding['reference_source'] = 'last host-completed MIT frame; no drive target readback'
                else:
                    observed = {i:hw.bus.drives[i].read_register(0x7016) for i in AXES}
                interrupted()
                if any(not math.isfinite(observed[i]) or not any(
                    abs(observed[i]-q) <= math.radians(.02) for q in candidates[i]) for i in AXES):
                    raise ValueError('保持参考读回不属于本会话最后发送的目标，停止并核查')
                references = np.array([observed[i] for i in AXES])
            with hw.lock:
                hw.command_reference = tuple(references)
                hw.holding['feedback_confirmed'] = True
                hw.holding['last_feedback_at'] = hw.clock()
                hw.error = f'通信异常已暂停：{reason}；电机保留原位置目标，反馈已恢复，重新接管后可提交新动作。'
                if hw.hold_takeover and hw.session:
                    try:
                        validate_takeover(hw)
                    except (ValueError,RuntimeError) as exc:
                        hw.session, hw.hold_takeover = '', False
                        hw.error = f'重新接管未通过：{exc}；仍保留原位置目标'
                        continue
                    # Only an explicit new lease arms a new command window.
                    # No old trajectory/input/target is reinstated here.
                    now = hw.clock()
                    permit.expires_at = now+2.
                    executor.heartbeat(executor.session, hw.input_seq+1, True)
                    hw.input_seq += 1
                    executor.last_axis_command = {i:now for i in AXES}
                    executor.command_targets = {i:float(references[i-1]) for i in AXES}
                    for i in AXES:
                        executor.command_history[i].append((now,float(references[i-1])))
                        hw.bus.drives[i].last_boundary_target = float(references[i-1])
                    hw.gripper.resume_hold()
                    hw.holding = None
                    hw.hold_takeover = False
                    hw.phase, hw.error = 'ACTIVE', ''
                    return references
        except TimeoutError as exc:
            with hw.lock:
                hw.holding['feedback_confirmed'] = False
                hw.holding['last_feedback_error'] = str(exc)
                hw.error = f'通信异常保持中：{exc}；未发失能，反馈未确认。'
                if hw.hold_takeover:
                    hw.session, hw.hold_takeover = '', False
            hw.bus.previous_samples.clear()
        hw.closing.wait(.05)
