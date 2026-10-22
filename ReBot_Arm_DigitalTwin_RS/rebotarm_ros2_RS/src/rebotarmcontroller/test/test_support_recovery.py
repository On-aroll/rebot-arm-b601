"""Offline policy tests for the one-use support recovery route."""
import math

import numpy as np
import pytest

from rebotarmcontroller.csp_official_limits import RATED_TORQUE
from rebotarmcontroller.csp_runtime.support_recovery import (
    RECOVERY_TORQUE_HARD_LIMIT_NM,
    SupportRecovery,
    build_support_recovery,
    gravity_limits_for_recovery_stage,
)


LIMITS = np.array([[-2.6179938779914944, 2.6179938779914944],
                   [0., 3.14], [0., 3.14], [-1.57, 1.57],
                   [-1.57, 1.57], [-3.14, 3.14]])
START = np.array([1.14925, 3.55215, 3.60157, -1.35688, 1.52773, 1.60623])
TARGET = np.array([START[0], 0., 0., 0., 0., 0.])


def policy():
    return SupportRecovery(build_support_recovery(START, TARGET, LIMITS), LIMITS)


def test_recovery_exception_is_limited_to_j2_j3_of_first_segment():
    first = gravity_limits_for_recovery_stage(0, RATED_TORQUE)
    later = gravity_limits_for_recovery_stage(1, RATED_TORQUE)
    np.testing.assert_array_equal(first, [11., 15., 15., 5., 5., 5.])
    np.testing.assert_array_equal(later, RATED_TORQUE)
    assert RECOVERY_TORQUE_HARD_LIMIT_NM == 15.
    with pytest.raises(ValueError):
        gravity_limits_for_recovery_stage(0, np.ones(5))


def test_only_known_j2_j3_overrun_and_inward_support_goal_are_plannable():
    plan = build_support_recovery(START, TARGET, LIMITS)
    assert plan['strategy'] == 'fold-j3-then-lower-j2-then-support'
    assert len(plan['waypoints']) == 3
    assert plan['waypoints'][0][2] == pytest.approx(1.5)
    invalid_target = TARGET.copy(); invalid_target[1] = LIMITS[1, 1] + .01
    with pytest.raises(ValueError, match='支撑目标'):
        build_support_recovery(START, invalid_target, LIMITS)
    too_far = START.copy();too_far[1] = LIMITS[1, 1] + math.radians(30.1)
    with pytest.raises(ValueError, match='超过'):
        build_support_recovery(too_far, TARGET, LIMITS)
    below = START.copy(); below[1] = -0.1
    with pytest.raises(ValueError, match='J2/J3'):
        build_support_recovery(below, TARGET, LIMITS)
    outside_wrist = START.copy(); outside_wrist[4] = LIMITS[4, 1] + .01
    with pytest.raises(ValueError, match='J2、J3|J1、J4–J6'):
        build_support_recovery(outside_wrist, TARGET, LIMITS)


def test_segment_start_is_idempotent_and_reference_must_progress_inward():
    recovery = policy()
    target = np.asarray(recovery.waypoints[0])
    recovery.begin_segment(START, target)
    recovery.begin_segment(START, target)
    first = START + (target - START) * .2
    assert recovery.allows_reference(first)
    recovery.advance_reference(first)
    recovery.begin_segment(START, target)
    assert recovery.allows_reference(first)
    outward = first.copy();outward[2] += .01
    assert not recovery.allows_reference(outward)
    assert recovery.allows_reference(target)
    recovery.advance_reference(target)
    with pytest.raises(ValueError, match='到位'):
        recovery.complete_segment(first)
    assert recovery.complete_segment(target) is False
    assert recovery.stage == 1 and not recovery.active
    assert recovery.allows_reference(target)
    assert not recovery.allows_reference(START)


def test_paused_route_can_resume_from_last_sent_progress_but_not_outward():
    recovery = policy()
    target = np.asarray(recovery.waypoints[0])
    recovery.begin_segment(START, target)
    progressed = START + (target - START) * .35
    recovery.advance_reference(progressed)

    resumed = progressed + (target - progressed) * .01
    recovery.begin_segment(resumed, target)
    assert recovery.allows_reference(resumed)

    outward = resumed.copy();outward[2] += .02
    with pytest.raises(ValueError, match='既定路段|向外偏离'):
        recovery.begin_segment(outward, target)


def test_unactivated_and_invalid_references_are_confined_to_current_stage():
    recovery = policy()
    assert recovery.allows_reference(START)
    assert not recovery.allows_reference(recovery.waypoints[0])
    assert not recovery.allows_reference([float('nan')] * 6)
    with pytest.raises(ValueError, match='范围'):
        build_support_recovery(START, TARGET, np.array([[-1., 1.]] * 6))


def test_hardware_gate_allows_the_exception_only_for_full_csp():
    from types import SimpleNamespace

    from rebotarmcontroller.csp_hardware import CspHardware
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import LIVE_FULL_MODE

    hardware = object.__new__(CspHardware)
    hardware.control_mode = 'full'
    hardware.config = SimpleNamespace(version=LIVE_FULL_MODE)
    hardware.bus = SimpleNamespace(drive_backend='csp')
    hardware._require_support_recovery_backend()

    hardware.bus.drive_backend = 'mit'
    with pytest.raises(RuntimeError, match='MIT/小幅'):
        hardware._require_support_recovery_backend()
    hardware.bus.drive_backend = 'csp'
    hardware.control_mode = 'small'
    with pytest.raises(RuntimeError, match='MIT/小幅'):
        hardware._require_support_recovery_backend()


def test_hardware_path_check_uses_stage_specific_recovery_torque_limits():
    from types import SimpleNamespace

    from rebotarmcontroller.csp_hardware import CspHardware
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import LIVE_FULL_MODE

    calls = []
    hardware = object.__new__(CspHardware)
    hardware.config = SimpleNamespace(version=LIVE_FULL_MODE)
    hardware.recovery_path_check = lambda *args: calls.append(args)
    recovery = SimpleNamespace(payload=lambda: {'strategy': 'fixed-recovery'})

    hardware._check_recovery_path(START, TARGET, recovery, 0)
    np.testing.assert_array_equal(calls[-1][4], [11., 15., 15., 5., 5., 5.])
    hardware._check_recovery_path(START, TARGET, recovery, 1)
    np.testing.assert_array_equal(calls[-1][4], RATED_TORQUE)


def hardware_for_recovery():
    from types import SimpleNamespace

    from rebotarmcontroller.csp_hardware import CspHardware
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import LIVE_FULL_MODE

    hardware = object.__new__(CspHardware)
    hardware.control_mode = 'full'
    hardware.config = SimpleNamespace(version=LIVE_FULL_MODE)
    hardware.bus = SimpleNamespace(drive_backend='csp', active=False)
    hardware.teleop_limits = LIMITS.tolist()
    hardware.min_twin_speed = .005
    hardware.max_twin_speed = .1
    hardware.support_recovery = None
    hardware.path_check = lambda *args: None
    hardware.recovery_path_check = lambda *args: None
    return hardware


def recovery_program(plan):
    from rebotarmcontroller.twin_program import CALIBRATION

    return dict(schema_version='1.0', robot_model='b601_rs', base_frame='base_link',
                calibration_id=CALIBRATION, units=dict(position='m', angle='rad', time='s'),
                steps=[dict(type='joint', joints=q, speed=.1) for q in plan['waypoints']])


def test_all_stages_can_be_previewed_without_advancing_runtime_policy():
    hardware = hardware_for_recovery()
    plan = hardware.plan_support_recovery(START, TARGET)
    recovery = SupportRecovery(plan, LIMITS)
    seed = START.copy()
    for stage, target in enumerate(recovery.waypoints):
        recovery.preview_segment(stage, seed, target)
        assert recovery.stage == 0 and not recovery.active
        seed = target
    assert hardware.support_recovery is None


def test_internal_recovery_compiles_all_stages_but_ordinary_program_stays_limited():
    from types import SimpleNamespace

    from rebotarmcontroller.twin_program import compile_program

    hardware = hardware_for_recovery()
    plan = build_support_recovery(START, TARGET, LIMITS)
    raw = recovery_program(plan)
    kin = SimpleNamespace(pose=lambda q: list(q))
    duration = lambda *args: 1.
    with pytest.raises(ValueError, match='关节目标越界'):
        compile_program(raw, START, kin, hardware, duration)
    _, steps = compile_program(raw, START, kin, hardware, duration, support_recovery_plan=plan)
    assert len(steps) == 3 and all(s['support_recovery'] for s in steps)
    np.testing.assert_array_equal(steps[0]['joints'], plan['waypoints'][0])
    raw['steps'][0]['joints'] = list(START)
    with pytest.raises(ValueError, match='不一致'):
        compile_program(raw, START, kin, hardware, duration, support_recovery_plan=plan)


def test_begin_recovery_accepts_only_preview_start_tolerance_without_rewriting_route():
    import threading
    from types import SimpleNamespace

    from rebotarmcontroller.csp_runtime.support_recovery import START_MATCH_RAD

    hardware = hardware_for_recovery()
    plan = build_support_recovery(START, TARGET, LIMITS)
    hardware.lock = threading.RLock()
    hardware.session = 'owner'
    hardware.phase = 'READONLY'
    hardware.clock = lambda: 10.
    hardware.feedback_timeout = .6
    hardware.samples = {i: SimpleNamespace(mode=0, qd=0., requested_at=9.9) for i in range(1, 7)}
    hardware._q = START + np.array([4e-5, -4e-5, 4e-5, -4e-5, 4e-5, -4e-5])
    result = hardware.begin_support_recovery('owner', plan)
    np.testing.assert_array_equal(result['waypoints'], plan['waypoints'])
    hardware.support_recovery.begin_segment(hardware._q, plan['waypoints'][0])
    hardware.support_recovery = None
    hardware._q[0] = START[0] + START_MATCH_RAD * 1.01
    with pytest.raises(RuntimeError, match='0.1°'):
        hardware.begin_support_recovery('owner', plan)


def test_full_route_requires_each_arrival_and_rejects_nonfinite_feedback():
    recovery = policy()
    seed = START.copy()
    for stage, target in enumerate(recovery.waypoints):
        recovery.preview_segment(stage, seed, target)
        recovery.begin_segment(seed, target)
        midpoint = (seed + target) / 2
        recovery.advance_reference(midpoint)
        with pytest.raises(ValueError, match='到位'):
            recovery.complete_segment(midpoint)
        recovery.advance_reference(target)
        invalid = target.copy(); invalid[1] = float('nan')
        with pytest.raises(ValueError, match='到位'):
            recovery.complete_segment(invalid)
        assert recovery.complete_segment(target) is (stage == 2)
        seed = target


@pytest.mark.parametrize('change', ['missing_stage', 'extra_step', 'non_joint', 'bad_offset', 'exported_flag'])
def test_recovery_cannot_be_extended_or_enabled_by_imported_fields(change):
    from types import SimpleNamespace

    from rebotarmcontroller.twin_program import compile_program

    hardware = hardware_for_recovery()
    plan = build_support_recovery(START, TARGET, LIMITS)
    raw = recovery_program(plan)
    offset = 0
    if change == 'missing_stage':raw['steps'].pop()
    elif change == 'extra_step':raw['steps'].append(dict(type='wait', seconds=.1))
    elif change == 'non_joint':raw['steps'][1] = dict(type='wait', seconds=.1)
    elif change == 'bad_offset':offset = True
    else:raw['steps'][0]['support_recovery'] = True
    with pytest.raises(ValueError):
        compile_program(raw, START, SimpleNamespace(pose=list), hardware, lambda *args: 1.,
                        support_recovery_plan=plan, support_recovery_stage_offset=offset)


def test_real_model_recovery_callback_initializes_each_worker_and_preserves_nominal_guard():
    from concurrent.futures import ThreadPoolExecutor

    from rebotarmcontroller.csp_twin_driver import CheckedKinematics

    kin = CheckedKinematics()
    hardware = hardware_for_recovery()
    hardware.path_check = lambda start, goal: kin.check_path(start, goal, gravity_limits=RATED_TORQUE)
    hardware.recovery_path_check = kin.check_support_recovery_path
    def preview_on_new_thread():
        assert not hasattr(kin.path_data, 'gravity')
        plan = hardware.plan_support_recovery(START, TARGET)
        assert hasattr(kin.path_data, 'gravity')
        with pytest.raises(ValueError, match='允许 11.00'):
            kin.check_path(START, plan['waypoints'][0], gravity_limits=RATED_TORQUE)
        return plan
    with ThreadPoolExecutor(max_workers=1) as worker:
        plan = worker.submit(preview_on_new_thread).result(timeout=15)
    # A distinct worker needs its own model scratch data as well.
    with ThreadPoolExecutor(max_workers=1) as worker:
        worker.submit(kin.check_support_recovery_path, START, plan['waypoints'][0], plan, 0,
                      gravity_limits_for_recovery_stage(0, RATED_TORQUE)).result(timeout=15)
    assert not hasattr(kin.path_data, 'gravity')


def test_memory_sdk_starts_recovery_from_actual_overrun_and_blocks_manual_commands():
    import time
    from test_csp_twin import Controller, Owner

    from rebotarmcontroller.csp_hardware import CspHardware
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZTransport

    class StartController(Controller):
        def add_robstride_motor(self, axis, host, model):
            motor = super().add_robstride_motor(axis, host, model)
            motor.q = float(START[axis-1])
            return motor
    sdk = StartController()
    hw = CspHardware(LIMITS, transport=NativeXYZTransport(factory=lambda **kwargs: sdk, ownership=Owner()),
                     path_check=lambda *args: None, recovery_path_check=lambda *args: None)
    hw.connect()
    try:
        hw.set_control_mode('full')
        plan = hw.plan_support_recovery(hw._q.copy(), TARGET)
        hw.prepare(); token = hw.teleop_acquire()
        hw.begin_support_recovery(token, plan)
        hw.teleop_resume(token)
        goal = plan['waypoints'][0]; speed = .2
        delta = float(np.max(np.abs(START-goal)))
        acceleration, jerk = hw.motion_limits()
        duration = max(1.875*delta/speed, math.sqrt(5.78*delta/acceleration), (60*delta/jerk)**(1/3))+.1
        began = None
        seq = 0
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            hw.teleop_heartbeat(token)
            if hw.motion_ready:
                if began is None: began = hw.clock()
                seq += 1
                hw.teleop_point(token, seq, hw.clock(), dict(start=START.tolist(), joints=goal,
                    duration=duration, began=began, support_recovery=True), speed)
                if hw._q[2] < START[2]-.0002:break
            assert hw.phase in ('READONLY', 'STARTING', 'ACTIVE'), hw.error
            time.sleep(.02)
        assert hw.motion_ready and hw._q[2] < START[2]-.0002, hw.error
        assert hw._q[1] > LIMITS[1,1] and hw.support_recovery.stage == 0
        with pytest.raises(ValueError, match='禁止普通手动'):
            hw.teleop_send(token, seq+1, hw.clock(), TARGET, None, speed)
    finally:
        hw.close()
    assert not any(m.enabled for m in sdk.motors.values())
