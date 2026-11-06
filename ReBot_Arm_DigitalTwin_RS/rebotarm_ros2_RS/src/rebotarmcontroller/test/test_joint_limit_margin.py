"""Requested limits are shared by planning and the final CAN write guard."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.csp_twin_driver import CheckedKinematics
from rebotarmcontroller.joint_limits import expanded_joint_limits
from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Envelope
from rebotarmcontroller.csp_runtime.rebot_native_csp import CspTransport
from rebotarmcontroller.csp_runtime.rebot_twin_mit import MitDrive
from rebotarmcontroller.csp_runtime.rebot_native_xyz import proposal, PADDED_MODE, PADDED_FULL_MODE, LIVE_FULL_MODE
from rebotarmcontroller.csp_reset import validate_snapshot
from test_csp_twin import Controller, Motor, Owner, wait_until
from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZTransport
import time


def test_all_six_real_joint_ranges_expand_exactly_two_degrees():
    kin = CheckedKinematics()
    expected = expanded_joint_limits(kin.nominal_limits)
    assert np.allclose(list(zip(kin.lower, kin.upper)), expected, rtol=0, atol=1e-12)
    assert np.degrees(kin.lower[1]) == pytest.approx(-2)
    assert np.degrees(kin.upper[1]) == pytest.approx(math.degrees(3.14) + 2)
    assert np.degrees(kin.lower[0]) == pytest.approx(-152)
    assert kin.start_tolerance == 0  # No extra one-degree envelope on top.


def test_requested_j2_round_trip_across_old_zero_is_inside_new_ik_range():
    kin = CheckedKinematics()
    start = np.array([.1, math.radians(.0186), .2, 0, 0, 0])
    target = start.copy()
    target[1] = math.radians(-.0467)
    solved, error = kin.solve(kin.pose(target), target)
    assert np.all(np.asarray(solved) >= kin.lower)
    assert np.linalg.norm(error[:3]) <= kin.position_tolerance


def test_margin_input_cannot_accept_missing_nonfinite_or_reversed_limits():
    for limits in ([(0, 1)] * 5, [(float('nan'), 1)] * 6, [(1, 0)] * 6):
        with pytest.raises(ValueError):
            expanded_joint_limits(limits)


@pytest.mark.parametrize('mode', ['small', 'full'])
def test_effective_config_does_not_stack_another_start_margin(mode):
    nominal = [(0., 1.)] * 6
    hw = CspHardware(expanded_joint_limits(nominal), nominal_limits=nominal)
    if mode == 'full':
        hw.phase = 'READONLY'
        hw.set_control_mode(mode)
    assert hw.config.version == (PADDED_MODE if mode == 'small' else PADDED_FULL_MODE)
    assert hw.config.start_boundary_tolerance == hw.start_tolerance == 0
    assert hw.control_settings()['joint_limit_margin_deg'] == 2
    with pytest.raises(ValueError, match='恰好'):
        CspHardware(expanded_joint_limits(hw.teleop_limits), nominal_limits=nominal)


@pytest.mark.parametrize('axis', range(1, 7))
@pytest.mark.parametrize('backend', ['csp', 'mit'])
def test_final_guard_allows_both_directions_across_old_bounds_and_refuses_beyond_two_degrees(axis, backend):
    nominal = [(0., 1.)] * 6
    config = Envelope.parse(proposal(expanded_joint_limits(nominal), mode=PADDED_FULL_MODE)[0])
    writes = []
    motor = SimpleNamespace(robstride_write_param_f32=lambda index, value: writes.append(value),
                            send_mit=lambda *frame: writes.append(frame[0]))
    kwargs = dict(axis=axis, clock=lambda: 10., ownership=SimpleNamespace(check=lambda: None))
    drive = (CspTransport(**kwargs) if backend == 'csp' else
             MitDrive(parameters=dict(kp=[3.] * 6, kd=[1.] * 6), **kwargs))
    drive.coordinated = True
    drive.envelope = config
    drive.active = {axis}
    drive.permit = SimpleNamespace(axes=tuple(range(1, 7)), expires_at=20., coordinated_start_checked=True)
    drive.boundary_start = {axis: math.radians(.0186)}
    drive.boundary_recovered = {axis}  # Old session flag must not relock the old zero.
    drive.latest_samples = {axis: SimpleNamespace(q=math.radians(.0186), qd=0., requested_at=10.)}
    drive.last_boundary_target = math.radians(.0186)
    drive.motors = {axis: motor}
    values = [math.radians(-.0467), math.radians(.0186), config.axes[axis-1].q_min,
              config.axes[axis-1].q_max, 1. + math.radians(.0467), 1. - math.radians(.0186)]
    for value in values:
        drive.write_register(0x7016, value, lambda: None, phase='trajectory', verify=False)
    assert writes == values
    for value in (config.axes[axis-1].q_min - 1e-8, config.axes[axis-1].q_max + 1e-8):
        before = len(writes)
        with pytest.raises((ValueError, PermissionError)):
            drive.write_register(0x7016, value, lambda: None, phase='trajectory', verify=False)
        assert len(writes) == before


def test_snapshot_recovery_accepts_exact_legacy_and_new_fingerprints_without_reinterpreting_sources():
    nominal = [(0., 1.)] * 6
    effective = expanded_joint_limits(nominal)
    sdk = Controller()
    snapshot = {str(i): {hex(k): v for k, v in Motor(i, sdk).regs.items() if k != 0x7016}
                for i in range(1, 7)}
    for limits, version in ((nominal, LIVE_FULL_MODE), (effective, PADDED_FULL_MODE)):
        source = dict(backend='csp-twin-v1', control_settings=dict(mode='full'), register_snapshot=snapshot,
                      config_fingerprint=Envelope.parse(proposal(limits, mode=version)[0]).fingerprint)
        _, selected = validate_snapshot(source, effective, nominal_limits=nominal)
        assert selected == version
        with pytest.raises(ValueError, match='方案不符'):
            validate_snapshot({**source, 'config_fingerprint': '0' * 64}, effective, nominal_limits=nominal)
        if version == LIVE_FULL_MODE:
            with pytest.raises(ValueError, match='方案不符'):
                validate_snapshot(source, effective, nominal_limits=[(-.5, .5)] * 6)


def test_actual_memory_worker_crosses_old_zero_both_ways_then_restores(tmp_path):
    nominal = [(0., 1.)] * 6
    sdk = Controller()
    hw = CspHardware(expanded_joint_limits(nominal), nominal_limits=nominal, output=tmp_path,
                     transport=NativeXYZTransport(factory=lambda **kwargs: sdk, ownership=Owner()))
    hw.connect()
    try:
        hw.set_control_mode('full')
        sdk.motors[2].q = math.radians(.0186)
        wait_until(lambda: abs(hw._q[1] - sdk.motors[2].q) < 1e-9 and abs(hw._v[1]) < .001)
        hw.prepare()
        token = hw.teleop_acquire()
        hw.teleop_resume(token)
        wait_until(lambda: hw.motion_ready)
        seq = 0
        for degrees in (-.0467, .0186):
            target = hw._q.copy()
            target[1] = math.radians(degrees)
            for _ in range(40):
                seq += 1
                hw.teleop_heartbeat(token)
                hw.teleop_send(token, seq, time.monotonic(), target, None, .05)
                time.sleep(.02)
            assert hw.motion_ready, hw.error
            assert abs(sdk.motors[2].q - target[1]) < math.radians(.01)
        hw.teleop_stop(token)
        wait_until(lambda: hw.phase == 'READONLY')
        assert hw.last_report['cleanup_ok'] and hw.last_report['restoration']['ok']
        assert not any(m.enabled for m in sdk.motors.values())
    finally:
        hw.close()
