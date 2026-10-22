"""Tuning persistence, inert selection and sampled feedback servo behaviour."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.twin_tuning import MotionSettings
from rebotarmcontroller.twin_servo import JointServo
from rebotarmcontroller.twin_algorithm import configuration
from rebotarmcontroller.csp_hardware import CspHardware
from test_twin_program import console


def config(engine='pid', **params):
    return configuration(dict(schema='b601-controller/1.0', engine=engine, name=engine,
                              output='joint_velocity', rate_hz=10, parameters=params))


def payload(stamp, q=0., target=.1, velocity=0., speed=.1):
    return dict(state=dict(position=[q]*6, velocity=[velocity]*6, feedback_stamp_s=stamp),
                reference=dict(position=[target]*6), limits=dict(velocity=speed))


def test_motion_profiles_persist_independently_and_failed_apply_is_atomic(tmp_path, monkeypatch):
    path=tmp_path/'motion-settings.json';s=MotionSettings(path)
    s.apply('ruckig', dict(acceleration_scale=.3, jerk_scale=.2))
    s.apply('quintic', dict(acceleration_scale=.5, jerk_scale=.4))
    s=MotionSettings(path)
    assert s.selected=='quintic' and s.limits(4,30)==(2,12)
    s.apply('ruckig');assert s.limits(4,30)==(1.2,6)
    assert s.limits(4,30,small=True)==(4,30)
    before=path.read_text()
    for bad in [dict(acceleration_scale=1.01,jerk_scale=1),dict(acceleration_scale=True,jerk_scale=1),
                dict(acceleration_scale=.01,jerk_scale=.2),dict(acceleration_scale=.2,jerk_scale=float('nan'))]:
        with pytest.raises(ValueError):s.apply('quintic',bad)
    assert path.read_text()==before and s.selected=='ruckig'
    monkeypatch.setattr(type(path),'replace',lambda *a:(_ for _ in ()).throw(OSError('disk')))
    with pytest.raises(OSError):s.apply('quintic')
    assert s.selected=='ruckig' and path.read_text()==before


def test_tuning_retimes_shared_program_and_can_reference_without_changing_envelope(tmp_path):
    t,hw,*_=console(tmp_path)
    normal=t._goal_duration([0]*6,[1]*6,1.)
    t.motion_settings.apply('ruckig',dict(acceleration_scale=.1,jerk_scale=.1))
    assert t._goal_duration([0]*6,[1]*6,1.)>normal
    real=CspHardware([(-2,2)]*6,output=tmp_path);real.phase='READONLY';real.set_control_mode('full')
    real.set_motion_profile('ruckig',dict(acceleration_scale=.1,jerk_scale=.1))
    assert real.motion_limits()==(.4,3.)
    assert all(a.a_max==4. and a.jerk_max==30. for a in real.config.axes)
    real.target=np.zeros(6);real.paused=False;real.last_sample=real.clock();real.stream_velocity=np.full(6,.1)
    q=np.zeros(6)
    for _ in range(5):
        real.last_sample=real.clock();q=real._desired_reference(real.clock(),q,.02)
        assert np.max(abs(real.stream_ruckig.acceleration))<=.4+1e-9
    assert np.max(abs(real.stream_ruckig.acceleration))<=3.*.1+1e-9
    real.armed=True
    with pytest.raises(RuntimeError):real.set_motion_profile('quintic',dict(acceleration_scale=1.,jerk_scale=1.))
    assert real.motion_limits()==(.4,3.)


def test_pd_damps_measured_motion_and_has_no_target_derivative_kick():
    pd=JointServo(config('pd',kp=.6,kd=.2));p=JointServo(config('p',kp=.6))
    assert pd.step(payload(1,velocity=.1))[0]<p.step(payload(1,velocity=.1))[0]
    assert np.allclose(JointServo(config('pd',kp=.6,kd=1.)).step(payload(1,target=.02)),.012)


def test_pid_duplicate_feedback_saturation_unwind_and_gap_reset():
    servo=JointServo(config(kp=1.,ki=1.,kd=0.,integral_limit_rad_s=.03))
    for i in range(40):servo.step(payload(i*.1,target=1.))
    assert not any(servo.integral) and servo.saturations==40
    servo.step(payload(4.0,target=.01));servo.step(payload(4.1,target=.01))
    integral=servo.integral.copy();out=servo.step(payload(4.1,target=.01))
    assert np.all(integral>0) and np.array_equal(servo.integral,integral)
    assert np.array_equal(out,servo.last_values)
    servo.step(payload(4.2,target=-.01));assert np.all(servo.integral<integral)
    servo.step(payload(5.,target=0.));assert not any(servo.integral)


def test_pid_reduces_fixed_bias_in_an_inert_velocity_plant():
    def run(engine):
        servo=JointServo(config(engine,kp=1.,ki=.4 if engine=='pid' else 0.,kd=0.));q=0.
        for i in range(1500):q+=(servo.step(payload(i*.02,q=q,target=.15))[0]-.008)*.02
        return abs(.15-q)
    assert run('pid')<run('p')/10


@pytest.mark.parametrize('params', [dict(kp=[1,2]),dict(kp=-1),dict(kd=6),dict(ki=float('nan')),
                                    dict(derivative_filter_s=0),dict(integral_limit_rad_s=1),dict(unknown=1)])
def test_invalid_servo_parameters_are_rejected(params):
    with pytest.raises(ValueError):config(**params)


def test_builtin_check_is_inert_and_start_stop_clear_algorithm_state(tmp_path,monkeypatch):
    t,hw,q,g,events,writes,checks,tick,_=console(tmp_path);c=t.extensions.controller
    monkeypatch.setattr('rebotarmcontroller.twin_algorithm.exchange',lambda *a:pytest.fail('builtin used HTTP'))
    c.check(dict(id='check',config=config(),target=[.01]*6,speed=.1));c.worker.join(1)
    assert c.checked and not writes and not c.active
    id=c.checked['id'];c.servo.integral.fill(.02)
    t.session='owner';t.mode='HOLD';c.start(dict(check_id=id))
    assert not any(c.servo.integral)
    for _ in range(8):
        tick(.05);c.worker.join(1)
    assert writes and c.stats['tracking']['samples']>=2
    t._hold('stop');assert c.servo is None and c.latest is None and c.checked is None
    n=len(writes);tick();assert len(writes)==n
