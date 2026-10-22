"""Actual numerical controllers and real MIT worker with an inert SDK only."""
import time

import numpy as np
import pytest

from rebotarmcontroller.twin_drive import DriveSettings, MIT_KP, MIT_KD
from rebotarmcontroller.twin_servo import JointServo
from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZTransport
from test_csp_twin import Motor, Controller, Owner, wait_until, protocol_console
from test_twin_tuning import config, payload
from test_twin_program import console


def sample(stamp, q=0., target=.1, velocity=0., applied=0.):
    row=payload(stamp,q,target,velocity)
    row.update(applied_velocity=[applied]*6,period_s=.1)
    row['limits']['position']=[(-1.,1.)]*6
    return row


@pytest.mark.parametrize('engine',['scheduled_pid','ladrc','mpc'])
def test_new_controllers_execute_inert_checks_and_shared_reference_loop(tmp_path,monkeypatch,engine):
    t,hw,q,g,events,writes,checks,tick,_=console(tmp_path);c=t.extensions.controller
    monkeypatch.setattr('rebotarmcontroller.twin_algorithm.exchange',lambda *a:pytest.fail('builtin used HTTP'))
    c.check(dict(id='check',config=config(engine),target=[.02]*6,speed=.1));c.worker.join(1)
    assert c.checked, c.state()
    assert not writes and not c.active
    id=c.checked['id'];t.session='owner';t.mode='HOLD';c.start(dict(check_id=id))
    for _ in range(10):
        tick(.05);c.worker.join(1)
    assert c.stats['cycles']>=3 and writes and c.state()['diagnostics']
    t._hold('stop');n=len(writes);tick();assert len(writes)==n and c.servo is None


def test_gain_schedule_is_smooth_bounded_and_preserves_integrator_rules():
    s=JointServo(config('scheduled_pid',kp=.6,ki=0.,kd=0.,schedule_ratio=2.))
    assert np.allclose(s.step(sample(1,target=.001)),.0006,atol=1e-6)
    assert np.allclose(s.step(sample(1.1,target=.05)),.045)
    assert np.allclose(s.step(sample(1.2,target=.2)),.1)
    assert max(s.diagnostics['effective_kp'])<=1.2
    with pytest.raises(ValueError):config('scheduled_pid',kp=8.,schedule_ratio=2.)


def test_ladrc_observes_inert_bias_uses_applied_input_and_resets_on_gap():
    def run(engine):
        s=JointServo(config(engine));q=0.;u=0.
        for k in range(300):
            measured=u-.008
            q+=measured*.1
            u=float(s.step(sample(k*.1,q=q,target=.15,velocity=measured,applied=u))[0])
        return s,abs(q-.15)
    ladrc,error=run('ladrc');_,p_error=run('p')
    assert error<p_error/8
    assert np.allclose(ladrc.disturbance,-.008,atol=.001)
    old=ladrc.disturbance.copy();ladrc.step(sample(ladrc.last_stamp));assert np.array_equal(old,ladrc.disturbance)
    ladrc.step(sample(31));assert not any(ladrc.disturbance)
    assert np.allclose(ladrc.estimate,0.)


def test_mpc_solves_quadratic_cost_and_respects_prediction_box():
    s=JointServo(config('mpc',horizon=8));row=sample(1,target=.01)
    u=s.step(row);n=8;dt=.1
    a=np.tril(np.ones((n,n)))*dt;d=np.eye(n)-np.eye(n,k=-1)
    h=a.T@a+.05*np.eye(n)+.1*d.T@d
    expected=np.linalg.solve(h,-a.T@np.full(n,-.01))[0]
    assert np.allclose(u,expected,atol=1e-4), (u,expected)
    row=sample(1.1,q=.999,target=1.);u=s.step(row)
    assert 0<u[0]<=.001/(n*dt)+1e-10
    assert max(s.diagnostics['predicted_max'])<=1.+1e-10
    assert s.diagnostics['projected_residual']<=1e-5


@pytest.mark.parametrize('engine,p',[('ladrc',dict(control_bandwidth=3,observer_bandwidth=1)),
    ('ladrc',dict(input_gain=0)),('mpc',dict(horizon=4.5)),('mpc',dict(delta_weight=0)),
    ('mpc',dict(kp=1)),('scheduled_pid',dict(schedule_ratio=4))])
def test_advanced_parameters_fail_closed(engine,p):
    with pytest.raises(ValueError):config(engine,**p)


class MitMotor(Motor):
    def __init__(self,*args):
        super().__init__(*args);self.mit_target=0.
    def advance(self):
        now=time.monotonic()
        if self.enabled:
            step=(now-self.motion_at)*self.regs[0x7017]*.8
            target=self.mit_target if self.regs[0x7005]==0 else self.regs[0x7016]
            self.q+=float(np.clip(target-self.q,-step,step))
        self.motion_at=now
    def send_mit(self,*values):
        self.advance();self.controller.calls.append(('mit',self.i,*values))
        assert self.enabled and self.regs[0x7005]==0
        if getattr(self.controller,'fail_mit',False):raise TimeoutError('MIT send outcome unknown')
        self.mit_target=values[0]


@pytest.fixture
def mit_fixture():
    sdk=Controller()
    def add(i,*_):
        motor=MitMotor(i,sdk);sdk.motors[i]=motor;return motor
    sdk.add_robstride_motor=add
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner())
    hw=CspHardware([(-1,1)]*6,transport=bus);hw.connect();hw.set_control_mode('full')
    try:yield hw,sdk
    finally:hw.close()


def start_mit(hw):
    hw.set_drive_config('mit');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready or hw.last_report)
    assert hw.motion_ready, (hw.error,hw.last_report)
    return token


def test_mit_selection_is_inert_and_gains_persist_without_backend_auto_selection(mit_fixture,tmp_path):
    hw,sdk=mit_fixture
    hw.set_drive_config('mit',dict(kp=[x*.8 for x in MIT_KP],kd=MIT_KD))
    assert sdk.calls==[] and hw.phase=='READONLY' and hw.drive_settings.selected=='mit'
    s=DriveSettings(tmp_path/'drive.json');s.apply('mit',dict(kp=MIT_KP,kd=MIT_KD))
    loaded=DriveSettings(s.path);assert loaded.selected=='csp' and loaded.parameters==s.parameters
    with pytest.raises(ValueError):hw.set_drive_config('mit',dict(kp=[151]*6,kd=MIT_KD))
    assert sdk.calls==[]


def test_mit_first_frames_shared_point_motion_and_restore_never_write_csp_targets(mit_fixture):
    hw,sdk=mit_fixture;token=start_mit(hw)
    for i in range(1,7):
        enable=sdk.calls.index(('enable',i))
        assert sdk.calls[enable+1]==('mit',i,0.,0.,MIT_KP[i-1],MIT_KD[i-1],0.)
    assert not any(c[0]=='write' and c[2]==0x7016 for c in sdk.calls)
    with pytest.raises(RuntimeError):hw.set_drive_config('csp')
    hw.teleop_heartbeat(token)
    start=time.monotonic();hw.teleop_point(token,1,start,dict(start=[0.]*6,joints=[.01]*6,duration=.9,began=start),.03)
    for _ in range(15):
        hw.teleop_heartbeat(token);time.sleep(.03)
    assert hw.phase=='ACTIVE',hw.error
    assert any(c[0]=='mit' and c[2]>0 for c in sdk.calls)
    hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok'] and hw.last_report['restoration']['ok']
    assert all(m.regs[0x7005]==0 and not m.enabled for m in sdk.motors.values())
    assert not any(c[0]=='write' and c[2] in (0x7016,0x701E,0x701F,0x7020,0x702B) for c in sdk.calls)


def test_mit_communication_loss_freezes_frames_without_disable_and_needs_takeover(mit_fixture):
    hw,sdk=mit_fixture;token=start_mit(hw)
    wait_until(lambda:hw.phase=='HOLDING' and hw.holding['feedback_confirmed'])
    count=len(sdk.calls);time.sleep(.15)
    assert len(sdk.calls)==count and all(m.enabled for m in sdk.motors.values())
    assert 'no drive target readback' in hw.holding['reference_source']
    assert not hw.session and hw.target is None
    token=hw.teleop_acquire();wait_until(lambda:hw.phase=='ACTIVE')
    assert hw.paused and hw.target is None
    hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY')


def test_mit_unknown_frame_cannot_be_certified_by_csp_register_readback(mit_fixture):
    hw,sdk=mit_fixture;token=start_mit(hw);sdk.fail_mit=True
    wait_until(lambda:hw.phase=='HOLDING')
    time.sleep(.15)
    assert not hw.holding['feedback_confirmed'] and all(m.enabled for m in sdk.motors.values())
    assert 'MIT末帧' in hw.holding.get('last_feedback_error','')
    with pytest.raises(RuntimeError):hw.teleop_acquire()


def test_ros_drive_configuration_invalidates_old_checks_without_actuation(mit_fixture):
    import json
    from std_msgs.msg import String
    hw,sdk=mit_fixture;t,command=protocol_console(hw)
    t.command(String(data=json.dumps(dict(op='drive_config',driver='mit',parameters=dict(kp=MIT_KP,kd=MIT_KD)))))
    result=json.loads(t.events.messages[-1].data)
    assert result['ok'] and result['motion_settings']['backend']=='MIT'
    assert not sdk.calls and not t.goal and not t.plan


def test_mit_mode_zero_restart_cannot_hide_unfinished_powered_session(mit_fixture,tmp_path):
    import json
    hw,sdk=mit_fixture;hw.output=tmp_path
    checked=[]
    for motor in sdk.motors.values():
        enable=motor.enable
        def guarded_enable(enable=enable):
            journals=list(tmp_path.glob('csp-session-*.json'))
            assert journals
            row=json.loads(journals[-1].read_text())
            assert row['write_intent'] and not row['cleanup_ok'] and len(row['register_snapshot'])==6
            checked.append(True);enable()
        motor.enable=guarded_enable
    token=start_mit(hw)
    assert len(checked)==6
    # Construction only; no second connection or CAN owner is opened.
    recovered=CspHardware([(-1,1)]*6,output=tmp_path)
    assert recovered.recovery_required and 'MIT启动' in recovered.recovery_required['reason']
    with pytest.raises(RuntimeError):recovered.prepare()
    hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY')
    assert CspHardware([(-1,1)]*6,output=tmp_path).recovery_required is None
