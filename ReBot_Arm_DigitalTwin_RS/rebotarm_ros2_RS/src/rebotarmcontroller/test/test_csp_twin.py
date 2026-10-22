"""Real CSP adapter and worker against an in-memory SDK, never real CAN."""
import math
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZTransport


class Owner:
    def acquire(self): pass
    def claim(self): pass
    def check(self): pass
    def close(self): pass


class Motor:
    def __init__(self,i,controller):
        self.i,self.controller=i,controller
        self.q=0.;self.enabled=False
        self.motion_at=time.monotonic()
        self.regs={0x7005:0,0x7017:1.,0x7018:(57. if i<=3 else 16.),0x700B:(36. if i<=3 else 14.),0x7016:0.,
                   0x701E:30.,0x701F:5.,0x7020:.02,0x702B:0.}

    def advance(self):
        # Simulate bounded continuous travel rather than teleporting at a
        # write. Teleports made differentiated encoder velocity depend on
        # thread scheduling and could spuriously trip the real speed guard.
        now=time.monotonic()
        if self.enabled:
            step=(now-self.motion_at)*self.regs[0x7017]*.8
            self.q+=max(-step,min(step,self.regs[0x7016]-self.q))
        self.motion_at=now

    def robstride_get_param_f32_host_id(self,index,host,timeout):
        if index==0x7019:
            self.advance()
            return self.q
        if index in (0x701B,0x701A,0x302C): return 0.
        return self.regs[index]

    def robstride_get_param_i8(self,index,timeout): return self.regs[index]

    def robstride_write_param_f32(self,index,value):
        self.advance()
        self.controller.calls.append(('write',self.i,index,value))
        self.regs[index]=value

    def robstride_write_param_i8(self,index,value):
        self.enabled=False
        self.robstride_write_param_f32(index,value)

    def enable(self):
        self.advance()
        self.controller.calls.append(('enable',self.i))
        self.enabled=True

    def disable(self):
        self.advance()
        self.controller.calls.append(('disable',self.i))
        self.enabled=False

    def get_state(self):
        mode=2 if self.controller.bad_stop else 0
        return SimpleNamespace(arbitration_id=(2<<24)|(mode<<22)|(self.i<<8)|0xFD)

    def close(self): pass


class Controller:
    def __init__(self): self.calls=[];self.motors={};self.bad_stop=False
    def add_robstride_motor(self,i,host,model):
        motor=Motor(i,self);self.motors[i]=motor;return motor
    def close_bus(self): pass
    def close(self): pass


def wait_until(predicate,timeout=3):
    deadline=time.monotonic()+timeout
    while not predicate() and time.monotonic()<deadline: time.sleep(.01)
    assert predicate()


@pytest.fixture
def fixture():
    sdk=Controller()
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner())
    hw=CspHardware([(-1,1)]*6,transport=bus)
    hw.connect()
    try: yield hw,sdk
    finally: hw.close()


def test_connect_prepare_acquire_are_readonly(fixture):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire();hw.teleop_pause(token)
    time.sleep(.05)
    assert sdk.calls==[]
    assert hw.phase=='READONLY' and not hw.motion_ready


@pytest.mark.parametrize('recovery_locked',[False,True])
def test_readonly_feedback_timeout_retries_without_writes_or_reviving_permission(fixture,recovery_locked):
    hw,sdk=fixture
    hw.prepare();hw.teleop_acquire()
    if recovery_locked:hw.recovery_required={'reason':'保留原始恢复锁定'}
    original=sdk.motors[4].robstride_get_param_f32_host_id
    sdk.motors[4].robstride_get_param_f32_host_id=lambda index,*args:float('nan') if index==0x7019 else original(index,*args)
    wait_until(lambda:hw.phase=='FEEDBACK_WAIT')
    assert hw.worker.is_alive() and not hw.armed and not hw.session
    assert hw.target is None and hw.command_reference is None and sdk.calls==[]
    with pytest.raises(RuntimeError):hw.prepare()
    with pytest.raises(RuntimeError):hw.teleop_acquire()
    sdk.motors[4].robstride_get_param_f32_host_id=original
    wait_until(lambda:hw.phase==('RECOVERY' if recovery_locked else 'READONLY'))
    assert hw.worker.is_alive() and not hw.armed and not hw.session and sdk.calls==[]
    assert hw.clock()-hw._feedback_refreshed_at < .6
    assert bool(hw.recovery_required)==recovery_locked


def test_unclean_stop_stays_observable_and_blocks_retry_after_restart(fixture,tmp_path):
    hw,sdk=fixture;hw.output=tmp_path
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    sdk.bad_stop=True
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='RECOVERY')
    assert hw.recovery_required and not hw.last_report['cleanup_ok']
    before=hw._feedback_refreshed_at
    wait_until(lambda:hw._feedback_refreshed_at>before)
    for action in (hw.prepare,hw.teleop_acquire,lambda:hw.set_control_mode('full')):
        with pytest.raises(RuntimeError,match='锁定'): action()
    hw.close()
    # Even a power cycle / fresh mode-zero SDK cannot certify the old snapshot.
    restarted_sdk=Controller()
    restarted=CspHardware([(-1,1)]*6,output=tmp_path,
        transport=NativeXYZTransport(factory=lambda **kwargs:restarted_sdk,ownership=Owner()))
    restarted.connect()
    try:
        assert restarted.phase=='RECOVERY' and restarted.recovery_required
        with pytest.raises(RuntimeError,match='锁定'): restarted.prepare()
        assert restarted_sdk.calls==[]
    finally: restarted.close()


def test_preflight_noop_report_cannot_mask_unrestored_session(tmp_path):
    import json
    failure=tmp_path/'csp-session-001.json'
    failure.write_text(json.dumps(dict(cleanup_ok=False,cleanup_error='未停稳',register_log=[{'axis':1}])))
    (tmp_path/'csp-session-002.json').write_text(json.dumps(dict(cleanup_ok=True,register_log=[])))
    hw=CspHardware([(-1,1)]*6,output=tmp_path)
    assert hw.recovery_required['report']==str(failure)
    assert hw.recovery_required['cleanup_error']=='未停稳'


def test_unexpected_start_mode_blocks_before_any_writes():
    sdk=Controller()
    def factory(**kwargs): return sdk
    original=sdk.add_robstride_motor
    def add(*args):
        motor=original(*args);motor.regs[0x7005]=5;return motor
    sdk.add_robstride_motor=add
    hw=CspHardware([(-1,1)]*6,transport=NativeXYZTransport(factory=factory,ownership=Owner()))
    hw.connect()
    try:
        assert hw.phase=='RECOVERY' and '5' in hw.error
        with pytest.raises(RuntimeError,match='锁定'): hw.prepare()
        assert sdk.calls==[] and all(not m.enabled for m in sdk.motors.values())
    finally: hw.close()


def test_mode_mismatch_reports_observed_and_required_values(fixture):
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZExecutor
    hw,sdk=fixture
    executor=NativeXYZExecutor(hw.config,hw.bus,fk=lambda q:[0,0,0])
    with pytest.raises(ValueError,match='读到 5.*要求 0'):
        executor.validate({1:SimpleNamespace(mode=5)})
    assert sdk.calls==[]


def protocol_console(hw):
    import json
    from std_msgs.msg import String
    from rebotarmcontroller.twin_control import TwinControl
    class Publisher:
        def __init__(self): self.messages=[]
        def publish(self,message): self.messages.append(message)
    node=SimpleNamespace(create_publisher=lambda *a,**k:Publisher(),
        create_service=lambda *a,**k:None,create_subscription=lambda *a,**k:None,
        create_timer=lambda *a,**k:None)
    twin=TwinControl(node,hw,'offline_hold',kinematics=SimpleNamespace())
    counter=iter(range(1,1000))
    def command(op,session=''):
        twin.command(String(data=json.dumps(dict(op=op,session=session,
            seq=next(counter),stamp_ms=time.time()*1000))))
        return json.loads(twin.events.messages[-1].data)
    return twin,command


def test_explicit_power_hold_needs_owner_and_preloads_measured_pose(fixture):
    import json
    hw,sdk=fixture
    for i,motor in sdk.motors.items(): motor.q=i*.01
    start=np.arange(1,7)*.01
    wait_until(lambda:np.max(abs(hw.get_joint_state()[0]-start))<1e-8 and np.max(abs(hw.get_joint_state()[1]))<.001)
    twin,command=protocol_console(hw)
    assert not command('power_hold')['ok'] and sdk.calls==[]
    hw.prepare();response=twin.acquire(None,SimpleNamespace())
    token=json.loads(response.message)['session']
    assert response.success and sdk.calls==[]
    assert not command('power_hold','foreign')['ok'] and sdk.calls==[]
    assert command('power_hold',token)['ok']
    wait_until(lambda:hw.motion_ready)
    assert twin.mode=='HOLD' and twin.goal is None
    assert np.allclose(hw.get_joint_state()[0],start)
    for i,motor in sdk.motors.items():
        before=sdk.calls[:sdk.calls.index(('enable',i))]
        assert [c for c in before if c[:3]==('write',i,0x7016)][-1][3]==start[i-1]
        assert motor.enabled
    assert twin.stop(None,SimpleNamespace()).success
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok'] and not any(m.enabled for m in sdk.motors.values())


def test_power_hold_model_rejection_precedes_writes_or_enable(fixture):
    import json
    hw,sdk=fixture
    def reject(start,target): raise ValueError('模型重力超过本次 CSP 力矩包络')
    hw.path_check=reject
    twin,command=protocol_console(hw)
    hw.prepare();response=twin.acquire(None,SimpleNamespace())
    token=json.loads(response.message)['session']
    assert command('power_hold',token)['ok']  # request accepted, worker validates asynchronously
    wait_until(lambda:hw.last_report is not None)
    assert sdk.calls==[]
    assert '模型重力' in hw.last_report['reason'] and hw.last_report['cleanup_ok']
    assert not hw.teleop_owned


@pytest.mark.parametrize('upper',[False,True])
@pytest.mark.parametrize('jitter',[-0.000042597763240337,0.000042597763240337])
def test_boundary_encoder_jitter_preloads_fresh_pose_and_only_recovers_inward(fixture,upper,jitter):
    hw,sdk=fixture
    hw.teleop_limits[1]=(0.,1.)
    hw.set_control_mode('full')
    motor=sdk.motors[2]
    motor.q=1.004474109970033169 if upper else -0.004474109970033169
    wait_until(lambda:abs(hw._q[1]-motor.q)<1e-9 and abs(hw._v[1])<.001)
    hw.prepare();token=hw.teleop_acquire()
    motor.q+=jitter
    wait_until(lambda:abs(hw._q[1]-motor.q)<1e-9 and abs(hw._v[1])<.001)
    fresh=motor.q
    hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready or hw.last_report is not None)
    assert hw.motion_ready,hw.error
    before=sdk.calls[:sdk.calls.index(('enable',2))]
    assert [c[3] for c in before if c[:3]==('write',2,0x7016)][-1]==fresh
    assert motor.regs[0x7016]==fresh
    outward=hw._q.copy();outward[1]=fresh+(.00001 if upper else -.00001)
    with pytest.raises(ValueError,match='J2 CSP 目标'):
        hw._check_target(outward)
    # Feedback noise while holding must not be copied into the reference.
    motor.q+=jitter
    for _ in range(5):hw.teleop_heartbeat(token);time.sleep(.02)
    assert motor.regs[0x7016]==fresh
    inward=hw._q.copy();inward[1]=.999 if upper else .001
    for seq in range(1,36):
        hw.teleop_heartbeat(token)
        hw.teleop_send(token,seq,time.monotonic(),inward,None,.02)
        time.sleep(.02)
    assert hw.motion_ready,hw.error
    assert abs(motor.q-inward[1])<.0005
    # Crossing the nominal boundary is one-way for this session.
    with pytest.raises(ValueError,match='J2 CSP 目标'):
        hw._check_target(outward)
    targets=[c[3] for c in sdk.calls if c[:3]==('write',2,0x7016)]
    assert all(q<=fresh+1e-12 if upper else q>=fresh-1e-12 for q in targets)
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok'] and not any(m.enabled for m in sdk.motors.values())


@pytest.mark.parametrize('kind',['delta','speed','replay','stale','nan'])
def test_rejects_invalid_targets_before_any_write(fixture,kind):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire()
    target=[0.]*6;speed=.01;seq=1;stamp=time.monotonic()
    if kind=='delta': target[5]=math.radians(.51)
    if kind=='speed': speed=.03
    if kind=='replay': seq=-1
    if kind=='stale': stamp-=1
    if kind=='nan': target[0]=float('nan')
    with pytest.raises((ValueError,RuntimeError)):
        hw.teleop_send(token,seq,stamp,target,None,speed)
    assert sdk.calls==[]


def test_worker_preloads_then_streams_stops_and_restores(fixture):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire()
    hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    target=[0.]*6;target[5]=math.radians(.2)
    for seq in range(1,35):
        assert hw.teleop_owned,hw.error
        hw.teleop_heartbeat(token)
        hw.teleop_send(token,seq,time.monotonic(),target,None,.01)
        time.sleep(.02)
    assert abs(hw.get_joint_state()[0][5]-target[5]) < math.radians(.02)
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok']
    for i,motor in sdk.motors.items():
        before=sdk.calls[:sdk.calls.index(('enable',i))]
        preload=[c for c in before if c[:3]==('write',i,0x7016)]
        assert preload[-1][3]==0.
        assert not motor.enabled
        assert [motor.regs[p] for p in (0x7005,0x7017,0x7018,0x700B)]==[0,1.,57. if i<=3 else 16.,36. if i<=3 else 14.]
    assert all(c[2] not in (0x701E,0x701F,0x7020,0x702B) for c in sdk.calls if c[0]=='write')
    assert set(sdk.motors)==set(range(1,7))


def test_heartbeat_loss_disables_and_clears_lease(fixture):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    wait_until(lambda:hw.last_report is not None,timeout=3)
    assert not hw.teleop_owned
    assert hw.last_report['cleanup_ok']
    assert not any(m.enabled for m in sdk.motors.values())


def test_stop_during_preparation_prevents_enable(fixture):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire()
    original=hw.bus.drives
    # Cancel from the real startup callback, before any runtime parameter write.
    authorize=hw.bus.authorize
    def cancel_after_authorize(*args):
        authorize(*args);hw.teleop_pause(token)
    hw.bus.authorize=cancel_after_authorize
    hw.teleop_resume(token)
    wait_until(lambda:hw.last_report is not None)
    assert not any(c[0]=='enable' for c in sdk.calls)
    assert hw.last_report['cleanup_ok']


def test_gateway_restore_checks_listener_and_socket(tmp_path,monkeypatch):
    from rebotarmcontroller.csp_runtime import gateway_lease
    process=SimpleNamespace(pid=123456,poll=lambda:None)
    lease=gateway_lease.GatewayLease(tmp_path,popen=lambda *a,**k:process)
    lease.argv=['verified-gateway'];lease.stopped=lease.active=lease.present=True
    calls=iter([None,process.pid])
    monkeypatch.setattr(lease,'_listener_pid',lambda:next(calls))
    sockets=[]
    monkeypatch.setattr(gateway_lease.socket,'create_connection',lambda *a,**k:SimpleNamespace(close=lambda:sockets.append(a[0])))
    assert lease.release()['restarted']
    assert sockets==[('127.0.0.1',9002)]
    assert lease.pidfile.read_text().strip()==str(process.pid)


def test_drift_during_preparation_rejects_before_enable(fixture):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire()
    authorize=hw.bus.authorize
    def drift_after_authorize(*args):
        authorize(*args);sdk.motors[5].q=math.radians(.17)
    hw.bus.authorize=drift_after_authorize
    hw.teleop_resume(token)
    wait_until(lambda:hw.last_report is not None)
    assert not any(c[0]=='enable' for c in sdk.calls)
    assert hw.last_report['cleanup_ok']
    assert '起点变化' in hw.last_report['reason']


def test_pause_freezes_reference_instead_of_following_feedback_noise(fixture):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    target=[0.]*6;target[5]=math.radians(.2)
    for seq in range(1,25):
        hw.teleop_heartbeat(token)
        hw.teleop_send(token,seq,time.monotonic(),target,None,.01)
        time.sleep(.02)
    hw.teleop_pause(token)
    held=sdk.motors[6].regs[0x7016]
    sdk.motors[6].q=held-math.radians(.03)
    for _ in range(5):
        hw.teleop_heartbeat(token);time.sleep(.03)
    assert sdk.motors[6].regs[0x7016]==held
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok']


def test_serial_read_retry_refreshes_same_reference_without_relaxing_deadline(fixture):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    # A normal scan plus one reply retry exceeds the 300 ms command window,
    # while each feedback sample remains younger than the 600 ms limit.
    delayed=[False]
    for i,motor in sdk.motors.items():
        f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
        def slow_f32(*args,read=f32):
            time.sleep(.009);return read(*args)
        def slow_i8(*args,read=i8,axis=i):
            time.sleep(.009)
            if axis==6 and not delayed[0]:
                delayed[0]=True;time.sleep(.12)
            return read(*args)
        motor.robstride_get_param_f32_host_id=slow_f32
        motor.robstride_get_param_i8=slow_i8
    for _ in range(20):
        hw.teleop_heartbeat(token);time.sleep(.05)
    assert hw.motion_ready,hw.error
    assert any(f.get('refresh') for f in hw.executor.frames)
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='READONLY',timeout=4)
    assert hw.last_report['cleanup_ok']


def test_startup_refreshes_enabling_axis_and_partial_hold(fixture):
    hw,sdk=fixture
    hw.set_control_mode('full')
    # A six-axis startup scan can exceed 300 ms while each individual reply
    # is valid. The first axis must not preload a sample from before that scan.
    for motor in sdk.motors.values():
        f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
        def slow_f32(*args,read=f32):time.sleep(.014);return read(*args)
        def slow_i8(*args,read=i8):time.sleep(.014);return read(*args)
        motor.robstride_get_param_f32_host_id=slow_f32
        motor.robstride_get_param_i8=slow_i8
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    deadline=time.monotonic()+12
    while not hw.motion_ready and hw.last_report is None and time.monotonic()<deadline:
        hw.teleop_heartbeat(token);time.sleep(.03)
    assert hw.motion_ready,hw.error or hw.last_report
    assert hw.config.policy.command_age==.5 and hw.config.policy.feedback_age==.6
    assert all(m.enabled for m in sdk.motors.values())
    assert any(f.get('refresh') and len(f['commands'])<6 for f in hw.executor.frames)
    for frame in hw.executor.frames:
        for command in frame['commands']:
            assert command['target_rad']==0.
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='READONLY',timeout=5)
    assert hw.last_report['cleanup_ok']


def test_expired_refreshed_preload_still_blocks_enable(fixture):
    hw,sdk=fixture
    hw.set_control_mode('full')
    read=hw._read
    def delayed_read(*args,**kwargs):
        samples=read(*args,**kwargs)
        if hw.phase=='STARTING' and kwargs.get('axes')==(1,):time.sleep(hw.config.policy.command_age+.01)
        return samples
    hw._read=delayed_read
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.last_report is not None)
    assert 'J1 首目标反馈过期' in hw.last_report['reason']
    assert not any(c[0]=='enable' for c in sdk.calls)
    assert hw.last_report['cleanup_ok']


def test_web_stop_clears_preparation_even_without_owner(fixture):
    from rebotarmcontroller.twin_control import TwinControl
    hw,sdk=fixture
    hw.prepare()
    twin=TwinControl.__new__(TwinControl)
    twin.hw=hw;twin.lock=threading.RLock();twin.session=''
    from rebotarmcontroller.twin_jog import JogRamp
    twin.jog_ramp=JogRamp()
    from rebotarmcontroller.twin_motion import RuckigVelocity
    twin.ruckig_jog=RuckigVelocity()
    response=twin.stop(None,SimpleNamespace())
    assert response.success and not hw.armed
    assert sdk.calls==[]


@pytest.mark.parametrize('notice_first',[True,False])
def test_stop_notice_survives_service_order_and_is_written_to_report(fixture,notice_first):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    reason='页面失去焦点，已释放控制'
    if notice_first:hw.note_stop_request(token,reason,time.time()*1000)
    hw.teleop_stop(token,'停止服务请求')
    if not notice_first:hw.note_stop_request(token,reason,time.time()*1000)
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['reason']=='停止服务请求'
    assert hw.last_report['browser_stop_request']['reason']==reason
    assert hw.last_report['cleanup_ok']


def test_stop_notice_is_diagnostic_only_and_rejects_foreign_or_stale_data(fixture):
    hw,sdk=fixture
    hw.prepare();token=hw.teleop_acquire()
    for lease,reason,stamp in [('other','stop',time.time()*1000),(token,'old',time.time()*1000-2100),
            (token,'bad',float('nan')),(token,'a'*161,time.time()*1000)]:
        hw.note_stop_request(lease,reason,stamp)
        assert hw.stop_request is None
    twin,_=protocol_console(hw)
    from std_msgs.msg import String
    import json
    twin.stop_notice(String(data='[]'))
    twin.stop_notice(String(data='not json'))
    twin.stop_notice(String(data=json.dumps(dict(session=token,reason='点击停止',stamp_ms=time.time()*1000))))
    assert hw.stop_request['reason']=='点击停止'
    assert hw.session==token and hw.armed and hw.phase=='READONLY' and sdk.calls==[]


def test_full_mode_is_readonly_configuration_and_uses_nominal_workspace(fixture):
    hw,sdk=fixture
    target=[0.]*5+[.5]
    with pytest.raises(ValueError,match='0.5'):
        hw.validate_preview([0.]*6,target)
    hw.set_control_mode('full')
    hw.validate_preview([0.]*6,target)
    assert hw.control_settings()['max_axis_delta_deg'] is None
    assert hw.max_twin_speed == 1. and hw.session_seconds is None
    assert hw.control_settings()['continuous'] and hw.control_settings()['session_limit_s'] is None
    assert not hw.armed and sdk.calls==[]
    with pytest.raises(ValueError,match='限位'):
        hw.validate_preview([0.]*6,[0.]*5+[1.01])
    hw.set_control_mode('small')
    assert hw.max_twin_speed == .02 and hw.session_seconds == 15
    with pytest.raises(ValueError): hw.set_control_mode('unbounded')


def test_control_range_cannot_change_while_prepared_owned_or_stopping(fixture):
    hw,sdk=fixture
    hw.prepare()
    with pytest.raises(RuntimeError): hw.set_control_mode('full')
    hw.teleop_acquire()
    with pytest.raises(RuntimeError): hw.set_control_mode('full')
    hw.disable()
    hw.phase='STOPPING'
    with pytest.raises(RuntimeError): hw.set_control_mode('full')
    hw.phase='READONLY'
    assert sdk.calls==[] and hw.control_mode == 'small'


def test_full_worker_moves_past_trial_boundary_then_stops_and_restores(fixture):
    hw,sdk=fixture
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire()
    hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    assert hw.executor.deadline is None
    assert 0 < hw.bus.permit.expires_at-time.monotonic() <= 2.
    initial_expiration=hw.bus.permit.expires_at
    assert all(d.runtime[0x7017] == 1. for d in hw.bus.drives.values())
    target=[.06,-.06,.05,-.05,.04,-.04]
    for seq in range(1,121):
        assert hw.teleop_owned,hw.error
        hw.teleop_heartbeat(token)
        hw.teleop_send(token,seq,time.monotonic(),target,None,.1)
        time.sleep(.02)
    assert np.max(np.abs(hw.get_joint_state()[0]-target)) < .001
    assert time.monotonic() > initial_expiration
    assert hw.bus.permit.expires_at > initial_expiration
    assert max(abs(v) for v in hw.get_joint_state()[0]) > math.radians(.5)
    with pytest.raises(RuntimeError): hw.set_control_mode('small')
    with pytest.raises(ValueError): hw.teleop_send(token,122,time.monotonic(),target,None,1.001)
    with pytest.raises(ValueError): hw.teleop_send(token,122,time.monotonic(),[1.01]*6,None,.1)
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok']
    assert hw.last_report['control_settings']['mode'] == 'full'
    for i,motor in sdk.motors.items():
        assert not motor.enabled
        assert [motor.regs[p] for p in (0x7005,0x7017,0x7018,0x700B)]==[0,1.,57. if i<=3 else 16.,36. if i<=3 else 14.]
    assert all(c[2] not in (0x701E,0x701F,0x7020,0x702B) for c in sdk.calls if c[0]=='write')


def test_full_mode_heartbeat_loss_latches_hold_without_disable_or_auto_resume(fixture):
    hw,sdk=fixture
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    wait_until(lambda:hw.phase=='HOLDING' and hw.holding['feedback_confirmed'])
    assert all(m.enabled for m in sdk.motors.values()) and not hw.teleop_owned
    with pytest.raises(RuntimeError,match='失效'):hw.teleop_heartbeat(token)
    held=len(sdk.calls);time.sleep(.35)
    assert hw.phase=='HOLDING' and len(sdk.calls)==held
    token=hw.teleop_acquire()
    wait_until(lambda:hw.motion_ready)
    assert all(c[0]=='write' and c[2]==0x7016 for c in sdk.calls[held:])
    assert hw.paused and hw.target is None
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok'] and not any(m.enabled for m in sdk.motors.values())


def test_full_fingerprint_rejects_arbitrary_speed_or_watchdog_changes():
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import FULL_MODE,NativeXYZExecutor,proposal
    from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Envelope
    for mutate in ('velocity','watchdog'):
        raw,_=proposal([(-1.,1.)]*6,mode=FULL_MODE)
        if mutate=='velocity': raw['axes'][0]['v_max']=1.
        else: raw['policy']['feedback_age']=10.
        with pytest.raises(ValueError,match='包络'):
            NativeXYZExecutor(Envelope.parse(raw),SimpleNamespace(),fk=lambda q:[0.,0.,0.])


def test_interleaved_poll_reads_all_positions_and_keeps_diagnostic_timestamps():
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import AXES
    tick=[100.]; queries=[]; sdk=Controller()
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner(),clock=lambda:tick[0])
    bus.open_readonly()
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:tick[0])
    try:
        for i,motor in sdk.motors.items():
            f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
            def read_f32(index,*args,read=f32,axis=i):
                tick[0]+=.008;queries.append((axis,index));return read(index,*args)
            def read_i8(index,*args,read=i8,axis=i):
                tick[0]+=.008;queries.append((axis,index));return read(index,*args)
            motor.robstride_get_param_f32_host_id=read_f32
            motor.robstride_get_param_i8=read_i8
        bus.read(AXES,hw.config.policy)
        initial=bus.latest_samples[6].diagnostics_requested_at
        for axis in AXES:
            queries.clear();began=tick[0]
            samples=bus.read_cycle(hw.config.policy,axis)
            assert len(queries)==8
            assert [i for i,key in queries if key==0x7019]==list(AXES)
            assert (axis,0x7005) in queries and (axis,0x701A) in queries
            assert tick[0]-began == pytest.approx(.064)
            assert max(tick[0]-s.requested_at for s in samples.values()) < .05
            if axis==1:
                assert samples[6].diagnostics_requested_at==initial
                assert samples[6].qd_raw is None
        assert max(tick[0]-s.diagnostics_requested_at for s in samples.values()) < .4
        assert sdk.calls==[]
    finally: bus.close()


def test_fresh_position_cannot_hide_expired_mode_or_current(fixture):
    from dataclasses import replace
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZExecutor
    hw,sdk=fixture
    executor=NativeXYZExecutor(hw.config,hw.bus,fk=lambda q:[0,0,0])
    s=hw.samples[1]
    for key in ('mode_requested_at','diagnostics_requested_at'):
        stale=replace(s,requested_at=time.monotonic(),**{key:time.monotonic()-.601})
        with pytest.raises(TimeoutError,match='反馈过期'):
            executor.validate({1:stale})
    assert sdk.calls==[]


def test_two_oldest_diagnostics_keep_retry_margin_without_widening_deadlines():
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import AXES
    tick=[100.];sdk=Controller();queries=[]
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner(),clock=lambda:tick[0])
    bus.open_readonly()
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:tick[0])
    try:
        for i,motor in sdk.motors.items():
            f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
            def read_f32(index,*args,read=f32,axis=i):
                tick[0]+=.008;queries.append((axis,index));return read(index,*args)
            def read_i8(index,*args,read=i8,axis=i):
                tick[0]+=.008;queries.append((axis,index));return read(index,*args)
            motor.robstride_get_param_f32_host_id=read_f32
            motor.robstride_get_param_i8=read_i8
        hw._read()
        def oldest(sample):
            return min(sample.mode_requested_at if sample.mode_requested_at is not None else sample.requested_at,
                sample.diagnostics_requested_at if sample.diagnostics_requested_at is not None else sample.requested_at)
        for cycle in range(12):
            expected=sorted(AXES,key=lambda i:oldest(bus.latest_samples[i]))[:2]
            queries.clear();began=tick[0]
            if cycle==4:tick[0]+=.1  # One slow/retried read budget, without relaxing 600 ms.
            samples=hw._read_cycle()
            assert len(queries)==10
            assert [i for i,key in queries if key==0x7019]==list(AXES)
            assert [i for i,key in queries if key==0x7005]==expected
            assert max(tick[0]-oldest(s) for s in samples.values()) < .45
            tick[0]=max(tick[0],began+.09)
        assert hw.config.policy.feedback_age==.6 and sdk.calls==[]
    finally:bus.close()


def test_active_commands_no_longer_wait_for_all_six_diagnostic_reads(fixture):
    import statistics
    hw,sdk=fixture
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    for motor in sdk.motors.values():
        f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
        def slow_f32(*args,read=f32): time.sleep(.008);return read(*args)
        def slow_i8(*args,read=i8): time.sleep(.008);return read(*args)
        motor.robstride_get_param_f32_host_id=slow_f32
        motor.robstride_get_param_i8=slow_i8
    began=time.monotonic()
    for _ in range(24):
        hw.teleop_heartbeat(token);time.sleep(.05)
    frames=[f for f in hw.executor.frames if f['phase']=='trajectory' and f['time']>began+.1]
    intervals=[b['time']-a['time'] for a,b in zip(frames,frames[1:])]
    assert hw.motion_ready,hw.error
    assert len(frames)>=10 and statistics.median(intervals)<.11
    assert max(time.monotonic()-s.diagnostics_requested_at for s in hw.samples.values()) < .6
    hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY',timeout=4)
    assert hw.last_report['cleanup_ok']


@pytest.mark.parametrize('speed',[.005,.027,.175,.9])
def test_full_accepts_custom_numeric_speed_without_enabling(fixture,speed):
    hw,sdk=fixture
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire()
    hw.teleop_send(token,1,time.monotonic(),[0.]*6,None,speed)
    assert hw.speed==speed and not hw.motion_ready and sdk.calls==[]
    with pytest.raises(ValueError,match='速度'):
        hw.teleop_send(token,2,time.monotonic(),[0.]*6,None,1.0001)


def test_legacy_and_new_full_recovery_match_exact_profiles(fixture):
    from rebotarmcontroller.csp_reset import validate_snapshot
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import FULL_MODE,SMOOTH_FULL_MODE,OFFICIAL_FULL_MODE,LIVE_FULL_MODE,proposal
    from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Envelope
    hw,sdk=fixture
    for version in (FULL_MODE,SMOOTH_FULL_MODE,OFFICIAL_FULL_MODE,LIVE_FULL_MODE):
        source=dict(backend='csp-twin-v1',control_settings={'mode':'full'},
            config_fingerprint=Envelope.parse(proposal(hw.teleop_limits,mode=version)[0]).fingerprint,
            register_snapshot={str(i):{hex(k):v for k,v in m.regs.items() if k!=0x7016} for i,m in sdk.motors.items()})
        originals,matched=validate_snapshot(source,hw.teleop_limits)
        assert matched==version and originals[1][0x7017]==1.
        source['config_fingerprint']='foreign'
        with pytest.raises(ValueError,match='方案不符'): validate_snapshot(source,hw.teleop_limits)


def test_new_profile_refuses_lower_existing_drive_limit_before_enable(fixture):
    hw,sdk=fixture
    for motor in sdk.motors.values(): motor.regs[0x7017]=.2
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.last_report is not None)
    assert not any(c[0]=='enable' for c in sdk.calls)
    assert all(m.regs[0x7017]==.2 for m in sdk.motors.values())


def test_official_full_releases_old_gravity_budget_without_using_peak_as_continuous():
    from rebotarmcontroller.csp_twin_driver import CheckedKinematics
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import TORQUE,gravity_limits_for_mode,OFFICIAL_FULL_MODE
    kin=CheckedKinematics();start=np.deg2rad([32.600110,-.141600,.187991,.652588,2.151123,9.406495])
    goal,_=kin.solve([.2,0,.2,0,0,0],start,orientation=False)
    with pytest.raises(ValueError,match='J2'):
        kin.check_path(start,goal,gravity_limits=TORQUE)
    kin.check_path(start,goal,gravity_limits=gravity_limits_for_mode(OFFICIAL_FULL_MODE))


def test_official_following_error_checks_sent_reference_and_rejects_stall_and_jump():
    from dataclasses import replace
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZExecutor,proposal,OFFICIAL_FULL_MODE
    from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Envelope,Sample
    tick=[100.];writes=[]
    config=Envelope.parse(proposal([(-1.,1.)]*6,mode=OFFICIAL_FULL_MODE)[0])
    bus=SimpleNamespace(drives={i:SimpleNamespace(runtime={0x7017:1.},write_register=lambda *a,**k:writes.append(a)) for i in range(1,7)})
    executor=NativeXYZExecutor(config,bus,fk=lambda q:[0,0,0],clock=lambda:tick[0])
    executor.check=lambda permit:None  # Isolate target/feedback checks, without a transport or permit.
    samples={i:Sample(i,0.,0.,5,tick[0],tick[0],iq_raw_a=0.) for i in range(1,7)}
    executor.send([0.]*6,samples,None,'trajectory')
    for prior,target in ((0.,.09),(.09,.18)):
        tick[0]+=.1
        samples={i:replace(s,q=prior if i==1 else 0.,requested_at=tick[0],received_at=tick[0]) for i,s in samples.items()}
        executor.send([target,0,0,0,0,0],samples,None,'trajectory')
    before=len(writes);tick[0]+=.1
    samples[1]=replace(samples[1],q=0.,requested_at=tick[0],received_at=tick[0])
    with pytest.raises(ValueError,match='跟踪误差'):executor.send([.18,0,0,0,0,0],samples,None,'trajectory')
    assert len(writes)==before
    samples[1]=replace(samples[1],q=.18)
    with pytest.raises(ValueError,match='限速预算'):executor.send([.8,0,0,0,0,0],samples,None,'trajectory')
    assert len(writes)==before


def test_official_runtime_keeps_lower_device_speed_and_restores_snapshot(fixture):
    hw,sdk=fixture;hw.set_control_mode('full')
    settings=hw.control_settings()
    assert settings['official_speed_max_rad_s']==1.5 and settings['max_speed_rad_s']==1.
    assert settings['acceleration_rad_s2']==4. and settings['manual_speed_fraction']==1.
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    assert all(d.runtime[0x7017]==1. for d in hw.bus.drives.values())
    assert hw.bus.drives[2].runtime[0x700B]==36. and hw.bus.drives[5].runtime[0x7018]==16.
    hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok']


def test_interleaved_reference_uses_query_time_not_a_future_command():
    from dataclasses import replace
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZExecutor,proposal,OFFICIAL_FULL_MODE
    from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Envelope,Sample
    tick=[100.];writes=[]
    config=Envelope.parse(proposal([(-1.,1.)]*6,mode=OFFICIAL_FULL_MODE)[0])
    bus=SimpleNamespace(drives={i:SimpleNamespace(runtime={0x7017:1.},write_register=lambda *a,**k:writes.append(a)) for i in range(1,7)})
    executor=NativeXYZExecutor(config,bus,fk=lambda q:[0,0,0],clock=lambda:tick[0]);executor.check=lambda permit:None
    samples={i:Sample(i,0.,0.,5,100.,100.,iq_raw_a=0.) for i in range(1,7)}
    executor.send([0.]*6,samples,None,'trajectory')
    samples={i:replace(s,requested_at=100.01,received_at=100.02) for i,s in samples.items()}
    for step in (1,2,3):
        tick[0]=100.+step*.04
        executor.send([step*.039,0,0,0,0,0],samples,None,'trajectory')
    # A genuinely later stalled sample is still rejected at the same 3° limit.
    before=len(writes);tick[0]+=.04
    samples[1]=replace(samples[1],requested_at=tick[0],received_at=tick[0])
    with pytest.raises(ValueError,match='跟踪误差'):executor.send([.117,0,0,0,0,0],samples,None,'trajectory')
    assert len(writes)==before


def test_delayed_sdk_advances_curve_between_queries_and_stop_clears_it(tmp_path):
    import statistics
    class DelayedMotor(Motor):
        def robstride_get_param_f32_host_id(self,*args):
            if (self.i==1 and args[0]==0x7019 and getattr(self.controller,'gap_at',float('inf'))<time.monotonic()
                    and not getattr(self.controller,'gap_done',False)):
                self.controller.gap_done=True;time.sleep(.12)
            time.sleep(.008);return super().robstride_get_param_f32_host_id(*args)
        def robstride_get_param_i8(self,*args):
            time.sleep(.008);return super().robstride_get_param_i8(*args)
    sdk=Controller()
    def add(i,host,model):
        sdk.motors[i]=DelayedMotor(i,sdk);return sdk.motors[i]
    sdk.add_robstride_motor=add
    hw=CspHardware([(-1,1)]*6,output=tmp_path,
        transport=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner()))
    done=threading.Event();heartbeat=None
    try:
        hw.connect();hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
        def keepalive():
            while not done.wait(.05):
                if hw.teleop_owned:hw.teleop_heartbeat(token)
        heartbeat=threading.Thread(target=keepalive);heartbeat.start()
        wait_until(lambda:hw.motion_ready,5)
        started=time.monotonic();curve=dict(start=list(hw.command_reference),joints=[.04,0,0,0,0,0],began=started,duration=1.5)
        sdk.gap_at=started+.6
        for seq in range(1,27):
            hw.teleop_point(token,seq,time.monotonic(),curve,.1);time.sleep(.05)
        assert hw.phase=='ACTIVE',hw.error
        frames=[f for f in hw.executor.frames if f['phase']=='trajectory' and not f.get('refresh')]
        intervals=[b['time']-a['time'] for a,b in zip(frames,frames[1:]) if a['time']>=started+.1]
        assert len(intervals)>30 and statistics.median(intervals)<.045
        assert sdk.gap_done and max(intervals)>.1
        for before,after in zip(frames,frames[1:]):
            delta=abs(after['commands'][0]['target_rad']-before['commands'][0]['target_rad'])
            # Frames are timestamped after check/path work; the requirement
            # here is the 50 ms jump cap even after the injected 120 ms stall.
            assert delta<=.1*.05+1e-9
        print('inert_8ms_sdk_reference_ms',dict(median=round(statistics.median(intervals)*1000,2),
            maximum=round(max(intervals)*1000,2),samples=len(intervals)))
        assert hw.control_settings()['reference_period_target_ms']==20
        hw.teleop_pause(token)
        assert hw.point_reference is None and hw.stream_velocity is None
        hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY',4)
        assert hw.last_report['cleanup_ok'] and not any(m.enabled for m in sdk.motors.values())
    finally:
        done.set()
        if heartbeat:heartbeat.join()
        hw.close()


def test_can_worker_keeps_fresh_manual_velocity_continuous_and_freezes_stale_input(fixture):
    hw,sdk=fixture
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    began=time.monotonic()
    hw.teleop_stream(token,1,began,list(hw.command_reference),[.03,0,0,0,0,0],.1)
    while time.monotonic()-began < .4:
        hw.teleop_heartbeat(token);time.sleep(.025)
    frames=[f for f in hw.executor.frames if f['phase']=='trajectory' and began+.07<f['time']<began+.21]
    positions=[f['commands'][0]['target_rad'] for f in frames]
    assert len(positions)>=4 and all(b>a for a,b in zip(positions,positions[1:]))
    assert hw.phase=='ACTIVE',hw.error
    frozen=tuple(hw.command_reference)
    for _ in range(4):hw.teleop_heartbeat(token);time.sleep(.025)
    assert tuple(hw.command_reference)==frozen and not np.any(hw.stream_ramp.velocity)
    hw.teleop_pause(token)
    assert hw.stream_velocity is None
    hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok']


@pytest.mark.parametrize('startup_jitter',[None,-.000042597763240337,.000042597763240337])
def test_full_control_protocol_preview_goal_pose_jog_and_feedback(fixture,startup_jitter):
    """Real FK/IK/collision + TwinControl + CSP worker; in-memory motors only."""
    import json
    from builtin_interfaces.msg import Time
    from std_msgs.msg import String
    from rebotarmcontroller.csp_twin_driver import CheckedKinematics
    from rebotarmcontroller.twin_control import TwinControl

    class Publisher:
        def __init__(self): self.messages=[]
        def publish(self,msg): self.messages.append(msg)
    node=SimpleNamespace(create_publisher=lambda *a,**k:Publisher(),
        create_service=lambda *a,**k:None,create_subscription=lambda *a,**k:None,
        create_timer=lambda *a,**k:None,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=Time)),
        get_logger=lambda:SimpleNamespace(warn=lambda message:None))
    hw,sdk=fixture;kin=CheckedKinematics()
    start=np.array([1.68,.001,.005,.011,.011,.052])
    if startup_jitter is not None:start[1]=-.004474109970033169
    for i,motor in sdk.motors.items(): motor.q=float(start[i-1])
    wait_until(lambda:np.max(abs(hw.get_joint_state()[0]-start)) < 1e-8 and np.max(abs(hw.get_joint_state()[1])) < .001)
    hw.teleop_limits=list(zip(kin.lower,kin.upper));hw.path_check=kin.check_path
    twin=TwinControl(node,hw,'offline_csp',kinematics=kin)
    seq=0;token=''
    def command(op,**args):
        nonlocal seq
        seq+=1
        twin.command(String(data=json.dumps(dict(op=op,session=token,seq=seq,stamp_ms=time.time()*1000,**args))))
        return json.loads(twin.events.messages[-1].data) if op!='heartbeat' else None
    result=command('control_mode',mode='full')
    assert result['ok'] and result['control_settings']['mode']=='full' and sdk.calls==[]
    hw.prepare();response=twin.acquire(None,SimpleNamespace())
    assert response.success;token=json.loads(response.message)['session']
    target=start.copy();target[5]+=.06
    if startup_jitter is not None:target[1]=.005
    preview=command('preview',pose=kin.pose(target),orientation=True,speed=.1)
    assert preview['ok'],preview
    assert max(abs(a-b) for a,b in zip(preview['plan']['joints'],start)) > math.radians(.5)
    if startup_jitter is not None:
        sdk.motors[2].q+=startup_jitter
        wait_until(lambda:abs(hw._q[1]-sdk.motors[2].q)<1e-9 and abs(hw._v[1])<.001)
    assert command('goal',plan_id=preview['plan']['id'])['ok']
    deadline=time.monotonic()+5
    while twin.mode=='GOAL' and time.monotonic()<deadline:
        command('heartbeat');twin.tick();time.sleep(.02)
    assert twin.mode=='HOLD',twin.message
    assert '已到达' in twin.message and twin.last_result['position_error_mm'] <= 2
    before=hw.get_joint_state()[0]
    jog_deadline=time.monotonic()+1
    while np.max(abs(hw.get_joint_state()[0]-before)) <= .005 and time.monotonic()<jog_deadline:
        result=command('jog',twist=[0,0,0,0,0,.04],speed=.1)
        assert result['ok'],result
        twin.tick();time.sleep(.02)
    assert np.max(abs(hw.get_joint_state()[0]-before)) > .005
    assert command('hold')['ok']
    twin._publish_state()
    state=json.loads(twin.pub.messages[-1].data)
    assert state['control_settings']['mode']=='full'
    assert np.allclose(state['actual'],hw.get_joint_state()[0])
    assert state['control_settings']['remaining_s'] is None
    assert state['control_settings']['continuous']
    assert not command('control_mode',mode='small')['ok']
    assert hw.control_mode=='full'
    assert twin.stop(None,SimpleNamespace()).success
    wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok'] and not any(m.enabled for m in sdk.motors.values())
    assert command('control_mode',mode='small')['ok']
    assert twin.plan is None


def reset_and_wait(hw):
    status = hw.request_reset()
    wait_until(lambda: hw.reset_status['status'] in ('succeeded','failed'),timeout=5)
    assert hw.reset_status['id'] == status['id']
    return hw.reset_status


def test_boundary_recovery_is_new_for_each_session_after_reset(fixture):
    hw,sdk=fixture
    hw.teleop_limits[1]=(0.,1.)
    hw.set_control_mode('full')
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    assert 2 in hw.bus.boundary_recovered
    hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY')
    assert reset_and_wait(hw)['status']=='succeeded'
    sdk.motors[2].q=math.radians(-.14)
    wait_until(lambda:abs(hw.get_joint_state()[0][1]-math.radians(-.14))<1e-8 and abs(hw.get_joint_state()[1][1])<.001)
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready or hw.last_report.get('reason')=='CSP 目标越界')
    assert hw.motion_ready,hw.error
    assert 2 not in hw.bus.boundary_recovered
    assert sdk.motors[2].regs[0x7016]==pytest.approx(math.radians(-.14))
    with pytest.raises(ValueError,match='越过'):
        hw.teleop_send(token,1,time.monotonic(),[0.,math.radians(-.15),0.,0.,0.,0.],None,.1)
    hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok']


def test_reset_readonly_clears_preparation_and_lease_without_enabling(fixture,tmp_path):
    hw,sdk=fixture;hw.output=tmp_path
    hw.set_control_mode('full')
    hw.prepare();token=hw.teleop_acquire()
    before=hw.get_joint_state()[0]
    status=reset_and_wait(hw)
    assert status['status']=='succeeded',status
    assert not hw.armed and not hw.session and hw.phase=='READONLY'
    assert hw.target is None and hw.origin is None and hw.last_seq==-1
    assert hw.control_mode=='full'  # range selection survives; preparation does not
    assert sdk.calls==[('disable',i) for i in range(1,7)]
    assert np.array_equal(hw.get_joint_state()[0],before)
    with pytest.raises(RuntimeError,match='失效'): hw.teleop_resume(token)


def failed_motion(hw,sdk,tmp_path):
    hw.output=tmp_path
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    sdk.bad_stop=True
    hw.teleop_stop(token)
    wait_until(lambda:hw.phase=='RECOVERY')
    return hw.recovery_required['report']


def test_reset_restores_snapshot_after_failed_stop_and_survives_restart(fixture,tmp_path):
    import json
    hw,sdk=fixture
    source=failed_motion(hw,sdk,tmp_path)
    saved=json.loads(open(source).read())
    before=len(sdk.calls)
    sdk.bad_stop=False
    status=reset_and_wait(hw)
    assert status['status']=='succeeded',status
    assert hw.phase=='READONLY' and not hw.recovery_required
    assert not any(c[0]=='enable' or (c[0]=='write' and c[2]==0x7016) for c in sdk.calls[before:])
    for i,motor in sdk.motors.items():
        for key,value in saved['register_snapshot'][str(i)].items():
            assert motor.regs[int(key,16)]==value
        assert not motor.enabled
    report=hw.last_report
    assert report['source_report']==source and len(report['source_sha256'])==64
    assert report['cleanup_ok'] and report['restoration']['ok']
    assert json.loads(open(source).read())==saved  # original incident retained
    hw.close()
    restarted=CspHardware([(-1,1)]*6,output=tmp_path,
        transport=NativeXYZTransport(factory=lambda **kw:sdk,ownership=Owner()))
    restarted.connect()
    try:
        assert restarted.phase=='READONLY' and not restarted.recovery_required
        calls=len(sdk.calls)
        assert reset_and_wait(restarted)['status']=='succeeded'
        assert sdk.calls[calls:]==[('disable',i) for i in range(1,7)]
    finally: restarted.close()


@pytest.mark.parametrize('issue',['disabled-status','protected-pid','foreign-runtime','missing-snapshot'])
def test_reset_refuses_unverified_recovery_and_retry_retains_original(fixture,tmp_path,issue):
    import json
    hw,sdk=fixture
    source=failed_motion(hw,sdk,tmp_path)
    sdk.bad_stop=False
    if issue=='disabled-status': sdk.bad_stop=True
    if issue=='protected-pid': sdk.motors[2].regs[0x701F]=77.
    if issue=='foreign-runtime': sdk.motors[2].regs[0x7017]=.4
    if issue=='missing-snapshot':
        report=json.loads(open(source).read());del report['register_snapshot']['2']
        open(source,'w').write(json.dumps(report))
    calls=len(sdk.calls)
    assert reset_and_wait(hw)['status']=='failed'
    assert hw.phase=='RECOVERY' and hw.recovery_required
    assert not any(c[0] in ('write','enable') for c in sdk.calls[calls:])
    assert hw.last_report['source_report']==source
    # Repeated resets may not use defaults to conceal missing/changed evidence.
    assert reset_and_wait(hw)['status']=='failed'
    assert hw.phase=='RECOVERY'
    if issue=='disabled-status':
        sdk.bad_stop=False
        assert reset_and_wait(hw)['status']=='succeeded'
        assert hw.phase=='READONLY'


def test_reset_unknown_mode_without_snapshot_keeps_lock():
    sdk=Controller()
    original=sdk.add_robstride_motor
    def add(*args):
        motor=original(*args);motor.regs[0x7005]=5;return motor
    sdk.add_robstride_motor=add
    hw=CspHardware([(-1,1)]*6,transport=NativeXYZTransport(factory=lambda **kw:sdk,ownership=Owner()))
    hw.connect()
    try:
        assert reset_and_wait(hw)['status']=='failed'
        assert '快照' in hw.reset_status['message']
        assert not any(c[0] in ('write','enable') for c in sdk.calls)
        assert hw.phase=='RECOVERY'
    finally: hw.close()


def test_reset_moving_feedback_never_restores(fixture,tmp_path):
    hw,sdk=fixture
    failed_motion(hw,sdk,tmp_path)
    sdk.bad_stop=False
    original=hw.bus.read
    from dataclasses import replace
    def moving(*args,**kwargs):
        return {i:replace(s,qd=.02) for i,s in original(*args,**kwargs).items()}
    hw.bus.read=moving
    calls=len(sdk.calls)
    assert reset_and_wait(hw)['status']=='failed'
    assert '停稳' in hw.reset_status['message']
    assert not any(c[0] in ('write','enable') for c in sdk.calls[calls:])


def test_reset_active_stops_before_reset_and_invalidates_old_inputs(fixture,tmp_path):
    import json
    hw,sdk=fixture;hw.output=tmp_path
    twin,command=protocol_console(hw)
    hw.prepare()
    token=json.loads(twin.acquire(None,SimpleNamespace()).message)['session']
    assert command('power_hold',token)['ok']
    wait_until(lambda:hw.motion_ready)
    twin.plan={'id':'old'};twin.goal={'joints':[.1]*6};twin.reference=[.1]*6
    result=twin.reset(None,SimpleNamespace())
    assert result.success
    job=json.loads(result.message)['id']
    duplicate=twin.reset(None,SimpleNamespace())
    assert json.loads(duplicate.message)['id']==job
    assert not twin.session and twin.plan is twin.goal is twin.reference is None
    assert twin.last_result is None and twin.input_seq==-1
    assert not command('power_hold',token)['ok']
    with pytest.raises(RuntimeError,match='重置'): hw.prepare()
    wait_until(lambda:hw.reset_status['status']=='succeeded',timeout=5)
    assert hw.phase=='READONLY' and not hw.teleop_owned and not hw.armed
    assert not any(m.enabled for m in sdk.motors.values())
    assert len([c for c in sdk.calls if c[0]=='enable'])==6  # no reset enable
    assert not command('power_hold',token)['ok']


def test_reset_partial_restore_failure_can_retry_from_same_original(fixture,tmp_path):
    hw,sdk=fixture
    source=failed_motion(hw,sdk,tmp_path)
    sdk.bad_stop=False
    motor=sdk.motors[2]
    write=motor.robstride_write_param_f32
    def broken(index,value):
        write(index,value)
        if index==0x7017: raise TimeoutError('模拟写后读回中断')
    motor.robstride_write_param_f32=broken
    calls=len(sdk.calls)
    assert reset_and_wait(hw)['status']=='failed'
    assert hw.last_report['restoration']['errors']
    assert hw.last_report['source_report']==source
    assert not any(c[0]=='enable' for c in sdk.calls[calls:])
    motor.robstride_write_param_f32=write
    assert reset_and_wait(hw)['status']=='succeeded'
    assert hw.last_report['source_report']==source
    assert all(m.regs[0x7005]==0 for m in sdk.motors.values())


def test_reset_rejects_delayed_preview_created_before_reset(fixture):
    import json
    from std_msgs.msg import String
    hw,sdk=fixture
    twin,_=protocol_console(hw)
    old=time.time()*1000
    assert twin.reset(None,SimpleNamespace()).success
    wait_until(lambda:hw.reset_status['status']=='succeeded')
    twin.command(String(data=json.dumps(dict(op='preview',stamp_ms=old,pose=[0]*6))))
    result=json.loads(twin.events.messages[-1].data)
    assert not result['ok'] and '旧请求' in result['message']
    assert twin.plan is None and not any(c[0]=='enable' for c in sdk.calls)


def test_fresh_early_axis_is_not_hidden_during_slow_six_axis_scan():
    # Reproduce the real 608 ms failure without changing the 600 ms threshold:
    # a 150 ms scheduling gap plus a 360 ms scan made the previous whole-scan
    # cache look old even after J1 had already replied in the current scan.
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZExecutor
    tick=[1.]
    sdk=Controller()
    bus=NativeXYZTransport(factory=lambda **kw:sdk,ownership=Owner(),clock=lambda:tick[0])
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:tick[0])
    bus.open_readonly()
    for motor in sdk.motors.values():
        f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
        def f(*a,read=f32):tick[0]+=.015;return read(*a)
        def g(*a,read=i8):tick[0]+=.015;return read(*a)
        motor.robstride_get_param_f32_host_id=f
        motor.robstride_get_param_i8=g
    try:
        hw._read()
        hw._read()  # warm steady-state timing after velocity-window bootstrap
        baseline=hw.samples[1].requested_at
        tick[0]+=.15
        executor=NativeXYZExecutor(hw.config,bus,fk=lambda q:[0,0,0],clock=lambda:tick[0])
        ages=[]
        def check():
            executor.validate(hw.samples,quiet=True)
            ages.append(tick[0]-hw.samples[1].requested_at)
        hw._read(check)
        assert hw.samples[1].requested_at>baseline
        assert max(ages)<.6
        assert hw.config.policy.feedback_age==.6
        assert sdk.calls==[]
        tick[0]+=.61
        with pytest.raises(TimeoutError,match='反馈过期'):
            hw._read(check)
        assert sdk.calls==[]
    finally:bus.close()


def test_continuous_mode_keeps_trajectory_budget_independent_of_session_age(fixture):
    hw,sdk=fixture
    twin,_=protocol_console(hw)
    hw.set_control_mode('full')
    twin._check_goal_budget(7200.)
    hw.set_control_mode('small')
    with pytest.raises(ValueError,match='时长'):twin._check_goal_budget(7200.)
    assert sdk.calls==[]


def test_continuous_expired_permit_is_stopped_not_revived(fixture):
    hw,sdk=fixture
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    hw.bus.permit.expires_at=time.monotonic()-.01
    wait_until(lambda:hw.phase=='HOLDING')
    assert '许可过期' in hw.holding['reason']
    assert not hw.session and all(m.enabled for m in sdk.motors.values())
    assert hw.bus.permit.expires_at < time.monotonic()


@pytest.mark.parametrize('cause',['command_age','feedback_missing'])
def test_full_communication_fault_inhibits_writes_without_disable(fixture,tmp_path,cause):
    import json
    hw,sdk=fixture;hw.output=tmp_path
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    original=sdk.motors[4].robstride_get_param_f32_host_id
    if cause=='feedback_missing':
        sdk.motors[4].robstride_get_param_f32_host_id=lambda index,*args:float('nan') if index==0x7019 else original(index,*args)
    else:hw.executor.last_axis_command[1]=hw.clock()-hw.config.policy.command_age-.01
    wait_until(lambda:hw.phase=='HOLDING')
    held=len(sdk.calls);time.sleep(.2)
    assert len(sdk.calls)==held and all(m.enabled for m in sdk.motors.values())
    assert not hw.teleop_owned and hw.target is None and hw.point_reference is None
    if cause=='feedback_missing':
        assert not hw.holding['feedback_confirmed']
        with pytest.raises(RuntimeError):hw.teleop_acquire()
        sdk.motors[4].robstride_get_param_f32_host_id=original
    wait_until(lambda:hw.holding['feedback_confirmed'])
    assert len(sdk.calls)==held  # recovered telemetry must never resume targets
    journal=json.loads(sorted(tmp_path.glob('csp-session-*.json'))[-1].read_text())
    assert not journal['cleanup_ok'] and journal['holding'] and journal['register_snapshot']
    hw.disable();wait_until(lambda:hw.phase=='READONLY')
    assert hw.last_report['cleanup_ok'] and not any(m.enabled for m in sdk.motors.values())


def test_csp_snapshot_is_recoverable_before_first_mode_write_and_enable(fixture,tmp_path):
    import json
    from rebotarmcontroller.csp_reset import validate_snapshot
    hw,sdk=fixture;hw.output=tmp_path;checked=[]
    hw.set_control_mode('full')
    for motor in sdk.motors.values():
        write=motor.robstride_write_param_f32
        def guarded_write(index,value,write=write):
            if index==0x7005 and value==5:
                journal=sorted(tmp_path.glob('csp-session-*.json'))[-1]
                row=json.loads(journal.read_text())
                assert row['write_intent'] and not row['cleanup_ok']
                originals,_=validate_snapshot(row,hw.teleop_limits)
                assert len(originals)==6;checked.append(True)
            write(index,value)
        motor.robstride_write_param_f32=guarded_write
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    assert len(checked)==6
    recovered=CspHardware([(-1,1)]*6,output=tmp_path)
    assert recovered.recovery_required and 'CSP启动' in recovered.recovery_required['reason']
    hw.disable();wait_until(lambda:hw.phase=='READONLY')
    assert CspHardware([(-1,1)]*6,output=tmp_path).recovery_required is None


def test_snapshot_flush_failure_never_enables_or_writes_runtime(fixture,tmp_path,monkeypatch):
    hw,sdk=fixture;hw.output=tmp_path;hw.set_control_mode('full')
    def fail(*args):raise OSError('disk flush failed')
    monkeypatch.setattr('rebotarmcontroller.csp_hardware.os.fsync',fail)
    hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.last_report is not None)
    assert 'disk flush failed' in hw.last_report['reason']
    assert not any(c[0]=='enable' or c[0]=='write' and c[2] in (0x7005,0x7017,0x7018,0x700B) for c in sdk.calls)


@pytest.mark.parametrize('bad',[None,[float('nan')]*6])
def test_invalid_generated_reference_holds_without_disabling(fixture,bad):
    hw,sdk=fixture;hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire();hw.teleop_resume(token)
    wait_until(lambda:hw.motion_ready)
    hw._desired_reference=lambda *a:bad
    wait_until(lambda:hw.phase=='HOLDING' and hw.holding['feedback_confirmed'])
    assert not hw.holding['automatic_manual_recovery']
    assert all(m.enabled for m in sdk.motors.values())
    held=len(sdk.calls);time.sleep(.15);assert len(sdk.calls)==held
    assert not hw.session and hw.target is None
