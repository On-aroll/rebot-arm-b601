"""J7 uses the existing inert SDK/CAN owner; never imports a physical SDK."""
import json
import os
import stat
import threading
import time
from types import SimpleNamespace

import pytest
from test_csp_twin import Controller, Owner, wait_until
from rebotarmcontroller.csp_gripper import CspGripper
from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZTransport
from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Sample


def inert_gripper():
    clock=[100.];sent=[]
    h=SimpleNamespace(clock=lambda:clock[0],output=None,motion_ready=True,
                      config=SimpleNamespace(policy=SimpleNamespace(command_age=.5)))
    g=CspGripper(h);g.reference=2.;g.speed_max=1.;g.authorized=True
    g.sample=Sample(7,2.,0.,5,clock[0],clock[0],iq_raw_a=0.)
    g.last_poll=clock[0];g.history=[(clock[0],g.reference)]
    g.view=SimpleNamespace(active={7},permit=SimpleNamespace(expires_at=102.),runtime={0x7018:16.},
                           write_register=lambda index,value,*a,**k:sent.append((clock[0],value)))
    def read(check):
        check();g.sample=Sample(7,g.reference,0.,5,clock[0],clock[0],iq_raw_a=0.)
        return g.sample
    g._read=read
    return g,clock,sent


def test_gripper_speed_change_preserves_acceleration_jerk_and_deadman_hold():
    g,clock,sent=inert_gripper();acceleration=0.
    for n in range(18):
        clock[0]+=.1;g.manual=dict(velocity=1. if n<8 else .1,at=clock[0]);g.step()
        assert not g.error,g.error
        assert 0<=g.manual_velocity<=1.
        a=float(g.manual_ramp.acceleration[0])
        assert abs(a)<=4.+1e-9 and abs(a-acceleration)<=30.*.1+1e-8
        acceleration=a
    assert g.manual_velocity==pytest.approx(.1)
    assert all(0<=q<=5 for _,q in sent)
    frozen=g.reference;g.pause();clock[0]+=.1;g.step()
    assert g.reference==frozen and not any(g.manual_ramp.velocity)


def test_gripper_point_clock_does_not_jump_forward_after_delayed_reply():
    g,clock,sent=inert_gripper();g.pending=(2.5,1.)
    clock[0]+=.1;g.step();before=g.reference
    clock[0]+=.4;g.step()
    assert not g.error,g.error
    assert g.curve['elapsed']==pytest.approx(.2)
    assert 0<g.reference-before<=.1+1e-9


def test_initialized_manual_step_reads_one_fresh_feedback_group():
    g,clock,sent=inert_gripper();read=g._read;reads=[]
    def counted(check):
        reads.append(clock[0]);return read(check)
    g._read=counted
    for _ in range(6):
        clock[0]+=.1;g.manual=dict(velocity=.1,at=clock[0]);g.step()
    assert len(reads)==3  # Feedback cadence is separate from bounded reference updates.
    assert len(sent)==6 and g.reference>2.01


def test_reference_updates_never_renew_cached_feedback_or_skip_deadman():
    g,clock,sent=inert_gripper();g.manual=dict(velocity=.1,at=clock[0])
    for _ in range(5):clock[0]+=.1;g.step(poll=False)
    assert g.sample.requested_at==100. and g.manual is None
    frozen=g.reference
    clock[0]+=.11
    with pytest.raises(TimeoutError,match='反馈或命令时限'):g.step(poll=False)
    assert g.reference==frozen and g.sample.requested_at==100.


def test_single_field_read_preserves_other_ages_and_requires_all_fresh():
    from dataclasses import replace
    from collections import deque
    g,clock,_=inert_gripper();calls=[]
    def f32(index,*args):
        calls.append(index);clock[0]+=.008;return 2. if index==0x7019 else .25
    motor=SimpleNamespace(robstride_get_param_f32_host_id=f32,
        robstride_get_param_i8=lambda *args:5)
    g.hw.bus=SimpleNamespace(latest_samples={},poll_io=deque(maxlen=64))
    g._motor=lambda:motor
    g.sample=replace(g.sample,mode_requested_at=99.9,diagnostics_requested_at=99.8)
    clock[0]=100.2;g.read_field()
    assert calls==[0x701A]
    assert g.sample.requested_at==100. and g.sample.mode_requested_at==99.9
    assert g.sample.diagnostics_requested_at==100.2 and g.feedback_time()==99.9
    clock[0]=100.51
    assert not g.state()['fresh']  # A new current cannot renew an old mode.
    with pytest.raises(TimeoutError,match='反馈或命令时限'):g.step(poll=False)


def test_partial_feedback_cannot_multiply_grasp_contact_evidence():
    from dataclasses import replace
    g,clock,_=inert_gripper();observations=[]
    g.grasp=SimpleNamespace(status='closing',update=lambda **row:observations.append(row))
    g.sample=replace(g.sample,mode_requested_at=100.01,diagnostics_requested_at=100.02)
    clock[0]=100.1;g._observe_grasp(g.sample)
    g.sample=replace(g.sample,requested_at=100.1)
    g._observe_grasp(g.sample)
    g.sample=replace(g.sample,mode_requested_at=100.11)
    g._observe_grasp(g.sample)
    assert len(observations)==1
    g.sample=replace(g.sample,diagnostics_requested_at=100.12)
    clock[0]=100.2;g._observe_grasp(g.sample)
    assert [row['stamp'] for row in observations]==[100.,100.1]


@pytest.mark.parametrize('active',[False,True])
def test_single_field_failure_keeps_existing_active_and_readonly_policy(active):
    g,clock,_=inert_gripper()
    if not active:g.view=None
    def unavailable(check):raise TimeoutError('query unavailable')
    g.read_field=unavailable;clock[0]+=.02
    if active:
        with pytest.raises(TimeoutError,match='unavailable'):g.step(one_field=True)
    else:g.step(one_field=True)
    assert g.error=='query unavailable' and g.sample.requested_at==100.


def test_inactive_sample_after_long_arm_startup_uses_original_full_read():
    g,clock,_=inert_gripper();g.view=None;reads=[];original=g._read
    def full(check):reads.append(clock[0]);return original(check)
    g._read=full;clock[0]+=5.
    g.read_field()
    assert reads==[105.] and g.sample.requested_at==105.


def test_active_single_position_read_never_bridges_expired_velocity_window():
    from collections import deque
    g,clock,_=inert_gripper()
    g.hw.bus=SimpleNamespace(latest_samples={},poll_io=deque(maxlen=64))
    g._motor=lambda:SimpleNamespace(robstride_get_param_f32_host_id=lambda *a:2.)
    clock[0]+=.7
    with pytest.raises(TimeoutError,match='差分窗口'):g.read_field()
    assert g.sample.requested_at==100.


@pytest.mark.parametrize('active',[False,True])
@pytest.mark.parametrize('slow_index',[0x7019,0x7005,0x701A])
def test_shared_poll_interleaves_j7_fields_across_296ms_reply(active,slow_index):
    from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZExecutor
    clock=[100.];sdk=Controller();slow=[False];queries=[]
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner(),clock=lambda:clock[0])
    bus.open_readonly()
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:clock[0])
    hw.phase='READONLY';hw.set_control_mode('full')
    executor=NativeXYZExecutor(hw.config,bus,fk=lambda q:[0,0,0],clock=lambda:clock[0])
    try:
        g=hw.gripper;g._motor()
        for axis,motor in sdk.motors.items():
            f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
            def elapsed(axis,index):
                if axis==7 and hw.phase=='ACTIVE':
                    assert g.feedback_time() <= min(stamp for s in hw.samples.values() for stamp in (
                        s.requested_at,s.mode_requested_at,s.diagnostics_requested_at))
                queries.append((axis,index));clock[0]+=.012
                if slow[0] and axis==7 and index==slow_index:
                    clock[0]+=.296;slow[0]=False
            def read_f32(index,*args,read=f32,axis=axis):
                elapsed(axis,index);return read(index,*args)
            def read_i8(index,*args,read=i8,axis=axis):
                elapsed(axis,index);return read(index,*args)
            motor.robstride_get_param_f32_host_id=read_f32;motor.robstride_get_param_i8=read_i8
        hw._read();g._read()
        if active:
            g.view=SimpleNamespace(active={7},permit=SimpleNamespace(expires_at=102.),runtime={0x7018:16.},
                write_register=lambda *a,**k:None)
            g.authorized=True;g.reference=g.sample.q;g.history=[(clock[0],g.reference)]
            sdk.motors[7].regs[0x7005]=5
            g._read()
        queries.clear()
        hw.phase='ACTIVE'
        next_reference=[clock[0]]
        def validate():
            if clock[0]>=next_reference[0]:
                clock[0]+=.002;next_reference[0]=clock[0]+.02
            executor.validate(dict(hw.samples))
            assert g.state()['fresh']
        for _ in range(4):hw._read_cycle(on_io=validate)
        slow[0]=True
        for _ in range(6):hw._read_cycle(on_io=validate)
        assert not slow[0] and hw.config.policy.feedback_age==.6
        assert not g.error
        assert len([axis for axis,index in queries if axis==7])>12
        assert bus.poll_io.maxlen==64 and len(bus.poll_io)<=64
    finally:bus.close()


@pytest.mark.parametrize('target,sign',[(0.,-1),(5.,1)])
def test_endpoint_reference_stays_inside_strict_feedback_range(target,sign):
    g,clock,sent=inert_gripper();g.pending=(target,1.)
    def measured(check):
        check()
        # Measured endpoint noise from the real 5 rad stop, with both signs.
        g.sample=Sample(7,g.reference+sign*.000087262,0.,5,clock[0],clock[0],iq_raw_a=.25)
        return g.sample
    g._read=measured
    for _ in range(90):clock[0]+=.1;g.step()
    assert g.arrived and g.state()['target']==target
    assert abs(g.reference-target)==pytest.approx(.001)
    assert g.state()['command_reference']==g.reference
    assert all(.001<=q<=4.999 for _,q in sent)
    g.curve=None
    for _ in range(12):
        clock[0]+=.1;g.manual=dict(velocity=sign*.1,at=clock[0]);g.step()
    assert all(.001<=q<=4.999 for _,q in sent)
    # Actual feedback protection is unchanged, even for the small overshoot.
    g.sample=Sample(7,target+sign*.000087262,0.,5,clock[0],clock[0],iq_raw_a=.25)
    clock[0]+=.03
    with pytest.raises(ValueError,match='限位'):g.step(poll=False)


def hardware(tmp_path):
    sdk=Controller();factories=[]
    def factory(**kwargs):factories.append(kwargs);return sdk
    hw=CspHardware([(-1,1)]*6,output=tmp_path,transport=NativeXYZTransport(factory=factory,ownership=Owner()))
    hw.connect()
    return hw,sdk,factories


@pytest.mark.parametrize('failure_at',[1,2])
def test_j7_durable_intent_failure_blocks_every_hardware_write(tmp_path,monkeypatch,failure_at):
    hw,sdk,_=hardware(tmp_path)
    try:
        calls=[]
        def fsync(fd):
            calls.append('directory' if stat.S_ISDIR(os.fstat(fd).st_mode) else 'file')
            if len(calls)==failure_at:raise OSError('injected durable write failure')
        monkeypatch.setattr('rebotarmcontroller.csp_gripper.os.fsync',fsync)
        hw.gripper.speed_max=1.;hw.gripper.authorized=True
        with pytest.raises(OSError,match='durable write'):hw.gripper._start(lambda:None)
        assert calls==['file','directory'][:failure_at]
        assert not sdk.calls  # no disable, mode/parameter write or enable
        assert hw.gripper.recovery['pending'] and hw.gripper.recovery['original']
        with pytest.raises(RuntimeError,match='恢复'):hw.gripper._start(lambda:None)
    finally:hw.close()


def test_j7_journal_syncs_file_then_directory_and_only_then_clears_recovery(tmp_path,monkeypatch):
    g,_,_=inert_gripper();g.journal=tmp_path/'csp-gripper-state.json'
    g.view.original={0x7005:0};g.view.register_log=[]
    calls=[]
    def fsync(fd):calls.append('directory' if stat.S_ISDIR(os.fstat(fd).st_mode) else 'file')
    monkeypatch.setattr('rebotarmcontroller.csp_gripper.os.fsync',fsync)
    g._save(True);assert g.recovery['pending'] and calls==['file','directory']
    calls.clear();g._save(False)
    assert g.recovery is None and calls==['file','directory']
    assert not json.loads(g.journal.read_text())['pending']
    g._save(True)
    def failing_fsync(fd):raise OSError('disk failure')
    monkeypatch.setattr('rebotarmcontroller.csp_gripper.os.fsync',failing_fsync)
    with pytest.raises(OSError):g._save(False)
    assert g.recovery['pending'] and json.loads(g.journal.read_text())['pending']


def test_gripper_probe_is_readonly_and_shares_controller(tmp_path):
    hw,sdk,factories=hardware(tmp_path)
    try:
        assert 7 not in sdk.motors and not hw.gripper_state()['authorized']
        hw.probe_gripper();wait_until(lambda:hw.gripper_state()['available'])
        assert len(factories)==1 and hw.bus.motors[7] is sdk.motors[7]
        assert hw.gripper_state()['fresh'] and hw.gripper_state()['speed_max']==1.
        assert not sdk.calls and not (tmp_path/'csp-gripper-state.json').exists()
        with pytest.raises(RuntimeError):hw.gripper_target('wrong',.1,.1)
    finally:hw.close()
    assert not any(c[0]=='enable' for c in sdk.calls)


def test_j7_explicit_authorization_preload_stop_and_exact_parameter_restore(tmp_path):
    hw,sdk,factories=hardware(tmp_path);done=threading.Event();beat=None
    try:
        hw.probe_gripper();wait_until(lambda:hw.gripper_state()['available'])
        original=dict(sdk.motors[7].regs)
        hw.prepare();token=hw.teleop_acquire()
        with pytest.raises(ValueError,match='完整控制'):hw.authorize_gripper(token,True)
        hw.teleop_stop(token);hw.disable();wait_until(lambda:not hw.teleop_owned and not hw.armed)
        hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire()
        hw.authorize_gripper(token,True)
        with pytest.raises(ValueError,match='使能'):hw.gripper_target(token,.01,.1)
        def heartbeat():
            while not done.wait(.05):
                if hw.teleop_owned:hw.teleop_heartbeat(token)
        beat=threading.Thread(target=heartbeat);beat.start()
        hw.teleop_resume(token);wait_until(lambda:hw.motion_ready)
        before=len(sdk.calls);hw.gripper_target(token,.01,.1)
        wait_until(lambda:hw.gripper_state()['active'])
        assert (tmp_path/'csp-gripper-state.json').exists()
        assert json.loads((tmp_path/'csp-gripper-state.json').read_text())['pending']
        wait_until(lambda:hw.gripper_state()['arrived'],4)
        calls=[c for c in sdk.calls[before:] if c[1]==7];enable=next(i for i,c in enumerate(calls) if c[0]=='enable')
        assert any(c[:3]==('write',7,0x7016) and c[3]==0. for c in calls[:enable])
        assert len(factories)==1 and hw.phase=='ACTIVE',hw.error
        hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY',5)
        assert not any(m.enabled for m in sdk.motors.values())
        assert hw.last_report['cleanup_ok'] and not hw.gripper_state()['authorized']
        assert hw.gripper_state()['target'] is None
        assert not json.loads((tmp_path/'csp-gripper-state.json').read_text())['pending']
        for register in (0x7005,0x7017,0x7018,0x700B,0x701E,0x701F,0x7020,0x702B):assert sdk.motors[7].regs[register]==original[register]
        assert not any(c[0]=='write' and c[1]==7 and c[2] in (0x701E,0x701F,0x7020,0x702B) for c in sdk.calls)
    finally:
        done.set()
        if beat:beat.join()
        hw.close()


def test_j7_start_with_revoked_authorization_never_renews_or_writes(tmp_path):
    hw,sdk,_=hardware(tmp_path)
    try:
        hw.gripper.speed_max=1.
        with pytest.raises(RuntimeError,match='授权已撤销'):hw.gripper._start(lambda:None)
        assert not sdk.calls and not hw.gripper.preparing
        assert not hw.gripper.state()['active']
    finally:hw.close()


def test_unknown_j7_recovery_blocks_prepare_and_cannot_be_erased_by_reset(tmp_path):
    (tmp_path/'csp-gripper-state.json').write_text(json.dumps(dict(pending=True,version='unknown')))
    hw,sdk,factories=hardware(tmp_path)
    try:
        assert hw.gripper_state()['recovery_required']
        with pytest.raises(RuntimeError,match='J7'):hw.prepare()
        hw.request_reset();wait_until(lambda:hw.reset_status.get('status')=='failed',5)
        assert hw.gripper_state()['recovery_required'] and not any(c[0]=='enable' for c in sdk.calls)
        assert json.loads((tmp_path/'csp-gripper-state.json').read_text())['pending']
    finally:hw.close()


def test_manual_j7_deadman_hold_and_communication_hold_share_owner(tmp_path):
    hw,sdk,factories=hardware(tmp_path);done=threading.Event();owner=['']
    def heartbeat():
        while not done.wait(.05):
            if owner[0] and hw.teleop_owned:
                try:hw.teleop_heartbeat(owner[0])
                except RuntimeError:pass
    beat=threading.Thread(target=heartbeat);beat.start()
    try:
        hw.probe_gripper();wait_until(lambda:hw.gripper_state()['available'])
        hw.set_control_mode('full');hw.prepare();owner[0]=hw.teleop_acquire()
        with pytest.raises(ValueError):hw.validate_gripper_jog(owner[0],1.1)
        assert not sdk.calls
        hw.teleop_resume(owner[0]);wait_until(lambda:hw.motion_ready)
        for _ in range(16):
            hw.gripper_jog(owner[0],.1);time.sleep(.05)
        assert hw.gripper_state()['position']>.025 and len(factories)==1
        hw.gripper_jog(owner[0],0)
        frozen=hw.gripper.reference;time.sleep(.3)
        assert hw.gripper.reference==frozen and hw.phase=='ACTIVE'
        hw.suspend('通信测试');wait_until(lambda:hw.phase=='HOLDING' and hw.holding['feedback_confirmed'])
        held=len(sdk.calls);time.sleep(.3)
        assert all(m.enabled for m in sdk.motors.values()) and len(sdk.calls)==held
        assert hw.gripper_state()['fresh'] and hw.gripper.manual is None
        with hw.lock:hw.gripper.error='通信异常暂停，等待重新接管'
        owner[0]=hw.teleop_acquire();wait_until(lambda:hw.motion_ready)
        time.sleep(.35)
        assert hw.phase=='ACTIVE',hw.error
        assert not hw.gripper_state()['error'] and hw.gripper.manual is None
        assert all(m.enabled for m in sdk.motors.values())
        hw.teleop_stop(owner[0]);owner[0]='';wait_until(lambda:hw.phase=='READONLY',5)
        assert not any(m.enabled for m in sdk.motors.values()) and hw.last_report['cleanup_ok']
    finally:
        done.set();beat.join();hw.close()


def test_partial_j7_start_cannot_be_resumed_as_initialized_hold():
    g,clock,sent=inert_gripper();g.reference=None;g.history=[]
    def read(check):
        check();return Sample(7,.54,0.,5,clock[0],clock[0],iq_raw_a=0.)
    g._read=read
    with pytest.raises(TimeoutError,match='J7启动未完成'):g.observe_hold(lambda:None)
    with pytest.raises(RuntimeError,match='J7启动未完成'):g.resume_hold()
    clock[0]+=.1
    with pytest.raises(RuntimeError,match='J7启动未完成'):g.step()
    assert not sent and not g.history


def test_partial_j7_start_is_not_reported_active_and_retains_recovery_lock():
    g,_,_=inert_gripper();g.reference=None;g.recovery={'pending':True}
    state=g.state()
    assert state['active'] is False
    assert state['recovery_required'] is True
    g.recovery=None
    def partial_failure(check):
        assert g.preparing
        g.recovery={'pending':True}
        raise TimeoutError('injected interrupted preparation')
    g._prepare_start=partial_failure
    with pytest.raises(TimeoutError,match='interrupted'):g._start(lambda:None)
    assert not g.preparing and g.state()['recovery_required'] and not g.state()['active']
    with pytest.raises(RuntimeError,match='恢复'):g._start(lambda:None)


def test_slow_first_keyboard_gripper_start_keeps_arm_feedback_and_command_deadlines(tmp_path):
    hw,sdk,factories=hardware(tmp_path);done=threading.Event();token='';beat=None;f32=i8=None
    try:
        hw.probe_gripper();wait_until(lambda:hw.gripper_state()['available'])
        hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire()
        def heartbeat():
            while not done.wait(.04):
                if hw.teleop_owned:
                    try:hw.teleop_heartbeat(token)
                    except RuntimeError:pass
        beat=threading.Thread(target=heartbeat);beat.start()
        hw.teleop_resume(token);wait_until(lambda:hw.motion_ready)
        motor=sdk.motors[7];f32=motor.robstride_get_param_f32_host_id;i8=motor.robstride_get_param_i8
        reads=[]
        def slow(read):
            def call(*args):
                time.sleep(.04);reads.append(threading.current_thread().name)
                return read(*args)
            return call
        motor.robstride_get_param_f32_host_id=slow(f32);motor.robstride_get_param_i8=slow(i8)
        began=time.monotonic();hw.gripper_jog(token,.1)
        wait_until(lambda:hw.gripper.reference is not None or hw.phase!='ACTIVE',timeout=6)
        assert hw.phase=='ACTIVE',hw.error
        assert time.monotonic()-began>.6  # The real startup work exceeds the guard without interleaving.
        assert hw.gripper.reference is not None and sdk.motors[7].enabled
        assert len(factories)==1 and set(reads)=={'csp-can-owner'}
        assert hw.clock()-hw._feedback_refreshed_at<hw.feedback_timeout
        assert not hw.holding
    finally:
        if f32 is not None:
            sdk.motors[7].robstride_get_param_f32_host_id=f32
            sdk.motors[7].robstride_get_param_i8=i8
        done.set()
        if beat:beat.join()
        hw.close()


def test_arm_poll_over_200ms_does_not_make_j7_probe_stale_on_arrival(tmp_path):
    hw,sdk,_=hardware(tmp_path);done=threading.Event();beat=None;originals=[]
    try:
        hw.set_control_mode('full');hw.probe_gripper()
        wait_until(lambda:hw.gripper_state()['fresh'])
        hw.prepare();token=hw.teleop_acquire()
        def heartbeat():
            while not done.wait(.04):
                if hw.teleop_owned:
                    try:hw.teleop_heartbeat(token)
                    except RuntimeError:pass
        beat=threading.Thread(target=heartbeat);beat.start()
        hw.teleop_resume(token);wait_until(lambda:hw.motion_ready)
        def delayed(read, delay):
            def call(*args):
                time.sleep(delay);return read(*args)
            return call
        for axis,motor in sdk.motors.items():
            for name in ('robstride_get_param_f32_host_id','robstride_get_param_i8'):
                original=getattr(motor,name);originals.append((motor,name,original))
                setattr(motor,name,delayed(original,.014 if axis<=6 else .008))
        began=hw.clock()
        wait_until(lambda:hw.gripper.sample.received_at>began,timeout=5)
        assert hw.phase=='ACTIVE',hw.error
        assert hw.gripper_state()['fresh'], 'J7 sample was already >600ms old on arrival'
        create_view=hw.gripper._new_view
        def long_snapshot(runtime):
            view=create_view(runtime);snapshot=view.snapshot_registers
            def scan(check):
                snapshot(check)
                for _ in range(40):
                    time.sleep(.02);check()
            view.snapshot_registers=scan
            return view
        hw.gripper._new_view=long_snapshot
        started=hw.clock()
        deadline=hw.clock()+12
        while hw.gripper.reference is None and hw.phase=='ACTIVE' and hw.clock()<deadline:
            try:hw.gripper_jog(token,.05)  # A held key supplies new input during startup.
            except ValueError as exc:pytest.fail(f'{exc}; J7={hw.gripper_state()}; now={hw.clock()}')
            time.sleep(.02)
        assert hw.phase=='ACTIVE',hw.error
        assert hw.gripper.reference is not None and hw.gripper_state()['fresh']
        assert hw.clock()-started>2  # Long preparation must renew a still-valid live permit.
    finally:
        for motor,name,original in originals:setattr(motor,name,original)
        done.set()
        if beat:beat.join()
        hw.close()


def test_j7_point_advances_between_slow_arm_replies_without_second_owner(tmp_path):
    hw,sdk,factories=hardware(tmp_path);done=threading.Event();beat=None;originals=[]
    try:
        hw.set_control_mode('full');hw.probe_gripper()
        wait_until(lambda:hw.gripper_state()['fresh'])
        hw.prepare();token=hw.teleop_acquire()
        def heartbeat():
            while not done.wait(.04):
                if hw.teleop_owned:
                    try:hw.teleop_heartbeat(token)
                    except RuntimeError:pass
        beat=threading.Thread(target=heartbeat);beat.start()
        hw.teleop_resume(token);wait_until(lambda:hw.motion_ready)
        hw.authorize_gripper(token,True);hw.gripper_target(token,.001,.2)
        wait_until(lambda:hw.gripper_state()['arrived'],4)
        threads=set();j7_positions=[]
        def delayed(read,axis):
            def call(*args):
                threads.add(threading.current_thread().name)
                time.sleep(.014 if axis<=6 else .008)
                if axis==7 and args[0]==0x7019:j7_positions.append(hw.clock())
                return read(*args)
            return call
        for axis,motor in sdk.motors.items():
            for name in ('robstride_get_param_f32_host_id','robstride_get_param_i8'):
                original=getattr(motor,name);originals.append((motor,name,original))
                setattr(motor,name,delayed(original,axis))
        began=hw.clock();hw.gripper_target(token,.201,.2)
        wait_until(lambda:hw.gripper_state()['arrived'] or hw.phase!='ACTIVE',3.4)
        assert hw.phase=='ACTIVE',hw.error
        assert hw.gripper_state()['arrived'],hw.gripper_state()
        assert hw.clock()-began<3.4
        for motor in sdk.motors.values():
            name='robstride_write_param_f32';original=getattr(motor,name)
            originals.append((motor,name,original))
            def delayed_write(*args,write=original):
                time.sleep(.001);return write(*args)
            setattr(motor,name,delayed_write)
        began=hw.clock();position=hw.gripper.sample.q
        while hw.clock()-began<2:
            assert hw.phase=='ACTIVE',hw.error
            hw.gripper_jog(token,.1);time.sleep(.02)
        hw.gripper_jog(token,0)
        assert hw.gripper.sample.q>position+.02
        assert hw.phase=='ACTIVE' and hw.clock()-hw._feedback_refreshed_at<.6,hw.error
        assert len(j7_positions)>10 and max(b-a for a,b in zip(j7_positions,j7_positions[1:]))<.5
        assert len(factories)==1 and threads=={'csp-can-owner'}
    finally:
        for motor,name,original in originals:setattr(motor,name,original)
        done.set()
        if beat:beat.join()
        hw.close()


def test_explicit_validated_hold_resume_clears_previous_error_without_replaying_input():
    g,clock,sent=inert_gripper()
    g.error='通信异常暂停，等待重新接管'
    g.manual=dict(at=clock[0],velocity=.1)
    g.pause();frozen=g.reference
    clock[0]+=.1
    g.resume_hold()
    assert not g.error
    assert g.reference==frozen and g.history==[(clock[0],frozen)]
    assert g.manual is None and g.pending is None and g.curve is None and not sent
    g.reference=None;g.error='启动未完成'
    with pytest.raises(RuntimeError,match='启动未完成'):g.resume_hold()
    assert g.error=='启动未完成' and not sent
