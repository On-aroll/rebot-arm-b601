"""500 ms command boundary and continuous references; no physical CAN."""
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Envelope
from rebotarmcontroller.csp_runtime.rebot_native_xyz import (
    AXES, MODE, PATH_MODE, FULL_MODE, SMOOTH_FULL_MODE, OFFICIAL_FULL_MODE,
    LIVE_FULL_MODE, NativeXYZExecutor, NativeXYZTransport, proposal,
)
from test_csp_twin import Controller, Owner
from test_twin_jog import console


def test_500ms_profile_preserves_historical_envelopes_and_all_other_limits():
    fingerprints = {
        MODE: '7fa4a9f2f859aa8c854637b0d74c36648b0117086d1e11dbe7d44e3d175bdbde',
        PATH_MODE: 'a9b2eb6aead64588d8db90d349a55479904ea0dd685705d71dc7f96ace256337',
        FULL_MODE: 'c58bb5469d2e80100b43f2c4b80a2e2ba76535a1ba608958a9e701d07383a26b',
        SMOOTH_FULL_MODE: '349aa74a9171c649a2f2548ae24c4e0914fb5bb4dfd13e488f8efdeefa6d5c10',
        OFFICIAL_FULL_MODE: '1b337f11d28cfa062109cf48bc54e37da9f98f604ee15a0fc38c293fc7e7a19e',
    }
    for mode, fingerprint in fingerprints.items():
        assert Envelope.parse(proposal([(-1,1)]*6,mode=mode)[0]).fingerprint == fingerprint
    old = proposal([(-1,1)]*6,mode=OFFICIAL_FULL_MODE)[0]
    current = proposal([(-1,1)]*6,mode=LIVE_FULL_MODE)[0]
    assert current['policy']['command_age'] == .5
    current['version'] = old['version'];current['policy']['command_age'] = .3
    assert current == old


def test_actual_executor_accepts_490ms_but_rejects_510ms_command_gap():
    clock = [100.]
    config = Envelope.parse(proposal([(-1,1)]*6,mode=LIVE_FULL_MODE)[0])
    executor = NativeXYZExecutor(config,SimpleNamespace(),fk=lambda q:[0,0,0],clock=lambda:clock[0])
    executor.heartbeat(executor.session,0,True)
    executor.last_axis_command = dict.fromkeys(AXES,100.)
    permit = SimpleNamespace(expires_at=102.)
    clock[0] = 100.49
    executor.check(permit)
    clock[0] = 100.51
    with pytest.raises(TimeoutError,match='命令有效期'):executor.check(permit)


@pytest.mark.parametrize('sign',[-1,1])
def test_manual_reference_continues_between_packets_then_expires_without_catchup(sign):
    hw = CspHardware([(-1,1)]*6,clock=lambda:100.)
    hw.phase = 'READONLY'  # Inert initialized state; never open a transport.
    hw.set_control_mode('full')
    hw.paused = False;hw.last_sample = 100.;hw.speed = .1
    hw.target = np.zeros(6);hw.stream_velocity = np.array([sign*.1,0,0,0,0,0])
    held = np.zeros(6);previous_velocity = 0.
    for n in range(1,13):
        wanted = hw._desired_reference(100.+n*.02,held,.02)
        velocity = (wanted[0]-held[0])/.02
        # Includes the >50 ms packet gap which used to flatten the reference.
        assert sign*(wanted[0]-held[0]) > 0
        assert abs(velocity-previous_velocity) <= 4*.02+1e-12
        assert abs(velocity) <= .1+1e-12
        held = wanted;previous_velocity = velocity
    assert np.array_equal(hw._desired_reference(100.251,held,.02),held)
    assert np.array_equal(hw._desired_reference(102.,held,1.75),held)
    assert not np.any(hw.stream_ramp.velocity)
    # A new packet cannot jump to its stale packet-integrated target.
    hw.last_sample = 102.;hw.target = np.full(6,-sign*.5)
    wanted = hw._desired_reference(102.02,held,.02)
    assert 0 < sign*(wanted[0]-held[0]) <= .1*.02
    hw.paused = True
    assert np.array_equal(hw._desired_reference(102.04,wanted,.02),wanted)


def test_stream_dispatch_does_not_renew_an_older_input_sample():
    clock = [100.]
    hw = CspHardware([(-1,1)]*6,clock=lambda:clock[0])
    hw.phase = 'READONLY';hw.set_control_mode('full')
    hw.session = 'inert';hw.origin = np.zeros(6);hw.paused = False
    clock[0] = 100.2
    hw.teleop_stream('inert',1,100.,[0]*6,[.1,0,0,0,0,0],.1)
    assert hw.last_sample == 100.
    held = hw._desired_reference(100.22,np.zeros(6),.02)
    assert held[0] > 0
    assert np.array_equal(hw._desired_reference(100.251,held,.02),held)


def test_manual_reference_respects_joint_boundary_during_packet_delay():
    hw = CspHardware([(-1,1)]*6,clock=lambda:100.)
    hw.phase = 'READONLY'
    hw.set_control_mode('full');hw.paused=False;hw.last_sample=100.;hw.speed=1.
    hw.target=np.full(6,.999);hw.stream_velocity=np.ones(6)
    held=hw.target.copy()
    for n in range(1,13):
        held=hw._desired_reference(100.+n*.02,held,.02)
        assert np.all(held <= 1.) and np.all(held >= .999)


def test_jog_ik_uses_live_can_reference_instead_of_previous_packet_endpoint():
    twin,hw,clock,writes,pauses,stops,jog = console()
    jog(.04)
    twin.reference=[.8]*6
    hw.continuous_jog_reference=True;hw.command_reference=[.2]*6
    seeds=[];original=twin.kin.jog
    def observed(seed,*args,**kwargs):
        seeds.append(list(seed));return original(seed,*args,**kwargs)
    twin.kin.jog=observed
    jog(.04)
    assert seeds == [[.2]*6]


def test_diagnostics_yield_after_every_reply_without_extra_queries_or_writes():
    clock=[100.];sdk=Controller();queries=[];yielded=[]
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner(),clock=lambda:clock[0])
    bus.open_readonly()
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:clock[0])
    try:
        for i,motor in sdk.motors.items():
            f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
            def read_f32(index,*args,read=f32,axis=i):
                clock[0]+=.008;queries.append((axis,index));return read(index,*args)
            def read_i8(index,*args,read=i8,axis=i):
                clock[0]+=.008;queries.append((axis,index));return read(index,*args)
            motor.robstride_get_param_f32_host_id=read_f32
            motor.robstride_get_param_i8=read_i8
        hw._read();queries.clear();began=clock[0]
        bus.read_cycle(hw.config.policy,(1,2),on_io=lambda:yielded.append(clock[0]))
        assert len(queries)==len(yielded)==10
        assert max(np.diff([began,*yielded])) == pytest.approx(.008)
        assert sdk.calls == []
    finally:bus.close()


def test_interleaved_sweep_counts_fields_refreshed_inside_io_callback():
    clock=[100.];sdk=Controller();queries=[]
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner(),clock=lambda:clock[0])
    bus.open_readonly()
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:clock[0])
    try:
        for i,motor in sdk.motors.items():
            f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
            def read_f32(index,*args,read=f32,axis=i):
                clock[0]+=.008;queries.append((axis,index));return read(index,*args)
            def read_i8(index,*args,read=i8,axis=i):
                clock[0]+=.008;queries.append((axis,index));return read(index,*args)
            motor.robstride_get_param_f32_host_id=read_f32;motor.robstride_get_param_i8=read_i8
        hw._read();queries.clear();serviced=[False]
        def service_other_io():
            if not serviced[0]:
                serviced[0]=True
                bus.read_cycle(hw.config.policy,tuple(range(1,7)),interleaved=True,max_fields=3)
        samples=bus.read_cycle(hw.config.policy,tuple(range(1,7)),interleaved=True,on_io=service_other_io)
        assert len(queries)==len(set(queries))==18
        assert all(clock[0]-s.requested_at<.2 for s in samples.values())
        assert sdk.calls==[]
    finally:bus.close()


def test_j7_read_groups_do_not_starve_oldest_arm_fields_under_host_overhead():
    clock=[100.];sdk=Controller();last_j7=[100.];j7_polls=[]
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner(),clock=lambda:clock[0])
    bus.open_readonly()
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:clock[0])
    hw.phase='READONLY';hw.set_control_mode('full')
    executor=NativeXYZExecutor(hw.config,bus,fk=lambda q:[0,0,0],clock=lambda:clock[0])
    try:
        for motor in sdk.motors.values():
            f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
            def read_f32(*args,read=f32):clock[0]+=.012;return read(*args)
            def read_i8(*args,read=i8):clock[0]+=.012;return read(*args)
            motor.robstride_get_param_f32_host_id=read_f32;motor.robstride_get_param_i8=read_i8
        hw._read();hw.phase='ACTIVE';last_j7[0]=clock[0]
        def validate():
            clock[0]+=.006
            executor.validate(dict(hw.samples))
        def gripper_step(check,*,poll=True,one_field=False):
            if not poll:return
            if clock[0]-last_j7[0]<.09:return
            assert clock[0]-last_j7[0]<.5,'J7 command deadline'
            last_j7[0]=clock[0];j7_polls.append(clock[0])
            hw.gripper.sample.requested_at=clock[0]
            for _ in range(3):clock[0]+=.020;check()
        hw.gripper=SimpleNamespace(view=SimpleNamespace(active={7}),reading=True,
                                  preparing=False,step=gripper_step,sample=SimpleNamespace(requested_at=clock[0]),
                                  request_probe=False,pending=None,pending_grasp=None,manual=None)
        hw.gripper.feedback_time=lambda:hw.gripper.sample.requested_at
        for _ in range(16):hw._read_cycle(on_io=validate)
        assert len(j7_polls)>10 and sdk.calls==[]
        assert hw.config.policy.feedback_age==.6
    finally:bus.close()


@pytest.mark.parametrize('stall_at',range(18))
def test_seven_axis_poll_recovers_one_300ms_host_gap_by_oldest_sample(stall_at):
    clock=[100.];sdk=Controller();polls=[];gap=[False];callbacks=[0];last_j7=[100.]
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner(),clock=lambda:clock[0])
    bus.open_readonly()
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:clock[0])
    hw.phase='READONLY';hw.set_control_mode('full')
    executor=NativeXYZExecutor(hw.config,bus,fk=lambda q:[0,0,0],clock=lambda:clock[0])
    try:
        for motor in sdk.motors.values():
            f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
            def read_f32(*args,read=f32):clock[0]+=.012;return read(*args)
            def read_i8(*args,read=i8):clock[0]+=.012;return read(*args)
            motor.robstride_get_param_f32_host_id=read_f32;motor.robstride_get_param_i8=read_i8
        hw._read();hw.phase='ACTIVE';last_j7[0]=clock[0]
        def validate():
            clock[0]+=.002
            if gap[0]:
                if callbacks[0]==stall_at:clock[0]+=.3;gap[0]=False
                callbacks[0]+=1
            executor.validate(dict(hw.samples))
        def gripper_step(check,*,poll=True,one_field=False):
            assert clock[0]-last_j7[0]<.6,'J7 feedback expired'
            if not poll or clock[0]-last_j7[0]<.18:return
            last_j7[0]=clock[0];polls.append(clock[0])
            hw.gripper.sample.requested_at=clock[0]
            for _ in range(3):clock[0]+=.008;check()
        hw.gripper=SimpleNamespace(view=SimpleNamespace(active={7}),reading=True,preparing=False,
                                  step=gripper_step,sample=SimpleNamespace(requested_at=clock[0]),
                                  request_probe=False,pending=None,pending_grasp=None,manual=None)
        hw.gripper.feedback_time=lambda:hw.gripper.sample.requested_at
        for _ in range(8):hw._read_cycle(on_io=validate)
        gap[0]=True
        for _ in range(8):hw._read_cycle(on_io=validate)
        assert not gap[0] and len(polls)>8
        assert hw.config.policy.feedback_age==.6 and sdk.calls==[]
    finally:bus.close()


@pytest.mark.parametrize('slow_query',range(18))
def test_active_poll_keeps_all_fields_fresh_across_one_330ms_sdk_reply(slow_query):
    """Replay the observed ~300 ms query stall at every place in a full sweep."""
    clock=[100.];sdk=Controller();queries=[];slow=[None]
    bus=NativeXYZTransport(factory=lambda **kwargs:sdk,ownership=Owner(),clock=lambda:clock[0])
    bus.open_readonly()
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:clock[0])
    hw.phase='READONLY'
    hw.set_control_mode('full')
    executor=NativeXYZExecutor(hw.config,bus,fk=lambda q:[0,0,0],clock=lambda:clock[0])
    try:
        def elapsed(axis,index):
            queries.append((axis,index))
            clock[0]+=.012  # Includes normal query and host processing cost.
            if slow[0] is not None:
                if slow[0]==0:clock[0]+=.330;slow[0]=None
                else:slow[0]-=1
        for i,motor in sdk.motors.items():
            f32,i8=motor.robstride_get_param_f32_host_id,motor.robstride_get_param_i8
            def read_f32(index,*args,read=f32,axis=i):
                elapsed(axis,index);return read(index,*args)
            def read_i8(index,*args,read=i8,axis=i):
                elapsed(axis,index);return read(index,*args)
            motor.robstride_get_param_f32_host_id=read_f32
            motor.robstride_get_param_i8=read_i8
        hw._read();hw.phase='ACTIVE'
        def validate():executor.validate(dict(hw.samples))  # No commands or enables.
        for cycle in range(12):
            if cycle==6:slow[0]=slow_query
            hw._read_cycle(on_io=validate)
        assert hw.config.policy.feedback_age==.6
        assert slow[0] is None and sdk.calls==[]
    finally:bus.close()


def test_stream_path_model_does_not_block_can_state_lock():
    import threading
    hw=CspHardware([(-1,1)]*6,clock=lambda:100.)
    hw.phase='READONLY';hw.set_control_mode('full');hw.session='inert';hw.origin=np.zeros(6)
    entered=threading.Event();workers=[]
    def inspect(start,target):
        def can_state():
            with hw.lock:entered.set()
        worker=threading.Thread(target=can_state);workers.append(worker);worker.start()
        assert entered.wait(.15),'Model checking held the CAN state lock'
    hw.path_check=inspect
    try:hw.teleop_stream('inert',1,100.,[.001]*6,[.01]*6,.1)
    finally:
        for worker in workers:worker.join(1)
    assert np.all(hw.target==.001) and hw.last_sample==100.


@pytest.mark.parametrize('change',['expired','revoked'])
def test_stream_revalidates_input_after_slow_model_before_committing(change):
    clock=[100.]
    hw=CspHardware([(-1,1)]*6,clock=lambda:clock[0])
    hw.phase='READONLY';hw.set_control_mode('full');hw.session='inert';hw.origin=np.zeros(6)
    def inspect(start,target):
        if change=='expired':clock[0]+=.251
        else:hw.session=''
    hw.path_check=inspect
    with pytest.raises((ValueError,RuntimeError),match='过期|租约'):
        hw.teleop_stream('inert',1,100.,[.001]*6,[.01]*6,.1)
    assert hw.target is None and hw.stream_velocity is None and hw.last_seq==-1
