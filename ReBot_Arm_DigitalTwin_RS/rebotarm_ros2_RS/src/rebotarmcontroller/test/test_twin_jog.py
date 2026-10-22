"""Online smoothing and stop precedence; inert transport, no CAN."""
import math
import json
import time
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from rebotarmcontroller.twin_jog import JogRamp
from rebotarmcontroller.twin_control import TwinControl


def test_ramp_start_reverse_and_neutral_obey_speed_and_acceleration():
    ramp=JogRamp();q=np.zeros(6);previous=np.zeros(6)
    for desired in ([.08,-.04,0,0,0,0],[-.08,.04,0,0,0,0],[0]*6):
        velocities=[]
        for _ in range(45):
            q=ramp.step(q,desired,.05,.1,.08,[-1]*6,[1]*6)
            assert np.max(abs(ramp.velocity)) <= .08+1e-12
            assert np.max(abs(ramp.velocity-previous)) <= .08*.05+1e-12
            previous=ramp.velocity.copy();velocities.append(previous)
        assert np.allclose(velocities[-1],desired,atol=1e-12)
    assert np.max(abs(ramp.velocity)) < 1e-12
    assert np.allclose(ramp.step(q,[0]*6,.05,.1,.08,[-1]*6,[1]*6),q)


@pytest.mark.parametrize('sign',[-1,1])
def test_boundary_braking_never_crosses_nominal_limit(sign):
    ramp=JogRamp();q=np.full(6,sign*.98);previous_speed=0.
    for _ in range(120):
        q=ramp.step(q,np.full(6,sign*.08),.05,.1,.08,[-1]*6,[1]*6)
        assert np.all(q>=-1) and np.all(q<=1)
        if abs(q[0])>.995: assert abs(ramp.velocity[0]) <= previous_speed+1e-12
        previous_speed=abs(ramp.velocity[0])
    assert abs(ramp.velocity[0]) < 1e-7


def test_delay_speed_reduction_and_reset_never_replay_old_motion():
    ramp=JogRamp();q=np.zeros(6)
    for _ in range(25):q=ramp.step(q,[1]*6,.05,.1,.08,[-1]*6,[1]*6)
    next_q=ramp.step(q,[1]*6,2.,.1,.08,[-1]*6,[1]*6)
    assert np.max(abs(next_q-q)) <= .004+1e-12
    next_q=ramp.step(q,[1]*6,.05,.02,.08,[-1]*6,[1]*6)
    assert np.max(abs(next_q-q)) <= .0008+1e-12
    ramp.reset()
    assert np.array_equal(ramp.step(q,[0]*6,.05,.1,.08,[-1]*6,[1]*6),q)


def console():
    clock=[10.];writes=[];pauses=[];stops=[]
    class Publisher:
        def publish(self,msg): pass
    node=SimpleNamespace(create_publisher=lambda *a,**k:Publisher(),
        create_service=lambda *a,**k:None,create_subscription=lambda *a,**k:None,
        create_timer=lambda *a,**k:None,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=Time)),
        get_logger=lambda:SimpleNamespace(warn=lambda text:None))
    actual=np.zeros(6)
    def send(token,seq,stamp,q,gripper,speed):
        actual[:]=q;writes.append(actual.copy())
    hw=SimpleNamespace(min_twin_speed=.005,max_twin_speed=.1,integrate_jog_reference=True,
        motion_profile='quintic',
        command_reference=tuple(actual),motion_ready=True,phase='ACTIVE',teleop_owned=True,
        config=SimpleNamespace(axes=[SimpleNamespace(a_max=.08)]*6),
        control_settings=lambda:dict(mode='full'),
        get_joint_state=lambda **kw:(actual.copy(),[0]*6,[0]*6),
        teleop_resume=lambda token:None,teleop_send=send,
        teleop_pause=lambda token:pauses.append(clock[0]),
        teleop_stop=lambda token,reason:stops.append(reason),disable=lambda:None,
        teleop_watchdog=lambda:None)
    kin=SimpleNamespace(lower=np.full(6,-1.),upper=np.full(6,1.),pose=lambda q:list(q),
        jog=lambda q,twist,lookahead,max_step:(np.asarray(q)+np.clip(twist,-max_step/lookahead,max_step/lookahead)*lookahead).tolist())
    twin=TwinControl(node,hw,'inert',clock=lambda:clock[0],kinematics=kin)
    twin.session='lease';twin.mode='HOLD';twin.last_heartbeat=clock[0]
    def jog(value):
        clock[0]+=.05
        twin.last_heartbeat=clock[0]
        twin._jog(dict(twist=[value,0,0,0,0,0],speed=.1))
    return twin,hw,clock,writes,pauses,stops,jog


def test_neutral_brakes_to_hold_but_neutral_alone_cannot_start():
    twin,hw,clock,writes,pauses,stops,jog=console()
    jog(0);assert twin.mode=='HOLD' and not writes
    for _ in range(22):jog(.04)
    previous=twin.jog_ramp.velocity[0];position=writes[-1][0]
    for _ in range(22):
        jog(0)
        assert 0<=twin.jog_ramp.velocity[0]<=previous+1e-12
        previous=twin.jog_ramp.velocity[0]
    assert twin.mode=='HOLD' and pauses and writes[-1][0]>position
    assert twin.reference is None and not np.any(twin.jog_ramp.velocity)
    count=len(writes);jog(0);assert len(writes)==count


@pytest.mark.parametrize('action',['hold','stop','timeout'])
def test_explicit_stop_and_input_timeout_discard_ramp_immediately(action):
    twin,hw,clock,writes,pauses,stops,jog=console()
    for _ in range(15):jog(.04)
    assert np.any(twin.jog_ramp.velocity)
    count=len(writes)
    if action=='hold':twin._hold('deadman released')
    elif action=='stop':twin.stop(None,SimpleNamespace())
    else:
        clock[0]+=.251;twin.last_heartbeat=clock[0];twin.tick()
    assert not np.any(twin.jog_ramp.velocity)
    assert len(writes)==count and twin.reference is None
    assert twin.mode==('IDLE' if action=='stop' else 'HOLD')
    if action!='stop':
        jog(.04)
        assert math.isclose(twin.jog_ramp.velocity[0],.004,abs_tol=1e-12)


def test_late_packet_restarts_from_zero_instead_of_resuming_old_velocity():
    twin,hw,clock,writes,pauses,stops,jog=console()
    for _ in range(15):jog(.04)
    clock[0]+=.3;jog(.04)
    assert math.isclose(twin.jog_ramp.velocity[0],.004,abs_tol=1e-12)


def test_neutral_during_csp_preparation_cancels_without_any_target():
    twin,hw,clock,writes,pauses,stops,jog=console()
    hw.motion_ready=False
    jog(.04);assert twin.mode=='JOG' and not writes
    jog(0);assert twin.mode=='HOLD' and pauses and not writes


def command_console():
    from std_msgs.msg import String
    twin, hw, clock, writes, pauses, stops, jog = console()
    hw._feedback_refreshed_at = clock[0]
    hw.feedback_timeout = .5
    hw.teleop_heartbeat = lambda token: None
    events = []
    twin.events.publish = lambda message: events.append(json.loads(message.data))
    def send(seq, op='jog', **args):
        twin.command(String(data=json.dumps({
            'op': op, 'session': 'lease', 'seq': seq,
            'stamp_ms': time.time() * 1000, 'twist': [.04, 0, 0, 0, 0, 0], 'speed': .1, **args,
        })))
    return twin, hw, clock, writes, pauses, stops, events, send


def test_command_jog_coalesces_bursts_and_dispatches_latest_sample_once():
    """Reliable ROS delivery must not replay stale manual targets."""
    twin, hw, clock, writes, pauses, stops, events, send = command_console()
    send(1)
    send(2, twist=[-.04, 0, 0, 0, 0, 0])
    assert not writes and twin.mode == 'HOLD'
    assert all(event['ok'] for event in events)
    assert twin._pending_jog['seq'] == 2 and twin._jog_dropped == 1
    clock[0] += .02
    twin.tick()
    assert len(writes) == 1 and writes[0][0] < 0
    assert twin._pending_jog is None
    clock[0] += .02
    twin.tick()
    assert len(writes) == 1  # No sample replay on later dispatcher ticks.


@pytest.mark.parametrize('action', ['hold', 'release', 'stop', 'suspend', 'reset'])
def test_stop_actions_discard_pending_jog_before_dispatch(action):
    twin, hw, clock, writes, pauses, stops, events, send = command_console()
    hw.suspend = lambda reason: True
    send(1)
    if action in ('hold', 'release'):
        send(2, action)
    else:
        getattr(twin, action)(None, SimpleNamespace())
    assert twin._pending_jog is None
    clock[0] += .02
    twin.tick()
    assert not writes


@pytest.mark.parametrize('args', [dict(twist=[math.nan]*6), dict(twist=[1]*6), dict(speed=1)])
def test_invalid_jog_rejected_synchronously_and_cannot_leave_old_input_pending(args):
    twin, hw, clock, writes, pauses, stops, events, send = command_console()
    send(1)
    send(2, **args)
    assert not events[-1]['ok'] and twin._pending_jog is None
    clock[0] += .02
    twin.tick()
    assert not writes


def test_queued_neutral_cancels_startup_without_a_motion_target():
    twin, hw, clock, writes, pauses, stops, events, send = command_console()
    hw.motion_ready = False
    send(1)
    clock[0] += .02;twin.tick()
    assert twin.mode == 'JOG' and not writes
    send(2)
    send(3, twist=[0]*6)
    clock[0] += .02;twin.tick()
    assert twin.mode == 'HOLD' and pauses and not writes


def test_expired_pending_input_is_not_revived_by_a_heartbeat():
    twin, hw, clock, writes, pauses, stops, events, send = command_console()
    send(1)
    clock[0] += .251
    send(2, 'heartbeat')
    twin.tick()
    assert twin.mode == 'HOLD' and twin._pending_jog is None and not writes
    assert '过期' in twin.message


def test_dispatch_delay_does_not_extend_input_lifetime_or_hardware_sample_time():
    twin, hw, clock, writes, pauses, stops, events, send = command_console()
    stamps = []
    hw.teleop_stream = lambda token, seq, stamp, q, velocity, speed: stamps.append(stamp)
    received_at = clock[0]
    send(1)
    clock[0] += .1;twin.tick()
    assert stamps == [received_at]
    assert twin.mode == 'JOG'
    clock[0] += .151
    send(2, 'heartbeat');twin.tick()
    assert twin.mode == 'HOLD' and len(stamps) == 1


def test_explicit_task_handoff_discards_pending_manual_input():
    twin, hw, clock, writes, pauses, stops, events, send = command_console()
    hw.phase = 'READONLY'
    send(1)
    send(2, 'power_hold')
    assert events[-1]['ok'] and twin._pending_jog is None
    clock[0] += .02;twin.tick()
    assert not writes and twin.mode == 'HOLD'


def test_pending_manual_input_blocks_expensive_preview():
    twin, hw, clock, writes, pauses, stops, events, send = command_console()
    send(1)
    send(2, 'preview', pose=[0]*6)
    assert not events[-1]['ok'] and '暂停' in events[-1]['message']
    assert twin._pending_jog is None and not writes
