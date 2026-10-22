"""Actual Ruckig references and their integration; no SDK or physical CAN."""
import json
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.twin_motion import RuckigVelocity, point_duration, point_sample
from test_twin_jog import console
from test_csp_twin import fixture, wait_until


def test_velocity_start_reverse_and_brake_obey_v_a_and_jerk():
    ramp = RuckigVelocity();q = np.zeros(6);old_a = np.zeros(6)
    for target in ([.3,-.15,0,0,0,0],[-.3,.15,0,0,0,0],[0]*6):
        for _ in range(130):
            old_q = q.copy()
            q = ramp.step(q,target,.02,.3,1.,[-5]*6,[5]*6,jerk=4.)
            assert np.max(abs(q-old_q)) <= .3*.02+1e-10
            assert np.max(abs(ramp.velocity)) <= .3+1e-10
            assert np.max(abs(ramp.acceleration)) <= 1.+1e-10
            assert np.max(abs(ramp.acceleration-old_a)) <= 4.*.02+1e-10
            old_a = ramp.acceleration.copy()
        assert np.allclose(ramp.velocity,target,atol=1e-9)
        assert np.max(abs(ramp.acceleration)) < 1e-9


@pytest.mark.parametrize('sign',[-1,1])
def test_nonzero_acceleration_stopping_envelope_never_crosses_limit(sign):
    ramp = RuckigVelocity();q = np.full(6,sign*.8);braked = False
    for _ in range(250):
        q = ramp.step(q,np.full(6,sign),.02,1.,4.,[-1]*6,[1]*6,jerk=30.)
        braked |= ramp.boundary_braking
        assert np.all(q <= 1.) and np.all(q >= -1.)
    assert braked and np.max(abs(ramp.velocity)) < 1e-9
    assert .95 < sign*q[0] <= 1.


def test_stall_has_only_one_capped_step_and_reset_discards_old_acceleration():
    ramp = RuckigVelocity();q = np.zeros(6)
    for _ in range(50):q = ramp.step(q,[.3]*6,.02,.3,1.,[-5]*6,[5]*6)
    before = q.copy()
    q = ramp.step(q,[.3]*6,2.,.3,1.,[-5]*6,[5]*6)
    assert np.max(abs(q-before)) <= .3*.05+1e-10
    ramp.reset()
    assert np.array_equal(ramp.step(q,[0]*6,.02,.3,1.,[-5]*6,[5]*6),q)
    assert not np.any(ramp.velocity) and not np.any(ramp.acceleration)


def test_point_curve_preserves_checked_line_and_limits_with_rest_endpoints():
    start = np.array([.2,-.1,0,.5,-.3,.1]);goal = start+np.array([.4,-.2,0,-.1,.05,.3])
    duration = point_duration(start,goal,.2,.5,2.,1.)
    dt = .001;times = np.arange(0,duration+dt,dt)
    q = np.array([point_sample('ruckig',start,goal,.2,.5,2.,duration,t) for t in times])
    blend = (q[:,0]-start[0])/.4
    assert np.allclose(q,start+(goal-start)*blend[:,None])
    assert np.all(blend >= -1e-12) and np.all(blend <= 1.+1e-12)
    v = np.diff(q,axis=0)/dt;a = np.diff(v,axis=0)/dt;j = np.diff(a,axis=0)/dt
    assert np.max(abs(v)) <= .2+1e-7
    assert np.max(abs(a)) <= .5+1e-6
    assert np.max(abs(j)) <= 2.+1e-5
    assert np.max(abs(v[[0,-1]])) < 1e-5
    assert np.allclose(q[-1],goal)


@pytest.mark.parametrize('action',['hold','stop','timeout'])
def test_ruckig_console_stop_clears_velocity_and_acceleration_immediately(action):
    twin,hw,clock,writes,pauses,stops,jog = console();hw.motion_profile='ruckig'
    for _ in range(15):jog(.04)
    assert np.any(twin.ruckig_jog.velocity)
    count = len(writes)
    if action == 'hold':twin._hold('deadman released')
    elif action == 'stop':twin.stop(None,SimpleNamespace())
    else:clock[0]+=.251;twin.last_heartbeat=clock[0];twin.tick()
    assert not np.any(twin.ruckig_jog.velocity) and not np.any(twin.ruckig_jog.acceleration)
    assert len(writes) == count and twin.reference is None


def test_console_neutral_brakes_before_hold_and_can_smoothing_is_not_duplicated():
    twin,hw,clock,writes,pauses,stops,jog = console();hw.motion_profile='ruckig'
    for _ in range(15):jog(.04)
    before = writes[-1][0]
    for _ in range(30):jog(0)
    assert twin.mode == 'HOLD' and pauses and writes[-1][0] > before
    hw.continuous_jog_reference=True;hw.stream_ruckig=RuckigVelocity()
    sent=[];hw.teleop_stream=lambda token,seq,at,q,v,speed:sent.append(v)
    jog(.04)
    assert sent[-1][0] == pytest.approx(.04)
    assert not np.any(twin.ruckig_jog.velocity) # smoothing runs on the CAN owner


def test_can_point_refresh_keeps_virtual_time_and_pause_clears_stream():
    clock = [100.];hw = CspHardware([(-1,1)]*6,clock=lambda:clock[0])
    hw.phase='READONLY';hw.set_control_mode('full')
    hw.session='lease';hw.origin=np.zeros(6);hw.paused=False
    goal = dict(start=[0]*6,joints=[.1]*6,duration=2.,began=100.)
    held = np.zeros(6)
    for seq in range(10):
        hw.teleop_point('lease',seq,clock[0],goal,.1)
        clock[0]+=.02
        held = hw._desired_reference(clock[0],held,.02)
    assert hw.point_reference['elapsed'] == pytest.approx(.2)
    expected = point_sample('ruckig',[0]*6,[.1]*6,.1,4.,30.,2.,.2)
    assert np.allclose(held,expected)
    clock[0]+=.2;hw.teleop_point('lease',10,clock[0],goal,.1)
    held = hw._desired_reference(clock[0],held,.2)
    assert hw.point_reference['elapsed'] == pytest.approx(.25)
    hw.stream_ruckig.velocity[:]=.1;hw.stream_ruckig.acceleration[:]=1.
    hw.teleop_pause('lease')
    assert hw.point_reference is None and not np.any(hw.stream_ruckig.velocity)
    assert not np.any(hw.stream_ruckig.acceleration)


def test_profile_switch_only_when_readonly_clears_preview_and_never_writes():
    twin,hw,clock,writes,pauses,stops,jog = console()
    events=[];twin._event=lambda req,ok,msg,**extra:events.append(dict(ok=ok,**extra))
    twin.command(SimpleNamespace(data=json.dumps(dict(op='motion_profile',profile='ruckig'))))
    assert not events[-1]['ok']
    twin._release('offline fixture');hw.teleop_owned=False
    del hw.motion_profile # Fake hardware selection belongs to TwinControl
    twin.plan={'stale':True}
    twin.command(SimpleNamespace(data=json.dumps(dict(op='motion_profile',profile='ruckig'))))
    assert events[-1]['ok'] and twin.plan is None and not writes
    assert twin._motion_profile() == 'ruckig'
    real = CspHardware([(-1,1)]*6);real.phase='READONLY'
    real.set_motion_profile('quintic');assert real.motion_profile == 'quintic'
    real.armed=True
    with pytest.raises(RuntimeError):real.set_motion_profile('ruckig')
    assert real.motion_profile == 'quintic'


def test_slow_reference_computation_skips_missed_slots_without_burst_or_false_speed_trip(fixture,monkeypatch):
    hw,sdk=fixture
    hw.set_control_mode('full');hw.prepare();token=hw.teleop_acquire()
    done=threading.Event();failures=[]
    goal=dict(start=[0]*6,joints=[.1]*6,duration=2.,began=time.monotonic())
    def keepalive():
        seq=0
        while not done.wait(.03):
            try:
                hw.teleop_heartbeat(token)
                if hw.motion_ready:
                    seq+=1;hw.teleop_point(token,seq,time.monotonic(),goal,.1)
            except Exception as exc:failures.append(str(exc));return
    original=hw._desired_reference;calls=[0];stalled=threading.Event()
    def delayed(now,held,dt=None):
        calls[0]+=1
        if calls[0]==8:time.sleep(.08);stalled.set()
        return original(now,held,dt)
    monkeypatch.setattr(hw,'_desired_reference',delayed)
    beat=threading.Thread(target=keepalive);beat.start()
    try:
        hw.teleop_resume(token);wait_until(lambda:hw.motion_ready)
        wait_until(stalled.is_set);time.sleep(.3)
        assert hw.phase=='ACTIVE' and not failures,hw.error
        frames=[f for f in hw.executor.frames if f['phase']=='trajectory']
        gaps=np.diff([f['time'] for f in frames])
        assert max(gaps)>.07
        assert min(gaps)>.005 # no compressed catch-up group after the calculation stall
        for old,new in zip(frames,frames[1:]):
            interval=new['time']-old['time']
            assert max(abs(a['target_rad']-b['target_rad']) for a,b in zip(new['commands'],old['commands']))<=.1*min(.05,interval)+1e-6
    finally:
        done.set();beat.join();hw.teleop_stop(token);wait_until(lambda:hw.phase=='READONLY',5)
