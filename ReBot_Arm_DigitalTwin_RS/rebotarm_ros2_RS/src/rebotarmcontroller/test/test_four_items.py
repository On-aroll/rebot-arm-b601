"""Offline checks only: no node startup, CAN connection or drive writes."""
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from test_twin_jog import console
from test_csp_twin import Controller, Owner
from rebotarmcontroller.csp_hardware import CspHardware
from rebotarmcontroller.csp_runtime.rebot_native_xyz import NativeXYZTransport
from rebotarmcontroller.csp_twin_driver import CheckedKinematics


def test_state_is_20hz_but_watchdogs_still_run_50hz():
    twin,hw,clock,*_ = console()
    twin.session=''
    published=[];checks=[]
    twin._publish_state=lambda:published.append(clock[0])
    hw.teleop_watchdog=lambda:checks.append(clock[0])
    for i in range(51):
        clock[0]=10+i*.02;twin.tick()
    assert len(checks)==51
    assert 20 <= len(published) <= 21
    # A stalled status publisher must not catch up with a burst.
    clock[0]+=1.;twin.tick();count=len(published);twin.tick()
    assert len(published)==count


def test_path_checks_do_not_wait_for_another_threads_scratch_data():
    kin=CheckedKinematics()
    entered=threading.Event();release=threading.Event();done=threading.Event()
    original=kin._check_path;seen=[];failures=[]
    def check(start,goal,gravity_limits):
        seen.append((id(kin.path_data.gravity),id(kin.path_data.collision)))
        if threading.current_thread().name=='preview':
            entered.set();assert release.wait(3)
        else:done.set()
        original(start,goal,gravity_limits)
    kin._check_path=check
    def run():
        try:kin.check_path(np.zeros(6),np.zeros(6),gravity_limits=np.full(6,100.))
        except Exception as exc:failures.append(exc)
    preview=threading.Thread(target=run,name='preview');preview.start()
    assert entered.wait(3)
    owner=threading.Thread(target=run,name='can-owner');owner.start()
    try:assert done.wait(2), 'CAN path check blocked behind preview'
    finally:release.set();preview.join(3);owner.join(3)
    assert not failures
    assert len(seen)==2 and all(a!=b for a,b in zip(*seen))
    kin._check_path=original
    with pytest.raises(ValueError,match='模型重力'):
        kin.check_path(np.zeros(6),np.zeros(6),gravity_limits=np.zeros(6))


def test_diagnostics_report_actual_snapshots_without_queries_or_writes():
    clock=[100.];sdk=Controller()
    bus=NativeXYZTransport(factory=lambda **kw:sdk,ownership=Owner(),clock=lambda:clock[0])
    hw=CspHardware([(-1,1)]*6,transport=bus,clock=lambda:clock[0])
    hw.phase='READONLY';hw.set_control_mode('full')
    # Populate memory only, as if an earlier authorized startup read these.
    bus.drives={3:SimpleNamespace(original={0x701E:30.,0x701F:6.,0x7020:.02,0x702B:0.})}
    hw.samples[3]=SimpleNamespace(q=.1,iq_raw_a=2.,requested_at=99.9)
    hw.command_reference=[0,0,.12,0,0,0]
    bus.poll_io.extend([dict(requested_at=99.,received_at=99.008,duration_ms=8.),dict(requested_at=99.058,received_at=99.070,duration_ms=12.)])
    settings=hw.control_settings();d=settings['motor_diagnostics']['3']
    assert d['csp_gains']['0x701f']==6 and d['current_a']==2
    assert d['position_age_ms']==pytest.approx(100.)
    assert d['following_error_deg']==pytest.approx(np.rad2deg(.02))
    assert settings['current_limits_a']==[57,57,57,16,16,16]
    assert settings['poll_timing']['max_between_queries_ms']==pytest.approx(50.)
    assert sdk.calls==[] and not bus.motors
