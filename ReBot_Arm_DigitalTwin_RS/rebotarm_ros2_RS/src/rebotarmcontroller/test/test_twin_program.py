"""Inert program compilation and shared executor semantics, no ROS graph/CAN."""
import copy
import json
import math
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from rebotarmcontroller.twin_control import TwinControl
from rebotarmcontroller.twin_program import CALIBRATION, compile_program, validate_program


def program(steps):
    return dict(schema_version='1.0',robot_model='b601_rs',base_frame='base_link',calibration_id=CALIBRATION,
                units=dict(position='m',angle='rad',time='s'),name='test',steps=steps)


class Kin:
    lower=np.full(6,-1.);upper=np.full(6,1.)
    def pose(self,q):return list(q)
    def error(self,q,p,orientation=True):return np.array(p)-np.array(q)
    def solve(self,p,seed,orientation,path_check=None):
        if path_check:path_check(seed,p)
        return list(p),0.
    def jog(self,q,twist,lookahead,max_step,orientation=True):return (np.array(q)+np.array(twist)*lookahead).tolist()


def console(tmp_path):
    now=[10.];q=[0.]*6;events=[];writes=[];checks=[];grip=dict(available=True,fresh=True,authorized=False,position=0.,speed_max=1.,arrived=True)
    class Publisher:
        def publish(self,msg):pass
    node=NS(create_publisher=lambda *a,**k:Publisher(),create_service=lambda *a,**k:None,
            create_subscription=lambda *a,**k:None,create_timer=lambda *a,**k:None,
            get_clock=lambda:NS(now=lambda:NS(to_msg=Time)),get_logger=lambda:NS(warn=lambda _:None))
    def send(token,seq,stamp,target,g,speed):q[:]=target;writes.append(list(target))
    def check(a,b):checks.append((list(a),list(b)))
    hw=NS(output=tmp_path,min_twin_speed=.005,max_twin_speed=1.,teleop_limits=[(-1,1)]*6,
          config=NS(fingerprint='current',axes=[NS(a_max=4.,jerk_max=30.)]*6),
          get_joint_state=lambda **kw:(q[:],[0.]*6,[0.]*6),validate_preview=check,
          control_settings=lambda:dict(remaining_s=None,session_limit_s=None),
          teleop_resume=lambda _:None,teleop_pause=lambda _:None,teleop_stop=lambda *a:None,
          teleop_owned=True,teleop_watchdog=lambda:None,teleop_send=send,
          gripper_state=lambda:grip.copy(),gripper_target=lambda token,pos,speed:grip.update(position=pos,arrived=True),
          authorize_gripper=lambda token,enabled:grip.update(authorized=enabled))
    t=TwinControl(node,hw,'inert',kinematics=Kin(),clock=lambda:now[0]);t.events=NS(publish=lambda msg:events.append(json.loads(msg.data)))
    def tick(delta=.05):now[0]+=delta;t.last_heartbeat=now[0];t.tick()
    def preview(p):
        assert t.extensions.readonly(dict(op='program_preview',program=p,session=t.session,id='preview'))
        deadline=time.monotonic()+3
        while t.extensions.compile_busy and time.monotonic()<deadline:time.sleep(.005)
        assert events[-1]['ok'],events[-1]
        return t.extensions.preview
    return t,hw,q,grip,events,writes,checks,tick,preview


@pytest.mark.parametrize('change',[dict(schema_version='2'),dict(calibration_id='different'),dict(base_frame='camera'),
    dict(units=dict(position='mm',angle='rad',time='s')),dict(steps=[dict(type='enable')]),
    dict(steps=[dict(type='wait',seconds=True)]),dict(steps=[dict(type='joint',joints=[float('nan')]*6)]),
    dict(steps=[dict(type='gripper',position=5.01)]),dict(steps=[dict(type='wait',seconds=1,code='bad')])])
def test_import_rejects_ambiguous_or_executable_input(change):
    p=program([dict(type='wait',seconds=1)]);p.update(change)
    with pytest.raises(ValueError):validate_program(p)


def test_compilation_checks_every_path_and_retimes_short_trajectory(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path)
    p=program([dict(type='joint',joints=[.1]*6,duration=.05,speed=.1),dict(type='pose',pose=[.15]*6,speed=.1),dict(type='wait',seconds=.1)])
    result=preview(p)
    assert not writes and len(checks)==2 and result['steps'][0]['retimed']
    assert result['steps'][0]['duration']>=1.875
    assert result['steps'][1]['start']==[.1]*6
    hw.validate_preview=lambda a,b:(_ for _ in ()).throw(ValueError('blocked path'))
    with pytest.raises(ValueError,match='第 1 步'):compile_program(p,q,Kin(),hw,t._goal_duration)


def test_program_runs_wait_joint_and_gripper_through_same_lease(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path);t.session='owner';t.mode='HOLD';g['authorized']=True
    p=preview(program([dict(type='joint',joints=[.01]*6,speed=.1),dict(type='gripper',position=.2,speed=.1),dict(type='wait',seconds=.1)]))
    t.extensions.command(dict(op='program_start',plan_id=p['id']))
    for _ in range(60):tick()
    assert t.extensions.run['status']=='completed' and g['position']==.2 and writes
    assert np.allclose(q,[.01]*6)


def test_pause_cancel_preview_expiry_and_drift(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path);t.session='owner';t.mode='HOLD'
    p=preview(program([dict(type='wait',seconds=1)]));q[0]=.1
    with pytest.raises(ValueError,match='起点'):t.extensions.command(dict(op='program_start',plan_id=p['id']))
    q[0]=0.;p=preview(program([dict(type='wait',seconds=1)]));p['at']-=61
    with pytest.raises(ValueError,match='过期'):t.extensions.command(dict(op='program_start',plan_id=p['id']))
    p=preview(program([dict(type='wait',seconds=1)]));t.extensions.command(dict(op='program_start',plan_id=p['id']));tick()
    t.extensions.command(dict(op='program_pause'));assert t.extensions.run['status']=='paused'
    t.extensions.command(dict(op='program_cancel'));assert t.extensions.run['status']=='cancelled' and not t.goal and not writes


def test_support_requires_verified_unpowered_feedback_and_preserves_current_base(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path)
    t.extensions.readonly(dict(op='support_preview',speed=.1))
    while t.extensions.compile_busy:time.sleep(.005)
    assert not events[-1]['ok'] and '尚无' in events[-1]['message']
    with pytest.raises(ValueError,match='确认'):t.extensions.readonly(dict(op='support_capture',verified=False))
    t.session='owner'
    with pytest.raises(ValueError,match='未使能'):t.extensions.readonly(dict(op='support_capture',session='owner',verified=True,support_note='fixture'))
    t.session='';q[:]=[.2,.1,.1,.1,.1,.1]
    t.extensions.readonly(dict(op='support_capture',name='fixture',verified=True,support_note='tested physical support'))
    q[0]=-.3
    t.extensions.readonly(dict(op='support_preview',speed=.1,id='support'))
    while t.extensions.compile_busy:time.sleep(.005)
    assert t.extensions.preview['steps'][0]['joints']==[-.3,.1,.1,.1,.1,.1]
    assert not writes and (tmp_path/'support-poses.json').exists()


def zero_support_console(tmp_path):
    (tmp_path/'support-poses.json').write_text(json.dumps([dict(
        id='initial-zero', name='初始姿态', joints=[.9,0,0,0,0,0], verified=True,
        calibration_id=CALIBRATION, arrival_tolerance_deg=3)]))
    result=console(tmp_path)
    t,hw,q,g,events,writes,checks,tick,preview=result
    q[:]=[-.3,.1,.1,.1,.1,.1]
    t.extensions.readonly(dict(op='support_preview',speed=.1))
    while t.extensions.compile_busy:time.sleep(.005)
    assert events[-1]['ok'] and not writes
    p=t.extensions.preview
    assert p['steps'][0]['joints']==[-.3,0,0,0,0,0]
    assert p['support_tolerance_rad']==pytest.approx(math.radians(3))
    t.session='owner';t.mode='HOLD'
    t.extensions.command(dict(op='program_start',plan_id=p['id'],release_after=True))
    tick()
    assert t.goal['support_tolerance_rad']==pytest.approx(math.radians(3))
    hw.teleop_send=lambda *args:None  # The feedback can lag behind the reference.
    return result


def test_support_recovery_preview_execute_pause_and_all_three_arrivals(tmp_path):
    from test_support_recovery import hardware_for_recovery, START, TARGET, LIMITS
    from rebotarmcontroller.twin_motion import point_sample

    (tmp_path/'support-poses.json').write_text(json.dumps([dict(
        id='initial-zero', name='初始姿态', joints=TARGET.tolist(), verified=True,
        calibration_id=CALIBRATION, arrival_tolerance_deg=3)]))
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path)
    q[:]=START.tolist()
    policy_hw=hardware_for_recovery()
    policy_hw.lock=threading.RLock();policy_hw.session='owner'
    policy_hw.phase='READONLY';policy_hw.executor=None
    policy_hw.feedback_timeout=.6;policy_hw.clock=t.clock
    hw.teleop_limits=LIMITS.tolist()
    hw.plan_support_recovery=policy_hw.plan_support_recovery
    hw.validate_support_recovery_preview=policy_hw.validate_support_recovery_preview
    hw.validate_preview=policy_hw.validate_preview
    def begin(token,plan):
        policy_hw._q=np.array(q)
        policy_hw.samples={i:NS(mode=0,qd=0.,requested_at=t.clock()) for i in range(1,7)}
        return policy_hw.begin_support_recovery(token,plan)
    hw.begin_support_recovery=begin
    completed=[]
    def complete(tolerance):
        completed.append((policy_hw.support_recovery.stage,tolerance))
        return policy_hw.complete_support_recovery_segment(tolerance)
    hw.complete_support_recovery_segment=complete
    hw.command_reference=q;hw.motion_ready=True
    hw.continuous_jog_reference=True;hw.joint_arrival_tolerance=math.radians(.1)
    def send_point(token,seq,stamp,trajectory,speed):
        recovery=policy_hw.support_recovery
        recovery.begin_segment(trajectory['start'],trajectory['joints'])
        assert trajectory['support_recovery'] is True
        sample=point_sample('quintic',trajectory['start'],trajectory['joints'],speed,4.,30.,
                            trajectory['duration'],t.clock()-trajectory['began'])
        recovery.advance_reference(sample)
        q[:]=sample.tolist();policy_hw._q=sample.copy();writes.append(q[:])
    hw.teleop_point=send_point
    # Preview alone creates neither a capability nor motion references.
    t.extensions.readonly(dict(op='support_preview',speed=.1,id='recovery-preview'))
    deadline=time.monotonic()+3
    while t.extensions.compile_busy and time.monotonic()<deadline:time.sleep(.005)
    assert events[-1]['ok'],events[-1]
    p=t.extensions.preview
    assert len(p['steps'])==3 and not writes and policy_hw.support_recovery is None
    t.session='owner';t.mode='HOLD'
    t.extensions.command(dict(op='program_start',plan_id=p['id']))
    tick()
    assert t.goal['support_tolerance_rad']==pytest.approx(math.radians(.1))
    for _ in range(20):tick()
    t.extensions.command(dict(op='program_pause'))
    assert t.extensions.run['status']=='paused' and policy_hw.support_recovery.active
    t.extensions.command(dict(op='program_resume',id='recovery-resume'))
    deadline=time.monotonic()+3
    while t.extensions.compile_busy and time.monotonic()<deadline:time.sleep(.005)
    assert events[-1]['ok'],events[-1]
    for _ in range(6000):
        tick()
        if t.extensions.run['status']=='completed':break
    assert t.extensions.run['status']=='completed',t.message
    assert [stage for stage,_ in completed]==[0,1,2]
    assert [tol for _,tol in completed]==pytest.approx([math.radians(.1),math.radians(.1),math.radians(3)])
    assert policy_hw.support_recovery is None
    np.testing.assert_allclose(q,TARGET,atol=math.radians(3))


@pytest.mark.parametrize('axis',range(1,6))
@pytest.mark.parametrize('degrees,arrived',[(3,True),(-3,True),(3.01,False),(-3.01,False)])
def test_support_tolerance_checks_each_of_five_axes_without_tcp_precision(tmp_path,axis,degrees,arrived):
    t,hw,q,g,events,writes,checks,tick,preview=zero_support_console(tmp_path)
    q[:]=[-.3,0,0,0,0,0];q[axis]=math.radians(degrees)
    t.goal['began']=t.clock()-t.goal['duration']
    t._advance_goal()
    assert (t.mode=='HOLD') is arrived
    assert t.last_result['support_joint_errors_deg'][axis-1]==pytest.approx(abs(degrees))
    assert g['position']==0 and not g['authorized']
    assert t.extensions.run['index']==int(arrived)


def test_support_does_not_release_before_trajectory_end_and_keeps_tolerance_on_resume(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=zero_support_console(tmp_path)
    q[:]=[-.3,0,0,0,0,0]
    t.goal['began']=t.clock()-t.goal['duration']/2
    t._advance_goal()
    assert t.mode=='GOAL' and t.extensions.run['index']==0
    t.extensions.command(dict(op='program_pause'))
    t.extensions.command(dict(op='program_resume'))
    while t.extensions.compile_busy:time.sleep(.005)
    assert events[-1]['ok']
    assert t.extensions.run['status'] == 'running' and '继续执行' in t.message
    assert '已暂停' not in t.message
    tick()
    assert t.goal['support_tolerance_rad']==pytest.approx(math.radians(3))


def test_ordinary_program_cannot_use_support_tolerance(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path)
    t.session='owner';t.mode='HOLD';hw.joint_arrival_tolerance=math.radians(.1)
    p=preview(program([dict(type='joint',joints=[.1]*6,speed=.1)]))
    t.extensions.command(dict(op='program_start',plan_id=p['id']))
    tick();hw.teleop_send=lambda *args:None
    q[:]=[.1+math.radians(1)]*6
    t.goal['began']=t.clock()-t.goal['duration']
    t._advance_goal()
    assert t.mode=='GOAL' and 'support_tolerance_rad' not in t.goal
    p=program([dict(type='joint',joints=[0]*6,speed=.1,support_tolerance_rad=math.radians(3))])
    with pytest.raises(ValueError):validate_program(p)


def test_stop_invalidates_async_preview_before_worker_returns(tmp_path,monkeypatch):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path);entered=threading.Event();release=threading.Event()
    import rebotarmcontroller.twin_extensions as module
    original=module.compile_program
    def slow(*args,**kwargs):entered.set();release.wait(2);return original(*args,**kwargs)
    monkeypatch.setattr(module,'compile_program',slow)
    t.extensions.readonly(dict(op='program_preview',id='late',program=program([dict(type='wait',seconds=0)])))
    assert entered.wait(1);t.extensions.stop('reset');release.set();time.sleep(.05)
    assert not t.extensions.preview and not events and not writes


def test_tcp_drag_expires_to_hold_without_replaying_input(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path);t.session='owner';t.mode='HOLD'
    t.extensions.command(dict(op='tcp_target',pose=[.01,0,0,0,0,0],speed=.1));tick()
    assert t.mode=='JOG' and writes
    tick(.3);assert t.mode=='HOLD' and t.extensions.tcp is None
    count=len(writes);tick();assert len(writes)==count


def test_active_program_blocks_competing_goal_jog_and_preview(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path);t.session='owner';t.mode='HOLD'
    p=preview(program([dict(type='wait',seconds=10)]));t.extensions.command(dict(op='program_start',plan_id=p['id']));tick()
    with pytest.raises(RuntimeError,match='程序'):t._jog(dict(twist=[0]*6,speed=.1))
    with pytest.raises(RuntimeError,match='暂停'):t._preview(dict(session='owner',pose=[0]*6,speed=.1))


def test_pause_wins_over_late_async_resume(tmp_path,monkeypatch):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path);t.session='owner';t.mode='HOLD'
    p=preview(program([dict(type='wait',seconds=5)]));t.extensions.command(dict(op='program_start',plan_id=p['id']));tick()
    t.extensions.command(dict(op='program_pause'))
    entered=threading.Event();release=threading.Event()
    import rebotarmcontroller.twin_extensions as module
    original=module.compile_program
    def slow(*args,**kwargs):entered.set();release.wait(2);return original(*args,**kwargs)
    monkeypatch.setattr(module,'compile_program',slow)
    assert t.extensions.command(dict(op='program_resume',id='late-resume'))=='pending'
    assert entered.wait(1);t._hold('pause again');release.set();time.sleep(.05)
    assert t.extensions.run['status']=='paused' and not writes
    assert not any(e.get('id')=='late-resume' for e in events)


def test_numeric_endpoint_noise_is_projected_but_real_outside_targets_are_rejected(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path);hw.teleop_limits=[(0,1)]*6
    p=preview(program([dict(type='joint',joints=[-1e-15]*6,speed=.1)]))
    assert p['steps'][0]['joints']==[0]*6 and not writes
    with pytest.raises(ValueError,match='越界'):compile_program(program([dict(type='joint',joints=[-.001]*6,speed=.1)]),q,Kin(),hw,t._goal_duration)


def test_gripper_compilation_uses_measured_distance_and_changed_start_rejects(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path);t.session='owner';t.mode='HOLD';g['authorized']=True
    p=preview(program([dict(type='gripper',position=.01,speed=.1),dict(type='gripper',position=.02,speed=.1)]))
    assert p['duration']<=1.1 and p['steps'][1]['start_position']==.01
    g['position']=.1
    with pytest.raises(ValueError,match='夹爪起点'):t.extensions.command(dict(op='program_start',plan_id=p['id']))
