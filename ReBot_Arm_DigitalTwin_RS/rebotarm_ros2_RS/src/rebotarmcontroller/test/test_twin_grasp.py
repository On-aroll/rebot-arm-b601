"""Contact evidence and the actual worker path, entirely in memory."""
import pytest
import threading
from types import SimpleNamespace

from rebotarmcontroller.twin_grasp import GraspMonitor, validate_profile, MAX_REFERENCE_LEAD
from rebotarmcontroller.csp_runtime.rebot_mit_hardware import Sample
from test_csp_gripper import inert_gripper
from test_twin_program import console, program


def profile():
    # Synthetic fixture values, NOT a calibrated real-machine profile.
    return dict(empty_position=0., empty_tolerance=.02, min_gap=.05,
                contact_current_a=.5, abort_current_a=2., stall_velocity=.01,
                min_travel=.05, dwell=.3, hold_time=.3, timeout=30.)


def feed(m, stamp, *, q=.2, velocity=0., current=1., reference=.18, now=None):
    return m.update(position=q, velocity=velocity, current=current,
                    stamp=stamp, now=stamp if now is None else now, reference=reference)


@pytest.mark.parametrize('change', [dict(contact_current_a=True), dict(contact_current_a=0.),
    dict(abort_current_a=.5), dict(min_gap=float('nan')), dict(timeout=.5), dict(extra=1), dict(timeout=10**1000)])
def test_profile_rejects_missing_unbounded_or_ambiguous_thresholds(change):
    p=profile();p.update(change)
    with pytest.raises(ValueError):validate_profile(p)
    with pytest.raises(ValueError):validate_profile({})


def test_contact_needs_distinct_samples_sustained_current_gap_and_travel():
    m=GraspMonitor(profile(),.4,10.,'test')
    assert feed(m,10.)=='contact'
    for _ in range(30):assert feed(m,10.,now=10.5)=='contact'
    assert feed(m,10.3)=='contact'
    assert feed(m,10.7)=='holding'
    assert m.state()['object_verified'] is False


@pytest.mark.parametrize('case', ['empty', 'low_current', 'moving', 'no_travel', 'no_load_error'])
def test_empty_or_ordinary_stops_never_pass(case):
    m=GraspMonitor(profile(),.4,10.,'test')
    kwargs={'empty':dict(q=.01), 'low_current':dict(current=.1), 'moving':dict(velocity=-.05),
            'no_travel':dict(q=.38,reference=.36), 'no_load_error':dict(reference=.2)}[case]
    for n in range(12):feed(m,10+n*.1,**kwargs)
    assert m.status != 'holding'
    if case=='empty':assert m.status=='failed' and '空夹' in m.reason


@pytest.mark.parametrize('case', ['stale', 'backwards', 'gap', 'timeout', 'current', 'noise'])
def test_bad_feedback_or_contact_uncertainty_fails_without_retry(case):
    p=profile();p['timeout']=2.
    m=GraspMonitor(p,.4,10.,'test');feed(m,10.)
    if case=='stale':feed(m,10.,now=10.7)
    elif case=='backwards':feed(m,9.99)
    elif case=='gap':feed(m,10.7)
    elif case=='timeout':
        for n in range(1,23):feed(m,10+n*.1,q=.3,velocity=-.05)
    elif case=='current':feed(m,10.1,current=2.)
    else:feed(m,10.1,current=.1)
    assert m.status=='failed'
    feed(m,10.2)
    assert m.status=='failed'


def test_holding_loss_is_latched_and_pause_does_not_invent_new_evidence():
    m=GraspMonitor(profile(),.4,10.,'test')
    for stamp in [10.,10.3,10.61]:feed(m,stamp)
    assert m.status=='holding'
    feed(m,10.8,current=0.);feed(m,11.01,current=0.)
    assert m.status=='failed' and '丢失' in m.reason
    m.cancel();assert m.status=='cancelled'


def test_can_worker_freezes_first_contact_and_never_builds_tracking_error():
    g,clock,sent=inert_gripper();g.reference=.4;g.sample=None
    g.history=[(clock[0],g.reference)]
    g.grasp=GraspMonitor(profile(),.4,clock[0],'worker')
    g.pending_grasp=dict(id='worker',profile=profile(),speed=.1)
    q=[.4];samples=[]
    def read(check):
        check();before=q[0];q[0]=max(.2,g.reference)
        velocity=(q[0]-before)/.1
        current=1. if q[0]-g.reference>=.002 else 0.
        g.sample=Sample(7,q[0],velocity,5,clock[0],clock[0],iq_raw_a=current)
        samples.append((q[0],g.reference))
        return g.sample
    g._read=read
    frozen=None
    for _ in range(180):
        clock[0]+=.1;g.step()
        if g.grasp.status=='contact' and frozen is None:frozen=g.reference
        if g.grasp.status=='holding':break
    assert g.grasp.status=='holding',g.grasp.state()
    assert not g.error and frozen is not None and g.reference==frozen
    assert all(q-ref <= MAX_REFERENCE_LEAD+1e-9 for q,ref in samples)
    assert g.reference > .17 and all(value>0 for _,value in sent)
    g.pause();clock[0]+=.1;g.step()
    assert g.grasp.status=='holding' and g.reference==frozen


def grasp_console(tmp_path):
    result=console(tmp_path)
    t,hw,q,g,events,writes,checks,tick,preview=result
    g.update(position=.4,velocity=0.,grasp_supported=True)
    def start(token,p,speed):
        g['grasp']=dict(id='job-1',status='closing',message='closing')
        return 'job-1'
    hw.gripper_grasp=start;t.session='owner';t.mode='HOLD'
    p=preview(program([dict(type='grasp',speed=.1,profile=profile()),
                       dict(type='joint',joints=[.01]*6,speed=.1)]))
    t.extensions.command(dict(op='program_start',plan_id=p['id']));tick()
    assert t.extensions.run['waiting'] and g['authorized'] and not writes
    return result


@pytest.mark.parametrize('case',['empty','expired','wrong_id','cancelled'])
def test_program_never_lifts_on_empty_stale_or_wrong_grasp(tmp_path,case):
    t,hw,q,g,events,writes,checks,tick,preview=grasp_console(tmp_path)
    if case=='empty':g['grasp'].update(status='failed',message='空夹')
    elif case=='expired':g['fresh']=False
    elif case=='wrong_id':g['grasp']['id']='old-job'
    else:g['grasp']['status']='cancelled'
    tick()
    assert t.extensions.run['status']=='failed' and t.mode=='HOLD' and not writes
    for _ in range(4):tick()
    assert not writes and t.extensions.run['index']==0


def test_program_checks_held_evidence_before_next_arm_sample(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=grasp_console(tmp_path)
    g['grasp'].update(status='holding',message='held');tick();tick();tick()
    assert writes and t.mode=='GOAL'
    count=len(writes)
    g['grasp'].update(status='failed',message='滑移')
    tick()
    assert t.mode=='HOLD' and t.extensions.run['status']=='failed'
    assert len(writes)==count


def test_compiler_requires_supported_driver_and_rejects_short_opening(tmp_path):
    t,hw,q,g,events,writes,checks,tick,preview=console(tmp_path)
    from rebotarmcontroller.twin_program import compile_program
    p=program([dict(type='grasp',profile=profile(),speed=.1)])
    with pytest.raises(ValueError,match='驱动'):compile_program(p,q,t.kin,hw,t._goal_duration)
    g['grasp_supported']=True;hw.gripper_grasp=lambda *args:None
    with pytest.raises(ValueError,match='张开'):compile_program(p,q,t.kin,hw,t._goal_duration)
    assert not writes


def test_fake_without_contact_physics_cannot_report_holding():
    from rebotarmcontroller.fake_leader_hardware import FakeLeaderHardware
    node=SimpleNamespace(_leader_lock=threading.RLock(),gripper_open_position=5.,
        max_joint_speed=1.,enabled=True,state_machine='IDLE',positions=[0.]*6,
        gripper_position=.4,gripper_target=.4,gripper_velocity=0.)
    h=FakeLeaderHardware(node);token=h.teleop_acquire();h.authorize_gripper(token,True)
    job=h.gripper_grasp(token,profile(),.1)
    for _ in range(20):assert h.gripper_state()['grasp']['status']!='holding'
    node.gripper_position=0.
    state=h.gripper_state()
    assert state['grasp']['id']==job and state['grasp']['status']=='failed'
    assert '空夹' in state['grasp']['message']


def test_public_grasp_api_checks_lease_authorization_and_caps_without_bus_io():
    from rebotarmcontroller.csp_hardware import CspHardware
    g,clock,sent=inert_gripper()
    g.sample=Sample(7,.4,0.,5,clock[0],clock[0],iq_raw_a=0.)
    h=CspHardware.__new__(CspHardware)
    h.lock=threading.RLock();h.clock=lambda:clock[0];h.session='owner';h.phase='ACTIVE';h.gripper=g
    with pytest.raises(RuntimeError):h.gripper_grasp('wrong',profile(),.1)
    g.authorized=False
    with pytest.raises(ValueError):h.gripper_grasp('owner',profile(),.1)
    g.authorized=True
    with pytest.raises(ValueError):h.gripper_grasp('owner',profile(),.2)
    g.speed_max=.05
    with pytest.raises(ValueError):h.gripper_grasp('owner',profile(),.1)
    g.speed_max=1.
    job=h.gripper_grasp('owner',profile(),.1)
    assert g.pending_grasp['id']==job and g.grasp.id==job and not sent
    g.reference=.4;g.history=[(clock[0],.4)]
    g.view.runtime[0x7018]=1.  # An original device cap below the submitted profile.
    clock[0]+=.1;g.step()
    assert g.grasp.status=='failed' and '限流' in g.grasp.reason
    assert g.reference==.4 and all(value==.4 for _,value in sent) and not g.error
