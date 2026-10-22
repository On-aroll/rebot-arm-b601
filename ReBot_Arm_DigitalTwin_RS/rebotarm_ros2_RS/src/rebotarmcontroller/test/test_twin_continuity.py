"""Continuity at consecutive point boundaries, using a real URDF and inert IO."""
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time

from rebotarmcontroller.twin_control import TwinControl
from rebotarmcontroller.twin_kinematics import TwinKinematics


def test_second_point_starts_at_held_reference_with_small_encoder_lag():
    clock=[10.];sent=[]
    kin=TwinKinematics();held=np.array([.7,.3,.4,-.1,.2,.2])
    actual=held.copy();actual[0]-=.004  # > first-enable 0.1°, within tracking guard.
    class Publisher:
        def publish(self,message):pass
    node=SimpleNamespace(create_publisher=lambda *a,**k:Publisher(),create_service=lambda *a,**k:None,
        create_subscription=lambda *a,**k:None,create_timer=lambda *a,**k:None,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=Time)))
    hw=SimpleNamespace(motion_ready=True,command_reference=held.tolist(),joint_arrival_tolerance=np.deg2rad(.1),
        get_joint_state=lambda **kw:(actual.copy(),[0]*6,[0]*6),_feedback_refreshed_at=10.,feedback_timeout=.6,
        min_twin_speed=.005,max_twin_speed=1.,teleop_limits=list(zip(kin.lower,kin.upper)),
        config=SimpleNamespace(axes=[SimpleNamespace(a_max=4.,jerk_max=30.)]*6),
        control_settings=lambda:dict(remaining_s=None,session_limit_s=None),
        validate_preview=lambda start,goal,**kwargs:None,teleop_resume=lambda token:None,
        teleop_point=lambda token,seq,stamp,goal,speed:sent.append(goal.copy()),goal_settle_timeout=10)
    twin=TwinControl(node,hw,'inert',kinematics=kin,clock=lambda:clock[0]);twin.session='owner';twin.mode='HOLD'
    pose=kin.pose(held+np.array([.003,.002,0,0,0,0]))
    twin._preview(dict(session='owner',pose=pose,orientation=False,speed=.1))
    assert not sent and np.array_equal(twin.plan['start'],held)
    assert np.array_equal(twin.plan['observed_start'],actual)
    twin._start_goal(dict(plan_id=twin.plan['id']))
    twin._advance_goal()
    assert sent and np.array_equal(sent[-1]['start'],held)
    assert not sent[-1]['align_start']


def test_rejection_logging_cannot_raise_on_nonfinite_input(tmp_path):
    messages=[]
    twin=TwinControl.__new__(TwinControl)
    twin.events=SimpleNamespace(publish=messages.append)
    twin.hw=SimpleNamespace(output=tmp_path)
    twin._positions=lambda:[0.]*6
    twin.reference=None
    twin._event(dict(op='preview',pose=[float('nan')]*6),False,'invalid pose')
    assert len(messages)==1
    assert 'invalid pose' in messages[0].data
