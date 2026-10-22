"""Real model IK regression tests; no actuator or CAN construction."""
import numpy as np
import pytest

from rebotarmcontroller.csp_twin_driver import CheckedKinematics
from rebotarmcontroller.twin_kinematics import IKFailure, TwinKinematics


@pytest.fixture
def kin():
    return CheckedKinematics()


SEED = np.array([1.68312,.000639,.005028,.011351,.011313,.051810])


@pytest.mark.parametrize('axis,offset',[(1,.005),(1,.010),(2,-.010)])
def test_position_targets_escape_old_clipped_dls_stall(kin,axis,offset):
    pose=np.array(kin.pose(SEED));pose[axis]+=offset
    joints,residual=kin.solve(pose,SEED,orientation=False)
    assert np.all(np.array(joints)>=kin.lower) and np.all(np.array(joints)<=kin.upper)
    assert np.linalg.norm(residual[:3])<=kin.position_tolerance
    assert np.linalg.norm(kin.error(joints,pose,False)[:3])<=kin.position_tolerance


def test_fixed_orientation_is_not_silently_relaxed(kin):
    pose=np.array(kin.pose(SEED));pose[1]+=.005
    with pytest.raises(IKFailure) as exc:
        kin.solve(pose,SEED,orientation=True)
    info=exc.value.diagnostic
    assert info['position_only_possible'] and info['orientation_constrained']
    assert 'J2' in info['near_limit_joints'] and '仅位置' in str(exc.value)
    assert info['position_error_mm']>kin.position_tolerance*1000


def test_fk_generated_target_and_rounded_current_pose(kin):
    target=SEED+np.array([.02,.02,.04,-.02,.01,.03])
    for pose in (kin.pose(target),np.round(kin.pose(SEED),4)):
        joints,residual=kin.solve(pose,SEED)
        assert np.linalg.norm(residual[:3])<=kin.position_tolerance
        assert np.linalg.norm(residual[3:])<=kin.rotation_tolerance


@pytest.mark.parametrize('pose',[[np.nan]*6,[np.inf]*6,[0]*5])
def test_invalid_pose_rejected(kin,pose):
    with pytest.raises(ValueError): kin.solve(pose,SEED)


def test_impossible_target_has_finite_diagnostic(kin):
    with pytest.raises(IKFailure) as exc: kin.solve([3,3,3,0,0,0],SEED,False)
    assert np.isfinite(exc.value.diagnostic['position_error_mm'])
    assert not exc.value.diagnostic['position_only_possible']
def test_nearby_position_targets_preserve_posture_and_report_path_rejection():
    from rebotarmcontroller.twin_kinematics import TwinKinematics,IKFailure
    kin=TwinKinematics();seed=np.array([.7,.3,.4,-.1,.2,.2])
    for step in range(1,16):
        target=kin.pose(seed+np.array([.001,.001,-.001,0,0,0]))
        goal,_=kin.solve(target,seed,False)
        assert np.max(abs(np.asarray(goal)-seed))<.02
        seed=np.asarray(goal)
    def reject(start,goal):raise ValueError('模型检测到自身碰撞')
    with pytest.raises(IKFailure) as caught:kin.solve(kin.pose(seed),seed,False,path_check=reject)
    assert caught.value.diagnostic['code']=='PATH_REJECTED'



def test_tcp_position_only_does_not_silently_lock_wrist_and_j5_is_available():
    k=TwinKinematics();q=np.array([.7,.3,.4,-.1,.2,.2])
    free=np.array(k.jog(q,[.01,0,0,0,0,0],orientation=False))-q
    locked=np.array(k.jog(q,[.01,0,0,0,0,0],orientation=True))-q
    j=k.jacobian(q)
    assert np.linalg.norm((j@free)[3:]) > .0001
    assert np.linalg.norm((j@locked)[3:]) < .00001
    assert (j@free)[0] > .0004
    wrist=np.array(k.jog(q,j[:,4]*.1))-q
    assert abs(wrist[4])>.0001  # no J5 lock or dropped Jacobian column
