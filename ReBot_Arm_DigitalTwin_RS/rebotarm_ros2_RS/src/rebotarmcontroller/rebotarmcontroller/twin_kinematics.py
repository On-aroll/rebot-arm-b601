"""Pure kinematics for the twin console. Never constructs a CAN/actuator object."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from ament_index_python.packages import get_package_share_directory


class IKFailure(ValueError):
    def __init__(self, message, diagnostic):
        super().__init__(message)
        self.diagnostic = diagnostic


class _IKBudgetExpired(Exception):
    pass


class TwinKinematics:
    def __init__(self, urdf=None, *, position_tolerance=.001, rotation_tolerance=.015, start_tolerance=0.):
        if urdf is None:
            urdf = Path(get_package_share_directory("rebotarm_bringup")) / "description/urdf/ReBot_Arm_RS.urdf"
        self.model = pin.buildModelFromUrdf(str(urdf))
        self.data = self.model.createData()
        self.frame = self.model.getFrameId("gripper_end")
        if self.frame >= self.model.nframes:
            raise ValueError("RS model has no gripper_end TCP frame")
        self.indices = [self.model.joints[self.model.getJointId(f"joint{i}")].idx_q for i in range(1, 7)]
        self.vel_indices = [self.model.joints[self.model.getJointId(f"joint{i}")].idx_v for i in range(1, 7)]
        self.lower = self.model.lowerPositionLimit[self.indices]
        self.upper = self.model.upperPositionLimit[self.indices]
        self.position_tolerance = position_tolerance
        self.rotation_tolerance = rotation_tolerance
        self.start_tolerance = start_tolerance

    def placement(self, joints):
        q = np.zeros(self.model.nq)
        q[self.indices] = joints
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.frame].copy()

    def pose(self, joints):
        transform = self.placement(joints)
        return np.r_[transform.translation, pin.rpy.matrixToRpy(transform.rotation)].tolist()

    def jacobian(self, joints):
        q = np.zeros(self.model.nq)
        q[self.indices] = joints
        return pin.computeFrameJacobian(
            self.model, self.data, q, self.frame, pin.LOCAL_WORLD_ALIGNED
        )[:, self.vel_indices]

    def error(self, joints, pose, orientation=True):
        current = self.placement(joints)
        position = np.asarray(pose[:3]) - current.translation
        rotation = pin.log3(pin.rpy.rpyToMatrix(*pose[3:]) @ current.rotation.T)
        return np.r_[position, rotation if orientation else np.zeros(3)]

    def solve(self, pose, seed, orientation=True, *, path_check=None):
        """Bounded trust-region IK; never relax orientation or accept a near miss."""
        pose,seed = np.asarray(pose,dtype=float),np.asarray(seed,dtype=float)
        if pose.shape != (6,) or seed.shape != (6,) or not np.all(np.isfinite(np.r_[pose,seed])):
            raise ValueError("IK 需要六个有限位姿数值和六轴起点")
        began = time.monotonic()
        path_rejections=[]
        q=np.clip(seed,self.lower,self.upper);error=self.error(q,pose,orientation)
        offsets=([0]*6,[0,.15,.15,-.1,0,0],[0,.4,.7,-.3,.2,0],[0,.7,.4,.3,-.2,0]) if path_check else ([0]*6,)
        for offset in offsets:
            if time.monotonic() >= began+.22: break
            q,error = self._bounded_solve(pose,seed+offset,orientation,began+.22)
            if self._solved(error):
                try:
                    if path_check: path_check(seed,q)
                except ValueError as exc:
                    path_rejections.append(str(exc))
                    continue
                return q.tolist(),error.tolist()
        if path_rejections:
            raise IKFailure('坐标存在逆解，但候选关节路径未通过检查：'+path_rejections[0]+'。未发送运动。',
                dict(code='PATH_REJECTED',stage='path',reasons=list(dict.fromkeys(path_rejections)),
                     orientation_constrained=bool(orientation)))
        # A position-only probe diagnoses the constraint, but its result is
        # never substituted for a requested orientation-constrained target.
        position_only = False
        if orientation:
            _,position_error = self._bounded_solve(pose,seed,False,began+.32)
            position_only = self._solved(position_error)
        near = [f"J{i+1}" for i,value in enumerate(seed)
                if min(abs(value-self.lower[i]),abs(value-self.upper[i])) < np.deg2rad(1)]
        diagnostic = dict(code="IK_NO_SOLUTION",position_error_mm=float(np.linalg.norm(error[:3])*1000),
            rotation_error_deg=float(np.rad2deg(np.linalg.norm(error[3:]))),
            position_only_possible=position_only,near_limit_joints=near,orientation_constrained=bool(orientation))
        message = (f"未找到满足关节限位与{'指定姿态' if orientation else '位置'}要求的解："
                   f"最接近结果的位置残差 {diagnostic['position_error_mm']:.2f} mm")
        if orientation:
            message += f"，姿态残差 {diagnostic['rotation_error_deg']:.2f}°"
        if near:
            message += f"；当前 {', '.join(near)} 接近限位"
        if position_only:
            message += "。同一坐标在「仅位置」模式可解，可切换后重新预览"
        message += "。未发送运动。"
        raise IKFailure(message,diagnostic)

    def _solved(self, error):
        return bool(np.linalg.norm(error[:3]) <= self.position_tolerance and
                    np.linalg.norm(error[3:]) <= self.rotation_tolerance)

    def _bounded_solve(self, pose, seed, orientation, deadline):
        # Bounds are part of the optimization, rather than clipping an
        # unconstrained DLS step and destroying its coordinated descent.
        start = np.clip(seed,self.lower,self.upper)
        weights = np.array([1.,1.,1.,.2,.2,.2]) if orientation else np.ones(3)
        error = self.error(start,pose,orientation)
        best = [float(np.linalg.norm((error if orientation else error[:3])*weights)),start,error]
        if self._solved(error):
            return start,error
        def residual(q):
            if time.monotonic() > deadline:
                raise _IKBudgetExpired()
            error = self.error(q,pose,orientation)
            value = (error if orientation else error[:3])*weights
            score = float(np.linalg.norm(value))
            if score < best[0]:
                best[:] = [score,q.copy(),error.copy()]
            # Position-only IK has three redundant DOFs. Prefer the nearby
            # posture rather than an arbitrary wrist/elbow solution for each
            # new point. Position/orientation tolerances remain unchanged.
            return np.r_[value,1e-4*(q-start)]
        # Deterministic alternate elbow seeds escape the straight/limit pose.
        offsets = ([0,0,0,0,0,0],[0,.15,.15,-.1,0,0],
                   [0,.4,.7,-.3,.2,0],[0,.7,.4,.3,-.2,0])
        for offset in offsets:
            try:
                result = least_squares(residual,np.clip(start+offset,self.lower,self.upper),
                    bounds=(self.lower,self.upper),x_scale="jac",max_nfev=100,
                    ftol=1e-10,xtol=1e-10,gtol=1e-10)
            except _IKBudgetExpired:
                break
            error = self.error(result.x,pose,orientation)
            if self._solved(error):
                return result.x,error
        return best[1],best[2]

    def jog(self, joints, twist, lookahead=0.05, max_step=0.006, orientation=True):
        jac = self.jacobian(joints)
        twist = np.asarray(twist)
        if not orientation:
            jac, twist = jac[:3], twist[:3]
        velocity = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(len(twist)) * 1e-4, twist)
        delta = velocity * lookahead
        # Preserve relative joint velocities when the slowest axis saturates.
        # Each target must be reachable before the next 20 Hz input sample;
        # otherwise independent motor slew caps can erase TCP rotation.
        delta *= min(1.0, max_step / max(np.max(np.abs(delta)), 1e-9))
        initial = np.asarray(joints)
        lower = np.where(initial >= self.lower-self.start_tolerance, np.minimum(initial,self.lower),self.lower)
        upper = np.where(initial <= self.upper+self.start_tolerance, np.maximum(initial,self.upper),self.upper)
        goal = np.clip(initial + delta, lower, upper)
        if np.linalg.norm(twist) > 1e-6 and np.max(np.abs(goal - joints)) < 1e-6:
            raise ValueError("已到关节限位，请反向移动")
        return goal.tolist()
