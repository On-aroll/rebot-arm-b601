"""RS simulation only. Canonical angles follow Seeed's control URDF, radians."""
from pathlib import Path
import sys
import numpy as np
import pinocchio as pin
import mujoco as mj

ROOT = Path(__file__).resolve().parents[1]
UPSTREAM = ROOT / 'external/reBotArm_control_py'
sys.path.insert(0, str(UPSTREAM))
from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik, IKParams
URDF = UPSTREAM / 'urdf/RS/urdf/ReBot_Arm_RS.urdf'
SCENE = ROOT / 'external/mujoco_menagerie/seeed_rebot_devarm/scene.xml'
HOME = np.array([0., .7, 1.1, 0., 0., 0.])

class Robot:
    def __init__(self, scene=SCENE):
        self.pm = pin.buildModelFromUrdf(str(URDF))
        self.pd = self.pm.createData()
        self.fid = self.pm.getFrameId('gripper_end')
        self.m = mj.MjModel.from_xml_path(str(scene))
        self.m.vis.global_.offwidth = 960
        self.m.vis.global_.offheight = 720
        self.d = mj.MjData(self.m)
        self.arm_ids = [self.m.joint(f'joint{i}').id for i in range(1, 7)]
        self.qadr = np.array([self.m.jnt_qposadr[j] for j in self.arm_ids])
        self.vadr = np.array([self.m.jnt_dofadr[j] for j in self.arm_ids])
        self.act_ids = np.array([self.m.actuator(f'joint{i}').id for i in range(1, 7)])
        self.finger_ids = [self.m.joint(n).id for n in ('joint_left', 'joint_right')]
        self.finger_qadr = [self.m.jnt_qposadr[j] for j in self.finger_ids]
        self.finger_acts = [self.m.actuator(n).id for n in ('joint_left', 'joint_right')]
        # All six axes are reversed between these two pinned upstream assets.
        self.lower = np.maximum(self.pm.lowerPositionLimit[:6], -self.m.jnt_range[self.arm_ids, 1])
        self.upper = np.minimum(self.pm.upperPositionLimit[:6], -self.m.jnt_range[self.arm_ids, 0])
        self.lower[0] = max(self.lower[0], -np.deg2rad(150))
        self.upper[0] = min(self.upper[0], np.deg2rad(150))
        self.pm.lowerPositionLimit[:6] = self.lower
        self.pm.upperPositionLimit[:6] = self.upper
        self.robot_bodies = {self.m.body(n).id for n in ['base_link'] + [f'link{i}' for i in range(1,7)] + ['gripper_end','gripper_left','gripper_right']}
        self.target = HOME.copy()
        self.finger_target = .015
        self.reset()

    def reset(self):
        mj.mj_resetData(self.m, self.d)
        self.target = HOME.copy()
        self.d.qpos[self.qadr] = -self.target
        self.d.qpos[self.finger_qadr] = self.finger_target
        self.command(self.target, self.finger_target)
        mj.mj_forward(self.m, self.d)

    def command(self, q, finger=None):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError('需要六个有限关节角')
        if np.any(q < self.lower - 1e-8) or np.any(q > self.upper + 1e-8):
            raise ValueError('目标超出当前模型可用关节范围')
        if finger is not None:
            if not np.isfinite(finger) or not 0 <= finger <= .05:
                raise ValueError('夹爪单指模型行程必须在 0–0.05 m')
            self.finger_target = float(finger)
        self.target = q.copy()
        self.d.ctrl[self.act_ids] = -q
        self.d.ctrl[self.finger_acts] = self.finger_target

    @property
    def q(self):
        return -self.d.qpos[self.qadr].copy()

    def motion_start(self):
        """Correct tiny simulator overshoot for targets; preserve raw feedback.

        Larger violations block motion. This is not a hardware calibration.
        """
        q = self.q
        if np.any(q < self.lower - 1e-4) or np.any(q > self.upper + 1e-4):
            raise ValueError('仿真反馈超出限位容差，请重置仿真后重试')
        return np.clip(q, self.lower, self.upper)

    def step(self):
        # Simulator-only bias compensation. Never reused as hardware gains.
        mj.mj_forward(self.m, self.d)
        self.d.qfrc_applied[:] = 0
        self.d.qfrc_applied[self.vadr] = self.d.qfrc_bias[self.vadr]
        mj.mj_step(self.m, self.d)
        if not np.all(np.isfinite(self.d.qpos)) or not np.all(np.isfinite(self.d.qvel)):
            raise RuntimeError('非有限仿真状态')

    def fk(self, q):
        full = np.zeros(self.pm.nq); full[:6] = q
        pin.forwardKinematics(self.pm, self.pd, full)
        pin.updateFramePlacements(self.pm, self.pd)
        return self.pd.oMf[self.fid].copy()

    def ik(self, xyz, rotation=None, seed=None, *, position_only=False):
        seed = self.q if seed is None else np.asarray(seed)
        rotation = self.fk(seed).rotation if rotation is None else np.asarray(rotation)
        xyz = np.asarray(xyz, dtype=float)
        if xyz.shape != (3,) or not np.all(np.isfinite(xyz)):
            raise ValueError('末端坐标必须为三个有限数值')
        result = solve_ik(self.pm, self.pm.createData(), self.fid,
                          pin.SE3(rotation, xyz), seed, IKParams(tolerance=1e-6),
                          controlled_joints=6, position_only=position_only)
        if not result.success:
            if position_only:
                raise ValueError(f'仅位置求解未收敛：位置残差 {result.error*1000:.2f} mm。可能超出可达范围，或当前姿态下求解失败；这不等于已证明目标不可达。')
            raise ValueError('保持当前朝向时未找到可行逆解。位置可能可达，但朝向约束或关节限位阻止了求解；可在末端调试中改用“只指定位置”。')
        self.check_path(seed, result.q)
        return result.q

    def contacts(self, q, finger=None):
        probe = mj.MjData(self.m)
        probe.qpos[:] = self.d.qpos
        probe.qpos[self.qadr] = -q
        if finger is not None:
            probe.qpos[self.finger_qadr] = finger
        mj.mj_forward(self.m, probe)
        return [(int(c.geom1), int(c.geom2), float(c.dist)) for c in probe.contact if c.dist < -1e-5 and (self.m.geom_bodyid[c.geom1] in self.robot_bodies or self.m.geom_bodyid[c.geom2] in self.robot_bodies)]

    def check_path(self, start, end):
        count = max(25, int(np.max(np.abs(end-start)) / .02) + 1)
        for t in np.linspace(0, 1, count):
            if self.contacts(start*(1-t)+end*t):
                raise ValueError('这段关节插值路径会碰撞；当前演示尚不包含绕障规划')

    def trajectory(self, kind='joints', seconds=10, hz=50):
        t = np.linspace(0, seconds, int(seconds*hz)+1)
        if kind == 'circle':
            pose = self.fk(HOME); seed = HOME.copy(); qs=[]
            for ti in t:
                theta = 2*np.pi*ti/seconds
                xyz=pose.translation + np.array([0,.025*np.sin(theta),.025*(1-np.cos(theta))])
                seed=self.ik(xyz,pose.rotation,seed); qs.append(seed.copy())
            qs=np.array(qs)
        else:
            amp=np.array([.18,.10,.12,.16,.12,.20])
            phase=2*np.pi*t/seconds
            qs=HOME + np.sin(phase)[:,None]**3*amp
        fingers=.025-.015*np.cos(2*np.pi*t/seconds)
        for q,f in zip(qs,fingers):
            if self.contacts(q,f): raise ValueError('演示轨迹包含碰撞')
        return t, qs, fingers

    def status(self):
        pose=self.fk(self.q)
        return dict(q_deg=np.rad2deg(self.q).tolist(),target_deg=np.rad2deg(self.target).tolist(),
                    q_rad=self.q.tolist(),target_rad=self.target.tolist(),velocity_rad_s=(-self.d.qvel[self.vadr]).tolist(),
                    finger_positions_m=self.d.qpos[self.finger_qadr].tolist(),finger_target_m=self.finger_target,
                    xyz=pose.translation.tolist(),finger_m=float(np.mean(self.d.qpos[self.finger_qadr])),
                    sim_time=float(self.d.time),contacts=int(self.d.ncon))
