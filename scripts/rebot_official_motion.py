"""Thin binding to Seeed's MIT trajectory callback; never connects hardware.

The upstream callback already computes g(q) from measured positions and sends
q/dq/kp/kd/tau. Capture its output, then use our existing checked transport.
Do not invoke upstream start/end, mode changes, homing or group lifecycle.
"""
import hashlib
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
OFFICIAL=ROOT/'external/reBotArm_control_py'
URDF=OFFICIAL/'urdf/RS/urdf/ReBot_Arm_RS.urdf'
COMMIT='6415d43130d1e143c70dc106096a857ac5556f81'
MODE='seeed-rs-gravity-v1'


def policy():
    return dict(mode=MODE,source_commit=COMMIT,model_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest())


class OfficialMIT:
    def __init__(self,config,model=None):
        if config.feedforward is None: raise ValueError('需要明确选择官方重力前馈')
        if config.feedforward.model_sha256!=policy()['model_sha256']:
            raise ValueError('RS 动力学模型已变化，请重新预演')
        # Same locked repository check used by the existing SDK adapter.
        from rebot_seeed_transport import official_module
        official_module()
        sys.path.insert(0,str(OFFICIAL)) if str(OFFICIAL) not in sys.path else None
        import numpy as np
        from reBotArm_control_py.controllers.rebotarm_endpose_controller import RebotArmEndPose
        from reBotArm_control_py.kinematics import load_robot_model
        ctrl=RebotArmEndPose.__new__(RebotArmEndPose)
        ctrl._n=6;ctrl._model=model if model is not None else load_robot_model(str(URDF))
        ctrl._data=ctrl._model.createData();ctrl._arm_control_mode='mit';ctrl._use_gravity_ff=True;ctrl._has_gripper=False
        binding=self
        class CapturedGroup:
            _mit_kp=np.asarray([a.kp for a in config.axes]);_mit_kd=np.asarray([a.kd for a in config.axes])
            def get_positions(self,request_feedback=False): return binding.measured.copy()
            def send_mit(self,pos,vel,kp,kd,tau):
                binding.output=[tuple(float(x) for x in row) for row in zip(pos,vel,kp,kd,tau)]
        ctrl._arm_group=CapturedGroup();self.ctrl=ctrl;self.measured=np.zeros(6);self.output=None

    def commands(self,target,velocity,measured):
        import numpy as np
        for value in (target,velocity,measured):
            if np.asarray(value).shape!=(6,) or not np.all(np.isfinite(value)):
                raise ValueError('官方 MIT 控制需要完整有限的六轴目标与反馈')
        self.measured=np.asarray(measured,dtype=float)
        self.ctrl._q_target=np.asarray(target,dtype=float);self.ctrl._qd_target=np.asarray(velocity,dtype=float)
        self.output=None
        self.ctrl._loop_cb(None,0.)
        if self.output is None or np.asarray(self.output).shape!=(6,5) or not np.all(np.isfinite(self.output)):
            raise ValueError('官方 MIT 重力前馈计算无效')
        return self.output
