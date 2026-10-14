"""Thin binding to Seeed's MIT trajectory callback; never connects hardware.

The upstream callback already computes g(q) from measured positions and sends
q/dq/kp/kd/tau. Capture its output, then use our existing checked transport.
Do not invoke upstream start/end, mode changes, homing or group lifecycle.
"""
import hashlib
from pathlib import Path
import sys
import subprocess

ROOT=Path(__file__).resolve().parents[1]
OFFICIAL=ROOT/'external/reBotArm_control_py'
URDF=OFFICIAL/'urdf/RS/urdf/ReBot_Arm_RS.urdf'
COMMIT='6415d43130d1e143c70dc106096a857ac5556f81'
MODE='seeed-rs-gravity-v1'
GAIN_MODE='seeed-rs-bounded-gains-v1'
SLOW_MODE='seeed-rs-slow-position-v1'
CONFIG=OFFICIAL/'config/rebotarm_rs.yaml'
REFERENCE=ROOT/'external/ReBot_Arm_DigitalTwin_RS'
REFERENCE_COMMIT='aa807cf2affe85b13b86221159871aae7742c01b'
REFERENCE_CONFIG=REFERENCE/'rebotarm_ros2_RS/src/rebotarm_bringup/config/rebotarm_hardware.yaml'


def policy():
    return dict(mode=MODE,source_commit=COMMIT,model_sha256=hashlib.sha256(URDF.read_bytes()).hexdigest())


def gain_policy():
    return dict(mode=GAIN_MODE,source_commit=COMMIT,config_sha256=hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
        reference_commit=REFERENCE_COMMIT,reference_config_sha256=hashlib.sha256(REFERENCE_CONFIG.read_bytes()).hexdigest(),
        startup_gravity_ramp_s=8.)


def official_gain_limits():
    import yaml
    data=yaml.safe_load(CONFIG.read_text())
    joints=[j for j in data['joints'] if j['name'] in {f'joint{i}' for i in range(1,7)}]
    if [j['name'] for j in joints]!=[f'joint{i}' for i in range(1,7)]:
        raise ValueError('官方六轴增益配置映射变化')
    caps=[(float(j['MIT']['kp']),float(j['MIT']['kd'])) for j in joints]
    if subprocess.check_output(['git','-C',str(REFERENCE),'rev-parse','HEAD'],text=True).strip()!=REFERENCE_COMMIT:
        raise ValueError('参考 ROS 工程版本已变化，增益需重新审核')
    if subprocess.check_output(['git','-C',str(REFERENCE),'diff','HEAD','--',str(REFERENCE_CONFIG.relative_to(REFERENCE))],text=True):
        raise ValueError('参考 ROS 增益配置被修改')
    control=yaml.safe_load(REFERENCE_CONFIG.read_text())['models']['rs']['overrides']['control']
    # Its measured J1 friction fix is 80/5; other axes retain Seeed caps.
    caps[0]=(max(caps[0][0],float(control['mit_kp'][0])),max(caps[0][1],float(control['mit_kd'][0])))
    return caps


def apply_slow_gains(raw):
    # Per-task values within the pinned SDK/reference position-loop gains. Never stored
    # in the drive or substituted for the user's saved software draft.
    raw['gain_review']=gain_policy()
    for axis,kp,kd in zip(raw['axes'],(80.,75.,150.,50.,40.,40.),(5.,6.,10.,5.,4.,4.)):
        axis.update(kp=kp,kd=kd,kp_max=kp,kd_max=kd)
    raw['axes'][0]['error_max']=min(raw['axes'][0]['error_max'],0.6*3.141592653589793/180.)
    # Reserve actual torque headroom instead of raising the existing 3 Nm cap.
    raw['axes'][1]['error_max']=min(raw['axes'][1]['error_max'],0.75*3.141592653589793/180.)
    # J3/J4 now use the actual SDK position-loop gains. Tighten their tracking
    # bounds to keep the existing 9/3 Nm effort ceilings, rather than raise them.
    raw['axes'][2]['error_max']=min(raw['axes'][2]['error_max'],0.6*3.141592653589793/180.)
    raw['axes'][3]['error_max']=min(raw['axes'][3]['error_max'],0.75*3.141592653589793/180.)


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
        from reBotArm_control_py.controllers import GravityCompensation
        self.transition=GravityCompensation.__new__(GravityCompensation)
        self.transition._transition_duration=config.gain_review.startup_gravity_ramp_s if config.gain_review else 0.

    def startup_blend(self,elapsed):
        # Reuse Seeed's pure smoothstep helper for a supported torque fade-in.
        # Upstream uses this shape for gain blending; this adaptation scales tau.
        return self.transition._transition_blend(elapsed)

    def commands(self,target,velocity,measured,gravity_scale=1.):
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
        if not np.isfinite(gravity_scale) or not 0<=gravity_scale<=1:
            raise ValueError('重力渐变系数必须在 0～1 之间')
        return [(*row[:4],row[4]*gravity_scale) for row in self.output]
