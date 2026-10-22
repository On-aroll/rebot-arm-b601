"""Inert drive selection and MIT frame gains. Never constructs a CAN handle."""
import json
from pathlib import Path

from .twin_program import vector

# Seeed SDK 6415d431, config/rebotarm_rs.yaml. Gains are frame fields,
# not the drive's CSP position/velocity PID registers.
MIT_KP = [50., 150., 150., 50., 50., 50.]
MIT_KD = [3., 10., 10., 5., 4., 4.]
MIT_SOURCE = 'https://github.com/Seeed-Projects/reBotArm_control_py/blob/6415d43130d1e143c70dc106096a857ac5556f81/config/rebotarm_rs.yaml'


def mit_parameters(raw):
    if not isinstance(raw, dict) or set(raw) != {'kp', 'kd'}:
        raise ValueError('MIT参数需要六轴kp/kd；本入口固定速度前馈与力矩前馈为0')
    p = {key: vector(raw[key], 6, f'MIT {key}') for key in ('kp', 'kd')}
    for key, ceilings in (('kp', MIT_KP), ('kd', MIT_KD)):
        if any(not 0 < value <= ceiling for value, ceiling in zip(p[key], ceilings)):
            raise ValueError(f'MIT {key}须大于0且不超过锁定官方逐轴增益{ceilings}')
    return p


class DriveSettings:
    def __init__(self, path=None):
        self.path = Path(path) if path else None
        # Reconnecting never silently opts a physical arm into a new backend.
        self.selected = 'csp'
        self.parameters = mit_parameters(dict(kp=MIT_KP, kd=MIT_KD))
        if self.path and self.path.exists():
            row = json.loads(self.path.read_text())
            if row.get('schema') != 'b601-drive/1.0':
                raise ValueError('MIT参数档案版本无效，请核查drive-settings.json')
            self.parameters = mit_parameters(row.get('mit'))

    def apply(self, selected, parameters=None):
        if selected not in ('csp', 'mit'):
            raise ValueError('驱动模式只支持csp / mit')
        p = mit_parameters(self.parameters if parameters is None else parameters)
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(dict(schema='b601-drive/1.0', mit=p), allow_nan=False, indent=2)+'\n')
            temporary.replace(self.path)
        self.selected, self.parameters = selected, p

    def state(self):
        return dict(selected=self.selected, available=['csp', 'mit'], parameters=self.parameters,
                    source=MIT_SOURCE, velocity_feedforward=0., torque_feedforward=0.,
                    physical_validation='pending', reconnect_default='csp')
