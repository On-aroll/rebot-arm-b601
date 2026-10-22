"""Host trajectory tuning, always a reduction of the existing device envelope."""
import json
import math
from pathlib import Path

from .twin_motion import PROFILES


def tuning(raw):
    if not isinstance(raw, dict) or set(raw) != {'acceleration_scale', 'jerk_scale'}:
        raise ValueError('轨迹参数须包含 acceleration_scale / jerk_scale')
    for value in raw.values():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not .05 <= value <= 1:
            raise ValueError('加速度与jerk比例须在0.05–1之间，不可提高设备包络')
    return dict(raw)


class MotionSettings:
    def __init__(self, path=None):
        self.path = Path(path) if path else None
        self.selected = 'ruckig'
        self.profiles = {p: dict(acceleration_scale=1., jerk_scale=1.) for p in PROFILES}
        if self.path and self.path.exists():
            row = json.loads(self.path.read_text())
            if row.get('schema') != 'b601-motion/1.0' or row.get('selected') not in PROFILES or set(row.get('profiles', {})) != set(PROFILES):
                raise ValueError('运动参数档案无效，请核查 motion-settings.json')
            self.profiles = {p: tuning(row['profiles'][p]) for p in PROFILES}
            self.selected = row['selected']

    def apply(self, profile, parameters=None):
        if profile not in PROFILES:
            raise ValueError('未知运动方案')
        profiles = {p: dict(v) for p, v in self.profiles.items()}
        if parameters is not None:
            profiles[profile] = tuning(parameters)
        row = dict(schema='b601-motion/1.0', selected=profile, profiles=profiles)
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(row, indent=2, allow_nan=False)+'\n')
            temporary.replace(self.path)
        self.selected, self.profiles = profile, profiles

    def limits(self, acceleration, jerk, *, small=False):
        p = self.profiles[self.selected] if not small else dict(acceleration_scale=1., jerk_scale=1.)
        return acceleration*p['acceleration_scale'], jerk*p['jerk_scale']
