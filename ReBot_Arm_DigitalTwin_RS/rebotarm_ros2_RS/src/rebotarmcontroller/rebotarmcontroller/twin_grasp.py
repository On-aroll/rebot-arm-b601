"""Calibrated motor-contact evidence; no bus access and no claim of object identity."""
import math


# The current thresholds and empty endpoint deliberately have no defaults.
PROFILE_LIMITS = {
    'empty_position': (0., 4.8), 'empty_tolerance': (.005, .1),
    'min_gap': (.02, .5), 'contact_current_a': (.01, 15.9),
    'abort_current_a': (.02, 16.), 'stall_velocity': (.001, .03),
    'min_travel': (.01, .5), 'dwell': (.3, 2.),
    'hold_time': (.3, 3.), 'timeout': (2., 180.),
}
MAX_GRASP_SPEED = .1
MAX_REFERENCE_LEAD = .02  # rad; inside the existing 3 degree tracking protection


def validate_profile(raw):
    if not isinstance(raw, dict) or set(raw) != set(PROFILE_LIMITS):
        raise ValueError('夹持标定需完整提供空夹位置、公差、间隙、电流及时间阈值')
    result = {}
    for key, (low, high) in PROFILE_LIMITS.items():
        value = raw[key]
        try:
            valid = type(value) in (int, float) and math.isfinite(value) and low <= value <= high
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f'夹持标定 {key} 需为 {low}–{high} 内有限数值')
        result[key] = float(value)
    if result['contact_current_a'] >= result['abort_current_a']:
        raise ValueError('接触电流必须小于中止电流')
    if result['stall_velocity'] >= MAX_GRASP_SPEED:
        raise ValueError('停滞速度阈值过高')
    if result['dwell'] + result['hold_time'] >= result['timeout']:
        raise ValueError('超时必须大于接触与保持确认时间')
    return result


class GraspMonitor:
    """Only distinct fresh samples can establish contact and sustained holding.

    The caller freezes its closing reference at the FIRST contact candidate.
    A noisy candidate fails instead of ratcheting further into an obstruction.
    """
    def __init__(self, profile, position, now, job_id):
        self.profile = validate_profile(profile)
        self.start = position
        self.began = now
        self.id = job_id
        self.status = 'closing'
        self.reason = '低速闭合，等待位置/速度/电流共同证据'
        self.last_stamp = None
        self.contact_at = None
        self.contact_position = None
        self.samples = 0
        self.evidence = {}
        self.bad_since = None
        p = self.profile
        if not math.isfinite(position) or not p['empty_position']+p['empty_tolerance']+p['min_gap']+p['min_travel'] <= position <= 5:
            raise ValueError('起点开度不足；先张开，不能从已堵转位置判定夹住')

    def state(self):
        return dict(id=self.id, status=self.status, message=self.reason,
                    evidence=dict(self.evidence), method='motor_contact', object_verified=False)

    def fail(self, message, status='failed'):
        self.status, self.reason = status, message

    def cancel(self):
        self.fail('夹持动作已中断，旧证据不能续跑', 'cancelled')

    def update(self, *, position, velocity, current, stamp, now, reference):
        if self.status in ('failed', 'cancelled'):return self.status
        values = (position, velocity, current, stamp, now, reference)
        if not all(type(x) in (int, float) and math.isfinite(x) for x in values):
            self.fail('夹持反馈包含无效数值');return self.status
        if not 0 <= now-stamp <= .6 or self.last_stamp is not None and stamp < self.last_stamp:
            self.fail('夹持反馈过期或时间倒退');return self.status
        if self.last_stamp == stamp:return self.status
        p = self.profile
        if self.last_stamp is not None and stamp-self.last_stamp > .6:
            self.fail('夹持采样中断，不能累积接触证据');return self.status
        self.last_stamp = stamp
        self.evidence = dict(position=position, velocity=velocity, current_a=current,
                             reference=reference, sample_at=stamp, elapsed_s=now-self.began)
        if not 0 <= position <= 5 or abs(current) >= p['abort_current_a']:
            self.fail('夹持位置越界或达到标定中止电流');return self.status
        if position <= p['empty_position']+p['empty_tolerance']:
            self.fail('空夹：已到空载闭合端点');return self.status
        if self.status != 'holding' and now-self.began > p['timeout']:
            self.fail('夹持超时，未获得持续接触证据');return self.status
        if self.status == 'holding':
            bad = (abs(position-self.contact_position) > p['min_gap']/2
                   or abs(current) < p['contact_current_a']/2
                   or abs(velocity) > p['stall_velocity'])
            self.bad_since = (self.bad_since if self.bad_since is not None else stamp) if bad else None
            if self.bad_since is not None and stamp-self.bad_since >= .2:
                self.fail('夹持保持证据丢失，疑似滑移或脱落')
            return self.status
        contact = (self.start-position >= p['min_travel']
                   and position > p['empty_position']+p['empty_tolerance']+p['min_gap']
                   and abs(velocity) <= p['stall_velocity']
                   and abs(current) >= p['contact_current_a']
                   and position-reference >= .002)
        if self.status == 'closing' and contact:
            self.status = 'contact'
            self.contact_at, self.contact_position, self.samples = stamp, position, 1
            self.reason = '检测到接触，冻结闭合参考并验证持续保持'
        elif self.status == 'contact':
            if not contact or abs(position-self.contact_position) > p['min_gap']/2:
                self.fail('接触证据不稳定，未判定夹持成功')
            else:
                self.samples += 1
                if self.samples >= 3 and stamp-self.contact_at >= p['dwell']+p['hold_time']:
                    self.status = 'holding'
                    self.reason = '电机接触与持续保持判定通过；仍需实物验证'
        self.evidence.update(contact_samples=self.samples, contact_position=self.contact_position)
        return self.status
