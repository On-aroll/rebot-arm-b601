"""Built-in/local HTTP feedback controllers never own CAN or a lease."""
import copy
import http.client
import json
import secrets
import threading
import time
from urllib.parse import urlsplit

import numpy as np

from .twin_servo import ENGINES, JointServo, parameters as servo_parameters
from .twin_jog import JogRamp
from .twin_motion import RuckigVelocity
from .twin_program import CALIBRATION, number, vector

SCHEMA = 'b601-controller/1.0'


def configuration(raw):
    if not isinstance(raw, dict) or raw.get('schema') != SCHEMA:
        raise ValueError('控制算法配置 schema 必须为 b601-controller/1.0')
    if set(raw) - {'schema', 'name', 'endpoint', 'rate_hz', 'output', 'parameters', 'engine'}:
        raise ValueError('控制算法配置包含未知字段')
    engine = raw.get('engine', 'external')
    if engine not in (*ENGINES, 'external'):
        raise ValueError(f'控制器仅支持 {" / ".join((*ENGINES,"external"))}')
    url = urlsplit(str(raw.get('endpoint', '')))
    if engine == 'external' and (url.scheme != 'http' or url.hostname not in ('127.0.0.1', '::1') or
            url.username or url.password or url.fragment or url.query or not url.port):
        raise ValueError('控制计算服务须位于机器人主机回环HTTP地址并指定端口，例如 http://127.0.0.1:8089/step')
    output = raw.get('output')
    if output not in ('joint_position', 'joint_velocity'):
        raise ValueError('上层控制器支持关节位置/速度参考；不支持直接力矩输出')
    parameters = raw.get('parameters', {})
    if not isinstance(parameters, dict) or len(json.dumps(parameters, allow_nan=False)) > 16384:
        raise ValueError('算法参数须为不超过16KB的有限JSON对象')
    if engine != 'external':
        if output != 'joint_velocity' or raw.get('endpoint'):
            raise ValueError('内置控制器只输出速度参考，不能配置外部地址')
        parameters = servo_parameters(engine, parameters)
    return dict(schema=SCHEMA, engine=engine, name=str(raw.get('name', '外部控制器'))[:80], endpoint=url.geturl(),
                rate_hz=number(raw.get('rate_hz', 10), '算法请求频率', 5, 20), output=output,
                parameters=copy.deepcopy(parameters))


def exchange(config, payload):
    """No proxy, DNS, redirect, retry, script loading, or actuator API."""
    url = urlsplit(config['endpoint'])
    conn = http.client.HTTPConnection(url.hostname, url.port, timeout=.18)
    try:
        conn.request('POST', url.path or '/', json.dumps(payload, allow_nan=False),
                     {'Content-Type': 'application/json'})
        response = conn.getresponse()
        if response.status != 200:
            raise ValueError(f'控制服务HTTP {response.status}；不跟随重定向')
        data = response.read(32769)
        if len(data) > 32768:
            raise ValueError('控制响应超过32KB')
        return json.loads(data)
    finally:
        conn.close()


class TwinAlgorithm:
    DEADLINE = .20
    MAX_AGE = .25

    def __init__(self, twin):
        self.t = twin
        self.generation = 0
        self.active = False
        self.pending = False
        self.checked = None
        self.latest = None
        self.config = None
        self.servo = None
        self.metric_sum = np.zeros(6)
        self.metric_peak = np.zeros(6)
        self.metric_count = 0
        self.metric_stamp = None
        self.ramp = JogRamp()
        self.ruckig_ramp = RuckigVelocity()
        self.stats = dict(status='idle', message='尚未接入控制算法', cycles=0, latency_ms=None,
                          requested_hz=None, applied_hz=None, requested=None, applied=None)
        self.next_at = self.last_apply = self.sent_at = 0.
        self.started = 0.
        self.applied_count = 0
        self.worker = None

    def state(self):
        return dict(self.stats, active=self.active, checking=self.pending and not self.active,
                    diagnostics=self.servo.diagnostics if self.servo else self.stats.get('diagnostics'),
                    name=(self.config or {}).get('name'), output=(self.config or {}).get('output'),
                    engine=(self.config or {}).get('engine'), stale_limit_ms=self.MAX_AGE*1000, deadline_ms=self.DEADLINE*1000)

    def stop(self, reason):
        if self.servo:self.stats['diagnostics']=dict(self.servo.diagnostics)
        self.ruckig_ramp.reset()
        was_active = self.active or self.pending
        self.generation += 1
        self.active = self.pending = False
        self.checked = self.latest = None
        self.ramp.reset()
        self.servo = None
        if was_active:
            self.stats.update(status='stopped', message=reason)

    def _settings(self, request):
        config = configuration(request.get('config'))
        target = vector(request.get('target'), 6, '控制器期望关节位置')
        speed = number(request.get('speed'), '速度', getattr(self.t.hw, 'min_twin_speed', .005),
                       getattr(self.t.hw, 'max_twin_speed', self.t.MAX_SPEED))
        limits = self.t.hw.teleop_limits
        if any(not lo-1e-9 <= q <= hi+1e-9 for q, (lo, hi) in zip(target, limits)):
            raise ValueError('控制器期望关节位置越过模型限位')
        target = [min(hi, max(lo, q)) for q, (lo, hi) in zip(target, limits)]
        return config, target, speed

    def _snapshot(self, dry_run):
        t = self.t
        q = t._positions()
        # MuJoCo velocities need to belong to the same feedback source as q.
        velocity = getattr(t, 'mujoco_velocity', None) if t._using_mujoco() else list(t.hw.get_joint_state(request_feedback=False)[1])
        feedback_at = t.mujoco_feedback_at if t._using_mujoco() else getattr(t.hw, '_feedback_refreshed_at', t.clock())
        now = t.clock()
        if self.active and feedback_at != self.metric_stamp:
            e = np.abs(np.asarray(self.target)-q)*180/np.pi
            self.metric_sum += e*e
            self.metric_peak = np.maximum(self.metric_peak, e)
            self.metric_count += 1
            self.metric_stamp = feedback_at
            self.stats['tracking'] = dict(samples=self.metric_count, rmse_deg=np.sqrt(self.metric_sum/self.metric_count).tolist(),
                peak_error_deg=self.metric_peak.tolist(), saturation_samples=self.servo.saturations if self.servo else None)
        profile=t._motion_profile()
        ramp=(getattr(t.hw,'stream_ruckig',self.ruckig_ramp) if profile == 'ruckig'
              else getattr(t.hw,'stream_ramp',self.ramp))
        applied_velocity=ramp.velocity.tolist() if self.active else [0.]*6
        return dict(schema=SCHEMA, request_id=secrets.token_hex(12), stamp_ms=time.time()*1000,
                    dry_run=dry_run, robot_model='b601_rs', namespace=t.namespace, calibration_id=CALIBRATION,
                    units=dict(position='rad', velocity='rad/s', time='s'),
                    state=dict(position=q, velocity=velocity, tcp_pose=t.kin.pose(q),
                               feedback_age_ms=max(0., (now-feedback_at)*1000), feedback_stamp_s=feedback_at),
                    reference=dict(position=self.target), applied_reference=t.reference, applied_velocity=applied_velocity,
                    period_s=1/self.config['rate_hz'], elapsed_s=0. if dry_run else now-self.started,
                    limits=dict(position=t.hw.teleop_limits, velocity=self.speed),
                    output=self.config['output'], parameters=self.config['parameters'])

    def _reply(self, row, payload):
        if not isinstance(row, dict) or set(row) != {'schema', 'request_id', 'output', 'values'}:
            raise ValueError('控制响应必须包含且仅包含schema/request_id/output/values')
        if row['schema'] != SCHEMA or row['request_id'] != payload['request_id'] or row['output'] != self.config['output']:
            raise ValueError('控制响应版本、请求标识或输出类型不匹配')
        values = vector(row['values'], 6, '控制算法输出')
        if row['output'] == 'joint_velocity':
            if max(abs(v) for v in values) > self.speed+1e-9:
                raise ValueError('控制算法输出速度超过本次输入上限')
        elif any(not lo-1e-9 <= q <= hi+1e-9 for q, (lo, hi) in zip(values, self.t.hw.teleop_limits)):
            raise ValueError('控制算法输出位置超出模型限位')
        return values

    def check(self, request):
        t = self.t
        t.extensions._idle()
        t._fresh()
        if self.worker and self.worker.is_alive():
            raise RuntimeError('上一控制计算尚未返回，请等待服务超时或重启该算法服务')
        self.config, self.target, self.speed = self._settings(request)
        self.servo = JointServo(self.config) if self.config['engine'] in ENGINES else None
        self.generation += 1
        self.checked = self.latest = None
        self.pending = True
        self.stats.update(status='checking', message='只读试算中，未发送运动', cycles=0)
        self._launch(request)

    def start(self, request):
        t = self.t
        t.extensions._idle()
        checked = self.checked
        if not checked or request.get('check_id') != checked['id'] or t.clock()-checked['at'] > 60:
            raise ValueError('控制算法试算已失效，请重新检查')
        if self.worker and self.worker.is_alive():
            raise RuntimeError('控制计算尚未退出，请稍后重试')
        fingerprint = getattr(getattr(t.hw, 'config', None), 'fingerprint', None)
        if checked['fingerprint'] != fingerprint or max(abs(a-b) for a, b in zip(t._positions(), checked['position'])) > .03:
            raise ValueError('试算起点或控制范围已变化，请重新检查')
        if hasattr(t.hw, 'motion_ready') and not t.hw.motion_ready:
            raise ValueError('实机请先准备、接管并启动电机保持，再试算并启动算法')
        self.active = True
        self.checked = self.latest = None
        self.ramp.reset()
        self.ruckig_ramp.reset()
        self.servo = JointServo(self.config) if self.config['engine'] in ENGINES else None
        self.metric_sum.fill(0.); self.metric_peak.fill(0.)
        self.metric_count = 0; self.metric_stamp = None
        self.stats['tracking'] = None
        self.stats['diagnostics'] = None
        self.started = self.last_apply = self.next_at = t.clock()
        self.applied_count = 0
        self.stats.update(status='running', message='闭环控制中', cycles=0, requested_hz=self.config['rate_hz'])
        t.plan = t.goal = t.reference = None
        t.hw.teleop_resume(t.session)
        t.mode, t.message = 'ALGORITHM', '反馈控制算法已启动，等待新鲜参考'

    def _launch(self, request=None):
        t = self.t
        payload = self._snapshot(request is not None)
        generation, session, config, servo = self.generation, t.session, self.config, self.servo
        sent_at = self.sent_at = t.clock()
        self.pending = True
        def work():
            try:
                row = (dict(schema=SCHEMA, request_id=payload['request_id'], output='joint_velocity',
                            values=servo.step(payload).tolist()) if servo else exchange(config, payload))
                with t.lock:
                    if generation != self.generation or session != t.session:
                        return
                    if t.clock()-sent_at > self.DEADLINE:
                        raise TimeoutError('控制算法响应超过200ms；参考未采用')
                    t._fresh()
                    values = self._reply(row, payload)
                    self.stats.update(latency_ms=round((t.clock()-sent_at)*1000, 2), requested=values)
                    if servo and self.stats.get('tracking'):
                        self.stats['tracking']['saturation_samples'] = servo.saturations
                    if request is not None:
                        self.checked = dict(id=secrets.token_hex(12), at=t.clock(), position=payload['state']['position'],
                                            fingerprint=getattr(getattr(t.hw, 'config', None), 'fingerprint', None))
                        self.stats.update(status='checked', message='协议与输出检查通过；这不是整个运行过程的路径预览')
                        t._event(request, True, self.stats['message'], check_id=self.checked['id'],
                                 controller=self.state())
                    else:
                        self.latest = dict(at=sent_at, values=values)
                        self.stats['cycles'] += 1
            except Exception as exc:
                with t.lock:
                    if generation != self.generation:
                        return
                    if self.active:
                        t._hold(f'控制算法已停止：{exc}')
                    self.stats.update(status='error', message=str(exc))
                    if request is not None:
                        t._event(request, False, str(exc))
            finally:
                with t.lock:
                    if generation == self.generation:
                        self.pending = False
        self.worker = threading.Thread(target=work, name='local-control-algorithm', daemon=True)
        self.worker.start()

    def tick(self):
        if not self.active:
            return
        t = self.t
        now = t.clock()
        if self.pending and now-self.sent_at > self.DEADLINE:
            t._hold('控制算法超过200ms未响应，保持当前位置')
            self.stats.update(status='error', message=t.message)
            return
        if self.latest and now-self.latest['at'] > self.MAX_AGE:
            t._hold('控制算法参考超过250ms未更新，保持当前位置')
            self.stats.update(status='error', message=t.message)
            return
        if now >= self.next_at and not self.pending:
            self.next_at = now+1/self.config['rate_hz']
            self._launch()
        if not self.latest:
            return
        dt = min(.05, max(0., now-self.last_apply))
        if dt <= 0:
            return
        seed = (getattr(t.hw,'command_reference',None) if getattr(t.hw,'continuous_jog_reference',False)
                else t.reference if t.reference is not None else getattr(t.hw, 'command_reference', None))
        seed = np.array(t._positions() if seed is None else seed)
        values = np.array(self.latest['values'])
        desired = values if self.config['output'] == 'joint_velocity' else (values-seed)/dt
        limits = np.array(t.hw.teleop_limits)
        acceleration, jerk = t._motion_limits()
        if t._motion_profile() == 'ruckig':
            if getattr(t.hw,'continuous_jog_reference',False):
                target = np.clip(seed+np.clip(desired,-self.speed,self.speed)*dt,
                                 np.minimum(seed,limits[:,0]),np.maximum(seed,limits[:,1]))
                velocity = (target-seed)/dt
            else:
                target = self.ruckig_ramp.step(seed,desired,dt,self.speed,acceleration,
                    limits[:,0],limits[:,1],jerk=jerk)
                velocity = self.ruckig_ramp.velocity
        else:
            target = self.ramp.step(seed, desired, dt, self.speed, acceleration, limits[:, 0], limits[:, 1], speed_fraction=1.)
            velocity = self.ramp.velocity
        # The shared hardware executor retains path, gravity, tracking and
        # current limits. The adapter does not reinterpret any of those limits.
        t._send(target.tolist(), self.speed, velocity=velocity.tolist())
        self.last_apply = now
        self.applied_count += 1
        self.stats.update(applied=target.tolist(), applied_hz=round(self.applied_count/max(.05, now-self.started), 1))
        t.mode, t.message = 'ALGORITHM', f'控制算法：{self.config["name"]}'
