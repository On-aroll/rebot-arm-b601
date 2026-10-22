"""Joint position error to bounded velocity. No motor register or torque API."""
import numpy as np

from .twin_program import number, vector

ENGINES = ('p', 'pd', 'pid', 'scheduled_pid', 'ladrc', 'mpc')


def parameters(engine, raw):
    if engine in ('ladrc', 'mpc'):
        fields = ({'control_bandwidth':(.6,.1,3.), 'observer_bandwidth':(3.,.5,12.),
                   'input_gain':(1.,.2,2.), 'disturbance_limit':(.03,0.,.2)} if engine == 'ladrc' else
                  {'horizon':(12,4,24), 'position_weight':(1.,.01,100.),
                   'velocity_weight':(.05,.001,10.), 'delta_weight':(.1,.001,10.)})
        if not isinstance(raw, dict) or set(raw)-set(fields):
            raise ValueError(f'{engine}包含未知参数，只支持{list(fields)}')
        result = {key:number(raw.get(key,default),key,low,high) for key,(default,low,high) in fields.items()}
        if engine == 'ladrc' and result['observer_bandwidth'] < 2*result['control_bandwidth']:
            raise ValueError('观测带宽至少为控制带宽的2倍；增大带宽会放大反馈噪声')
        if engine == 'mpc':
            if int(result['horizon']) != result['horizon']:raise ValueError('预测步数必须为整数')
            result['horizon'] = int(result['horizon'])
        return result
    allowed = {'kp', 'ki', 'kd', 'derivative_filter_s', 'integral_limit_rad_s'}
    if engine == 'scheduled_pid':allowed.update(('schedule_ratio','schedule_error_rad'))
    if not isinstance(raw, dict) or set(raw)-allowed:
        raise ValueError('内置控制器参数只支持 kp/ki/kd/derivative_filter_s/integral_limit_rad_s')
    result = {}
    for key, default, maximum in [('kp', .6, 10.), ('ki', .02 if engine in ('pid','scheduled_pid') else 0., 10.),
                                   ('kd', .03 if engine != 'p' else 0., 5.)]:
        values = raw.get(key, [default]*6)
        if isinstance(values, (int, float)) and not isinstance(values, bool): values = [values]*6
        result[key] = vector(values, 6, key)
        if any(v < 0 or v > maximum for v in result[key]):
            raise ValueError(f'{key} 必须在0–{maximum:g}之间')
    if (engine not in ('pid','scheduled_pid') and any(result['ki'])) or (engine == 'p' and any(result['kd'])):
        raise ValueError('P不使用Ki/Kd；PD不使用Ki，请选择对应控制器')
    result['derivative_filter_s'] = number(raw.get('derivative_filter_s', .1), '微分滤波时间', .02, 1.)
    result['integral_limit_rad_s'] = number(raw.get('integral_limit_rad_s', .03), '积分速度贡献上限', 0., .2)
    if engine == 'scheduled_pid':
        result['schedule_ratio'] = number(raw.get('schedule_ratio',1.5),'远端Kp倍率',1.,3.)
        result['schedule_error_rad'] = number(raw.get('schedule_error_rad',.1),'调度误差尺度',.002,1.)
        if max(result['kp'])*result['schedule_ratio'] > 10.:raise ValueError('调度后Kp也不能超过10')
    return result


class JointServo:
    def __init__(self, config):
        self.engine = config['engine']
        self.p = parameters(config['engine'], config['parameters'])
        self.integral = np.zeros(6)
        self.filtered_velocity = np.zeros(6)
        self.last_stamp = None
        self.last_values = None
        self.saturations = 0
        self.estimate = None
        self.disturbance = np.zeros(6)
        self.diagnostics = {}

    def step(self, payload):
        q = np.asarray(vector(payload['state']['position'], 6, '反馈位置'))
        v = np.asarray(vector(payload['state']['velocity'], 6, '反馈速度'))
        error = np.asarray(payload['reference']['position'])-q
        stamp = payload['state']['feedback_stamp_s']
        speed = payload['limits']['velocity']
        if self.last_stamp is not None and stamp == self.last_stamp:
            return self.last_values.copy()  # Duplicate ROS ticks must not integrate old feedback.
        dt = 0. if self.last_stamp is None else stamp-self.last_stamp
        if dt < 0 or dt > .25:
            self.integral.fill(0.)
            self.estimate = None
            self.disturbance.fill(0.)
            dt = 0.
        if self.engine == 'ladrc':
            # Discrete first-order ESO for q_dot=b0*u+d. Observer poles are
            # exp(-omega_o*dt), avoiding an unstable Euler bandwidth setting.
            applied = np.asarray(vector(payload['applied_velocity'],6,'已施加速度参考'))
            if self.estimate is None:self.estimate=q.copy()
            elif dt > 0:
                pole = np.exp(-self.p['observer_bandwidth']*dt)
                prediction = self.estimate+dt*(self.p['input_gain']*applied+self.disturbance)
                innovation = q-prediction
                self.estimate = prediction+(1-pole*pole)*innovation
                self.disturbance = np.clip(self.disturbance+(1-pole)**2/dt*innovation,
                                           -self.p['disturbance_limit'],self.p['disturbance_limit'])
            raw = (self.p['control_bandwidth']*(np.asarray(payload['reference']['position'])-self.estimate)-self.disturbance)/self.p['input_gain']
            self.diagnostics = dict(disturbance_rad_s=self.disturbance.tolist(), estimated_position=self.estimate.tolist())
        elif self.engine == 'mpc':
            raw = self._mpc(q, payload)
        else:
            raw = self._pid(error,v,dt,speed)
        self.saturations += int(np.any(np.abs(raw) > speed+1e-12))
        self.last_values = np.clip(raw, -speed, speed)
        self.last_stamp = stamp
        return self.last_values.copy()

    def _pid(self,error,v,dt,speed):
        alpha = 1. if dt == 0 else dt/(self.p['derivative_filter_s']+dt)
        self.filtered_velocity += alpha*(v-self.filtered_velocity)
        kp = np.asarray(self.p['kp'])
        if self.engine == 'scheduled_pid':
            ratio=np.clip(np.abs(error)/self.p['schedule_error_rad'],0.,1.)
            blend=ratio*ratio*(3-2*ratio)
            kp=kp*(1+(self.p['schedule_ratio']-1)*blend)
            self.diagnostics = dict(effective_kp=kp.tolist())
        base = kp*error-np.asarray(self.p['kd'])*self.filtered_velocity
        candidate = np.clip(self.integral+np.asarray(self.p['ki'])*error*dt,
                            -self.p['integral_limit_rad_s'], self.p['integral_limit_rad_s'])
        # Conditional integration: retain unwind, reject growth into saturation.
        reject = ((base+candidate > speed) & (candidate > self.integral)) | ((base+candidate < -speed) & (candidate < self.integral))
        self.integral = np.where(reject, self.integral, candidate)
        return base+self.integral

    def _mpc(self,q,payload):
        n=self.p['horizon'];dt=payload['period_s']
        a=np.tril(np.ones((n,n)))*dt
        d=np.eye(n)-np.eye(n,k=-1)
        h=self.p['position_weight']*(a.T@a)+self.p['velocity_weight']*np.eye(n)+self.p['delta_weight']*(d.T@d)
        previous=np.asarray(vector(payload['applied_velocity'],6,'已施加速度参考'))
        linear=self.p['position_weight']*a.T@np.tile(q-np.asarray(payload['reference']['position']),(n,1))
        linear[0]-=self.p['delta_weight']*previous
        bounds=np.asarray(payload['limits']['position'])
        # Conservative box constraints guarantee every predicted position is
        # within [min(q,lower),max(q,upper)] for this integrator model. The
        # shared executor independently checks actual path, jerk and tracking.
        low=np.maximum(-payload['limits']['velocity'],(np.minimum(q,bounds[:,0])-q)/(n*dt))
        high=np.minimum(payload['limits']['velocity'],(np.maximum(q,bounds[:,1])-q)/(n*dt))
        u=np.zeros((n,6))
        residual=float('inf')
        for iteration in range(200):
            for j in range(n):u[j]=np.clip(u[j]-(h[j]@u+linear[j])/h[j,j],low,high)
            projected=u-np.clip(u-(h@u+linear)/np.diag(h)[:,None],low,high)
            residual=float(np.max(np.abs(projected)))
            if residual <= 1e-5:break
        if residual > 1e-5:raise RuntimeError('MPC在200次迭代内未收敛，未采用参考')
        predicted=q+a@u
        self.diagnostics=dict(iterations=iteration+1,projected_residual=residual,
                              predicted_min=predicted.min(axis=0).tolist(),predicted_max=predicted.max(axis=0).tolist(),
                              model='q[k+1]=q[k]+dt*v_reference[k]; not identified arm dynamics')
        return u[0]
