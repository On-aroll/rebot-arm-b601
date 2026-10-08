"""Simulation action programs. Targets and measured states remain distinct.

The only implemented backend is MuJoCo. No hardware transport is enabled here.
All tick/control calls must be serialized by the owning simulator thread/lock.
"""
import copy
import json
import time
import uuid
from pathlib import Path

import numpy as np

from rebot_core import Robot, ROOT

DOWN = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)
ACTIVE = {'running', 'paused'}


def number(value, lo, hi, label):
    value = float(value)
    if not np.isfinite(value) or not lo <= value <= hi:
        raise ValueError(f'{label} 必须在 {lo}–{hi} 范围内')
    return value


def vector(value, size, label):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.all(np.isfinite(a)):
        raise ValueError(f'{label} 需要 {size} 个有限数值')
    return a


def pick_place_steps(source=(.25, 0, .05), destination=(.25, .065, .05)):
    """40 mm cube, 30 g; positions are cube centers in the simulation base frame."""
    src, dst = vector(source, 3, 'source'), vector(destination, 3, 'destination')
    def move(name, pos):
        return dict(op='move_tcp', name=name, xyz=pos.tolist(), orientation='down', seconds=3)
    def check(name, pos, tol):
        return dict(op='assert_object', name=name, object='block', xyz=pos.tolist(), tolerance_m=tol)
    return [
        check('确认物体起点', src, .012),
        dict(op='gripper', name='张开夹爪', finger_m=.032, seconds=1, completion='position'),
        move('接近物体', src + [0, 0, .09]),
        move('下降抓取', src + [0, 0, -.01]),
        dict(op='gripper', name='闭合夹爪', finger_m=.016, seconds=2, completion='contact'),
        move('抬升', src + [0, 0, .09]),
        dict(op='wait', name='保持抓取', seconds=1),
        check('检查物体已抬起', src + [0, 0, .10], .025),
        move('搬运', dst + [0, 0, .09]),
        move('下降放置', dst + [0, 0, -.01]),
        dict(op='gripper', name='释放物体', finger_m=.032, seconds=2, completion='position'),
        move('撤离', dst + [0, 0, .09]),
        check('检查放置结果', dst, .012),
    ]


def compile_program(document, robot):
    """Validate the entire program before taking ownership of the command stream.

    Robot-only geometry is checked during IK. Scene collisions are checked against
    measured contacts during execution; this is not an obstacle-avoiding planner.
    """
    if not isinstance(document, dict) or document.get('version') != 1:
        raise ValueError('动作程序需要 version: 1')
    if set(document) - {'version', 'name', 'steps'}:
        raise ValueError('动作程序包含未知字段')
    if not isinstance(document.get('name', ''), str) or len(document.get('name', '')) > 120:
        raise ValueError('程序名称过长或不是文本')
    steps = document.get('steps')
    if not isinstance(steps, list) or not 1 <= len(steps) <= 200:
        raise ValueError('动作程序需要 1–200 个步骤')
    if sum(isinstance(s,dict) and s.get('op')=='trace_path' for s in steps)>1:
        raise ValueError('每个程序目前只接受一条连续 trace_path；请把连线合并进同一条路径')
    planner = Robot()
    seed = robot.q
    finger = float(np.mean(robot.d.qpos[robot.finger_qadr]))
    compiled = []
    total = 0.
    fields = {
        'move_joints': {'q_deg', 'seconds'},
        'move_tcp': {'xyz', 'orientation', 'seconds'},
        'gripper': {'finger_m', 'seconds', 'completion'},
        'wait': {'seconds'},
        'trace_path': {'points', 'styles', 'orientation', 'speed_m_s'},
        'assert_object': {'object', 'xyz', 'tolerance_m'},
    }
    for i, raw in enumerate(steps):
        if not isinstance(raw, dict) or raw.get('op') not in fields:
            raise ValueError(f'第 {i+1} 步：未知操作')
        op = raw['op']
        if set(raw) - fields[op] - {'op', 'name'}:
            raise ValueError(f'第 {i+1} 步：未知字段')
        label = raw.get('name', op)
        if not isinstance(label, str) or len(label) > 120:
            raise ValueError('步骤名称过长或不是文本')
        trace = None
        target = seed.copy()
        duration = number(raw.get('seconds', 1), .1, 60, 'seconds')
        completion = 'position'
        if op == 'move_joints':
            target = np.deg2rad(vector(raw['q_deg'], 6, 'q_deg'))
            planner.command(target, finger)
            planner.check_path(seed, target)
        elif op == 'move_tcp':
            from rebot_writing import orientation
            rotation = orientation(raw.get('orientation', 'keep'), planner, seed)
            target = planner.ik(vector(raw['xyz'], 3, 'xyz'), rotation, seed)
        elif op == 'trace_path':
            from rebot_writing import compile_trace
            trace = compile_trace(raw, planner, seed)
            target = trace['target'];duration = trace['seconds']
        elif op == 'gripper':
            finger = number(raw['finger_m'], 0, .05, 'finger_m')
            completion = raw.get('completion', 'position')
            if completion not in ('contact', 'position'):
                raise ValueError('夹爪 completion 必须为 contact / position')
        elif op == 'assert_object':
            if raw.get('object') != 'block' or robot.m.body('block').id < 0:
                raise ValueError('当前场景仅提供 block 物体断言')
            vector(raw['xyz'], 3, 'xyz')
            number(raw['tolerance_m'], .001, .05, 'tolerance_m')
            duration = 0
        # Smoothstep peak speed is 1.5 * displacement / duration.
        if op in ('move_tcp', 'move_joints'):
            duration = max(duration, float(1.5 * np.max(np.abs(target-seed)) / .7))
        total += duration + (3 if op != 'assert_object' else 0)
        if total > 1200:
            raise ValueError('动作程序总时长过长，请拆分')
        compiled.append(dict(op=op, name=label, target=target.copy(), finger=finger,
                             seconds=duration, completion=completion, raw=copy.deepcopy(raw), trace=trace))
        seed = target
    return compiled


class ProgramRunner:
    def __init__(self, robot, output_dir=None):
        from rebot_writing import Trail
        self.trail = Trail()
        self.robot = robot
        self.output_dir = Path(output_dir or ROOT / 'outputs/jobs')
        self.job = None
        self.steps = []
        self.index = 0
        self.elapsed = 0.
        self.stable = 0.
        self._log_clock = 0.
        self.history = []
        # Jobs cannot silently resume after losing their simulator state.
        if self.output_dir.exists():
            for path in self.output_dir.glob('*.json'):
                report = json.loads(path.read_text())
                if report.get('status') in ACTIVE:
                    report.update(status='failed', error='上次仿真服务中断，程序不能自动续跑', finished_at=time.time())
                    temp = path.with_suffix('.tmp')
                    temp.write_text(json.dumps(report, ensure_ascii=False, indent=2))
                    temp.replace(path)

    @property
    def active(self):
        return self.job is not None and self.job['status'] in ACTIVE

    def submit(self, document, *, compiled=None):
        if self.active:
            raise ValueError('已有动作程序执行中；请先取消或等待完成')
        if compiled is None:
            compiled = compile_program(document, self.robot)
        self.trail.clear()
        self.steps = compiled
        self.index = 0
        self.history = []
        self.job = dict(id=uuid.uuid4().hex, name=document.get('name', '动作程序'),
                        status='running', backend='mujoco', feedback_source='simulation',
                        started_at=time.time(), finished_at=None, error='',
                        step_count=len(compiled), completed_steps=[], program=copy.deepcopy(document))
        self.warning_baseline = self.robot.d.warning.number.copy()
        self._log_clock = 0.
        self._begin()
        self._save()
        return self.snapshot()

    def _begin(self):
        self.elapsed = 0.
        self.stable = 0.
        self.q0 = self.robot.motion_start()
        self.f0 = float(np.mean(self.robot.d.qpos[self.robot.finger_qadr]))
        self.duration = self.steps[self.index]['seconds']
        self.trace_style = 0
        if self.steps[self.index]['op']=='trace_path':
            self.trail.enabled=True
            self.trail.sample(self.robot.fk(self.robot.q).translation,0)

    def snapshot(self):
        if self.job is None:
            return None
        result = {k: copy.deepcopy(v) for k, v in self.job.items() if k != 'program'}
        result.update(step_index=self.index, step_name=self.steps[self.index]['name'],
                      step_elapsed_s=float(self.elapsed), step_duration_s=float(self.duration),
                      trace_points=len(self.trail.points),trace_length_m=self.trail.length)
        return result

    def _hold(self):
        # Keep gripper force/target to avoid dropping a held object on pause/cancel.
        self.robot.command(self.robot.motion_start(), self.robot.finger_target)

    def _save(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        path = self.output_dir / (self.job['id'] + '.json')
        payload = self.snapshot()
        payload.update(program=self.job['program'], samples=self.history)
        if self.trail.points:
            payload.update(trace_xyz=[p.tolist() for p in self.trail.points],trace_styles=self.trail.styles)
        temp = path.with_suffix('.tmp')
        temp.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
        temp.replace(path)

    def _finish(self, status, error=''):
        self.job.update(status=status, error=error, finished_at=time.time())
        self._hold()
        self._record()
        self._save()

    def fail(self, reason):
        if self.active:
            self._finish('failed', str(reason))

    def cancel(self):
        if not self.active:
            raise ValueError('没有正在执行的程序')
        self._finish('cancelled', '用户取消')

    def pause(self):
        if not self.active or self.job['status'] != 'running':
            raise ValueError('没有可以暂停的程序')
        self.job['status'] = 'paused'
        self.pause_elapsed=self.elapsed
        self.remaining = max(0., self.duration - self.elapsed)
        self._hold()
        self._save()

    def resume(self):
        if not self.active or self.job['status'] != 'paused':
            raise ValueError('没有已暂停的程序')
        self.job['status'] = 'running'
        if self.steps[self.index]['op']=='trace_path':
            self.elapsed=self.pause_elapsed;self.stable=0.
            self._save();return
        self._begin()
        self.duration = max(.1, self.remaining, float(1.5 * np.max(np.abs(self.steps[self.index]['target']-self.q0)) / .7))
        self._save()

    def _grasp_contact(self):
        r = self.robot
        block = r.m.body('block').id
        fingers = {r.m.body(n).id for n in ('gripper_left', 'gripper_right')}
        touching = set()
        for c in r.d.contact:
            bodies = {int(r.m.geom_bodyid[c.geom1]), int(r.m.geom_bodyid[c.geom2])}
            if block in bodies and c.dist <= .0005:
                touching |= bodies & fingers
        return touching == fingers

    def _check_collisions(self):
        r = self.robot
        fingers = {r.m.body(n).id for n in ('gripper_left', 'gripper_right')}
        block = r.m.body('block').id
        for c in r.d.contact:
            a, b = int(r.m.geom_bodyid[c.geom1]), int(r.m.geom_bodyid[c.geom2])
            if c.dist >= -.001 or not ({a, b} & r.robot_bodies):
                continue
            if block in (a, b) and ({a, b} & fingers):
                continue  # Declared grasp contact, not arbitrary obstacle contact.
            raise RuntimeError(f'非预期碰撞：{r.m.geom(c.geom1).name} / {r.m.geom(c.geom2).name}')

    def _record(self):
        r = self.robot
        self.history.append(dict(sim_time_s=float(r.d.time), step=self.index,
                                 target_rad=r.target.tolist(), actual_rad=r.q.tolist(),
                                 finger_target_m=float(r.finger_target),
                                 finger_actual_m=r.d.qpos[r.finger_qadr].tolist(),
                                 tcp_xyz=r.fk(r.q).translation.tolist(),
                                 block_xyz=r.d.xpos[r.m.body('block').id].tolist()))

    def after_step(self):
        if self.active and self.job['status']=='running' and self.steps[self.index]['op']=='trace_path':
            self.trail.sample(self.robot.fk(self.robot.q).translation,self.trace_style)

    def tick(self, dt):
        """Call once before each physics step, with the simulator time increment."""
        if not self.active:
            return
        try:
            self._check_collisions()
            if np.any(self.robot.d.warning.number > self.warning_baseline):
                raise RuntimeError('MuJoCo 报告物理求解警告')
            if self.job['status'] == 'paused':
                return
            self.elapsed += dt
            self._log_clock += dt
            step = self.steps[self.index]
            r = self.robot
            if step['op'] == 'assert_object':
                actual = r.d.xpos[r.m.body('block').id]
                if np.linalg.norm(actual - np.array(step['raw']['xyz'])) > step['raw']['tolerance_m']:
                    raise RuntimeError(f"{step['name']}失败：物体实际位置 {actual.tolist()}")
                done = True
            else:
                u = min(1., self.elapsed / self.duration)
                s = u*u*(3-2*u)
                if step['op']=='trace_path':
                    trace=step['trace'];t=min(self.elapsed,self.duration)
                    q=np.array([np.interp(t,trace['times'],trace['qs'][:,j]) for j in range(6)])
                    sample=min(int(np.searchsorted(trace['times'],t,side='right')),len(trace['times'])-1)
                    self.trace_style=trace['styles'][int(trace['edge_ids'][sample])]
                    r.command(q,step['finger'])
                else:
                    r.command(self.q0*(1-s)+step['target']*s, self.f0*(1-s)+step['finger']*s)
                joint_ok = np.max(np.abs(r.q-step['target'])) < .02 and np.max(np.abs(r.d.qvel[r.vadr])) < .12
                if step['op'] == 'gripper':
                    finger_ok = self._grasp_contact() if step['completion'] == 'contact' else bool(np.max(np.abs(r.d.qpos[r.finger_qadr]-step['finger'])) < .002)
                else:
                    finger_ok = True
                self.stable = self.stable + dt if u >= 1 and joint_ok and finger_ok else 0.
                done = self.stable >= .1
                if self.elapsed > self.duration + 3 and not done:
                    raise RuntimeError(f"{step['name']}等待实际反馈到位超时")
            if self._log_clock >= .05:
                self._record()
                self._log_clock = 0.
            if done:
                if step['op']=='trace_path':self.trail.sample(r.fk(r.q).translation,self.trace_style,force=True)
                self.job['completed_steps'].append(dict(name=step['name'], elapsed_s=self.elapsed,
                                                        actual_rad=r.q.tolist(),
                                                        tcp_xyz=r.fk(r.q).translation.tolist(),
                                 block_xyz=r.d.xpos[r.m.body('block').id].tolist()))
                if self.index + 1 == len(self.steps):
                    self._finish('succeeded')
                else:
                    self.index += 1
                    self._begin()
                    self._save()
        except (ValueError, RuntimeError) as exc:
            self._finish('failed', str(exc))
