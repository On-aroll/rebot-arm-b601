"""Programs, support poses and dragged TCP share TwinControl's guarded executor."""
import json
import math
import secrets
import threading
from pathlib import Path

import numpy as np

from .twin_program import CALIBRATION, compile_program, number, vector
from .twin_algorithm import TwinAlgorithm


class TwinExtensions:
    def __init__(self, twin):
        self.twin = twin
        self.preview = None
        self.run = None
        self.tcp = None
        self.compile_generation = 0
        self.compile_busy = False
        self.controller = TwinAlgorithm(twin)
        root = Path(getattr(twin.hw, 'output', None) or Path.cwd() / 'outputs/twin-console/simulation')
        self.support_file = root / 'support-poses.json'
        self.supports = []
        if self.support_file.exists():
            try:
                rows = json.loads(self.support_file.read_text())
                self.supports = [r for r in rows if r.get('calibration_id') == CALIBRATION and r.get('verified') is True
                                 and len(vector(r.get('joints'), 6, '支撑姿态')) == 6]
            except (ValueError, OSError, TypeError, AttributeError):
                self.supports = []

    def stop(self, reason='已取消', *, preserve_support_recovery=False):
        self.controller.stop(reason)
        self.compile_generation += 1
        self.compile_busy = False
        self.tcp = None
        self.preview = None
        if self.run and self.run['status'] not in ('completed', 'cancelled', 'failed'):
            if preserve_support_recovery and self.run.get('support_recovery_plan'):
                self.run.update(status='paused', message='支撑恢复已暂停；原路线许可保留，可重新检查后继续')
            else:
                self.run.update(status='cancelled', message=reason)

    def on_hold(self, reason):
        self.controller.stop(reason)
        # A later pause wins over an in-flight resume/preview worker.
        self.compile_generation += 1
        self.compile_busy = False
        self.preview = None
        self.tcp = None
        if self.run and self.run['status'] == 'running':
            self.run.update(status='paused', message=reason)

    def state(self):
        return dict(calibration_id=CALIBRATION, support_poses=self.supports,
                    program={k: v for k, v in (self.run or {}).items() if k not in ('steps', 'program')},
                    gripper=self.twin.hw.gripper_state() if hasattr(self.twin.hw, 'gripper_state') else {},
                    tcp_drag=bool(self.tcp), program_compiling=self.compile_busy, controller=self.controller.state())

    def _idle(self):
        if self.controller.active or self.controller.pending or self.twin._pending_jog is not None or self.twin.mode in ('GOAL', 'JOG') or self.run and self.run['status'] == 'running':
            raise RuntimeError('请先暂停当前运动')

    def readonly(self, request):
        t = self.twin
        op = request.get('op')
        if op not in ('program_preview', 'support_capture', 'support_preview', 'gripper_probe', 'controller_check'):
            return False
        if t.session and request.get('session') != t.session:
            raise RuntimeError('其它页面已接管，仅可观察')
        t._fresh()
        self._idle()
        if op == 'controller_check':
            self.controller.check(request)
        elif op == 'gripper_probe':
            if not hasattr(t.hw, 'probe_gripper'):
                raise RuntimeError('此驱动暂不支持 J7')
            t.hw.probe_gripper()
            t._event(request, True, '已请求同一 CAN 所有者检查 J7；不会使能')
        elif op == 'support_capture':
            if t.session or getattr(t.hw, 'armed', False) or getattr(t.hw, 'motion_ready', False):
                raise ValueError('记录支撑姿态时须停止、释放并保持未使能')
            if hasattr(t.hw, 'phase') and t.hw.phase != 'READONLY':
                raise ValueError('控制器须处于只读且停止/恢复已确认的状态')
            if request.get('verified') is not True or not str(request.get('support_note', '')).strip():
                raise ValueError('请确认实际无电支撑稳定，并填写支撑/工具/负载说明')
            if len(self.supports) >= 30:
                raise ValueError('支撑姿态库最多30项，请导出备份后整理')
            q = t._positions()
            if any(abs(v) > .01 for v in t.hw.get_joint_state(request_feedback=False)[1]):
                raise ValueError('当前仍有运动，不能记录支撑姿态')
            row = dict(id=secrets.token_hex(8), name=str(request.get('name', '支撑姿态'))[:80], joints=q,
                       verified=True, calibration_id=CALIBRATION, support_note=str(request['support_note'])[:500])
            rows = self.supports + [row]
            self.support_file.parent.mkdir(parents=True, exist_ok=True)
            temp = self.support_file.with_suffix('.tmp')
            temp.write_text(json.dumps(rows, ensure_ascii=False, indent=2));temp.replace(self.support_file)
            self.supports = rows
            t._event(request, True, '已保存实际支撑姿态；返回时保留当前 J1', support=row)
        else:
            if self.compile_busy:raise RuntimeError('正在检查程序路径，请等待结果')
            self.preview = None
            observed = t._positions()
            seed = list(t.hw.command_reference) if getattr(t.hw, 'motion_ready', False) else observed
            raw = request.get('program')
            def choose_support():
                if not self.supports:
                    raise ValueError('尚无验证过的无电支撑姿态；请先停止后记录，不能用零位猜测')
                speed = number(request.get('speed', .1), '返回速度', .005, getattr(t.hw, 'max_twin_speed', .3))
                options = []
                failures = []
                for row in self.supports:
                    q = [seed[0], *row['joints'][1:]]
                    try:
                        recovery_plan = None
                        limits=getattr(t.hw,'teleop_limits',None)
                        overrun_axes=(tuple(i for i,(value,(_,high)) in enumerate(zip(seed,limits))
                                            if value>high) if limits else ())
                        if hasattr(t.hw, 'plan_support_recovery') and overrun_axes == (1,2):
                            recovery_plan = t.hw.plan_support_recovery(seed, q)
                            duration = sum(t._goal_duration(a, b, speed)
                                           for a, b in zip([seed, *recovery_plan['waypoints'][:-1]], recovery_plan['waypoints']))
                            options.append((duration, row, q, recovery_plan))
                        else:
                            if hasattr(t.hw, 'validate_preview'):t.hw.validate_preview(seed, q)
                            options.append((t._goal_duration(seed, q, speed), row, q, None))
                    except (ValueError, RuntimeError) as exc:
                        failures.append(str(exc))
                if not options:
                    raise ValueError('支撑姿态路径均未通过检查：' + '; '.join(failures[:3]))
                _, selected, q, recovery_plan = min(options, key=lambda item: item[0])
                steps = recovery_plan['waypoints'] if recovery_plan else [q]
                raw = dict(schema_version='1.0', robot_model='b601_rs', base_frame='base_link',
                           calibration_id=CALIBRATION, units=dict(position='m', angle='rad', time='s'),
                           name='返回支撑：' + selected['name'],
                           steps=[dict(type='joint', joints=list(goal), speed=speed,
                                       label=f'支撑恢复第 {i+1}/{len(steps)} 段')
                                  for i, goal in enumerate(steps)])
                return raw, selected, recovery_plan
            self.compile_generation += 1
            generation = self.compile_generation
            fingerprint = getattr(getattr(t.hw, 'config', None), 'fingerprint', None)
            self.compile_busy = True
            def build():
                try:
                    if op == 'support_preview':
                        source, support, recovery_plan = choose_support()
                    else:
                        source, support, recovery_plan = raw, None, None
                    # Pinocchio model data is mutable: use an isolated IK
                    # instance so status publication and stops remain responsive.
                    kin = type(t.kin)()
                    program, steps = compile_program(source, seed, kin, t.hw, t._goal_duration,
                                                     support_recovery_plan=recovery_plan)
                    support_tolerance = (math.radians(number(support['arrival_tolerance_deg'], '支撑到位容差', .001, 3))
                                         if support and 'arrival_tolerance_deg' in support else None)
                    duration = sum(s['duration'] for s in steps)
                    with t.lock:
                        if generation != self.compile_generation:return
                        if fingerprint != getattr(getattr(t.hw, 'config', None), 'fingerprint', None):
                            raise ValueError('控制范围在预览中改变，请重新预览')
                        t._check_goal_budget(duration)
                        self.preview = dict(id=secrets.token_hex(12), at=t.clock(), observed_start=observed, start=seed,
                                            program=program, steps=steps, duration=duration, support=support,
                                            support_tolerance_rad=support_tolerance,
                                            support_recovery_plan=recovery_plan,
                                            fingerprint=fingerprint)
                        first_grip = next((s for s in steps if s['type'] in ('gripper', 'grasp')), None)
                        if first_grip:self.preview['observed_gripper'] = first_grip['start_position']
                        t._event(request, True, '整套程序路径检查通过；尚未执行', program_plan=self.preview)
                except Exception as exc:
                    with t.lock:
                        if generation == self.compile_generation:t._event(request, False, str(exc))
                finally:
                    with t.lock:
                        if generation == self.compile_generation:self.compile_busy = False
            threading.Thread(target=build, name='inert-program-preview', daemon=True).start()
        return True

    def command(self, request):
        t = self.twin;op = request.get('op')
        if op == 'controller_start':
            if self.compile_busy:raise RuntimeError('请等待正在进行的路径检查')
            self.controller.start(request)
        elif op == 'controller_stop':
            t._hold('控制算法已停止，保持当前位置')
        elif op == 'program_start':
            self._idle()
            p = self.preview
            if not p or request.get('plan_id') != p['id'] or t.clock() - p['at'] > 60:
                raise ValueError('程序预览已过期，请重新检查整套路径')
            if p.get('fingerprint') != getattr(getattr(t.hw, 'config', None), 'fingerprint', None):
                raise ValueError('控制范围已变化，请重新预览')
            if max(abs(a-b) for a, b in zip(t._positions(), p['observed_start'])) > .03:
                raise ValueError('程序起点已变化，请重新预览')
            if 'observed_gripper' in p:
                grip = t.hw.gripper_state()
                if not grip.get('fresh') or abs(grip['position']-p['observed_gripper']) > .03:
                    raise ValueError('夹爪起点已变化或反馈过期，请重新预览')
                # The explicit program start covers its checked J7 steps.
                t.hw.authorize_gripper(t.session, True)
            if p.get('support_recovery_plan') and hasattr(t.hw, 'begin_support_recovery'):
                t.hw.begin_support_recovery(t.session, p['support_recovery_plan'])
            self.run = dict(id=p['id'], program=p['program'], steps=p['steps'], index=0,
                            count=len(p['steps']), status='running', message='程序已开始', waiting=False,
                            release_after=bool(request.get('release_after')) and bool(p.get('support')),
                            support=bool(p.get('support')), support_tolerance_rad=p.get('support_tolerance_rad'),
                            support_recovery_plan=p.get('support_recovery_plan'), deadline=None)
            self.preview = None;t.message = '正在执行程序'
        elif op == 'program_pause':
            t._hold('程序已暂停；继续前将从当前反馈重新检查剩余路径')
        elif op == 'program_resume':
            if not self.run or self.run['status'] != 'paused':raise ValueError('没有暂停的程序')
            if self.compile_busy:raise ValueError('正在重新检查剩余路径')
            remaining = dict(self.run['program'], steps=self.run['program']['steps'][self.run['index']:])
            seed = list(t.hw.command_reference) if getattr(t.hw, 'motion_ready', False) else t._positions()
            run = self.run;index = run['index'];generation = self.compile_generation = self.compile_generation+1
            self.compile_busy = True
            def resume():
                try:
                    _, steps = compile_program(remaining, seed, type(t.kin)(), t.hw, t._goal_duration,
                                               support_recovery_plan=run.get('support_recovery_plan'),
                                               support_recovery_stage_offset=run['index'])
                    with t.lock:
                        if generation != self.compile_generation or self.run is not run:return
                        if run['status'] != 'paused':raise ValueError('程序状态已改变，请重新继续')
                        t._fresh()
                        t._check_goal_budget(sum(s['duration'] for s in steps))
                        run['steps'] = run['steps'][:index] + steps
                        run.update(status='running', waiting=False, deadline=None, message='剩余路径检查通过，继续执行')
                        t.message = run['message']
                        t._event(request, True, run['message'])
                except Exception as exc:
                    with t.lock:
                        if generation == self.compile_generation:t._event(request, False, str(exc))
                finally:
                    with t.lock:
                        if generation == self.compile_generation:self.compile_busy = False
            threading.Thread(target=resume, name='inert-program-resume', daemon=True).start()
            return 'pending'
        elif op == 'program_cancel':
            if self.run and self.run.get('support_recovery_plan') and self.run['status'] == 'running':
                message='支撑恢复已暂停；原路线许可保留，可重新检查后继续'
                self.stop(message,preserve_support_recovery=True)
                t._hold(message,preserve_program=True)
            else:
                self.stop('程序已取消');t._hold('程序已取消，保持当前位置')
        elif op == 'gripper_authorize':
            t.hw.authorize_gripper(t.session, request.get('enabled') is True)
        elif op == 'gripper':
            self._idle();t.hw.gripper_target(t.session, request.get('position'), request.get('speed', .1))
        elif op == 'tcp_target':
            if self.controller.active or t.mode == 'GOAL' or self.run and self.run['status'] == 'running':
                raise ValueError('程序或到点运动中不能拖拽；请先暂停')
            pose = vector(request.get('pose'), 6, 'TCP目标')
            if np.linalg.norm(pose[:3]) > .8:raise ValueError('TCP目标超出工作空间参考')
            self.tcp = dict(pose=pose, orientation=request.get('orientation') is True,
                            speed=number(request.get('speed', .1), '速度', .005, getattr(t.hw, 'max_twin_speed', .3)),
                            linear=number(request.get('linear', .025), '平移速度', .0005, .04),
                            angular=number(request.get('angular', math.radians(10)), '旋转速度', math.radians(.5), math.radians(20)),
                            at=t.clock())
        else:
            return False
        return True

    def goal_finished(self, success, message):
        if self.run and self.run['status'] == 'running':
            if success:
                if self.run.get('support_recovery_plan') and hasattr(self.twin.hw,'complete_support_recovery_segment'):
                    final = self.run['index'] == self.run['count']-1
                    tolerance = self.run.get('support_tolerance_rad') if final else None
                    self.twin.hw.complete_support_recovery_segment(tolerance or math.radians(.1))
                self.run['index'] += 1;self.run['waiting'] = False
            else:
                status='paused' if self.run.get('support_recovery_plan') else 'failed'
                self.run.update(status=status, message=message)

    def check_grasp(self):
        """Check the held-object gate BEFORE another arm goal sample is sent."""
        t = self.twin
        run = self.run
        if run and run['status'] == 'running' and run.get('grasp_id'):
            state = t.hw.gripper_state()
            grasp = state.get('grasp', {})
            if (not state.get('fresh') or not state.get('authorized') or not state.get('active', True)
                    or state.get('error') or grasp.get('id') != run['grasp_id'] or grasp.get('status') != 'holding'):
                message = grasp.get('message') or '夹持证据已失效，未继续后续动作'
                t._hold(message)
                run.update(status='failed', message=message, grasp_result=grasp)
                return False
        return True

    def tick(self):
        t = self.twin
        self.controller.tick()
        if self.tcp:
            if t.clock() - self.tcp['at'] > .25:
                t._hold('TCP拖拽已松开或过期：保持当前位置')
            else:
                p = self.tcp
                error = t.kin.error(t._positions(), p['pose'], p['orientation'])
                v = error[:3] * 2;w = error[3:] * 2
                v *= min(1., p['linear'] / max(np.linalg.norm(v), 1e-9))
                w *= min(1., p['angular'] / max(np.linalg.norm(w), 1e-9))
                t._jog(dict(twist=[*v, *w], speed=p['speed'], orientation=p['orientation']))
        run = self.run
        if not run or run['status'] != 'running':return
        if not self.check_grasp():return
        if run['index'] >= len(run['steps']):
            message = '夹取流程完成：电机接触与保持判定通过，原位保持待实物验证' if run.get('grasp_id') else '全部步骤已确认到达'
            run.update(status='completed', message=message)
            t.message = run['message']
            if run.get('release_after'):
                t._release('已到达已验证支撑姿态，按本次选择撤去驱动力')
            return
        step = run['steps'][run['index']]
        run['message'] = f'第 {run["index"]+1}/{run["count"]} 步：{step["label"]}'
        if run['waiting']:
            if step['type'] == 'wait' and t.clock() >= run['deadline']:
                run['index'] += 1;run['waiting'] = False
            elif step['type'] == 'gripper':
                state = t.hw.gripper_state()
                if state.get('error'):raise RuntimeError(state['error'])
                if state.get('arrived'):
                    run['index'] += 1;run['waiting'] = False
                elif t.clock() > run['deadline']:raise TimeoutError('夹爪到位等待超时')
            elif step['type'] == 'grasp':
                state = t.hw.gripper_state()
                grasp = state.get('grasp', {})
                if grasp.get('id') != run.get('pending_grasp_id'):
                    message = '夹持任务标识已变化，未继续后续动作'
                elif not state.get('fresh') or state.get('error'):
                    message = state.get('error') or 'J7夹持反馈过期'
                elif grasp.get('status') in ('failed', 'cancelled'):
                    message = grasp.get('message', '夹持未通过')
                elif t.clock() > run['deadline']:
                    message = '夹持任务等待超时'
                else:
                    run['message'] = grasp.get('message', run['message'])
                    if grasp.get('status') == 'holding':
                        run.update(index=run['index']+1, waiting=False,
                                   grasp_id=grasp['id'], grasp_result=grasp)
                    return
                t._hold(message)
                run.update(status='failed', message=message, grasp_result=grasp)
            return
        if t.mode != 'HOLD':return
        if step['type'] == 'wait':
            run.update(waiting=True, deadline=t.clock()+step['seconds'])
        elif step['type'] == 'gripper':
            if hasattr(t.hw, 'motion_ready') and not t.hw.motion_ready:
                t.hw.teleop_resume(t.session);return
            run.pop('grasp_id', None)
            t.hw.gripper_target(t.session, step['position'], step['speed'])
            run.update(waiting=True, deadline=t.clock()+step['duration']+10)
        elif step['type'] == 'grasp':
            if hasattr(t.hw, 'motion_ready') and not t.hw.motion_ready:
                t.hw.teleop_resume(t.session);return
            job_id = t.hw.gripper_grasp(t.session, step['profile'], step['speed'])
            run.update(waiting=True, deadline=t.clock()+step['duration']+10, pending_grasp_id=job_id)
        else:
            observed = t._positions()
            seed = list(t.hw.command_reference) if getattr(t.hw, 'motion_ready', False) else observed
            if step.get('support_recovery') is True and hasattr(t.hw, 'validate_preview'):
                t.hw.validate_preview(seed, step['joints'],
                    support_recovery=True)
            elif hasattr(t.hw, 'validate_preview'):
                t.hw.validate_preview(seed, step['joints'])
            duration = max(step['duration'], t._goal_duration(seed, step['joints'], step['speed']))
            t.plan = dict(id=secrets.token_hex(12), at=t.clock(), start=seed, observed_start=observed,
                          joints=step['joints'], pose=step['pose'], orientation=True,
                          speed=step['speed'], duration=duration,
                          support_recovery=bool(step.get('support_recovery')))
            if run.get('support') and run.get('support_tolerance_rad') is not None:
                intermediate = run.get('support_recovery_plan') and run['index'] < run['count']-1
                t.plan['support_tolerance_rad'] = math.radians(.1) if intermediate else run['support_tolerance_rad']
            t._start_goal(dict(plan_id=t.plan['id']));run['waiting'] = True
