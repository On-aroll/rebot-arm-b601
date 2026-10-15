"""Local operator-confirmed J1 or reviewed XYZ trial; no CAN on import.

The UI prepares a read-only plan, then explicitly confirms that exact plan once.
A child process uses the audited SDK environment. Software profiles stay untouched.
"""
import copy
import json
import math
import os
from pathlib import Path
import secrets
import subprocess
import threading
import time
from rebot_mit_hardware import Envelope

PREFIX = 'REBOT_POWER_JSON '
CONFIRMATIONS = ('operator_present', 'power_cut_ready', 'supported', 'space_clear', 'accept_proposal')
TERMINAL = ('ARRIVED', 'CANCELLED', 'FAULT')


def trial_proposal(state, limits):
    for i in range(1,7):
        p = state['drafts'].get(str(i)) or state['approved'].get(str(i)) or {}
        f = p.get('fields', {})
        values=tuple(f.get(k,{}).get('value') for k in ('kp','kd','tau_ff'))
        if values != (3,1,0):
            raise ValueError(f'J{i} 已保存参数 Kp={values[0]}、Kd={values[1]}、前馈={values[2]}，与本次试验的 3/1/0 不一致；请核对现有草稿，未覆盖任何值')
    # An explicit disposable proposal, displayed in full before confirmation.
    # Passive axes receive NO frames; their model/velocity limits remain guards.
    axes = [dict(q_min=float(lo),q_max=float(hi),v_max=.02,a_max=.04,jerk_max=.2,
                 kp=3.,kd=1.,tau_ff=0.,kp_max=3.,kd_max=1.,torque_max=.2,error_max=math.pi/180)
            for lo,hi in limits]
    policy = dict(period=.1,io_timeout=.05,feedback_age=.6,command_age=.3,jitter_max=.15,
                  lease_timeout=.75,start_speed=.01,arrival_q=math.radians(.1),arrival_qd=.01,
                  settle_time=.3,task_timeout=5.)
    raw = dict(source='explicit-onsite-trial-proposal',version=f'j1-hold-v1-config-{state["revision"]}',axes=axes,policy=policy)
    Envelope.parse(raw)
    public = dict(title='J1 底座 · 当次使能保持试验',config_revision=state['revision'],
        summary='只使能 J1，以执行前重新读取的位置为目标，保持约 3 秒后自动撤力；不移动到新坐标、不自动返回。Kp=3、Kd=1、前馈=0；速度上限 0.02 rad/s，位置偏差上限 1°，软件力矩估算上限 0.2 N·m。停止、失能或窗口失焦均结束本次保持并对 J1 去使能；J2–J7 不发送控制指令。',
        limitations='SDK 阻塞会延迟撤力；网页按钮不是物理急停。单次速度读数不能证明持续停止。3/1/0 尚未实机验证，不保证支撑或克服静摩擦。',
        limits=axes[0],passive_model_limits=[{'axis':i+1,'q_min':a['q_min'],'q_max':a['q_max']} for i,a in enumerate(axes)],
        policy=policy,parameter_source='用户 3/1/0 + 本次明确展示的临时保护方案；不写入电机寄存器或软件配置',
        stop_policy='只对本次已尝试使能的 J1 撤力；其他轴保持原状态',seconds=3)
    return raw, public


class MotorPowerAPI:
    def __init__(self, store, port, root, limits, check_pose, idle=lambda:True, *, point_plan=None, popen=subprocess.Popen, clock=time.monotonic):
        self.store,self.port,self.root,self.limits=store,port,Path(root),limits
        self.check_pose,self.idle,self.popen,self.clock=check_pose,idle,popen,clock
        self.point_plan=point_plan
        self.lock=threading.RLock(); self.pipe_lock=threading.Lock()
        self.sessions={}; self.plan=None; self.process=None; self.owner=None; self.sequence=0
        self.proposal=None; self.stop_requested=False; self.closed=False; self.generation=0
        self.snapshot=dict(state='IDLE',busy=False,axis=1,reason='等待用户确认本次已预演的实机任务；尚未下发。',physical_acceptance=False,feedback={})

    def status(self):
        with self.lock:
            result=copy.deepcopy(self.snapshot)
            if self.proposal: result['proposal']=copy.deepcopy(self.proposal)
            result['enabled_feedback_available']=False
            wall=time.time(); mono=time.monotonic()
            result['observed_at_ms']=wall*1000
            result['motors']={str(i):dict(id=int(i),model='rs-06' if int(i)<=3 else 'rs-00',
                mechPosRad=s['q'],positionUpdatedAt=(wall-(mono-s['requested_at']))*1000,error='')
                for i,s in result.get('feedback',{}).items() if all(k in s for k in ('q','requested_at'))}
            return result

    def _new_process(self):
        python=self.root/'.venv/bin/python'
        if not python.is_file(): raise ValueError('缺少已审核 motorbridge 0.5.6 环境 .venv/bin/python；未启动实机')
        env=dict(os.environ,PYTHONPATH=str(self.root/'scripts'))
        return self.popen([str(python),'-u',str(self.root/'scripts/rebot_power_worker.py')],
                          stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,
                          cwd=str(self.root),env=env,start_new_session=True)

    def _send(self, body):
        # Small writes, separate from the status lock; STOP never queues behind a probe.
        with self.pipe_lock:
            if self.process is None or self.process.poll() is not None: raise ValueError('试验进程已结束，请查看停止结果')
            self.process.stdin.write(json.dumps(body,ensure_ascii=False,allow_nan=False)+'\n')
            self.process.stdin.flush()

    def _readonly_probe(self, raw, *, native=False, native_axis=3):
        p=self._new_process()
        try:
            output,_=p.communicate(json.dumps(dict(operation=f'probe-csp-j{native_axis}' if native else 'probe',config=raw))+'\n',timeout=12)
        except subprocess.TimeoutExpired:
            # This process has only ever received operation=probe; it cannot enable.
            p.kill(); p.communicate(); raise ValueError('只读检查超时；未发送使能')
        records=[json.loads(line[len(PREFIX):]) for line in output.splitlines() if line.startswith(PREFIX)]
        if not records: raise ValueError('SDK 检查进程未返回结果；未发送使能')
        return records[-1]

    def prepare(self, token, *, native=False, native_axis=3):
        with self.lock:
            if self.closed or self.snapshot['busy']: raise ValueError('已有试验/检查，请先停止或等待')
            if not self.idle(): raise ValueError('请先结束仿真任务，再检查单轴试验')
            self.plan=None; self.proposal=None; self.generation+=1; generation=self.generation
            self.snapshot=dict(state='CHECKING',busy=True,axis=native_axis if native else 1,reason='正在只读检查六轴位置、速度和模式；未使能',feedback={},physical_acceptance=False)
        try:
            state=self.store.status()['state']
            if native:
                from rebot_native_csp import proposal,DELTA
                if native_axis == 7:
                    from rebot_gripper_csp import proposal
                raw,public=proposal(self.limits,axis=native_axis)
            else: raw,public=trial_proposal(state,self.limits)
            with self.lock: self.proposal=public
            result=self._readonly_probe(raw,native=True,native_axis=native_axis) if native else self._readonly_probe(raw)
            with self.lock: self.snapshot['feedback']=result.get('feedback',{})
            if result.get('ok'):
                self.check_pose(result['expected_start'][:6])
                if native:
                    from rebot_mit_mock import Trajectory
                    target=list(result['expected_start']); target[native_axis-1]+=DELTA
                    if native_axis == 7:
                        from rebot_gripper_csp import GripperEnvelope,GripperTrajectory
                        parsed=GripperEnvelope.parse(raw);curve=GripperTrajectory
                    else: parsed=Envelope.parse(raw);curve=Trajectory
                    trajectory=curve.create(result['expected_start'],[target],parsed,axes=(native_axis,))
                    for n in range(101):
                        self.check_pose(trajectory.sample(trajectory.duration*n/100)[0][:6])
                    public.update(start_deg=[math.degrees(v) for v in result['expected_start']],
                                  target_deg=[math.degrees(v) for v in target],curve_seconds=trajectory.duration)
                if state['revision']!=self.store.status()['state']['revision']:
                    raise ValueError('检查期间软件配置发生变化，请重新检查')
            with self.lock:
                if self.closed or generation!=self.generation:
                    self.snapshot.update(state='IDLE',busy=False,reason='准备已取消，未发送使能',feedback=result.get('feedback',{}))
                    return dict(status=self.status())
                if result.get('ok'):
                    plan_id=secrets.token_urlsafe(24)
                    self.plan=dict(id=plan_id,session=token,expires=self.clock()+60,config=raw,revision=state['revision'],
                                   expected_start=result['expected_start'],operation=f'csp-j{native_axis}-trial' if native else 'hold-j1')
                    if native: self.plan['target']=target
                    self.snapshot.update(state='READY',busy=False,reason='检查通过；尚未使能。请核对并确认本次方案。',feedback=result['feedback'])
                    return dict(plan_id=plan_id,proposal=public,status=self.status())
                self.snapshot.update(state='FAULT',busy=False,reason=result.get('reason') or '只读检查失败；未使能',feedback=result.get('feedback',{}))
                return dict(status=self.status(),proposal=public)
        except Exception as e:
            with self.lock: self.snapshot.update(state='FAULT',busy=False,reason=str(e))
            return dict(status=self.status())

    def start(self, token, body, *, point=False, diagnostic=False, native=False, native_axis=3):
        with self.lock:
            if self.closed or self.snapshot['busy']: raise ValueError('已有试验或服务正在关闭')
            if not isinstance(body.get('confirmed'),dict) or any(body['confirmed'].get(k) is not True for k in CONFIRMATIONS):
                raise ValueError('请确认当前在场、独立断电、可靠支撑、周围空间及本次完整方案')
            if point:
                if self.point_plan is None: raise ValueError('XYZ 执行器未接入')
                plan=self.point_plan(body.get('plan_id'))
                plan=dict(plan,session=token)
            else: plan=self.plan
            if not plan or plan['id']!=body.get('plan_id') or plan['session']!=token or self.clock()>=plan['expires']:
                raise ValueError('准备方案已过期、已消费或不属于当前页面，请重新检查')
            if not point and plan.get('operation','hold-j1')!=(f'csp-j{native_axis}-trial' if native else 'hold-j1'):
                raise ValueError('准备方案与执行类型不一致')
            if not self.idle(): raise ValueError('仿真任务尚未结束')
            if plan['revision']!=self.store.status()['state']['revision']:
                self.plan=None; raise ValueError('软件配置已变化，请重新检查；没有覆盖修改')
            if native and native_axis == 7:
                from rebot_gripper_csp import GripperEnvelope
                config=GripperEnvelope.parse(plan['config'])
            else: config=Envelope.parse(plan['config'])
            native_xyz=point and config.version=='native-csp-xyz-v1'
            if diagnostic:
                if not point or config.gain_review is None or config.feedforward is None:
                    raise ValueError('启动采集仅支持已审核的官方范围增益方案')
            self.plan=None; self.owner=token; self.sequence=0; self.stop_requested=False
            self.snapshot.update(state='PREPARING',busy=True,axes=[native_axis] if native else list(range(1,7)) if point else [1],
                                 reason=f'正在复读起点，准备当次 J{native_axis} CSP +0.5° 试验' if native else '正在复读真实起点；仅做 3 秒启动采集，不执行 XYZ' if diagnostic else
                                 '正在复读真实起点；仅执行本次已确认目标' if point else '正在复读真实起点；随后仅尝试使能 J1',feedback={})
            try:
                self.process=self._new_process()
                message=dict(operation=f'csp-j{native_axis}-trial' if native else 'native-xyz' if native_xyz else 'probe-startup' if diagnostic else 'move-xyz' if point else 'hold-j1',seconds=config.policy.task_timeout if native or native_xyz else 3,config=plan['config'],expected_start=plan['expected_start'],
                             request_id=secrets.token_hex(16),evidence='本机用户点击确认执行已预演 XYZ，沿用七轴现场确认' if point else '当前本机页面明确确认 J1 3 秒保持、机械支撑及撤力方案')
                if point or native: message['target']=plan['target']
                if diagnostic: message['evidence']='本机用户明确点击启动采集：首次使能起最多 3 秒自动撤力，不执行 XYZ；沿用现场确认'
                if native: message['evidence']=f'已明确批准 J{native_axis} CSP +0.5°、0.02 rad/s、{config.policy.task_timeout:g} 秒试验及临时四项运行参数；PID/零位/保存草稿不写入'
                if native_xyz: message['evidence']='用户明确确认当前预演的六轴原生 XYZ 小幅试验：每轴最多0.5°、0.02 rad/s、15秒及临时运行限制；PID/零位/保存草稿不写入'
                self._send(message)
                threading.Thread(target=self._read_worker,args=(self.process,),daemon=True,name='j1-power-status').start()
            except Exception as e:
                self.snapshot.update(state='FAULT',busy=False,reason=str(e))
                if self.process and self.process.poll() is None:
                    self.snapshot.update(state='STOPPING',busy=True,reason='启动通信异常，撤力状态待确认：'+str(e))
                    if self.process.stdin: self.process.stdin.close()
                    threading.Thread(target=self._read_worker,args=(self.process,),daemon=True,name='j1-failed-start').start()
            return dict(status=self.status())

    def _read_worker(self, process):
        terminal=None
        command_log=None
        try:
            for line in process.stdout:
                if not line.startswith(PREFIX): continue
                data=json.loads(line[len(PREFIX):]); kind=data.pop('kind',None)
                if kind=='result': command_log=data.pop('command_log',None)
                with self.lock:
                    if self.process is not process: continue
                    self.snapshot.update(data)
                    self.snapshot['busy']=True  # Until child exit and stop cleanup complete.
                    if self.stop_requested and data.get('state') not in TERMINAL: self.snapshot['state']='STOPPING'
                if kind=='result': terminal=data
            process.wait()
        except Exception as e:
            terminal=dict(state='FAULT',reason='试验进程通信异常，撤力待确认：'+str(e),physical_acceptance=False)
            with self.lock: self.snapshot.update(state='STOPPING',busy=True,reason=terminal['reason'])
            # EOF cancels the worker. Keep exclusive task ownership until it exits.
            # A blocked SDK must not be hidden by marking the service idle.
            try: process.stdin.close()
            except OSError: pass
            process.wait()
        finally:
            with self.lock:
                if self.process is process:
                    self.snapshot.update(terminal or dict(state='FAULT',reason='试验进程异常退出，撤力结果未知，请使用现场独立停止手段'))
                    self.snapshot['busy']=False
                    self.snapshot['result']=copy.deepcopy(terminal)
                    self.owner=None
                    result=self.status()
            if process.stdin: process.stdin.close()
            directory=self.root/'outputs/mit-p5/runtime'; directory.mkdir(parents=True,exist_ok=True)
            if command_log is not None: result['command_log']=command_log
            (directory/(time.strftime('%Y%m%d-%H%M%S')+'-'+secrets.token_hex(4)+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')

    def stop(self):
        with self.lock:
            self.plan=None; self.generation+=1
            if not self.snapshot['busy'] or self.process is None or self.process.poll() is not None:
                if self.snapshot['state']=='READY': self.snapshot.update(state='IDLE',reason='当次准备已取消，未使能')
                return dict(status=self.status(),note='当前没有本会话拥有的使能轴；未对未知状态设备发送批量失能')
            self.stop_requested=True
            self.snapshot.update(state='STOPPING',reason='已请求停止并撤去本任务运动轴力矩，等待 SDK 与速度反馈；异常时使用独立断电')
        try: self._send(dict(action='stop'))
        except (OSError,ValueError) as e:
            with self.lock: self.snapshot.update(reason='停止请求传输失败，实际状态未知：'+str(e))
        return dict(status=self.status())

    def request(self, path, headers, peer, body):
        point=path.startswith('/hardware-point/')
        if point: path=path.replace('/hardware-point/','/motor-power/',1)
        host=headers.get('Host','')
        if peer not in ('127.0.0.1','::1') or host not in (f'127.0.0.1:{self.port}',f'localhost:{self.port}') or headers.get('Origin')!='http://'+host or headers.get('Sec-Fetch-Site') not in (None,'same-origin'):
            raise ValueError('实机试验只接受本机同源页面请求')
        with self.lock:
            self.sessions={k:v for k,v in self.sessions.items() if v>self.clock()}
            if path=='/motor-power/session':
                if len(self.sessions)>=128: raise ValueError('会话过多')
                token=secrets.token_urlsafe(32); self.sessions[token]=self.clock()+1800
                return dict(token=token,status=self.status())
            token=headers.get('X-Motor-Session')
            if path!='/motor-power/action' or token not in self.sessions: raise ValueError('单轴操作会话失效，请重新打开操作入口')
        action=body.get('action')
        if point and action not in ('execute','probe-startup','prepare-csp-j3','csp-j3-trial','prepare-csp-j6','csp-j6-trial','prepare-csp-j7','csp-j7-trial','stop','heartbeat'):
            raise ValueError('XYZ 入口只接受已预演任务、启动采集、停止和会话心跳')
        if action=='prepare': return self.prepare(token)
        if action=='start': return self.start(token,body)
        if action=='execute' and point: return self.start(token,body,point=True)
        if action=='probe-startup' and point: return self.start(token,body,point=True,diagnostic=True)
        if action=='prepare-csp-j3' and point: return self.prepare(token,native=True)
        if action=='csp-j3-trial' and point: return self.start(token,body,native=True)
        if action=='prepare-csp-j6' and point: return self.prepare(token,native=True,native_axis=6)
        if action=='csp-j6-trial' and point: return self.start(token,body,native=True,native_axis=6)
        if action=='prepare-csp-j7' and point: return self.prepare(token,native=True,native_axis=7)
        if action=='csp-j7-trial' and point: return self.start(token,body,native=True,native_axis=7)
        if action=='stop': return self.stop()
        if action=='heartbeat':
            with self.lock:
                seq=body.get('sequence')
                if token!=self.owner or type(seq) is not int or seq<=self.sequence: raise ValueError('试验心跳重复、过期或页面不匹配')
                self.sequence=seq
            self._send(dict(action='heartbeat',sequence=seq,focused=body.get('focused') is True))
            return dict(ok=True)
        raise ValueError('不支持该操作；不能通过本入口修改参数、模式、零点或控制其他轴')

    def close(self):
        self.closed=True; self.stop()
        if self.process and self.process.stdin:
            try: self.process.stdin.close()
            except OSError: pass
