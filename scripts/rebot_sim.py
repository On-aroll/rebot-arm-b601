#!/usr/bin/env python3
"""Local RS simulation dashboard, verification and reproducible demonstrations."""
import os
import argparse, json, time, threading, io, csv, subprocess, hashlib, platform, uuid, mimetypes
from urllib.parse import unquote, urlsplit
from pathlib import Path
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import numpy as np
import mujoco as mj
import pinocchio as pin
from PIL import Image
from rebot_core import Robot, ROOT, URDF, SCENE, HOME
from rebot_point import PointPlanner, point_capabilities
from rebot_config import ConfigStore, ConfigAPI
from rebot_hardware_readiness import readiness
from rebot_motor_power import MotorPowerAPI
from rebot_ros_proxy import ros_request

OUT=Path(os.environ.get('REBOT_OUTPUT_DIR', str(ROOT/'outputs')))
STUDIO_LOCALES=('en','zh','es')
# Client-side routes that must survive a hard reload. /studio/writing stays as a
# compatibility alias for the renamed 虚实联调控制台.
STUDIO_ROUTES=('/studio/','/studio/console','/studio/writing','/studio/simu')
MACHINE_LIVE=OUT/'machine_live.json'
MACHINE_PARAMS=OUT/'machine_state.json'

def read_report(path,source,max_age_s=None):
    """Expose a hardware report file as read-only JSON with freshness metadata.

    This service never opens the CAN socket. The hardware side is published by
    rebot_twin_sync (live joints) and rebot_can_readstate (parameter sweep), so a
    missing or stale file must surface as an explicit state rather than a pose of
    zeros that would look like a real reading.
    """
    if not path.is_file():
        return dict(present=False,source=source,reason='not_published',
                    hint='实机只读数据尚未发布：先运行 scripts/rebot_twin_sync.py --follow')
    try:
        data=json.loads(path.read_text())
    except (json.JSONDecodeError,OSError) as e:
        return dict(present=False,source=source,reason='unreadable',error=str(e))
    stamp=float(data.get('time_unix_s') or path.stat().st_mtime)
    data['present']=True; data['source']=source
    data['report_time_unix_s']=stamp
    data['age_s']=round(max(0.,time.time()-stamp),2)
    if max_age_s is not None: data['stale']=data['age_s']>max_age_s
    return data

def camera():
    c=mj.MjvCamera();c.lookat[:]=[.1,0,.30];c.distance=1.4;c.azimuth=125;c.elevation=-20
    return c

def verify():
    r=Robot(); rng=np.random.default_rng(20261006)
    pos_errors=[]; rot_errors=[]
    for _ in range(100):
        q=rng.uniform(r.lower,r.upper)
        r.d.qpos[r.qadr]=-q; mj.mj_forward(r.m,r.d)
        pose=r.fk(q); bid=r.m.body('gripper_end').id
        pos_errors.append(float(np.linalg.norm(pose.translation-r.d.xpos[bid])))
        rot_errors.append(float(np.linalg.norm(pin.log3(pose.rotation.T@r.d.xmat[bid].reshape(3,3)))))
    r.reset(); ik_errors=[]
    for _ in range(20):
        q=HOME+rng.uniform(-.12,.12,6); pose=r.fk(q)
        solved=r.ik(pose.translation,pose.rotation,HOME)
        got=r.fk(solved)
        ik_errors.append(float(np.linalg.norm(got.translation-pose.translation)))
    try:
        r.ik([10,10,10]); raise AssertionError('Unreachable target accepted')
    except ValueError: pass
    for malformed in ([float('nan')]*6, [999]*6, [0,0]):
        try:
            r.command(malformed); raise AssertionError('Invalid command accepted')
        except ValueError: pass
    t,qs,fs=r.trajectory('joints',seconds=10)
    r.reset(); r.command(qs[0],fs[0]); r.d.qpos[r.finger_qadr]=fs[0]
    start=time.perf_counter(); max_error=0.;max_contacts=0; min_time=0.
    steps=round(60/r.m.opt.timestep)
    for i in range(steps):
        elapsed=i*r.m.opt.timestep; phase=elapsed%10
        q=np.array([np.interp(phase,t,qs[:,j]) for j in range(6)])
        f=float(np.interp(phase,t,fs));r.command(q,f);r.step()
        max_error=max(max_error,float(np.max(np.abs(r.q-q))))
        max_contacts=max(max_contacts,r.d.ncon)
        assert r.d.time > min_time, 'Unexpected simulator reset'
        min_time=r.d.time
    wall=time.perf_counter()-start
    report=dict(python=platform.python_version(),mujoco=mj.__version__,pinocchio=pin.__version__,
                fk_samples=100,fk_max_position_m=max(pos_errors),fk_max_rotation_rad=max(rot_errors),
                ik_samples=20,ik_max_position_m=max(ik_errors),
                simulated_seconds=float(r.d.time),wall_seconds=wall,realtime_factor=60/wall,
                joint_max_tracking_error_deg=float(np.rad2deg(max_error)),max_contacts=int(max_contacts),
                warning_counts=r.d.warning.number.tolist(),
                mass_mjcf_kg=float(r.m.body_mass.sum()),
                joint_sign_mapping=[-1]*6,
                limits_canonical_rad=np.column_stack((r.lower,r.upper)).tolist(),
                note='Kinematic agreement only; URDF/MJCF inertias differ. Simulated bias compensation enabled.')
    assert max(pos_errors)<.001 and max(rot_errors)<np.deg2rad(1)
    assert max(ik_errors)<.001 and not any(report['warning_counts'])
    assert max_contacts==0 and max_error<np.deg2rad(5)
    (OUT/'verification.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

def demo(kind,seconds,video):
    import imageio.v2 as imageio
    r=Robot();t,qs,fs=r.trajectory(kind,seconds); r.command(qs[0],fs[0]);r.d.qpos[r.finger_qadr]=fs[0]
    writer=imageio.get_writer(str(OUT/f'{kind}_demo.mp4'),fps=30) if video else None
    renderer=mj.Renderer(r.m,height=720,width=960) if video else None
    cam=camera(); rows=[];next_frame=0
    try:
        for i in range(round(seconds/r.m.opt.timestep)+1):
            elapsed=i*r.m.opt.timestep
            q=np.array([np.interp(elapsed,t,qs[:,j]) for j in range(6)])
            f=float(np.interp(elapsed,t,fs));r.command(q,f);r.step()
            if elapsed+1e-8>=next_frame:
                pose=r.fk(r.q)
                rows.append([r.d.time,*q,*r.q,*pose.translation,float(np.mean(r.d.qpos[r.finger_qadr]))])
                if renderer:
                    renderer.update_scene(r.d,camera=cam); frame=renderer.render();writer.append_data(frame)
                    if not (OUT/f'{kind}_preview.png').exists(): Image.fromarray(frame).save(OUT/f'{kind}_preview.png')
                next_frame+=1/30
    finally:
        if writer: writer.close()
        if renderer: renderer.close()
    with (OUT/f'{kind}_trajectory.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['time_s']+[f'target_j{i}_rad' for i in range(1,7)]+[f'actual_j{i}_rad' for i in range(1,7)]+['x_m','y_m','z_m','finger_travel_m']);w.writerows(rows)
    print(f'Completed {kind}: {OUT}')

def lan_addresses():
    """Non-loopback IPv4 addresses this host can be reached at on the LAN."""
    import socket
    found=[]
    try:
        s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.connect(('8.8.8.8',80))
        found.append(s.getsockname()[0]);s.close()
    except OSError:pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(),None,socket.AF_INET):found.append(info[4][0])
    except OSError:pass
    return [ip for ip in dict.fromkeys(found) if not ip.startswith('127.')]

def serve(port,host='127.0.0.1'):
    from grasp_demo import build_scene
    from rebot_pose_sync import synchronize_pose, feedback_pose
    from rebot_program import ProgramRunner, pick_place_steps, compile_program
    from rebot_writing import writing_program, CENTER
    config_store=ConfigStore(OUT/'private_config'); config_api=ConfigAPI(config_store, port)
    r=Robot(build_scene()); runner=ProgramRunner(r, output_dir=OUT/'jobs'); point_planner=PointPlanner(r, configuration=config_store.fingerprint)
    writing_document=writing_program();writing_compiled=compile_program(writing_document,r)
    r.m.vis.quality.offsamples=0
    lock=threading.RLock(); stop=threading.Event()
    def hardware_configuration():
        from rebot_motor_power import trial_proposal
        from rebot_mit_hardware import Envelope
        trial_proposal(config_store.status()['state'],list(zip(r.lower,r.upper)))
        raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
        envelope=Envelope.parse(raw)
        if any(abs(a.q_min-lo)>1e-8 or abs(a.q_max-hi)>1e-8 for a,lo,hi in zip(envelope.axes,r.lower,r.upper)):
            raise ValueError('当前模型限位与既有实机试验方案不同；未覆盖参数')
        return raw,envelope

    def resolve_hardware_point(plan_id):
        with lock:
            plan=point_planner.plans.get(plan_id)
            if not plan or plan['source']!='hardware-preview' or point_planner.clock()>=plan['expires']:
                raise ValueError('实机预演不存在、过期或已消费，请重新预演')
            if plan['configuration']!=config_store.fingerprint():
                raise ValueError('配置已改变，请重新预演')
            if plan.get('config_revision')!=config_store.status()['state']['revision']:
                raise ValueError('已保存的软件草稿已改变，请重新预演')
            from rebot_xyz_trial import assess_motion
            _,base=hardware_configuration()
            if plan.get('hardware_base_hash')!=base.fingerprint:
                raise ValueError('实机试验参数已变化或预演未绑定参数，请重新预演')
            from rebot_mit_hardware import Envelope
            raw=plan['hardware_config'];envelope=Envelope.parse(raw)
            if plan.get('hardware_config_hash')!=envelope.fingerprint:
                raise ValueError('本次任务设置已改变，请重新预演')
            assessment=assess_motion(r,plan['start'],plan['target'],envelope)
            if not assessment['execution_ready']:
                raise ValueError('；'.join(assessment['execution_reasons']))
            point_planner.plans.pop(plan_id)
            return dict(id=plan_id,expires=plan['expires'],config=raw,revision=config_store.status()['state']['revision'],
                        expected_start=plan['start'].tolist(),target=plan['target'].tolist())
    state=dict(stream_id=uuid.uuid4().hex,sequence=0,feedback_time_unix_s=0.,frame=b'',error='',motion=None,playing=False,azimuth=125.,elevation=-20.,lookat=[.1,0,.30],distance=1.4,ghost_robot=False)
    trajectories={kind:r.trajectory(kind) for kind in ('joints','circle')}
    def animate():
        try:
            with mj.Renderer(r.m,height=480,width=640) as renderer:
                cam=camera()
                while not stop.is_set():
                    start=time.monotonic()
                    with lock:
                        if state['motion'] is not None:
                            motion=state['motion'];u=min(1.,(r.d.time-motion['start'])/motion['seconds']);s=u*u*(3-2*u)
                            r.command(motion['q0']*(1-s)+motion['q1']*s,motion['f0']*(1-s)+motion['f1']*s)
                            if u>=1: state['motion']=None
                        if state['playing']:
                            t,qs,fs=trajectories[state['playing']];phase=(r.d.time-state['play_start'])%t[-1]
                            r.command(np.array([np.interp(phase,t,qs[:,j]) for j in range(6)]),float(np.interp(phase,t,fs)))
                        for _ in range(20):
                            runner.tick(float(r.m.opt.timestep))
                            r.step();runner.after_step()
                        state['sequence']+=1;state['feedback_time_unix_s']=time.time()
                        cam.azimuth=state['azimuth'];cam.elevation=state['elevation'];cam.lookat[:]=state['lookat'];cam.distance=state['distance']
                        renderer.update_scene(r.d,camera=cam)
                        runner.trail.render(renderer.scene,r,ghost_robot=state['ghost_robot'])
                        renderer.scene.flags[mj.mjtRndFlag.mjRND_SHADOW]=0
                        renderer.scene.flags[mj.mjtRndFlag.mjRND_REFLECTION]=0
                        frame=renderer.render()
                        b=io.BytesIO();Image.fromarray(frame).save(b,format='JPEG',quality=80);state['frame']=b.getvalue()
                    # Yield even when rendering exceeds the frame budget so HTTP readers
                    # and commands can acquire the shared simulation lock promptly.
                    stop.wait(max(.001,.04-(time.monotonic()-start)))
        except Exception as e:
            with lock: runner.fail(e)
            state['error']=str(e);stop.set();print('Simulation stopped:',e,flush=True)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args): pass
        def send(self,body,ctype='application/json',status=200):
            if isinstance(body,dict):body=json.dumps(body,allow_nan=False).encode()
            self.send_response(status);self.send_header('Content-Type',ctype);self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store');self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_GET(self):
            path=self.path.split('?')[0]
            # Studio's i18n moves the locale to the front of the address bar
            # (src/i18n/index.js: buildPathWithLocale), so a locale-prefixed deep
            # link has to resolve to the same resource as the unprefixed one.
            split=path.split('/',2)
            if len(split)>2 and split[1] in STUDIO_LOCALES: path='/'+split[2]
            if path.startswith('/studio/') or path.startswith('/studio-model/'):
                model = path.startswith('/studio-model/')
                root = (URDF.parent.parent if model else ROOT/'external/motorbridge-studio/dist').resolve()
                prefix = '/studio-model/' if model else '/studio/'
                resource = (root / unquote(path[len(prefix):])).resolve()
                if not resource.is_relative_to(root):
                    return self.send(dict(error='Invalid resource path'),status=403)
                if not model and path in STUDIO_ROUTES:
                    resource = root/'index.html'
                if not resource.is_file():
                    return self.send(dict(error='Studio resource not found; build Studio first'),status=404)
                return self.send(resource.read_bytes(),mimetypes.guess_type(str(resource))[0] or 'application/octet-stream')
            if path=='/':return self.send((ROOT/'scripts/dashboard.html').read_bytes(),'text/html; charset=utf-8')
            if path in ('/dashboard.css','/dashboard.js'):
                return self.send((ROOT/'scripts'/path[1:]).read_bytes(),'text/css; charset=utf-8' if path.endswith('.css') else 'text/javascript; charset=utf-8')
            if path=='/frame.jpg':
                with lock: frame=state['frame']
                return self.send(frame,'image/jpeg')
            if path=='/state':
                with lock: data=r.status();data.update(job=runner.snapshot(),backend='mujoco',feedback_source='simulation',feedback_time_unix_s=state['feedback_time_unix_s'],sequence=state['sequence'],stream_id=state['stream_id'],error=state['error'],playing=state['playing'],manual_motion=state['motion'] is not None,camera={k:state[k] for k in ('azimuth','elevation','distance','ghost_robot')},lower_deg=np.rad2deg(r.lower).tolist(),upper_deg=np.rad2deg(r.upper).tolist())
                return self.send(data)
            if path=='/studio-state':
                # Pose, task and trail must come from one coherent simulation tick.
                with lock:
                    data=r.status()
                    data.update(job=runner.snapshot(),backend='mujoco',feedback_source='simulation',
                                stream_id=state['stream_id'],sequence=state['sequence'],
                                feedback_time_unix_s=state['feedback_time_unix_s'],error=state['error'],
                                manual_motion=state['motion'] is not None,playing=state['playing'],pose_sync=state.get('pose_sync'),
                                limits_deg=np.column_stack((np.rad2deg(r.lower),np.rad2deg(r.upper))).tolist(),
                                trace_xyz=[p.tolist() for p in runner.trail.points],
                                trace_styles=list(runner.trail.styles))
                return self.send(data)
            if path=='/ros-control/state':
                try: return self.send(ros_request())
                except ValueError as e: return self.send(dict(online=False,backend='ros2-fake',hardware=False,error=str(e)))
            if path=='/motor-power':
                return self.send(motor_power.status())
            if path=='/hardware-readiness':
                return self.send(dict(readiness(config_store),power_trial=motor_power.status()))
            if path=='/point-capabilities':
                result=point_capabilities()
                try:
                    from rebot_xyz_trial import hardware_motion_options
                    raw=json.loads((ROOT/'configs/trials/seeed-nearby-xyz-pending.json').read_text())['envelope']
                    result['hardware_motion_options']=hardware_motion_options(raw)
                except (ValueError,OSError) as error: result['hardware_settings_error']=str(error)
                return self.send(result)
            if path=='/hardware-point/state': return self.send(motor_power.status())
            if path=='/machine-state':
                # 实机只读关节读数，由 rebot_twin_sync 发布；本服务不开 CAN。
                return self.send(read_report(MACHINE_LIVE,'twin_sync',max_age_s=2.))
            if path=='/machine-params':
                # 最近一次只读参数扫描，由 rebot_can_readstate 发布。
                return self.send(read_report(MACHINE_PARAMS,'can_readstate'))
            if path=='/writing-program':
                return self.send(writing_document)
            if path=='/program-example':
                return self.send(json.loads((ROOT/'programs/pick_place.json').read_text()))
            if path.startswith('/jobs/'):
                import re
                job_id=path.rsplit('/',1)[-1]
                if re.fullmatch(r'[a-f0-9]{32}',job_id):
                    with lock:
                        if runner.job and runner.job['id']==job_id:
                            return self.send(runner.snapshot())
                        report=runner.output_dir/(job_id+'.json')
                        if report.exists():
                            data=json.loads(report.read_text());data.pop('samples',None);data.pop('program',None)
                            return self.send(data)
            self.send(dict(error='Not found'),status=404)
        def do_POST(self):
            try:
                # Same-origin only: the Origin must match the Host the browser actually
                # used, so loopback, LAN IPs and hostnames all work while a cross-site
                # page (different Origin) is still rejected.
                origin=self.headers.get('Origin')
                if origin and urlsplit(origin).netloc!=self.headers.get('Host',''):raise ValueError('Origin rejected')
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<65536:raise ValueError('Invalid payload length')
                body=json.loads(self.rfile.read(n))
                if not isinstance(body,dict): raise ValueError('Expected JSON object')
                if self.path=='/kinematics/fk':
                    from rebot_kinematics import coordinate_preview
                    with lock: result=coordinate_preview(r, body.get('q_deg'))
                    return self.send(result)
                if self.path.startswith('/hardware-point/'):
                    return self.send(motor_power.request(self.path,self.headers,self.client_address[0],body))
                if self.path.startswith('/motor-power/'):
                    raise ValueError('自研实发入口暂停；本轮使用 ROS 虚拟控制栈验证，不允许真实使能')
                if self.path=='/ros-control/action':
                    if self.client_address[0] not in ('127.0.0.1','::1') or origin != 'http://'+self.headers.get('Host',''):
                        raise ValueError('只允许本机同源 ROS 验证请求')
                    op=body.get('action')
                    if op in ('connect','enable','disable','stop'): return self.send(ros_request(op))
                    with motor_power.lock, lock:
                        if runner.active or state['motion'] is not None or state['playing']: raise ValueError('请先结束仿真任务')
                        current=ros_request()
                        if not current.get('fresh'): raise ValueError('ROS 反馈过期')
                        if current.get('job',{}).get('status') in ('starting','running','cancelling'): raise ValueError('请先停止 ROS 任务')
                        if op=='preview':
                            # Pure planning from fresh Fake feedback; no enable, sync or motion.
                            return self.send(point_planner.preview({**body, 'target':'simulation', 'execution_source':'ros2-fake'}, start=current['q_rad']))
                        if op=='sync':
                            r.initialize_pose(current['q_rad']);point_planner.plans.clear();runner.trail.clear()
                            return self.send(dict(ok=True,source='ros2-fake',hardware=False,q_rad=r.q.tolist(),max_error_rad=float(np.max(np.abs(r.q-np.array(current['q_rad']))))))
                        if op=='execute':
                            plan=point_planner.plans.get(body.get('plan_id'))
                            if plan is None: raise ValueError('请先计算 XYZ 预览')
                            if not current.get('enabled'): raise ValueError('请先使能虚拟电机')
                            if np.max(np.abs(np.array(current['q_rad'])-plan['start']))>np.deg2rad(.5): raise ValueError('ROS 起点与预演不同，请重新预演')
                            document=point_planner.take(dict(plan_id=body['plan_id'],target='simulation'), start=current['q_rad'], allowed_sources=('mujoco','ros2-fake'))
                            step=document['steps'][0]
                            result=ros_request('execute',start=current['q_rad'],target=np.deg2rad(step['q_deg']).tolist(),seconds=step['seconds'])
                            state['ros_owned']=True
                            return self.send(result)
                        raise ValueError('未知 ROS 操作')
                if self.path.startswith('/config/'):
                    return self.send(config_api.request('POST', self.path, self.headers, self.client_address[0], body))
                if self.path not in ('/program','/command'): raise ValueError('Unknown endpoint')
                action='program' if self.path=='/program' else body['action']
                with motor_power.lock, lock:
                    if state['error']: raise ValueError(state['error'])
                    if state.get('ros_owned') and action not in ('camera','camera_writing','camera_robot'):
                        # Fail closed if the owner cannot be contacted. Stop remains a separate endpoint.
                        ros_state=ros_request()
                        if ros_state.get('job',{}).get('status') in ('starting','running','cancelling'):
                            raise ValueError('ROS 任务持有控制权，请先停止或等待结束')
                        state['ros_owned']=False
                    if motor_power.status()['busy'] and action not in ('camera','camera_writing','camera_robot','program_cancel','pause'):
                        raise ValueError('单轴实机检查/试验持有控制权，请先停止该试验')
                    if runner.active and action not in ('camera','camera_writing','camera_robot','program_pause','program_resume','program_cancel','pause'):
                        raise ValueError('动作程序持有控制权；请先取消或等待结束')
                    if action == 'sync_hardware_pose':
                        if state['motion'] is not None or state['playing']:
                            raise ValueError('请先停止当前仿真任务，再初始化姿态')
                        result=synchronize_pose(r, body.get('motors'), body.get('mapping'))
                        point_planner.plans.clear(); runner.trail.clear()
                        state['pose_sync']=result
                        return self.send(result)
                    if action in ('point_plan','point_execute'):
                        if state['motion'] is not None or state['playing']:
                            raise ValueError('请先停止当前仿真运动，再操作目标点')
                        if action=='point_plan':
                            if body.get('execution_source')=='hardware-preview':
                                q,_=feedback_pose(r,body.get('motors'),body.get('mapping'),start_tolerance=np.deg2rad(1))
                                result=point_planner.preview(body,start=q)
                                try:
                                    from rebot_xyz_trial import assess_motion,configure_motion
                                    base_raw,base=hardware_configuration()
                                    settings=body.get('hardware_motion')
                                    if isinstance(settings,dict) and settings.get('speed_rad_s')!=body.get('speed_rad_s'):
                                        raise ValueError('实机速度与预演速度不同，请重新预演')
                                    raw,envelope=configure_motion(base_raw,q,np.deg2rad(result['q_deg']),settings)
                                    plan=point_planner.plans[result['plan_id']]
                                    plan.update(hardware_config=raw,hardware_config_hash=envelope.fingerprint,
                                                hardware_base_hash=base.fingerprint,config_revision=config_store.status()['state']['revision'])
                                    assessment=assess_motion(r,q,np.deg2rad(result['q_deg']),envelope)
                                    result['hardware_preflight']=dict(assessment,parameters=raw,motion_settings=settings,
                                        task_timeout_s=envelope.policy.task_timeout,
                                        parameter_source='existing gains and limits; task speed and duration selected in webpage; not device parameters')
                                    if np.isfinite(assessment.get('hardware_seconds',np.nan)):
                                        result['seconds']=assessment['hardware_seconds']
                                        plan['seconds']=result['seconds']
                                except (ValueError,OSError) as error:
                                    result['hardware_preflight']=dict(execution_ready=False,execution_reasons=[str(error)])
                                return self.send(result)
                            return self.send(point_planner.preview(body))
                        document=point_planner.take(body)
                        job=runner.submit(document)
                        return self.send(dict(ok=True,job=job),status=202)
                    if action not in ('camera','camera_writing','camera_robot','ik_check'):
                        point_planner.plans.clear()
                    if action not in ('camera','camera_writing','camera_robot','program_pause','program_resume','program_cancel','pause','program','grasp_demo','print_demo','ik_check'):
                        state['playing']=False
                    if action=='camera':
                        az=float(body['azimuth']);el=float(body['elevation'])
                        if not np.isfinite(az+el): raise ValueError('Invalid camera angle')
                        distance=float(body.get('distance',state['distance']))
                        if not np.isfinite(distance) or not .35<=distance<=2.2:raise ValueError('Invalid camera distance')
                        state['azimuth']=az;state['elevation']=float(np.clip(el,-80,-5));state['distance']=distance
                    elif action in ('camera_writing','camera_robot'):
                        if action=='camera_writing':state.update(lookat=CENTER.tolist(),distance=.48,azimuth=180.,elevation=-30.,ghost_robot=True)
                        else:state.update(lookat=[.26,0,.35],distance=1.15,azimuth=140.,elevation=-25.,ghost_robot=False)
                    elif action in ('program_pause','program_resume','program_cancel'):
                        {'program_pause':runner.pause,'program_resume':runner.resume,'program_cancel':runner.cancel}[action]()
                    elif action=='pause':
                        if runner.active:
                            if runner.job['status']=='running': runner.pause()
                        else:
                            state['playing']=False;state['motion']=None;r.command(r.motion_start())
                    elif action in ('program','grasp_demo','print_demo'):
                        if state['motion'] is not None or state['playing']:
                            raise ValueError('请先保持当前位置，再提交动作程序')
                        if action=='print_demo':
                            if np.max(np.abs(r.q-HOME))>.005:raise ValueError('请先恢复初始姿态，再执行一笔书写')
                            job=runner.submit(writing_document,compiled=writing_compiled)
                        else:
                            document=body if action=='program' else dict(version=1,name='抓取并放置',steps=pick_place_steps())
                            job=runner.submit(document)
                        return self.send(dict(ok=True,job=job),status=202)
                    elif action=='reset':state['motion']=None;r.reset();runner.trail.clear()
                    elif action in ('joints','ik','ik_check'):
                        speed=float(body.get('speed_rad_s',.35))
                        if not np.isfinite(speed) or not .05<=speed<=.5:raise ValueError('speed_rad_s must be between 0.05 and 0.5')
                        mode=body.get('orientation_mode','hold')
                        if mode not in ('hold','position'):raise ValueError('orientation_mode must be hold or position')
                        q=np.deg2rad(body['q_deg']) if action=='joints' else r.ik(body['xyz'],position_only=mode=='position')
                        f=float(body.get('finger_m',r.finger_target));old_target=r.target.copy();old_f=r.finger_target
                        try:
                            r.command(q,f);r.check_path(r.motion_start(),q)
                        finally:
                            r.command(old_target,old_f)
                        if action=='ik_check':
                            return self.send(dict(ok=True,orientation_mode=mode,q_deg=np.rad2deg(q).tolist(),xyz=r.fk(q).translation.tolist(),position_error_mm=float(np.linalg.norm(r.fk(q).translation-np.asarray(body['xyz']))*1000)))
                        state['motion']=dict(q0=r.motion_start(),q1=np.asarray(q),f0=float(np.mean(r.d.qpos[r.finger_qadr])),f1=f,start=r.d.time,seconds=max(1.,1.5*float(np.max(np.abs(q-r.q)))/speed))
                        r.command(old_target,old_f)
                    elif action in ('joints_demo','circle_demo'):
                        r.reset();state['motion']=None;state['playing']=action.split('_')[0];state['play_start']=r.d.time
                    else:raise ValueError('Unknown action')
                self.send(dict(ok=True))
            except (ValueError,KeyError,TypeError,OSError) as e:self.send(dict(error=str(e)),status=400)
    def power_idle():
        with lock: return not runner.active and state['motion'] is None and not state['playing'] and not state['error']
    def power_check_pose(q):
        with lock:
            if r.contacts(np.asarray(q)): raise ValueError('当前真实姿态在模型中有碰撞，请核对现场与仿真场景')
    motor_power=MotorPowerAPI(config_store,port,ROOT,list(zip(r.lower,r.upper)),power_check_pose,power_idle,point_plan=resolve_hardware_point)
    worker=threading.Thread(target=animate,daemon=True);worker.start()
    server=ThreadingHTTPServer((host,port),Handler)
    print(f'RS 仿真控制台：http://127.0.0.1:{port}',flush=True)
    if host not in ('127.0.0.1','localhost'):
        for ip in lan_addresses():print(f'  局域网共享：http://{ip}:{port}/studio/console',flush=True)
        print('  ⚠️ 局域网模式无鉴权：同网段任何人都能下发仿真指令；/machine-state 会暴露实机读数。',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        motor_power.close()
        stop.set();server.server_close();worker.join(timeout=5)
        with lock: runner.fail('仿真服务关闭')

def meshcat_demo():
    import sys
    sys.path.insert(0,str(ROOT/'external/reBotArm_control_py'))
    from example.sim.visualizer import Visualizer
    r=Robot();viz=Visualizer(urdf_path=str(URDF));t,qs,fs=r.trajectory('circle')
    points=[r.fk(q).translation.tolist() for q in qs];viz.draw_ref_path(points)
    print('Meshcat: Ctrl+C 退出',flush=True)
    try:
        while True:
            for q,f in zip(qs,fs):viz.update(np.r_[q,f,f]);time.sleep(.02)
    except KeyboardInterrupt:pass

def main():
    parser=argparse.ArgumentParser(description='reBot RS 纯仿真工具')
    parser.add_argument('mode',choices=['serve','verify','demo','meshcat','grasp'],nargs='?',default='serve')
    parser.add_argument('--trials',type=int,default=1)
    parser.add_argument('--port',type=int,default=8765);parser.add_argument('--host',default='127.0.0.1',help='绑定地址，局域网共享用 0.0.0.0（默认仅本机可访问）');parser.add_argument('--kind',choices=['joints','circle'],default='joints');parser.add_argument('--seconds',type=float,default=10);parser.add_argument('--video',action='store_true')
    a=parser.parse_args();OUT.mkdir(exist_ok=True)
    if a.mode=='verify':verify()
    elif a.mode=='demo':demo(a.kind,a.seconds,a.video)
    elif a.mode=='meshcat':meshcat_demo()
    elif a.mode=='grasp':
        from grasp_demo import run
        run(a.video,a.trials)
    else:serve(a.port,a.host)
if __name__=='__main__':main()
