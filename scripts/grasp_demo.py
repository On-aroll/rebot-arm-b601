"""Contact-only pick/place demonstration; no attachment constraints or object teleportation."""
import argparse,json,xml.etree.ElementTree as ET,time
import numpy as np
import mujoco as mj
from rebot_core import Robot,ROOT,HOME

def build_scene():
    source=ROOT/'external/mujoco_menagerie/seeed_rebot_devarm/seeed_rebot_devarm.xml'
    tree=ET.parse(source);root=tree.getroot()
    root.find('compiler').set('meshdir','../external/mujoco_menagerie/seeed_rebot_devarm/assets')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='960',offheight='720')
    ET.SubElement(visual,'headlight',ambient='.4 .4 .4',diffuse='.7 .7 .7')
    world=root.find('worldbody')
    ET.SubElement(world,'light',pos='0 -1 2',dir='0 0 -1',directional='true')
    ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 .05',rgba='.15 .22 .27 1',friction='1 .005 .0001')
    ET.SubElement(world,'geom',name='table',type='box',pos='.3 0 .015',size='.20 .20 .015',rgba='.38 .47 .52 1',friction='1 .005 .0001')
    block=ET.SubElement(world,'body',name='block',pos='.25 0 .05')
    ET.SubElement(block,'freejoint',name='block_free')
    ET.SubElement(block,'geom',name='block_geom',type='box',size='.02 .02 .02',mass='.03',rgba='.96 .49 .13 1',friction='1 .01 .001',condim='4',solref='.005 1')
    # Restrict the gripper's huge upstream force rating for this light test object.
    for actuator in root.find('actuator'):
        if actuator.get('name') in ('joint_left','joint_right'): actuator.set('forcerange','-20 20')
    path=ROOT/'models/rs_grasp_scene.xml';tree.write(path,encoding='unicode');return path

def plan():
    r=Robot();R=np.array([[0,0,1],[0,1,0],[-1,0,0]],float)
    poses=[('approach',[.25,0,.14],.032,3),('descend',[.25,0,.04],.032,3),
           ('close',[.25,0,.04],.016,2),('lift',[.25,0,.14],.016,3),
           ('hold',[.25,0,.14],.016,2),('transfer',[.25,.065,.14],.016,3),
           ('place',[.25,.065,.04],.016,3),('release',[.25,.065,.04],.032,2),
           ('retreat',[.25,.065,.14],.032,3)]
    qs=[];seed=HOME.copy()
    for name,xyz,grip,duration in poses:
        seed=r.ik(xyz,R,seed);qs.append((name,seed.copy(),grip,duration))
    return qs

def run(video=False,trials=1):
    scene=build_scene();schedule=plan();reports=[]
    for trial in range(trials):
        r=Robot(scene);bid=r.m.body('block').id;r.finger_target=.032;r.reset()
        # Small declared position variations in subsequent trials.
        qa=r.m.jnt_qposadr[r.m.joint('block_free').id]
        offset=(trial%5-2)*.0005 if trials>1 else 0.
        r.d.qpos[qa+1]+=offset;mj.mj_forward(r.m,r.d)
        renderer=writer=None
        if video and trial==0:
            import imageio.v2 as imageio
            renderer=mj.Renderer(r.m,height=720,width=960)
            writer=imageio.get_writer(str(ROOT/'outputs/grasp_demo.mp4'),fps=30)
            cam=mj.MjvCamera();cam.lookat[:]=[.19,0,.23];cam.distance=1.10;cam.azimuth=125;cam.elevation=-25
        max_z=.05;hold_z=[];stages=[];contact_steps=0;next_frame=0.;tracking=0
        start=time.perf_counter()
        try:
            for name,target,finger,duration in schedule:
                q0=r.q;f0=float(np.mean(r.d.qpos[r.finger_qadr]))
                for i in range(round(duration/r.m.opt.timestep)):
                    u=min(1,(i+1)*r.m.opt.timestep/duration);s=u*u*(3-2*u)
                    r.command(q0*(1-s)+target*s,f0*(1-s)+finger*s);r.step()
                    z=float(r.d.xpos[bid,2]);max_z=max(max_z,z)
                    if name=='hold':hold_z.append(z)
                    if any(bid in (r.m.geom_bodyid[c.geom1],r.m.geom_bodyid[c.geom2]) and (r.m.geom_bodyid[c.geom1] in [r.m.body('gripper_left').id,r.m.body('gripper_right').id] or r.m.geom_bodyid[c.geom2] in [r.m.body('gripper_left').id,r.m.body('gripper_right').id]) for c in r.d.contact):contact_steps+=1
                    tracking=max(tracking,float(np.max(np.abs(r.q-r.target))))
                    if renderer and r.d.time>=next_frame:
                        renderer.update_scene(r.d,camera=cam);frame=renderer.render();writer.append_data(frame);next_frame+=1/30
                        if name=='hold' and not (ROOT/'outputs/grasp_preview.png').exists():
                            from PIL import Image
                            Image.fromarray(frame).save(ROOT/'outputs/grasp_preview.png')
                stages.append(dict(stage=name,block_xyz=r.d.xpos[bid].tolist(),fingers=r.d.qpos[r.finger_qadr].tolist()))
            final=r.d.xpos[bid].copy()
            success=bool(min(hold_z)>.10 and np.linalg.norm(final[:2]-[.25,.065])<.025 and abs(final[2]-.05)<.008)
            report=dict(trial=trial,initial_y_offset_m=offset,success=success,max_lift_m=max_z-.05,hold_min_lift_m=min(hold_z)-.05,hold_seconds=2,final_xyz=final.tolist(),contact_steps=contact_steps,joint_max_error_deg=float(np.rad2deg(tracking)),warnings=r.d.warning.number.tolist(),stages=stages,wall_seconds=time.perf_counter()-start)
            reports.append(report);print(json.dumps(report),flush=True)
        finally:
            if renderer:renderer.close()
            if writer:writer.close()
    result=dict(successes=sum(x['success'] for x in reports),trials=trials,description='40 mm cube, 30 g, contact/friction only, +/-1 mm initial y offsets for multi-trial verification; model gripper max force 20 N per finger; simulator arm bias compensation.',results=reports)
    (ROOT/'outputs/grasp_verification.json').write_text(json.dumps(result,indent=2))
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--video',action='store_true');p.add_argument('--trials',type=int,default=1);a=p.parse_args();run(a.video,a.trials)
