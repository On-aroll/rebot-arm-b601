"""Validate actual physics feedback tracing one continuous 国庆快乐 path."""
import json,sys,time
from pathlib import Path
import numpy as np
import mujoco as mj
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_core import Robot,ROOT
from grasp_demo import build_scene
from rebot_program import ProgramRunner
from rebot_writing import writing_program,print_camera,CENTER,RIGHT,UP,NORMAL

r=Robot(build_scene());runner=ProgramRunner(r,ROOT/'outputs/writing-tests')
start=time.monotonic();runner.submit(writing_program());trace=runner.steps[4]['trace']
errors=[];max_q_error=0.
while runner.active and r.d.time<600:
    runner.tick(float(r.m.opt.timestep));r.step();runner.after_step()
    if runner.index==4:
        actual=r.fk(r.q).translation
        expected=np.array([np.interp(min(runner.elapsed,runner.duration),trace['times'],trace['xyz'][:,i]) for i in range(3)])
        errors.append(float(np.linalg.norm(actual-expected)))
        max_q_error=max(max_q_error,float(np.max(np.abs(r.q-r.target))))
assert runner.job['status']=='succeeded',runner.snapshot()
points=np.array(runner.trail.points)
assert len(points)>1000 and len(runner.trail.styles)==len(points)-1
assert set(runner.trail.styles)=={0,1,2,3,4}
# Distance from actual trace samples to the nearest intended segment (shape error).
a=trace['points'][:-1];b=trace['points'][1:];ab=b-a;denom=np.sum(ab*ab,axis=1)
shape_errors=[]
for p in points:
    t=np.clip(np.sum((p-a)*ab,axis=1)/denom,0,1)
    shape_errors.append(float(np.min(np.linalg.norm(a+t[:,None]*ab-p,axis=1))))
assert max(shape_errors)<.003, max(shape_errors)
# Every intended text segment must actually have been traced in its letter color.
coverage=[]
styles=np.array(runner.trail.styles)
for a0,b0,style in zip(trace['points'],trace['points'][1:],trace['styles']):
    if style==4:continue
    ink=points[1:][styles==style]
    for t in np.linspace(0,1,7):coverage.append(float(np.min(np.linalg.norm(ink-(a0+(b0-a0)*t),axis=1))))
assert max(coverage)<.0025,max(coverage)
assert not any(r.d.warning.number)
assert np.max(np.linalg.norm(np.diff(points,axis=0),axis=1))<.003
renderer=mj.Renderer(r.m,height=720,width=960,max_geom=10000)
cam=print_camera();cam.lookat[:]=CENTER;cam.distance=.48;cam.azimuth=180;cam.elevation=-30
renderer.update_scene(r.d,camera=cam);runner.trail.render(renderer.scene,r,ghost_robot=True)
Image.fromarray(renderer.render()).save(ROOT/'outputs/national_day_one_stroke.png')
cam.azimuth=140;cam.elevation=-17;cam.distance=.90;cam.lookat[:]=[.29,0,.36]
renderer.update_scene(r.d,camera=cam);runner.trail.render(renderer.scene,r)
Image.fromarray(renderer.render()).save(ROOT/'outputs/national_day_3d.png');renderer.close()
report=dict(success=True,text='国庆快乐',continuous_path=True,pen_lifts_within_text=0,
            actual_trace_points=len(points),actual_path_length_m=runner.trail.length,
            simulated_seconds=float(r.d.time),wall_seconds=time.monotonic()-start,
            max_time_aligned_tracking_error_m=max(errors),max_shape_error_m=max(shape_errors),max_text_coverage_error_m=max(coverage),
            max_joint_tracking_error_deg=float(np.rad2deg(max_q_error)),
            writing_plane_tilt_deg=60,connector_depth_m=.012,
            actual_depth_range_m=float(np.ptp((points-CENTER)@NORMAL)),
            warnings=r.d.warning.number.tolist(),job_id=runner.job['id'])
(ROOT/'outputs/writing_verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False,indent=2))
