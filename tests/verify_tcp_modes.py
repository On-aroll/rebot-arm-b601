"""Regression for user target (200, 150, 200) mm and explicit TCP orientation."""
import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_core import Robot,HOME
from grasp_demo import build_scene

xyz=np.array([.2,.15,.2])
writing=np.deg2rad([-36.19793854739449,70.67192568764415,25.78170497126607,80.37390983706315,-30.691049408446553,20.050958353515142])
results=[]
for name,seed in [('home',HOME),('writing_end',writing)]:
    r=Robot(build_scene());r.d.qpos[r.qadr]=-seed;r.command(seed)
    try:r.ik(xyz,seed=seed)
    except ValueError as e:assert '朝向' in str(e)
    else:raise AssertionError('Expected fixed-orientation reproduction')
    target=r.ik(xyz,seed=seed,position_only=True)
    assert np.all(target>=r.lower-1e-8) and np.all(target<=r.upper+1e-8)
    assert np.linalg.norm(r.fk(target).translation-xyz)<1e-6
    duration=max(1.,1.5*np.max(np.abs(target-seed))/.25)
    dt=r.m.opt.timestep
    for i in range(int((duration+2)/dt)+1):
        u=min(1.,i*dt/duration);smooth=u*u*(3-2*u)
        r.command(seed*(1-smooth)+target*smooth);r.step()
    error=float(np.linalg.norm(r.fk(r.q).translation-xyz)*1000)
    assert error<2,error
    assert not any(w.number for w in r.d.warning)
    results.append(dict(seed=name,q_deg=np.rad2deg(target).tolist(),actual_xyz_mm=(r.fk(r.q).translation*1000).tolist(),actual_error_mm=error))
# Existing full-pose behavior remains available and preserves orientation.
r=Robot(build_scene());rot=r.fk(HOME).rotation.copy();q=r.ik([.4,0,.4]);assert np.linalg.norm(r.fk(q).rotation-rot)<1e-5
report=dict(success=True,target_xyz_mm=(xyz*1000).tolist(),fixed_pose_regression=True,runs=results)
(Path(__file__).resolve().parents[1]/'outputs/tcp_modes_verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
