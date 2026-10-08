"""Trace validation and pause/resume/cancel preserve path progress."""
import sys,json,tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_core import Robot,ROOT
from rebot_program import ProgramRunner
from grasp_demo import build_scene

r=Robot(build_scene());p=r.fk(r.q).translation
path=dict(op='trace_path',points=[p.tolist(),(p+[0,.012,0]).tolist(),(p+[.006,.012,0]).tolist(),p.tolist()],speed_m_s=.02,styles=[0,1,2])
with tempfile.TemporaryDirectory() as tmp:
    runner=ProgramRunner(r,tmp);runner.submit(dict(version=1,steps=[path]))
    for _ in range(200):runner.tick(.002);r.step();runner.after_step()
    runner.pause();elapsed=runner.elapsed;count=len(runner.trail.points)
    for _ in range(150):runner.tick(.002);r.step();runner.after_step()
    assert runner.elapsed==elapsed and len(runner.trail.points)==count
    runner.resume();assert runner.elapsed==elapsed
    while runner.active and r.d.time<20:runner.tick(.002);r.step();runner.after_step()
    assert runner.job['status']=='succeeded'
    assert np.linalg.norm(r.fk(r.q).translation-p)<.001
    r.reset();runner.submit(dict(version=1,steps=[path]));runner.tick(.002);r.step();runner.after_step();runner.cancel()
    count=len(runner.trail.points)
    for _ in range(50):runner.tick(.002);r.step();runner.after_step()
    assert len(runner.trail.points)==count and runner.job['status']=='cancelled'
    r.reset()
    for bad in [dict(path,points=[p.tolist(),p.tolist()]),dict(path,styles=[9]*3),dict(path,speed_m_s=float('nan')),dict(path,orientation=[[1,0,0]]*3)]:
        before=r.target.copy()
        try:runner.submit(dict(version=1,steps=[bad]));raise AssertionError('Invalid path accepted')
        except ValueError:pass
        assert np.array_equal(before,r.target)
report=dict(success=True,pause_resume_progress_preserved=True,cancel_stops_recording=True,invalid_path_cases_rejected=4)
(ROOT/'outputs/trace_control_verification.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
