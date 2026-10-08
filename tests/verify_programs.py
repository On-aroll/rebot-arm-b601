"""Physics integration tests: useful motion, failure, ownership and interruption."""
import json
import sys
import tempfile
from pathlib import Path
import numpy as np
import mujoco as mj
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rebot_core import Robot, ROOT
from grasp_demo import build_scene
from rebot_program import ProgramRunner, pick_place_steps

results=[]

def run(name, document, expected='succeeded', setup=None, intervention=None, limit=180):
    r=Robot(build_scene())
    if setup: setup(r)
    runner=ProgramRunner(r, ROOT/'outputs/program-tests')
    runner.submit(document)
    interrupted=False
    while runner.active and r.d.time < limit:
        if intervention and not interrupted and r.d.time>2:
            intervention(r,runner);interrupted=True
        runner.tick(float(r.m.opt.timestep));r.step()
    result=dict(name=name,status=runner.job['status'],error=runner.job['error'],
                sim_time_s=float(r.d.time),completed_steps=len(runner.job['completed_steps']),
                block_xyz=r.d.xpos[r.m.body('block').id].tolist(),
                warnings=r.d.warning.number.tolist(),job_id=runner.job['id'])
    print(json.dumps(result,ensure_ascii=False),flush=True)
    assert result['status']==expected,result
    assert not any(result['warnings']),result
    results.append(result)
    return r,runner

program=dict(version=1,name='往返搬运',steps=pick_place_steps()+pick_place_steps((.25,.065,.05),(.25,0,.05)))
run('pick_and_return',program)

def pause_resume(r,runner):
    try:
        runner.submit(program)
        raise AssertionError('Concurrent program accepted')
    except ValueError: pass
    runner.pause();elapsed=runner.elapsed;q=r.q.copy()
    for _ in range(250): runner.tick(float(r.m.opt.timestep));r.step()
    assert runner.elapsed==elapsed and runner.job['status']=='paused'
    assert np.max(np.abs(r.q-q)) < .04
    runner.resume()
run('pause_resume_and_exclusive_writer',dict(version=1,name='暂停继续',steps=pick_place_steps()),intervention=pause_resume)
run('cancel',program,expected='cancelled',intervention=lambda r,p:p.cancel())
run('wrong_object_location',dict(version=1,name='错误断言',steps=[dict(op='assert_object',object='block',xyz=[.25,.15,.05],tolerance_m=.01)]),expected='failed')
run('no_object_grip_timeout',dict(version=1,name='空抓检测',steps=[dict(op='gripper',finger_m=.016,completion='contact',seconds=.5)]),expected='failed')
# Freeze measured state: elapsed time alone must never yield success.
r=Robot(build_scene());runner=ProgramRunner(r,ROOT/'outputs/program-tests')
runner.submit(dict(version=1,steps=[dict(op='move_joints',q_deg=np.rad2deg(r.q+[.1,0,0,0,0,0]).tolist(),seconds=.5)]))
for _ in range(2200): runner.tick(.002)
assert runner.job['status']=='failed' and '超时' in runner.job['error']
results.append(dict(name='frozen_feedback_timeout',status=runner.job['status'],error=runner.job['error']))
# Invalid later steps must reject the whole program without changing targets.
for bad in [dict(op='move_joints',q_deg=[999]*6),dict(op='move_tcp',xyz=[10,10,10]),dict(op='wait',seconds=float('nan')),dict(op='wait',typo=1)]:
    r=Robot(build_scene());runner=ProgramRunner(r,ROOT/'outputs/program-tests');before=r.target.copy()
    try:
        runner.submit(dict(version=1,steps=[dict(op='wait',seconds=1),bad]))
        raise AssertionError('Invalid program accepted')
    except ValueError: pass
    assert runner.job is None and np.array_equal(r.target,before)
results.append(dict(name='invalid_program_atomic_rejection',cases=4,status='passed'))
(ROOT/'outputs/program_verification.json').write_text(json.dumps(dict(results=results),ensure_ascii=False,indent=2))
