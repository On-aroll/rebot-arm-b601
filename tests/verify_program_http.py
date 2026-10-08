"""Local API contract tests, run against an idle simulation server."""
import json
import sys
import tempfile
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from rebot_client import RebotClient
from rebot_core import Robot, ROOT
from grasp_demo import build_scene
from rebot_program import ProgramRunner

# Abruptly interrupted tasks become failed on next start, not forever running.
with tempfile.TemporaryDirectory() as tmp:
    path=Path(tmp)/('a'*32+'.json')
    path.write_text(json.dumps(dict(status='running')))
    runner=ProgramRunner(Robot(build_scene()),tmp)
    assert json.loads(path.read_text())['status']=='failed'

client=RebotClient()
state=client.state()
if state.get('job',{}):
    assert state['job']['status'] not in ('running','paused'),'Wait for current demonstration first'
client.control('reset')
q=client.state()['q_deg']
program=dict(version=1,name='API 验证',steps=[dict(op='wait',seconds=2)])
job=client.submit(program)
for action in ('reset','joints_demo'):
    try: client.control(action);raise AssertionError('Manual command overrode program')
    except RuntimeError as e: assert '控制权' in str(e)
try:client.submit(program);raise AssertionError('Concurrent submission accepted')
except RuntimeError:pass
client.control('program_pause');assert client.job(job['id'])['status']=='paused'
client.control('program_resume');client.wait(job['id'],timeout=20)
first=job['id']
job=client.submit(dict(version=1,name='取消验证',steps=[dict(op='wait',seconds=10)]))
client.control('program_cancel');assert client.job(job['id'])['status']=='cancelled'
assert client.job(first)['status']=='succeeded','Historical job lookup lost'
try:client.submit(dict(version=1,steps=[dict(op='move_joints',q_deg=[999]*6)]));raise AssertionError('Invalid joint accepted')
except RuntimeError:pass
assert client.job(job['id'])['status']=='cancelled'
report=dict(success=True,exclusive_writer=True,pause_resume=True,cancel=True,history_lookup=True,invalid_program_rejected=True,interrupted_job_recovery=True)
(ROOT/'outputs/program_http_verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
