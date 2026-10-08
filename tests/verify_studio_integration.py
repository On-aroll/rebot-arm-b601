"""Read-only HTTP checks and numerical-boundary regression. Does not start motion."""
import json
import sys
import urllib.request
import urllib.error
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from rebot_core import Robot, URDF

BASE = 'http://127.0.0.1:8765'
def get(path):
    return urllib.request.urlopen(BASE+path, timeout=10)

assert b'/studio/assets/' in get('/studio/writing').read()
assert get('/studio-model/urdf/ReBot_Arm_RS.urdf').read() == URDF.read_bytes()
for path in ('/studio-model/%2e%2e/reBotArm_control_py/__init__.py', '/studio/%2e%2e/package.json'):
    try:
        get(path)
        raise AssertionError('Traversal was accepted')
    except urllib.error.HTTPError as error:
        assert error.code == 403
state = json.load(get('/studio-state'))
assert state['feedback_source'] == 'simulation' and state['backend'] == 'mujoco'
assert not state['error']
assert len(state['q_rad']) == 6
assert len(state['trace_styles']) == max(0, len(state['trace_xyz'])-1)
robot = Robot()
q = robot.q.copy(); q[1] = robot.lower[1] - 1.02e-8
robot.d.qpos[robot.qadr] = -q
start = robot.motion_start()
assert robot.q[1] < robot.lower[1], 'Raw feedback must remain intact'
robot.command(start)
assert start[1] == robot.lower[1]
q[1] = robot.lower[1] - 0.01
robot.d.qpos[robot.qadr] = -q
try:
    robot.motion_start()
    raise AssertionError('Large violation should not be silently clipped')
except ValueError:
    pass
report = dict(success=True, studio_assets=True, identical_rs_model=True,
              traversal_rejected=True, coherent_feedback=True,
              tiny_sim_overshoot_handled=True, large_violation_rejected=True,
              raw_feedback_preserved=True, hardware_motion=False)
(ROOT/'outputs/studio_integration_verification.json').write_text(json.dumps(report, indent=2))
print(json.dumps(report))
