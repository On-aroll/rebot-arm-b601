"""Offline tests: never opens CAN and uses a temporary job directory."""
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from rebot_core import Robot
from rebot_point import PointPlanner
from rebot_program import ProgramRunner
from grasp_demo import build_scene

r = Robot(build_scene())
now = [0.]
p = PointPlanner(r, clock=lambda: now[0])
body = dict(xyz_mm=[200, 150, 200], orientation_mode='position', speed_rad_s=.15)
old_q, old_target, old_f = r.q.copy(), r.target.copy(), r.finger_target
plan = p.preview(body)
assert plan['position_error_mm'] < .01
assert len(plan['path_xyz']) == 61
assert np.array_equal(r.q, old_q) and np.array_equal(r.target, old_target)
assert r.finger_target == old_f
assert plan['hardware'] is False


def rejected(callback, contains):
    try:
        callback()
    except ValueError as error:
        assert contains in str(error), str(error)
    else:
        raise AssertionError('Expected rejection: ' + contains)


for invalid in ([1, 2], [None, 0, 0], [float('nan'), 0, 0]):
    rejected(lambda: p.preview({**body, 'xyz_mm': invalid}), '有限')
rejected(lambda: p.preview({**body, 'speed_rad_s': 0}), '速度')
rejected(lambda: p.preview({**body, 'xyz_mm': [9000, 9000, 9000]}), '求解')
rejected(lambda: p.preview({**body, 'target': 'hardware'}), '实机同步')
rejected(lambda: p.take({'target': 'hardware', 'plan_id': plan['plan_id']}), '实机同步')
now[0] = 61
rejected(lambda: p.take({'plan_id': plan['plan_id']}), '过期')
plan = p.preview(body)
r.d.qpos[r.qadr[0]] -= .02
rejected(lambda: p.take({'plan_id': plan['plan_id']}), '姿态已改变')
r.reset()
plan = p.preview(body)
document = p.take({'plan_id': plan['plan_id']})
rejected(lambda: p.take({'plan_id': plan['plan_id']}), '预览不存在')
with tempfile.TemporaryDirectory() as folder:
    runner = ProgramRunner(r, output_dir=folder)
    runner.submit(document)
    for _ in range(int((plan['seconds'] + 4) / r.m.opt.timestep)):
        runner.tick(float(r.m.opt.timestep))
        r.step()
        runner.after_step()
        if not runner.active:
            break
    assert runner.snapshot()['status'] == 'succeeded', runner.snapshot()
    error = float(np.linalg.norm(r.fk(r.q).translation * 1000 - body['xyz_mm']))
    assert error < 2, error
    print(f'PASS: preview read-only; invalid/expired/drift/replay/hardware rejected; actual arrival error {error:.4f} mm')

version = ["v1"]
p = PointPlanner(r, configuration=lambda: version[0])
plan = p.preview(body)
version[0] = "v2"
rejected(lambda: p.take({'plan_id': plan['plan_id']}), '版本已改变')
print('PASS: configuration change invalidates point plan')
