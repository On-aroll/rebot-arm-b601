"""Run a simulation-only point move against the local idle server; never uses CAN."""
import json
import math
import re
import time
import urllib.error
import urllib.request
from pathlib import Path

BASE = 'http://127.0.0.1:8765'


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=15) as response:
        return json.load(response)


def command(body):
    request = urllib.request.Request(BASE + '/command', data=json.dumps(body).encode(),
                                     headers={'Content-Type': 'application/json', 'Origin': BASE})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


state = get('/studio-state')
assert not state['manual_motion'] and not state['playing']
assert (state.get('job') or {}).get('status') not in ['running', 'paused']
assert not state['error']
capabilities = get('/point-capabilities')
assert capabilities['simulation'] is True and capabilities['hardware'] is False
with urllib.request.urlopen(BASE + '/zh/studio/console', timeout=15) as response:
    html = response.read().decode()
asset = re.search(r'src="(/studio/assets/[^\"]+\.js)"', html).group(1)
with urllib.request.urlopen(BASE + asset, timeout=15) as response:
    assert '执行仿真到点' in response.read().decode()
body = dict(action='point_plan', target='simulation', xyz_mm=[200, 150, 200],
            orientation_mode='position', speed_rad_s=.15)
plan = command(body)
after = get('/studio-state')
assert after['target_deg'] == state['target_deg'], 'Preview changed targets'
assert after['job'] == state['job'], 'Preview changed job'
try:
    command(dict(action='point_execute', target='hardware', plan_id=plan['plan_id']))
except urllib.error.HTTPError as error:
    assert error.code == 400 and '实机同步' in json.load(error)['error']
else:
    raise AssertionError('Hardware execution must be rejected')
execution = command(dict(action='point_execute', target='simulation', plan_id=plan['plan_id']))
job_id = execution['job']['id']
print(f'Started simulation point job {job_id}; nominal duration {plan["seconds"]:.1f}s', flush=True)
deadline = time.monotonic() + 120
while time.monotonic() < deadline:
    state = get('/studio-state')
    assert not state['error'], state['error']
    job = state['job']
    assert job['id'] == job_id, 'Another task replaced the point job'
    if job['status'] not in ['running', 'paused']:
        break
    time.sleep(.4)
else:
    command({'action':'program_cancel'})
    raise AssertionError('Simulation point execution timed out')
assert job['status'] == 'succeeded', job
error_mm = math.dist([v * 1000 for v in state['xyz']], plan['xyz_mm'])
assert error_mm < 2, error_mm
report = dict(success=True, hardware_commands_sent=False, target_xyz_mm=plan['xyz_mm'],
              actual_xyz_mm=[v * 1000 for v in state['xyz']], error_mm=error_mm,
              job_id=job_id, frontend_asset=asset, preview_read_only=True,
              hardware_execution_rejected=True, service_stream_id=state['stream_id'])
(Path(__file__).resolve().parents[1] / 'outputs/point_http_verification.json').write_text(
    json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False, indent=2))
