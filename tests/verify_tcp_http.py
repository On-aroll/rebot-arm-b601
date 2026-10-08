"""Read-only TCP preview must not send joint or gripper commands."""
import json,urllib.request,urllib.error
from pathlib import Path
BASE='http://127.0.0.1:8765'
def state():return json.load(urllib.request.urlopen(BASE+'/state',timeout=5))
def check(**fields):
    body=dict(action='ik_check',xyz=[.2,.15,.2],**fields)
    req=urllib.request.Request(BASE+'/command',data=json.dumps(body).encode(),headers={'Content-Type':'application/json'})
    return json.load(urllib.request.urlopen(req,timeout=5))
a=state();assert not a['manual_motion'] and not a['playing'] and not a.get('job')
result=check(orientation_mode='position');assert result['position_error_mm']<.01
b=state()
for key in ['target_deg','finger_target_m','manual_motion','playing','job']:assert a[key]==b[key],key
assert max(abs(x-y) for x,y in zip(a['q_deg'],b['q_deg']))<.01
try:check(orientation_mode='hold')
except urllib.error.HTTPError as e:assert e.code==400 and '朝向' in json.load(e)['error']
else:raise AssertionError('Hold mode should reproduce failure from HOME')
try:check(orientation_mode='unknown')
except urllib.error.HTTPError as e:assert e.code==400
else:raise AssertionError('Invalid orientation mode accepted')
report=dict(success=True,preview_without_motion=True,preview_keeps_targets=True,user_target_position_mode=True,hold_failure_explained=True,invalid_mode_rejected=True)
(Path(__file__).resolve().parents[1]/'outputs/tcp_http_verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
