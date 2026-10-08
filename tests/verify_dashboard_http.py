"""Dashboard API checks against an idle, local simulation. No hardware access."""
import json
import time
import urllib.error
import urllib.request
from pathlib import Path

BASE='http://127.0.0.1:8765'
def get(path='/state'):
    return json.load(urllib.request.urlopen(BASE+path,timeout=5))
def command(action,**fields):
    req=urllib.request.Request(BASE+'/command',data=json.dumps(dict(action=action,**fields)).encode(),headers={'Content-Type':'application/json'})
    return json.load(urllib.request.urlopen(req,timeout=5))
def rejected(action,**fields):
    try:command(action,**fields)
    except urllib.error.HTTPError as e:
        assert e.code==400
        return json.load(e)['error']
    raise AssertionError('Invalid command accepted')

s=get()
assert not s.get('job') or s['job']['status'] not in ('running','paused')
assert not s['manual_motion'] and not s['playing']
for path,ctype in [('/dashboard.js','text/javascript'),('/dashboard.css','text/css')]:
    with urllib.request.urlopen(BASE+path) as response:
        assert ctype in response.headers['Content-Type'] and len(response.read())>1000
assert s['feedback_source']=='simulation' and s['backend']=='mujoco'
assert 0<=time.time()-s['feedback_time_unix_s']<2
for speed in [0,1,float('nan')]:
    rejected('joints',q_deg=s['q_deg'],speed_rad_s=speed)
assert not get()['manual_motion']
rejected('camera',azimuth=140,elevation=-25,distance=0)
assert get()['camera']==s['camera']
command('camera',azimuth=140,elevation=-25,distance=1.1)
assert get()['camera']['distance']==1.1
command('camera',**{k:s['camera'][k] for k in ('azimuth','elevation','distance')})
command('joints',q_deg=s['q_deg'],speed_rad_s=.1)
assert get()['manual_motion']
command('pause')
assert not get()['manual_motion']
report=dict(success=True,static_assets=True,fresh_simulation_feedback=True,camera_zoom=True,invalid_camera_rejected=True,manual_motion_status=True,invalid_speed_rejected=True,manual_hold=True)
(Path(__file__).resolve().parents[1]/'outputs/dashboard_http_verification.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
