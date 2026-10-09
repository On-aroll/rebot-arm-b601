"""Synthetic telemetry -> live local simulation HTTP; no CAN/SDK/browser action."""
import json
import math
from pathlib import Path
import time
import urllib.request
import urllib.error
BASE='http://127.0.0.1:8765'
def get(path):
    with urllib.request.urlopen(BASE+path,timeout=10) as response: return json.load(response)
def post(body):
    req=urllib.request.Request(BASE+'/command',json.dumps(body).encode(),{'Content-Type':'application/json','Origin':BASE})
    with urllib.request.urlopen(req,timeout=20) as response: return json.load(response)
def rejected(body):
    try: post(body)
    except urllib.error.HTTPError as error:
        assert error.code==400; return json.load(error)['error']
    raise AssertionError('invalid request accepted')
state=get('/studio-state'); assert not state['manual_motion'] and not state['playing'] and (state.get('job') or {}).get('status') not in ('running','paused')
ready=get('/hardware-readiness'); assert ready['hardware_allowed'] is False and ready['auto_connect'] is False
assert ready['configuration_revision']>=0 and ready['reasons']
plan=post(dict(action='point_plan',xyz_mm=[v*1000 for v in state['xyz']],orientation_mode='position',speed_rad_s=.15))
q=[0.03,.72,1.09,.01,0.,.01]
mapping=dict(version=1,axes=[dict(motorId=i,direction=1,offsetDeg=0) for i in range(1,7)])
def body():
    return dict(action='sync_hardware_pose',mapping=mapping,motors={str(i):dict(id=i,model='rs-06' if i<=3 else 'rs-00',mechPosRad=q[i-1],positionUpdatedAt=time.time()*1000,error=None) for i in range(1,7)})
result=post(body()); assert result['ok'] and not result['hardware_motion'] and not result['hardware_start_authority']
assert result['max_error_rad']<1e-10
st=get('/studio-state'); assert st['pose_sync']['synchronization_id']==result['synchronization_id']
assert max(abs(math.radians(deg)-rad) for deg,rad in zip(st['target_deg'],q))<1e-8
assert '不存在' in rejected(dict(action='point_execute',plan_id=plan['plan_id'],target='simulation'))
bad=body(); bad['motors']['1']['positionUpdatedAt']=1
assert '超过' in rejected(bad)
assert get('/studio-state')['pose_sync']['synchronization_id']==result['synchronization_id']
again=post(body()); assert again['max_error_rad']<1e-10
assert '实机同步' in rejected(dict(action='point_plan',target='hardware',xyz_mm=[200,150,200]))
report=dict(passed=True,source='synthetic-http-telemetry',real_CAN_queries=0,real_motion_commands=0,
    error_rad=result['max_error_rad'],fk_xyz_mm=result['xyz_mm'],qpos_and_target_verified=True,
    old_plan_invalidated=True,stale_rejected=True,repeated_sync_passed=True,hardware_locked=True)
path=Path('outputs/mit-p4/pose-sync-http.json');path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False))
