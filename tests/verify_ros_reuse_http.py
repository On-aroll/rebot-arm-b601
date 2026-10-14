"""Integration of existing IK/MuJoCo with upstream ROS Fake; never CAN."""
import json,math,time,urllib.request,urllib.error
from pathlib import Path
BASE='http://127.0.0.1:8765'
def get(path):return json.load(urllib.request.urlopen(BASE+path,timeout=10))
def post(path,body):
 return json.load(urllib.request.urlopen(urllib.request.Request(BASE+path,json.dumps(body).encode(),{'Content-Type':'application/json','Origin':BASE}),timeout=30))
report=dict(hardware=False,physical_acceptance=False)
s=get('/ros-control/state');assert s['backend']=='ros2-fake' and s['fresh']
post('/ros-control/action',{'action':'enable'})
sync=post('/ros-control/action',{'action':'sync'});assert sync['max_error_rad']<1e-9
plan=post('/command',{'action':'point_plan','target':'simulation','xyz_mm':[200,150,200],'orientation_mode':'position','speed_rad_s':.3})
post('/ros-control/action',{'action':'execute','plan_id':plan['plan_id']})
try:
 post('/command',{'action':'reset'});raise AssertionError('concurrent simulation reset accepted')
except urllib.error.HTTPError as e:
 assert 'ROS' in json.load(e)['error']
try:
 post('/ros-control/action',{'action':'execute','plan_id':plan['plan_id']});raise AssertionError('replayed plan accepted')
except urllib.error.HTTPError:pass
for _ in range(650):
 time.sleep(.1);s=get('/ros-control/state')
 if s['job']['status'] not in ('starting','running','cancelling'):break
assert s['job']['status']=='succeeded',s
sync=post('/ros-control/action',{'action':'sync'})
state=get('/studio-state');error=math.dist([v*1000 for v in state['xyz']],[200,150,200]);assert error<2,error
report.update(concurrent_reset_rejected=True,passed=True,xyz_mm=[200,150,200],position_error_mm=error,ros_result=s['job'],scheduling=s['scheduling'],sync=sync)
post('/ros-control/action',{'action':'disable'})
path=Path('outputs/mit-p5/ros-reuse/http.json');path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False))
