"""Local-only HTTP software configuration/permission checks. No CAN/driver calls."""
import json
import sys
import urllib.request
import urllib.error
from urllib.parse import urlsplit
BASE=sys.argv[1] if len(sys.argv)>1 else 'http://127.0.0.1:8766'
assert urlsplit(BASE).hostname in ('127.0.0.1','localhost')

def post(path, body, headers=None):
    req=urllib.request.Request(BASE+path,json.dumps(body).encode(),{'Content-Type':'application/json', **(headers or {})})
    try:
        with urllib.request.urlopen(req,timeout=30) as response:return response.status,json.load(response)
    except urllib.error.HTTPError as error:return error.code,json.load(error)

assert post('/config/session',{})[0]==400
assert post('/config/session',{}, {'Origin':'https://external.lhr.life'})[0]==400
headers={'Origin':BASE}
code,result=post('/config/session',{},headers)
assert code==200 and result['hardware'] is False and result['armed'] is False and result['drill'] is True
headers['X-Config-Session']=result['token']; revision=result['state']['revision']
assert post('/config/action',{'action':'status','seq':1},headers)[0]==200
assert post('/config/action',{'action':'status','seq':1},headers)[0]==400
for seq,op in enumerate(('enable','disable','stop','ensure_mode','mit','pos_vel','vel','robstride_write_param','set_zero'),2):
    assert post('/config/action',{'action':op,'seq':seq,'revision':revision,'armed':True,'drill':False,'hardware':True},headers)[0]==400
    assert post('/command',{'action':op},headers)[0]==400
assert post('/command',{'action':'point_plan','target':'hardware','xyz_mm':[200,150,200]},headers)[0]==400
assert post('/command',{'action':'point_execute','target':'hardware','plan_id':'fake'},headers)[0]==400
print('PASS: loopback same-origin session, replay rejection, nine hardware ops and hardware point requests denied')
