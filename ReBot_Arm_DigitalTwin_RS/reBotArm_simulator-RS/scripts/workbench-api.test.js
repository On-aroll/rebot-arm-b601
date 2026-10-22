'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const http=require('node:http'),fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {createWorkbench,contracts,validateProgram}=require('../workbench-api');
async function fixture(t) {
  const directory=fs.mkdtempSync(path.join(os.tmpdir(),'b601-workbench-')),received=[];
  let responder=(b,url)=>url==='/health'?{health:'ok'}:{schema_version:'1.0',request_id:b.request_id,stamp_ms:Date.now(),objects:[]};
  const upstream=http.createServer(async(req,res)=>{let data='';for await(const p of req)data+=p;const body=data?JSON.parse(data):{};received.push({body,url:req.url,authorization:req.headers.authorization});res.setHeader('Content-Type','application/json');res.end(JSON.stringify(responder(body,req.url)));});
  await new Promise(resolve=>upstream.listen(0,'127.0.0.1',resolve));
  const adapter=createWorkbench({directory}),server=http.createServer((req,res)=>void adapter.handle(req,res));
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const base=`http://127.0.0.1:${server.address().port}`,remote=`http://127.0.0.1:${upstream.address().port}`;
  t.after(async()=>{await Promise.all([new Promise(r=>server.close(r)),new Promise(r=>upstream.close(r))]);fs.rmSync(directory,{recursive:true,force:true});});
  async function api(route,body,origin=base){const res=await fetch(base+'/api/workbench/'+route,{method:body===undefined?'GET':'POST',headers:{Origin:origin,'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body)});return {status:res.status,...await res.json()};}
  async function save(kind,extras={}){const r=await api('connector',{name:kind,kind,protocol:'http-json',endpoint:remote+'/'+kind,health_url:remote+'/health',api_key:'secret-test',...extras});assert.equal(r.ok,true,r.error);return r.profiles.at(-1).id;}
  return {api,save,directory,remote,received,setResponder(fn){responder=fn;}};
}
const robot=()=>({actual:[0,0,0,0,0,0],pose:[.2,-.2,.3,0,0,0],calibration_id:'rs-identity-v1',stamp_ms:Date.now(),speed_max_rad_s:1,requested_speed_rad_s:.1});
const frame=()=>({mime:'image/jpeg',data_base64:'AA==',stamp_ms:Date.now(),frame_id:'camera_optical_frame'});
test('local API stores secret only server-side; key retained only at same endpoint',async t=>{
  const f=await fixture(t),id=await f.save('vision');
  const c=await f.api('config');assert.equal(c.profiles[0].has_key,true);assert.ok(!JSON.stringify(c).includes('secret-test'));
  assert.equal(fs.statSync(path.join(f.directory,'connectors.json')).mode&0o777,0o600);
  await f.api('test',{id});assert.equal(f.received.at(-1).authorization,'Bearer secret-test');
  await f.api('connector',{...c.profiles[0],api_key:''});await f.api('test',{id});assert.equal(f.received.at(-1).authorization,'Bearer secret-test');
  await f.api('connector',{...c.profiles[0],endpoint:f.remote+'/changed',api_key:''});
  assert.equal((await f.api('config')).profiles[0].has_key,false);
  assert.equal((await f.api('connector',{kind:'vision',protocol:'http-json',endpoint:'file:///etc/passwd'})).ok,false);
  assert.equal((await f.api('connector',{kind:'vision',protocol:'chat-completions',endpoint:f.remote})).ok,false);
});
test('mutations reject missing or foreign Origin; reads reject foreign Origin',async t=>{
  const f=await fixture(t);
  assert.equal((await f.api('programs',contracts.program,'https://foreign.example')).status,403);
  assert.equal((await f.api('config',undefined,'https://foreign.example')).status,403);
  assert.equal((await f.api('programs',contracts.program,'')).status,403);
});
test('program import is inert, validates actions/units and library round-trips',async t=>{
  const f=await fixture(t);const row=await f.api('programs',contracts.program);assert.equal(row.ok,true);
  assert.deepEqual((await f.api('programs')).programs[0].program,contracts.program);assert.equal(f.received.length,0);
  assert.equal((await f.api('programs',{...contracts.program,units:{position:'mm',angle:'rad',time:'s'}})).ok,false);
  assert.equal((await f.api('programs',{...contracts.program,steps:[{type:'shell',command:'no'}]})).ok,false);
  for(const s of [{type:'joint',joints:[0,0,0,0,0,0],speed:2},{type:'wait',seconds:-1},{type:'gripper',position:6},{type:'wait',seconds:1,enable:true}])assert.throws(()=>validateProgram({...contracts.program,steps:[s]}));
  const grasp={...contracts.program,steps:[{type:'grasp',speed:.1,profile:{empty_position:0,empty_tolerance:.02,
    min_gap:.05,contact_current_a:.5,abort_current_a:2,stall_velocity:.01,min_travel:.05,dwell:.3,hold_time:.3,timeout:30}}]};
  const saved=await f.api('programs',grasp);assert.equal(saved.ok,true,saved.error);
  assert.ok((await f.api('programs')).programs.some(p=>JSON.stringify(p.program)===JSON.stringify(grasp)));
  assert.equal(f.received.length,0);
});
test('vision request uses explicit consent, request identity, timestamp and calibrated 3D transform',async t=>{
  const f=await fixture(t),id=await f.save('vision');
  assert.equal((await f.api('vision',{id,frame:frame()})).ok,false);assert.equal(f.received.length,0);
  assert.equal((await f.api('calibration',{id:'camera-1',camera_to_base:[[1,0,0,.1],[0,1,0,0],[0,0,1,.2],[0,0,0,1]],intrinsics:[500,0,320,0,500,240,0,0,1]})).ok,true);
  f.setResponder(b=>({schema_version:'1.0',request_id:b.request_id,stamp_ms:Date.now(),objects:[{label:'box',confidence:.9,frame_id:'camera_optical_frame',position:[.1,.2,.3]}]}));
  const result=await f.api('vision',{id,allow_image:true,frame:frame()});assert.equal(result.ok,true,result.error);
  assert.deepEqual(result.observation.objects[0].position,[.2,.2,.5]);assert.equal(result.observation.objects[0].frame_id,'base_link');
  assert.equal(f.received.at(-1).body.frame.mime,'image/jpeg');
  f.setResponder(()=>({schema_version:'1.0',request_id:'wrong',stamp_ms:Date.now(),objects:[]}));
  assert.equal((await f.api('vision',{id,allow_image:true,frame:frame()})).ok,false);
  assert.equal((await f.api('vision',{id,allow_image:true,frame:{...frame(),stamp_ms:Date.now()-10000}})).ok,false);
  assert.equal((await f.api('calibration',{id:'bad',camera_to_base:[[2,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]})).ok,false);
});
test('planner returns a proposal only; stale context and unknown actions are rejected',async t=>{
  const f=await fixture(t),id=await f.save('planner');f.setResponder(b=>({request_id:b.request_id,program:contracts.program,explanation:'proposal only'}));
  const result=await f.api('plan',{id,task:'wait',robot:robot()});assert.equal(result.ok,true,result.error);assert.deepEqual(result.program,contracts.program);
  assert.equal(f.received.at(-1).body.robot.requested_speed_rad_s,.1);
  assert.equal((await f.api('plan',{id,task:'wait',robot:{...robot(),stamp_ms:1}})).ok,false);
  assert.equal((await f.api('plan',{id,task:'wait',robot:robot(),observation:{schema_version:'1.0',stamp_ms:1,objects:[]}})).ok,false);
  f.setResponder(b=>({request_id:b.request_id,program:{...contracts.program,steps:[{type:'enable'}]}}));
  assert.equal((await f.api('plan',{id,robot:robot()})).ok,false);
});
test('chat-completions adapter handles JSON proposals and explanations without execution',async t=>{
  const f=await fixture(t),id=await f.save('llm',{protocol:'chat-completions',endpoint:f.remote+'/chat/completions',model:'mock-model'});
  f.setResponder(()=>({choices:[{message:{content:JSON.stringify({explanation:'test proposal',program:contracts.program})}}]}));
  let result=await f.api('chat',{id,task:'wait one second',robot:robot()});assert.equal(result.ok,true,result.error);assert.deepEqual(result.program,contracts.program);
  const sent=f.received.at(-1).body;assert.equal(sent.model,'mock-model');assert.ok(sent.messages[0].content.includes('待检查提案'));assert.ok(!JSON.stringify(sent).includes('image_url'));
  f.setResponder(()=>({choices:[{message:{content:'Please supply object coordinates.'}}]}));
  result=await f.api('chat',{id,task:'pick it up',robot:robot()});assert.equal(result.program,null);assert.match(result.explanation,/coordinates/);
  assert.equal((await f.api('chat',{id,task:'image',robot:robot(),frame:frame()})).ok,false);
});
