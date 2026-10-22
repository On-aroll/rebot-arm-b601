const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {spawn}=require('node:child_process');
const {createShutdownManager,classify,stopProcesses,readProcess,sameProcess}=require('../project-shutdown');
const tick=()=>new Promise(resolve=>setImmediate(resolve));
const temp=()=>fs.mkdtempSync(path.join(os.tmpdir(),'rebot-exit-test-'));
async function settled(manager){for(let i=0;i<100;i++){const v=manager.status();if(v.job.state!=='running')return v.job;await tick();}throw new Error('not settled');}
test('only exact project entry points are selected, unrelated processes are excluded',()=>{
  const root='/tmp/project';
  assert.equal(classify(['python3','-m','rebotarmcontroller.csp_twin_driver'],root,root),'hardware');
  assert.equal(classify(['node','/tmp/project/FakeRsDriver'],root,root),'simulation');
  assert.equal(classify(['python3','rosbridge_websocket'],'/tmp/other',root),null);
  assert.equal(classify(['node','-e','console.log("FakeRsDriver")'],root,root),null);
  assert.equal(classify(['python3','unrelated.py'],root,root),null);
});
test('confirmation and startup-in-progress gates cannot send any signal',()=>{
  let calls=0;const m=createShutdownManager({root:temp(),inventory:()=>[],stopProcesses:async()=>calls++,starting:()=>true});
  assert.throws(()=>m.start(false),/确认/);assert.throws(()=>m.start(true),/启动步骤/);assert.equal(calls,0);
});
test('unconfirmed or active real hardware blocks the entire shutdown before any signal',async()=>{
  const calls=[];const m=createShutdownManager({root:temp(),inventory:()=>[{kind:'hardware',pid:1}],
    sealHardware:async()=>{calls.push('seal');throw new Error('HOLDING：请先支撑并失能');},stopProcesses:async()=>calls.push('stop')});
  m.start(true);assert.equal((await settled(m)).state,'error');assert.deepEqual(calls,['seal']);
  assert.equal(m.blocked(),false);assert.throws(()=>m.finish('wrong'));
});
test('driver acknowledgement precedes termination; finish requires all components gone',async()=>{
  let rows=['hardware','text','mcp','simulation','bridge'].map((kind,i)=>({kind,pid:i+10}));const order=[];
  const m=createShutdownManager({root:temp(),inventory:()=>rows,sealHardware:async()=>order.push('seal'),
    stopProcesses:async targets=>{if(targets.length)order.push(targets[0].kind);rows=rows.filter(p=>!targets.includes(p));}});
  const first=m.start(true);assert.equal(m.start(true).job.id,first.job.id);
  assert.equal(m.blocked(),true);assert.throws(()=>m.finish(first.job.id),/尚未/);
  const done=await settled(m);assert.equal(done.state,'ready');
  assert.deepEqual(order,['seal','hardware','text','mcp','simulation','bridge']);
  assert.throws(()=>m.finish('wrong'),/尚未/);assert.equal(m.finish(done.id).job.state,'exiting');
});
test('failure keeps later services and HTTP alive; unexpected new processes block final exit',async()=>{
  let rows=[{kind:'simulation',pid:1},{kind:'bridge',pid:2}];const seen=[];
  const m=createShutdownManager({root:temp(),inventory:()=>rows,stopProcesses:async targets=>{
    if(targets.length){seen.push(targets[0].kind);throw new Error('未退出');}}});
  m.start(true);assert.equal((await settled(m)).state,'error');assert.deepEqual(seen,['simulation']);assert.equal(rows.length,2);
  const clean=createShutdownManager({root:temp(),inventory:()=>rows,stopProcesses:async()=>{rows=[];}});
  clean.start(true);const ready=await settled(clean);rows=[{kind:'simulation',pid:99}];
  assert.throws(()=>clean.finish(ready.id),/重新出现/);
});
test('physical driver is never SIGKILLed; recycled pids are not signalled',async()=>{
  const calls=[];const real={kind:'hardware',pid:42},recycled={kind:'simulation',pid:43};
  await assert.rejects(stopProcesses([real,recycled],{timeout:100,wait:async()=>{},alive:p=>p===real,
    signal:(pid,s)=>calls.push([pid,s])}),/不强制/);
  assert.deepEqual(calls,[[42,'SIGTERM']]);
});
test('actual HTTP server exits after receipt; sandbox services and descendants stop, outsiders survive',async(t)=>{
  const root=temp(),web=path.join(root,'reBotArm_simulator-RS');fs.mkdirSync(web);
  for(const name of ['server.js','local-api.js','project-shutdown.js','startup-manager.js','hardware-launcher.js','workbench-api.js'])
    fs.copyFileSync(path.join(__dirname,'..',name),path.join(web,name));
  const workers=[];
  function worker(name,cwd){const file=path.join(cwd,name);fs.writeFileSync(file,'setInterval(()=>{},1000)');const p=spawn(process.execPath,[file],{cwd,stdio:'ignore'});workers.push(p);return p;}
  const fake=worker('FakeRsDriver',root),bridge=worker('rosbridge_websocket',root),outside=worker('FakeRsDriver',temp());
  const childfile=path.join(root,'child.pid'),helper=path.join(root,'child-helper.js');
  fs.writeFileSync(helper,"const p=require('node:child_process').spawn(process.execPath,['-e','setInterval(()=>{},1000)'],{cwd:'/tmp',stdio:'ignore'});require('node:fs').writeFileSync("+JSON.stringify(childfile)+",String(p.pid));setInterval(()=>{},1000)");
  const parentfile=path.join(root,'mujoco_rs_sync');fs.copyFileSync(helper,parentfile);
  const parent=spawn(process.execPath,[parentfile],{cwd:root,stdio:'ignore'});workers.push(parent);
  for(let i=0;i<100&&!fs.existsSync(childfile);i++)await new Promise(r=>setTimeout(r,10));
  const child=readProcess(Number(fs.readFileSync(childfile,'utf8')));
  t.after(()=>{if(child&&sameProcess(child))process.kill(child.pid,'SIGKILL');});
  const port=await new Promise(resolve=>{const s=require('node:net').createServer();s.listen(0,'127.0.0.1',()=>{const p=s.address().port;s.close(()=>resolve(p));});});
  const server=spawn(process.execPath,['server.js'],{cwd:web,env:{...process.env,PORT:String(port)},stdio:'ignore'});
  t.after(()=>{for(const p of [...workers,server])if(p.exitCode===null)p.kill('SIGKILL');});
  const base='http://127.0.0.1:'+port;
  for(let i=0;i<100;i++){try{if((await fetch(base+'/api/startup/shutdown/status')).ok)break;}catch(_){}await new Promise(r=>setTimeout(r,20));}
  let response=await fetch(base+'/api/startup/shutdown/start',{method:'POST',headers:{Origin:'https://other.example','Content-Type':'application/json'},body:'{"confirmed":true}'});
  assert.equal(response.status,403);assert.equal(fake.exitCode,null);
  response=await fetch(base+'/api/startup/shutdown/start',{method:'POST',headers:{Origin:base,'Content-Type':'application/json'},body:'{"confirmed":false}'});
  assert.equal(response.status,400);
  const exited=new Promise(resolve=>server.once('exit',code=>resolve(code)));
  response=await fetch(base+'/api/startup/shutdown/start',{method:'POST',headers:{Origin:base,'Content-Type':'application/json'},body:'{"confirmed":true}'});
  assert.equal(response.status,202);let job;
  for(let i=0;i<100;i++){job=(await(await fetch(base+'/api/startup/shutdown/status')).json()).job;if(job.state!=='running')break;await new Promise(r=>setTimeout(r,30));}
  assert.equal(job.state,'ready',job.message);
  assert.equal((await fetch(base+'/api/hardware/connect',{method:'POST',headers:{Origin:base}})).status,409);
  assert.ok(fake.exitCode!==null||fake.signalCode);assert.ok(bridge.exitCode!==null||bridge.signalCode);
  assert.equal(sameProcess(child),false);assert.equal(outside.exitCode,null);assert.equal(outside.signalCode,null);
  response=await fetch(base+'/api/startup/shutdown/finish',{method:'POST',headers:{Origin:base,'Content-Type':'application/json'},body:JSON.stringify({id:job.id})});
  assert.equal(response.status,200);assert.equal((await response.json()).job.state,'exiting');
  assert.equal(await Promise.race([exited,new Promise((_,reject)=>setTimeout(()=>reject(new Error('HTTP did not exit')),3000))]),0);
  await assert.rejects(fetch(base+'/api/startup/shutdown/status'));
});
