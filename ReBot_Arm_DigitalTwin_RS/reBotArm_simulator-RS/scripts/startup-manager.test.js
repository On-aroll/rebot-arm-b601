'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const {createStartupManager}=require('../startup-manager');
const tick=()=>new Promise(resolve=>setImmediate(resolve));
function fixture() {
  const calls=[],feedback={connected:false},proc={},ports=new Set();
  let can={ready:false,present:true,message:'DOWN'},failRos=false;
  const manager=createStartupManager({
    observer:{snapshot:()=>feedback,close(){}},processes:()=>proc,portOpen:async port=>ports.has(port),canStatus:async()=>can,
    hardware:{runningPid:()=>proc.hardware?42:null,connect(){calls.push('hardware');proc.hardware=true;feedback.hardware={fresh:true,valid:true,phase:'READONLY'};return {pid:42};}},
    launch(key){calls.push(key);if(key==='ros')return {completion:failRos?Promise.reject(new Error('ROS missing')):Promise.resolve()};
      if(key==='can')can={ready:true,present:true};
      if(key==='bridge'){proc.bridge=true;ports.add(9090);feedback.connected=true;}
      if(key==='simulation'){proc.simulation=true;feedback.simulation={fresh:true,valid:true};feedback.mujoco={fresh:true};}
      if(key==='mcp'){proc.mcp=true;ports.add(8081);}
      return {completion:key==='text'?Promise.reject(new Error('bad model setup')):Promise.resolve()};}
  });
  return {manager,calls,proc,feedback,ports,setCan(v){can=v;},failRos(){failRos=true;}};
}
async function run(f,key) { f.manager.start(key);for(let i=0;i<100;i++){await new Promise(resolve=>setTimeout(resolve,5));const job=(await f.manager.status()).services[key].job;if(job?.state!=='running')return job;}throw new Error('Job did not finish'); }
test('startup status and unknown actions never spawn or acquire hardware',async()=>{
  const f=fixture();await f.manager.status();assert.deepEqual(f.calls,[]);
  for(const key of ['stop','enable','reset','../../command'])assert.throws(()=>f.manager.start(key));
  assert.deepEqual(f.calls,[]);
});
test('startup enforces ROS, bridge and CAN dependencies before physical startup',async()=>{
  const f=fixture();assert.match((await run(f,'hardware')).message,/第 1 步/);assert.deepEqual(f.calls,[]);
  await run(f,'ros');assert.match((await run(f,'hardware')).message,/联动接口/);
  await run(f,'bridge');assert.match((await run(f,'hardware')).message,/CAN/);
  await run(f,'can');assert.equal((await run(f,'hardware')).state,'done');assert.deepEqual(f.calls,['ros','bridge','can','hardware']);
});
test('duplicate clicks share one job, ready services reuse without a second process',async()=>{
  const f=fixture();f.manager.start('ros');assert.equal(f.manager.start('ros').reused,true);await tick();
  await run(f,'bridge');await run(f,'bridge');await run(f,'can');await run(f,'hardware');await run(f,'hardware');
  assert.deepEqual(f.calls,['ros','bridge','can','hardware']);
});
test('stale existing driver and occupied port do not cause automatic restart',async()=>{
  const f=fixture();await run(f,'ros');f.ports.add(9090);
  assert.match((await run(f,'bridge')).message,/占用/);assert.deepEqual(f.calls,['ros']);
  f.proc.bridge=true;f.feedback.connected=true;f.proc.hardware=true;f.setCan({ready:true});
  assert.match((await run(f,'hardware')).message,/进程已存在/);assert.deepEqual(f.calls,['ros']);
});
test('CAN cannot be reconfigured while a real driver exists',async()=>{
  const f=fixture();await run(f,'ros');f.proc.hardware=true;
  assert.match((await run(f,'can')).message,/禁止重新配置 CAN/);assert.deepEqual(f.calls,['ros']);
});
test('failed ROS recheck revokes ready state; simulation needs both fresh streams',async()=>{
  const f=fixture();await run(f,'ros');f.failRos();assert.equal((await run(f,'ros')).state,'error');
  assert.equal((await f.manager.status()).services.ros.ready,false);
  f.feedback.connected=true;f.feedback.simulation={fresh:true,valid:true};f.proc.simulation=true;
  assert.equal((await f.manager.status()).services.simulation.ready,false);
  f.feedback.mujoco={fresh:true};assert.equal((await f.manager.status()).services.simulation.ready,true);
  f.feedback.simulation.fresh=false;assert.equal((await f.manager.status()).services.simulation.ready,false);
});
test('optional agents require simulation and failures remain visible',async()=>{
  const f=fixture();await run(f,'ros');await run(f,'bridge');
  assert.match((await run(f,'mcp')).message,/先启动仿真/);await run(f,'simulation');await run(f,'mcp');
  assert.equal((await run(f,'text')).state,'error');assert.equal((await f.manager.status()).services.text.ready,false);
});
