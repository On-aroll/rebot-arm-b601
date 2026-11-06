'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const {createStartupManager}=require('../startup-manager');
const tick=()=>new Promise(resolve=>setImmediate(resolve));
function fixture() {
  const calls=[],feedback={connected:false},proc={},ports=new Set();
  let can={ready:false,present:true,message:'DOWN'},failRos=false,failHardware=false,hasCamera=true,failCamera=false;
  const manager=createStartupManager({
    observer:{snapshot:()=>feedback,close(){}},processes:()=>proc,portOpen:async port=>ports.has(port),canStatus:async()=>can,
    cameraPresent:()=>hasCamera,
    hardware:{runningPid:()=>proc.hardware?42:null,connect(){calls.push('hardware');
      if(failHardware)return {pid:42,completion:Promise.reject(new Error('实机反馈服务已退出：0x7019 unavailable'))};
      proc.hardware=true;feedback.hardware={fresh:true,valid:true,phase:'READONLY'};return {pid:42};}},
    launch(key){calls.push(key);if(key==='ros')return {completion:failRos?Promise.reject(new Error('ROS missing')):Promise.resolve()};
      if(key==='can')can={ready:true,present:true};
      if(key==='bridge'){proc.bridge=true;ports.add(9090);feedback.connected=true;}
      if(key==='simulation'){proc.simulation=true;feedback.simulation={fresh:true,valid:true};feedback.mujoco={fresh:true};}
      if(key==='mcp'){proc.mcp=true;ports.add(8081);}
      if(key==='camera'){
        if(failCamera)return {completion:Promise.reject(new Error('相机无法打开'))};
        proc.camera=true;feedback.camera={fresh:true,valid:true};
      }
      return {completion:key==='text'?Promise.reject(new Error('bad model setup')):Promise.resolve()};}
  });
  return {manager,calls,proc,feedback,ports,setCan(v){can=v;},failRos(){failRos=true;},failHardware(){failHardware=true;},noCamera(){hasCamera=false;},failCamera(){failCamera=true;}};
}
async function run(f,key) { f.manager.start(key);for(let i=0;i<100;i++){await new Promise(resolve=>setTimeout(resolve,5));const job=(await f.manager.status()).services[key].job;if(job?.state!=='running')return job;}throw new Error('Job did not finish'); }
test('startup status and unknown actions never spawn or acquire hardware',async()=>{
  const f=fixture();await f.manager.status();assert.deepEqual(f.calls,[]);
  for(const key of ['stop','enable','reset','../../command'])assert.throws(()=>f.manager.start(key));
  assert.deepEqual(f.calls,[]);
});
async function runAll(f) {
  f.manager.start('all');
  for(let i=0;i<200;i++) {const result=await f.manager.status();if(result.bundle.state!=='running')return result.bundle;await new Promise(r=>setTimeout(r,5));}
  throw new Error('bundle did not finish');
}
test('one click starts readonly dependencies and camera serially; duplicate clicks reuse the bundle',async()=>{
  const f=fixture();const first=f.manager.start('all');assert.equal(f.manager.start('all').job,first.job);
  assert.throws(()=>f.manager.start('hardware'),/一键启动/);
  const result=await runAll(f);assert.equal(result.state,'done');
  assert.deepEqual(f.calls,['ros','bridge','can','hardware','camera']);
  await runAll(f);assert.deepEqual(f.calls,['ros','bridge','can','hardware','camera','ros']);
});
test('camera absence or failure is visible without denying healthy readonly hardware',async()=>{
  const absent=fixture();absent.noCamera();const missing=await runAll(absent);
  assert.equal(missing.state,'done');assert.match(missing.message,/未发现相机/);assert.ok(!absent.calls.includes('camera'));
  const failed=fixture();failed.failCamera();const result=await runAll(failed);
  assert.equal(result.state,'done');assert.match(result.message,/相机无法打开/);assert.equal((await failed.manager.status()).services.hardware.ready,true);
});
test('camera requires live frames and recovery flags are separate from feedback readiness',async()=>{
  const f=fixture();f.feedback.connected=true;f.proc.camera=true;
  assert.equal((await f.manager.status()).services.camera.ready,false);
  f.feedback.camera={fresh:true,valid:true};assert.equal((await f.manager.status()).services.camera.ready,true);
  f.proc.hardware=true;f.feedback.hardware={fresh:true,valid:true,phase:'RECOVERY',recoveryRequired:true};
  const s=(await f.manager.status()).services.hardware;
  assert.equal(s.ready,true);assert.equal(s.needsRecovery,true);assert.match(s.message,/需在重置与维护中恢复/);
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
test('cached frames from an exited hardware process do not count as a ready driver',async()=>{
  const f=fixture();f.feedback.connected=true;f.feedback.hardware={fresh:true,valid:true,phase:'READONLY'};
  assert.equal((await f.manager.status()).services.hardware.ready,false);
  f.proc.hardware=true;assert.equal((await f.manager.status()).services.hardware.ready,true);
});
test('driver early exit ends the startup job with its error instead of a 30 second timeout',async()=>{
  const f=fixture();await run(f,'ros');await run(f,'bridge');await run(f,'can');f.failHardware();
  const job=await run(f,'hardware');
  assert.equal(job.state,'error');assert.match(job.message,/0x7019 unavailable/);
  assert.deepEqual(f.calls,['ros','bridge','can','hardware']);
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
test('unavailable ROS websocket does not crash the startup observer process',async()=>{
  const {execFile}=require('node:child_process'),path=require('node:path');
  const source=`
    const net=require('node:net'),{createStartupManager}=require('./startup-manager');
    const reservation=net.createServer();
    reservation.listen(0,'127.0.0.1',()=>{
      const port=reservation.address().port;
      reservation.close(async()=>{
        const NativeWebSocket=global.WebSocket;
        global.WebSocket=class extends NativeWebSocket {constructor(){super('ws://127.0.0.1:'+port);}};
        const manager=createStartupManager({processes:()=>({}),canStatus:async()=>({ready:false}),
          portOpen:async()=>false,hardware:{runningPid:()=>null}});
        await manager.status();
        setTimeout(()=>{manager.close();console.log('observer survived disconnected bridge');},150);
      });
    });`;
  const stdout=await new Promise((resolve,reject)=>execFile(process.execPath,['-e',source],
    {cwd:path.resolve(__dirname,'..'),timeout:5000},(error,stdout,stderr)=>error?reject(new Error(stderr||error.message)):resolve(stdout)));
  assert.match(stdout,/observer survived disconnected bridge/);
});
test('observer retries an error even when the websocket emits no close',async()=>{
  const {execFile}=require('node:child_process'),path=require('node:path');
  const source=`
    const assert=require('node:assert/strict'),{createStartupManager}=require('./startup-manager');
    const sockets=[];
    global.WebSocket=class {static OPEN=1;readyState=0;constructor(){sockets.push(this);}close(){}};
    (async()=>{
      const manager=createStartupManager({processes:()=>({}),canStatus:async()=>({ready:false}),
        portOpen:async()=>false,hardware:{runningPid:()=>null}});
      await manager.status();
      sockets[0].onerror();
      const now=Date.now;Date.now=()=>now()+2100;
      await manager.status();Date.now=now;
      assert.equal(sockets.length,2);
      manager.close();console.log('observer retried error without close');
    })();`;
  const stdout=await new Promise((resolve,reject)=>execFile(process.execPath,['-e',source],
    {cwd:path.resolve(__dirname,'..'),timeout:5000},(error,stdout,stderr)=>error?reject(new Error(stderr||error.message)):resolve(stdout)));
  assert.match(stdout,/observer retried error without close/);
});
