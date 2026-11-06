'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const fs=require('node:fs'),os=require('node:os'),path=require('node:path');
const {createMaintenanceManager,resetHardware,recovered}=require('../maintenance-recovery');
const makeClient=require('../public/js/maintenance-recovery');
const tick=()=>new Promise(resolve=>setImmediate(resolve));
const safeState=()=>({feedback_valid:true,hardware_phase:'READONLY',mode:'IDLE',owned:false,control_prepared:false,
  goal:null,reference:null,recovery_required:null,cleanup_ok:true,drive_modes:{1:0,2:0,3:0,4:0,5:0,6:0},
  extensions:{gripper:{active:false,authorized:false,recovery_required:null}},
  reset_status:{id:'r1',status:'succeeded',message:'已恢复'}});
function fixture(t,{healthy=false,rows=[{kind:'hardware',pid:42,birth:'one'}],blocked=false}={}) {
  const root=fs.mkdtempSync(path.join(os.tmpdir(),'rebot-maintenance-'));t.after(()=>fs.rmSync(root,{recursive:true,force:true}));
  const calls=[];let failure='',stopFailure=false,changeDriver=false,busy=false,scans=0;
  const startup={busy:()=>busy,async status(){return {services:{bridge:{ready:true}},
    feedback:{hardware:{fresh:true,valid:healthy,phase:healthy?'RECOVERY':'FAULT'}}};},
    start(key){calls.push(key);if(key==='hardware')rows=[{kind:'hardware',pid:43,birth:'two'}];return {job:{state:'done'}};}};
  const manager=createMaintenanceManager({root,startup,sleep:tick,blocked:()=>blocked,inventory:()=>{
    if(changeDriver&&++scans>1)return [{kind:'hardware',pid:42,birth:'recycled'}];
    return rows;
  },stopProcesses:async targets=>{calls.push('stop');if(stopFailure)throw new Error('驱动未正常退出');rows=rows.filter(p=>!targets.includes(p));},
  resetHardware:async()=>{calls.push('reset');if(failure)throw new Error(failure);return {resetId:'r1',message:'已恢复'};}});
  const status=startup.status;startup.status=async()=>{calls.push('status');return status();};
  return {manager,calls,root,startup,setFailure(v){failure=v;},stopFailure(){stopFailure=true;},changeDriver(){changeDriver=true;},busy(){busy=true;}};
}
async function settled(manager) {
  for(let i=0;i<200;i++){const {job}=manager.status();if(job.state!=='running')return job;await tick();}
  throw new Error('maintenance did not settle');
}
test('status and unconfirmed calls never act; confirmation is persisted before the operation',async t=>{
  const f=fixture(t);assert.equal(f.manager.status().job.state,'idle');assert.deepEqual(f.calls,[]);
  for(const value of [false,undefined,'true',1])assert.throws(()=>f.manager.start(value),/明确确认/);
  assert.deepEqual(f.calls,[]);
  const first=f.manager.start(true).job;
  const saved=JSON.parse(fs.readFileSync(path.join(f.root,'outputs/maintenance',first.id+'.json')));
  assert.match(saved.confirmation,/用户点击恢复按钮/);
  assert.equal((await settled(f.manager)).state,'done');
});
test('healthy driver resets through the current owner without restarts or CAN configuration',async t=>{
  const f=fixture(t,{healthy:true});f.manager.start(true);const job=await settled(f.manager);
  assert.equal(job.state,'done');assert.equal(job.result.resetId,'r1');
  assert.deepEqual(f.calls,['status','reset','status']);assert.equal(job.driver.pid,42);
});
test('a live driver with retrying or stale feedback is retained',async t=>{
  for(const [phase,returns] of [['FEEDBACK_WAIT',true],['FEEDBACK_WAIT',false],['FAULT',false]]) {
    const f=fixture(t,{healthy:true}),read=f.startup.status;
    let samples=0;
    f.startup.status=async()=>{
      const result=await read();
      if(++samples===1 || !returns)result.feedback.hardware={fresh:phase==='FEEDBACK_WAIT',valid:false,phase};
      return result;
    };
    f.manager.start(true);
    const job=await settled(f.manager);
    assert.equal(job.state,returns?'done':'error');
    assert.equal(f.calls.includes('reset'),returns);
    assert.ok(f.calls.every(call=>['status','reset'].includes(call)));
    if(returns)assert.equal(job.driver.pid,42);
    else assert.match(job.message,/已保留 CAN 连接/);
  }
});
test('fault recovery stops the exact old driver before ROS/CAN and one readonly replacement',async t=>{
  const f=fixture(t);const first=f.manager.start(true);assert.equal(f.manager.start(true).job.id,first.job.id);
  assert.equal((await settled(f.manager)).state,'done');
  assert.deepEqual(f.calls,['status','stop','ros','status','can','hardware','reset','status']);
  assert.equal(f.manager.status().job.previousDrivers[0].birth,'one');
  assert.equal(f.manager.status().job.driver.birth,'two');
});
test('maintenance also restarts missing camera capture, without starting a second physical driver',async t=>{
  const f=fixture(t,{healthy:true}),read=f.startup.status;
  f.startup.status=async()=>{const result=await read();return {...result,services:{...result.services,ros:{ready:true},camera:{present:true,ready:false,running:false}}};};
  f.manager.start(true);assert.equal((await settled(f.manager)).state,'done');
  assert.deepEqual(f.calls,['status','reset','status','camera']);
});
test('a stuck driver, duplicate owner or changed birth cannot reach CAN/recovery',async t=>{
  const stuck=fixture(t);stuck.stopFailure();stuck.manager.start(true);
  assert.equal((await settled(stuck.manager)).state,'error');assert.deepEqual(stuck.calls,['status','stop']);
  const duplicate=fixture(t,{rows:[{kind:'hardware',pid:42},{kind:'hardware',pid:43}]});duplicate.manager.start(true);
  assert.match((await settled(duplicate.manager)).message,/多份/);assert.deepEqual(duplicate.calls,['status']);
  const changed=fixture(t,{healthy:true});changed.changeDriver();changed.manager.start(true);
  assert.match((await settled(changed.manager)).message,/身份发生变化/);assert.deepEqual(changed.calls,['status']);
});
test('missing snapshots and unknown parameters remain errors, never retries or lock-file edits',async t=>{
  const f=fixture(t,{healthy:true});f.setFailure('缺少事故前完整参数快照；保持锁定');f.manager.start(true);
  const job=await settled(f.manager);assert.equal(job.state,'error');assert.match(job.message,/保持锁定/);
  assert.deepEqual(f.calls,['status','reset']);assert.equal(fs.existsSync(path.join(f.root,'outputs/twin-console/hardware')),false);
});
test('startup/shutdown conflicts and an unwritable confirmation record cannot operate',t=>{
  const f=fixture(t,{blocked:true});assert.throws(()=>f.manager.start(true),/正在进行/);assert.deepEqual(f.calls,[]);
  const starting=fixture(t);starting.busy();assert.throws(()=>starting.manager.start(true),/正在进行/);
  const disk=fixture(t);fs.writeFileSync(path.join(disk.root,'outputs'),'not a directory');
  assert.throws(()=>disk.manager.start(true),/记录保存失败/);assert.deepEqual(disk.calls,[]);
});
function socketFixture() {
  let socket;
  class Socket {
    constructor(){socket=this;this.sent=[];queueMicrotask(()=>this.onopen());}
    send(data){this.sent.push(JSON.parse(data));}
    close(){this.onclose?.();}
    state(state){this.onmessage({data:JSON.stringify({op:'publish',topic:'/rebotarm/twin/state',msg:{data:JSON.stringify(state)}})});}
    ack(id='r1'){this.onmessage({data:JSON.stringify({op:'service_response',id:this.sent.find(m=>m.op==='call_service').id,result:true,
      values:{success:true,message:JSON.stringify({id,status:'queued'})}})});}
  }
  return {Socket,get socket(){return socket;}};
}
test('actual ROS transport only subscribes and calls reset once, matching the result id even if frames arrive first',async()=>{
  const f=socketFixture(),p=resetHardware({Socket:f.Socket,timeout:1000});await tick();
  f.socket.state({...safeState(),reset_status:null});f.socket.state(safeState());f.socket.ack();
  assert.equal((await p).resetId,'r1');
  assert.deepEqual(f.socket.sent.map(m=>[m.op,m.service||m.topic]),[['subscribe','/rebotarm/twin/state'],['call_service','/rebotarm/twin/reset']]);
});
test('terminal reset failure, stale feedback, mismatched result and connection loss never claim success',async()=>{
  const failed=socketFixture(),p=resetHardware({Socket:failed.Socket,timeout:1000});
  const rejection=assert.rejects(p,/未知参数/);await tick();failed.socket.state({...safeState(),reset_status:null});failed.socket.ack();
  failed.socket.state({...safeState(),reset_status:{id:'r1',status:'failed',message:'未知参数，保持锁定'}});await rejection;
  const stale=socketFixture(),q=resetHardware({Socket:stale.Socket,timeout:1000}),invalid=assert.rejects(q,/尚无有效反馈/);
  await tick();stale.socket.state({...safeState(),feedback_valid:false});await invalid;
  assert.equal(stale.socket.sent.filter(m=>m.op==='call_service').length,0);
  const other=socketFixture(),r=resetHardware({Socket:other.Socket,timeout:25}),timeout=assert.rejects(r,/超时/);
  await tick();other.socket.state({...safeState(),reset_status:null});other.socket.ack('new-job');other.socket.state(safeState());await timeout;
  const lost=socketFixture(),s=resetHardware({Socket:lost.Socket,timeout:1000}),closed=assert.rejects(s,/中断/);
  await tick();lost.socket.close();await closed;
});
test('a successful reset with a lingering owner or active mode fails final verification',async()=>{
  const f=socketFixture(),p=resetHardware({Socket:f.Socket,timeout:1000}),rejection=assert.rejects(p,/读回尚未通过/);
  await tick();f.socket.state({...safeState(),reset_status:null});f.socket.ack();f.socket.state({...safeState(),owned:true});await rejection;
});
test('success requires six disabled modes, no owner, old goals, gripper activity or recovery lock',()=>{
  assert.equal(recovered(safeState()),true);
  for(const extra of [{feedback_valid:false},{owned:true},{control_prepared:true},{goal:[0]},{reference:[0]},
    {hardware_phase:'RECOVERY'},{cleanup_ok:false},{drive_modes:{1:0}},{recovery_required:{report:'fault.json'}},
    {extensions:{gripper:{active:true}}},{extensions:{gripper:{authorized:true}}}])assert.equal(recovered({...safeState(),...extra}),false);
});
test('browser click supplies the confirmation once; polling and reloads cannot initiate recovery',async()=>{
  const calls=[],states=[{id:'a',state:'running'},{id:'a',state:'done',message:'已恢复'}];
  const client=makeClient(async(url,options)=>{calls.push({url,...options});return {ok:true,json:async()=>({ok:true,job:states.shift()})};},async()=>{});
  assert.equal((await client.restore(()=>{})).state,'done');
  assert.deepEqual(calls.map(c=>c.method),['POST','GET']);assert.deepEqual(JSON.parse(calls[0].body),{confirmed:true});
  calls.length=0;states.push({state:'idle'});await client.resume(()=>{});assert.deepEqual(calls.map(c=>c.method),['GET']);
});
