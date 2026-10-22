// Execute the shipped scheduler with an inert transport; never open a gateway.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const source=fs.readFileSync(require.resolve('../public/js/motorbridge/rebot-motorbridge-ui.js'),'utf8');
const turn=()=>new Promise(resolve=>setImmediate(resolve));
function scheduler() {
  const calls=[],resolvers=[];
  const ctx={controlTransactionLocked:false,enabled:true,mode:true,
    MOTOR_IDS:[1,2,3,4,5,6,7],enabledMotors:new Set([3]),
    liveMoveLatest:null,liveMoveInFlight:false,liveMoveInFlightMotorId:null,
    liveMoveGeneration:0,liveMoveInterval:null,liveDragTarget:null,liveDragActive:false,
    liveDragReleaseTimer:null,commandQueueRunning:false,pendingMoveTargets:new Map(),liveMirrorMotorIds:new Set(),
    isLiveSingleAxisEnabled:()=>ctx.enabled,isMotorBridgeControlMode:()=>ctx.mode,
    clampMotorTarget:(id,q)=>Math.max(-1,Math.min(1,q)),
    getVlim:()=>.1,getLocKp:()=>30,ensureLiveFeedbackStream:async()=>{},
    updateMotorPositionControls(){},restoreMotorPositionControls(){},cancelAllTargetVerifications(){},
    writeLog(){},setSyncStatus(){},setMessage(){},scheduleLiveFeedbackStop(){},flushMotorMoves(){},
    client:{connected:true,movePosVel(...args){calls.push(args);return new Promise(resolve=>resolvers.push(resolve));}},
    window:{setInterval(fn,ms){assert.equal(ms,80);return 1;},clearInterval(){},clearTimeout(){}}};
  vm.createContext(ctx);
  const cancelStart=source.indexOf('  function cancelLiveMotorMoves()');
  const cancelEnd=source.indexOf('  function hasLiveMovePending()',cancelStart);
  const start=source.indexOf('  function scheduleLiveMotorMove(');
  const end=source.indexOf('  function queueMotorMove(',start);
  assert.ok(cancelStart>=0 && cancelEnd>cancelStart && start>=0 && end>start);
  vm.runInContext(source.slice(cancelStart,cancelEnd)+source.slice(start,end),ctx);
  return {ctx,calls,resolvers};
}
test('direct axis sends immediately, permits one request and coalesces to latest target',async()=>{
  const {ctx,calls,resolvers}=scheduler();
  ctx.scheduleLiveMotorMove(3,.1);await turn();
  assert.deepEqual(calls,[[3,.1,.1,30]]);
  ctx.scheduleLiveMotorMove(3,.2);ctx.scheduleLiveMotorMove(3,.3);
  await ctx.flushLiveMotorMove();assert.equal(calls.length,1);
  resolvers.shift()();await turn();
  const next=ctx.flushLiveMotorMove();await turn();
  assert.deepEqual(calls,[[3,.1,.1,30],[3,.3,.1,30]]);
  resolvers.shift()();await next;
  await ctx.flushLiveMotorMove();assert.equal(calls.length,2);
});
test('cancellation during feedback preparation cannot later send the old target',async()=>{
  const {ctx,calls}=scheduler();
  let ready;
  ctx.ensureLiveFeedbackStream=()=>new Promise(resolve=>{ready=resolve;});
  ctx.scheduleLiveMotorMove(3,.1);
  ctx.scheduleLiveMotorMove(3,.2);
  ctx.cancelLiveMotorMoves();ready();await turn();
  await ctx.flushLiveMotorMove();
  assert.equal(calls.length,0);assert.equal(ctx.liveMoveLatest,null);
});
test('mode, connection and per-axis enable gates remain enforced',async()=>{
  const {ctx,calls}=scheduler();
  assert.equal(ctx.scheduleLiveMotorMove(2,.1),false);
  ctx.mode=false;assert.equal(ctx.scheduleLiveMotorMove(3,.1),false);
  ctx.mode=true;ctx.client.connected=false;
  assert.equal(ctx.scheduleLiveMotorMove(3,.1),false);
  assert.equal(calls.length,0);
});
