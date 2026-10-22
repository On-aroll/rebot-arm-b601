const {test}=require('node:test');
const assert=require('node:assert/strict');
const {ManualRecovery,gamepad}=require('../public/js/twin-input.js');
const hold=()=>({hardware_phase:'HOLDING',owned:false,recovery_required:false,communication_hold:{since:42,feedback_confirmed:true,automatic_manual_recovery:true}});
const c={fresh:true,neutral:true,allowed:true};
test('only the prior manual owner can recover; no program or manual pause recovery',()=>{
 const r=new ManualRecovery(),s=hold();r.lost(s);assert.equal(r.ready(s,1000,c),false);
 r.arm();r.lost({...s,communication_hold:{...s.communication_hold,automatic_manual_recovery:false}});assert.equal(r.pending,false);
 r.arm();r.lost(s);assert.equal(r.ready(s,0,c),false);assert.equal(r.ready(s,500,c),true);
});
test('lease-revoked-before-HOLDING race still waits for typed feedback recovery',()=>{
 const r=new ManualRecovery();r.arm();r.lost({hardware_phase:'ACTIVE'});assert.ok(r.awaitingHold);assert.equal(r.pending,false);
 const s=hold();r.lost(s);assert.ok(r.pending);assert.equal(r.awaitingHold,false);
 r.ready(s,0,c);assert.equal(r.ready(s,500,{...c,neutral:false}),false);assert.equal(r.ready(s,520,c),true);
});
test('requires continuous fresh feedback, neutral input, original hold and no competing owner',()=>{
 const r=new ManualRecovery(),s=hold();r.arm();r.lost(s);r.ready(s,0,c);
 assert.equal(r.ready(s,400,{...c,fresh:false}),false);assert.equal(r.ready(s,500,c),false);assert.equal(r.ready(s,1000,c),true);
 assert.equal(r.ready({...s,owned:true},1010,c),false);assert.equal(r.ready(s,1100,c),false);
 assert.equal(r.ready({...s,communication_hold:{...s.communication_hold,since:43}},1600,c),false);assert.equal(r.pending,false);
});
test('never recovers after stop, permission withdrawal or fault and bounds retries',()=>{
 const r=new ManualRecovery(),s=hold();r.arm();r.lost(s);r.ready(s,0,c);r.begin(500);
 assert.equal(r.ready(s,2000,c),false);r.finish(false);r.ready(s,2100,c);assert.equal(r.ready(s,2600,c),true);
 r.begin(2600);r.finish(false);r.begin(4200);r.finish(false);r.ready(s,6000,c);assert.equal(r.ready(s,7000,c),false);
 r.cancel();assert.equal(r.ready(s,8000,c),false);
 for(const state of [{...s,hardware_phase:'READONLY'},{...s,hardware_phase:'FAULT'},{...s,recovery_required:true}]){r.arm();r.lost(s);assert.equal(r.ready(state,9000,c),false);assert.equal(r.pending,false);}
 r.arm();r.lost(s);r.ready(s,0,{...c,allowed:false});assert.equal(r.armed,false);
});
test('success only restores readiness and requires an ACTIVE rearm; LT/RT and A/B still require RB',()=>{
 const r=new ManualRecovery();r.arm();r.lost(hold());r.begin(500);r.finish(true);assert.equal(r.pending,false);assert.ok(r.waitingActive);
 const p={mapping:'standard',axes:[0,0,0,0],buttons:Array.from({length:17},()=>({value:0}))};
 p.buttons[0].value=1;p.buttons[6].value=1;let s=gamepad(p);assert.equal(s.gripper,0);assert.equal(s.values[3],0);
 p.buttons[5].value=1;s=gamepad(p);assert.equal(s.gripper,1);assert.equal(s.values[3],-1);
 p.buttons[4].value=1;s=gamepad(p);assert.equal(s.gripper,.25);assert.equal(s.values[3],-.25);
 p.buttons[6].value=0;p.buttons[7].value=1;p.buttons[0].value=0;p.buttons[1].value=1;s=gamepad(p);assert.equal(s.gripper,-.25);assert.equal(s.values[3],.25);
});
