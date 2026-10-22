// Actual gesture coordinator and pointer-release code with inert control methods.
const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const source=fs.readFileSync(require.resolve('../public/js/twin-workbench.js'),'utf8');
function gesture(){
  const calls=[],listeners=new Map(),messages=[];
  const c={requestEpoch:0,drag:null,dragFlight:null,document:{hidden:false,body:{dataset:{hubView:'tcp'}}},
    $:()=>({checked:true}),status:(_id,message)=>messages.push(message),
    twin:{owned:()=>true,fresh:()=>true,state:()=>({pose:[.1,.2,.3,1,2,3]}),hold:()=>calls.push('hold'),
      async drag(p,options){options.check();calls.push({pose:[...p],orientation:options.orientation});return {plan:{duration:1}};}},
    window:{addEventListener:(name,fn)=>listeners.set(name,fn)}};
  vm.createContext(c);vm.runInContext(source.slice(source.indexOf('  async function finishTcpDrag'),source.indexOf('  function fillProfiles')),c);
  return {c,calls,messages,fire(phase,position={x:.1,y:.2,z:.3}){listeners.get('rebot-twin-drag')({detail:{phase,position}});}};
}
test('drag chooses the last absolute point, submits once on release, preserves start orientation',async()=>{
  const p=gesture();p.fire('start');p.fire('move',{x:.2,y:.25,z:.35});p.fire('move',{x:.3,y:.26,z:.4});
  assert.deepEqual(p.calls,['hold']);p.fire('end');await Promise.resolve();
  assert.deepEqual(p.calls[1],{pose:[.3,.26,.4,1,2,3],orientation:true});
  p.fire('end');assert.equal(p.calls.length,2);
});
test('cancelled gesture, simple click, lost ownership and changed page never submit a point',async()=>{
  for(const phase of ['cancel','click','unowned','page']){
    const p=gesture();p.fire('start');if(phase!=='click')p.fire('move',{x:.2,y:.2,z:.3});
    if(phase==='cancel')p.fire('cancel');
    if(phase==='unowned'){p.c.twin.owned=()=>false;p.c.twin.drag=async()=>{throw Error('已失去控制权');};}
    if(phase==='page')p.c.document.body.dataset.hubView='keyboard';
    p.fire('end');await Promise.resolve();assert.equal(p.calls.length,1,phase);
  }
});
const sim=fs.readFileSync(require.resolve('../public/js/rebot-sim.js'),'utf8');
const release=sim.slice(sim.indexOf('  function endTcpDrag('),sim.indexOf('    draggingTcp = false;',sim.indexOf('      if(!committed)setGhostDisplay')));
test('only matching pointerup commits; pointercancel, lost capture and mode changes cancel',()=>{
  for(const type of ['pointerup','pointercancel','lostpointercapture',null]){
    const events=[];const c={draggingTcp:true,twinDrag:true,dragPointerId:7,dragPlane:{},dragTarget:{x:1,y:2,z:3},els:{},
      threeToRos:p=>p,setGhostDisplay(){},CustomEvent:class{constructor(type,options){Object.assign(this,{type,...options});}},window:{dispatchEvent:e=>events.push(e)}};
    vm.createContext(c);vm.runInContext(release+'  }',c);c.endTcpDrag(type?{type,pointerId:7}:undefined);
    assert.equal(events[0].detail.phase,type==='pointerup'?'end':'cancel');assert.equal(c.draggingTcp,false);
    c.endTcpDrag({type:'pointerup',pointerId:7});assert.equal(events.length,1);
  }
});
