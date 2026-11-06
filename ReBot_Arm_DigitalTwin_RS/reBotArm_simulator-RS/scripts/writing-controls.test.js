'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const geometry=require('../public/js/writing-plane');
const templates=require('../public/programs/writing-strokes.json');
const tick=()=>new Promise(resolve=>setImmediate(resolve));
function page(storage=new Map(),badFrame=false,hardware=false){
  const elements=new Map(),listeners=new Map(),loaded=[],commands=[],requests=[],actions=[];
  const state={feedback_valid:true,pose:[.3,.1,.2,0,0,0],extensions:{program:{}}};
  function el(id){if(!elements.has(id))elements.set(id,{value:'',textContent:'',dataset:{},disabled:false,children:[],handlers:{},
    addEventListener(type,fn){this.handlers[type]=fn;},async emit(type){await this.handlers[type]?.({});},
    async click(){await this.emit('click');},scrollIntoView(){},setAttribute(k,v){this[k]=v;},
    replaceChildren(...nodes){this.children=nodes;},append(...nodes){this.children.push(...nodes);}});return elements.get(id);}
  el('writing-text').value='BIT103';
  const window={ReBotWritingPlane:geometry,ReBotTwin:{hardware,state:()=>state,fresh:()=>true,jointSpeed:()=>.05,request(...args){commands.push(args);}},
    addEventListener(type,fn){listeners.set(type,fn);},ReBotWritingEditor:{invalidate(){},load(program){loaded.push(program);listeners.get('rebot-program-loaded')?.({detail:{program}});},
      async preview(){actions.push('preview');listeners.get('rebot-program-checked')?.({detail:{program:loaded.at(-1)}});},visual(){actions.push('visual');},async execute(){actions.push('execute');}}};
  class DOMParser{parseFromString(){return {querySelector:()=>({getAttribute:()=> 'fixed',querySelector(tag){
    const attrs=tag==='origin'?{xyz:'0 0 0',rpy:'0 0 0'}:tag==='parent'?{link:badFrame?'wrong_link':'link6'}:{link:'gripper_end'};
    return {getAttribute:key=>attrs[key]};}})};}}
  const context={window,document:{getElementById:el,createElementNS:(_,tag)=>{const e={tag,attrs:{},textContent:'',setAttribute(k,v){this.attrs[k]=v;}};return e;}},DOMParser,
    localStorage:{getItem:key=>storage.get(key),setItem:(key,v)=>storage.set(key,v)},
    fetch:async url=>{requests.push(url);return {ok:true,text:async()=>'<robot/>',json:async()=>templates};}};
  vm.runInNewContext(fs.readFileSync(require.resolve('../public/js/writing-controls.js'),'utf8'),context);
  return {el,state,loaded,commands,requests,storage,actions,workflow:window.ReBotWritingWorkflow,
    emit:(type,detail)=>listeners.get(type)?.({detail}),select:async text=>{el('writing-text').value=text;await el('writing-text').emit('change');}};
}
test('capture, angles and generation remain local and use a frozen pose',async()=>{
  const p=page();await tick();p.el('writing-axis').value='+X';await p.el('writing-axis').emit('input');
  assert.equal(p.el('writing-generate').disabled,true);
  await p.el('writing-capture').click();assert.equal(p.el('writing-generate').disabled,false);
  p.state.pose=[.8,.9,1,0,0,0];p.el('writing-tilt-y').value='25';await p.el('writing-tilt-y').emit('input');
  await p.el('writing-generate').click();const program=p.loaded[0];
  assert.deepEqual(Array.from(program.writing_plane.captured_pose.slice(0,3)),[.3,.1,.2]);
  assert.equal(program.writing_plane.preview_only,true);assert.equal(program.writing_plane.configuration.tilt_deg[1],25);
  assert.deepEqual(p.commands,[]);assert.deepEqual(p.requests,['/api/urdf','/programs/writing-strokes.json']);
});
test('the two texts retain independent settings across reload, while captured pose is never silently reused',async()=>{
  const storage=new Map(),p=page(storage);await tick();
  p.el('writing-axis').value='-Y';p.el('writing-spin').value='30';await p.el('writing-spin').emit('input');
  await p.select('国庆快乐');p.el('writing-axis').value='+Z';p.el('writing-spin').value='-60';await p.el('writing-spin').emit('input');
  await p.select('BIT103');assert.equal(p.el('writing-axis').value,'-Y');assert.equal(p.el('writing-spin').value,'30');
  const reloaded=page(storage);await tick();assert.equal(reloaded.el('writing-axis').value,'-Y');
  assert.equal(reloaded.el('writing-generate').disabled,true);
  await reloaded.select('国庆快乐');assert.equal(reloaded.el('writing-spin').value,'-60');
  assert.ok(!storage.get('rebot-writing-settings-v1').includes('captured_pose'));
});
test('an unknown wrist transform disables generation rather than guessing a direction',async()=>{
  const p=page(new Map(),true);await tick();assert.match(p.el('writing-status').textContent,/不匹配/);
  p.el('writing-axis').value='+X';await p.el('writing-capture').click();assert.equal(p.el('writing-generate').disabled,true);
  assert.deepEqual(p.commands,[]);assert.equal(p.loaded.length,0);
});
test('a writing job advances only through current capture, explicit confirmation and a checked path',async()=>{
  const p=page();await tick();
  p.el('writing-axis').value='+Z';await p.el('writing-axis').emit('input');await p.el('writing-capture').click();
  assert.equal(p.el('writing-step-3').dataset.state,'current');assert.equal(p.el('writing-check').disabled,true);
  await p.el('writing-generate').click();const program=p.loaded.at(-1);
  assert.doesNotThrow(()=>p.workflow.assertCurrent(program));assert.throws(()=>p.workflow.assertCurrent(program,true),/先检查/);
  assert.equal(p.el('writing-step-4').dataset.state,'current');assert.deepEqual(p.actions,[]);assert.deepEqual(p.commands,[]);
  await p.el('writing-check').click();assert.doesNotThrow(()=>p.workflow.assertCurrent(program,true));
  assert.equal(p.el('writing-step-5').dataset.state,'current');assert.equal(p.el('writing-execute').disabled,false);
  await p.el('writing-visual').click();await p.el('writing-execute').click();assert.deepEqual(p.actions,['preview','visual','execute']);
});
test('changing geometry or the program revokes confirmation and rejects delayed old check results',async()=>{
  const p=page();await tick();p.el('writing-axis').value='+X';await p.el('writing-capture').click();await p.el('writing-generate').click();
  const old=p.loaded.at(-1);await p.el('writing-check').click();
  p.el('writing-tilt-y').value='15';await p.el('writing-tilt-y').emit('input');
  assert.throws(()=>p.workflow.assertCurrent(old,true),/重新确认/);p.emit('rebot-program-checked',{program:old});
  assert.equal(p.el('writing-execute').disabled,true);assert.equal(p.el('writing-step-3').dataset.state,'current');
  await p.el('writing-generate').click();const current=p.loaded.at(-1),changed=JSON.parse(JSON.stringify(current));changed.steps[0].pose[0]+=.01;
  assert.throws(()=>p.workflow.assertCurrent(changed),/重新确认/);p.workflow.invalidate();assert.throws(()=>p.workflow.assertCurrent(current),/重新确认/);
  assert.deepEqual(p.commands,[]);assert.deepEqual(p.actions,['preview']);
});
test('a saved writing plane cannot carry previous approval into a new job or unlock hardware writing',async()=>{
  const p=page(new Map(),false,true);await tick();p.el('writing-axis').value='+Y';await p.el('writing-capture').click();await p.el('writing-generate').click();
  await p.el('writing-check').click();const program=p.loaded.at(-1);
  assert.equal(p.el('writing-execute').disabled,true);assert.match(p.el('writing-execute-note').textContent,/尚未标定/);
  p.emit('rebot-program-loaded',{program:JSON.parse(JSON.stringify(program))});
  assert.equal(p.el('writing-generate').disabled,true);assert.throws(()=>p.workflow.assertCurrent(program,true),/重新确认/);
  assert.equal(p.el('writing-step-1').dataset.state,'current');assert.deepEqual(p.commands,[]);
});
