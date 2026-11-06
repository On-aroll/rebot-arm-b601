// Run the page's actual program coordinator against inert controller methods.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const source=fs.readFileSync(require.resolve('../public/js/twin-workbench.js'),'utf8');
const start=source.indexOf('  async function executeProgram()');
const end=source.indexOf("  action('wb-program-run'",start);
assert.ok(start>=0&&end>start);
function fixture({rejectPreview=false,deferPreview=false,program={steps:[]},writingAssert=()=>{},writingActive=false}={}){
  const calls=[];let owned=false,resolvePreview,previewReached;
  const previewReady=new Promise(resolve=>{previewReached=resolve;});
  const twin={hardware:true,state:()=>({}),owned:()=>owned,
    async prepareProgram(check){check();calls.push('prepare');owned=true;},
    async request(op,args){calls.push(op);if(op==='program_preview'){
      if(rejectPreview)throw new Error('路径超限');
      if(deferPreview)await new Promise(resolve=>{resolvePreview=resolve;previewReached();});
      return {program_plan:{id:'plan-1',program:{steps:[]},steps:[]}};
    }assert.equal(args.plan_id,'plan-1');return {};},
    async stop(){calls.push('stop');owned=false;}};
  const context=vm.createContext({twin,window:{ReBotWritingWorkflow:{assertCurrent:writingAssert,active:()=>writingActive}},document:{hidden:false},editorProgram:()=>program,status(){},showPlan(){}});
  vm.runInContext('var programBusy=false,submittedRun="",simTrial=null,requestEpoch=0,watchedRun="",plan=null,ghostEpoch=0;'+source.slice(start,end),context);
  return {calls,context,previewReady,run:()=>context.executeProgram(),finishPreview:()=>resolvePreview()};
}
test('one execution action prepares, validates and starts without simulation',async()=>{
  const f=fixture();await f.run();
  assert.deepEqual(f.calls,['prepare','program_preview','program_start']);
  await assert.rejects(f.run(),/已有程序/);
  assert.equal(f.calls.filter(c=>c==='program_start').length,1);
});
test('rejected program never starts and releases the lease',async()=>{
  const f=fixture({rejectPreview:true});await assert.rejects(f.run(),/路径超限/);
  assert.deepEqual(f.calls,['prepare','program_preview','stop']);
});
test('stop or editor change while validation is pending prevents delayed start',async()=>{
  const f=fixture({deferPreview:true}),running=f.run();
  await f.previewReady;f.context.requestEpoch++;f.finishPreview();
  await assert.rejects(running,/已取消/);
  assert.ok(!f.calls.includes('program_start'));
});
test('uncalibrated writing geometry is rejected before prepare, enable or acquisition',async()=>{
  const f=fixture({program:{steps:[],writing_plane:{preview_only:true,pen_tip_calibrated:false}}});
  await assert.rejects(f.run(),/几何预览/);assert.deepEqual(f.calls,[]);
});
test('the shared execute button cannot bypass an invalidated writing-plane confirmation',async()=>{
  const f=fixture({program:{steps:[],writing_setup:{text:'BIT103'}},writingAssert(){throw new Error('请重新确认平面');}});
  await assert.rejects(f.run(),/重新确认/);assert.deepEqual(f.calls,[]);
});
test('selecting a writing template cannot accidentally execute an unrelated old editor program',async()=>{
  const f=fixture({writingActive:true,writingAssert(){throw new Error('请先确认新的书写平面');}});
  await assert.rejects(f.run(),/确认新的书写平面/);assert.deepEqual(f.calls,[]);
});
test('editing the confirmed plane during preparation prevents a delayed writing start',async()=>{
  let valid=true;
  const f=fixture({deferPreview:true,program:{steps:[],writing_setup:{text:'BIT103'}},writingAssert(){if(!valid)throw new Error('书写确认已作废');}});
  const running=f.run();await f.previewReady;valid=false;f.context.requestEpoch++;f.finishPreview();
  await assert.rejects(running,/已取消/);assert.ok(!f.calls.includes('program_start'));
});
test('an obsolete writing path reply cannot restore a cleared preview or mark the new settings checked',async()=>{
  const a=source.indexOf('  async function previewProgram()'),b=source.indexOf("  action('wb-program-preview'",a);
  let finish;const deferred=new Promise(resolve=>{finish=resolve;}),shown=[],events=[];
  const context=vm.createContext({editorProgram:()=>({writing_setup:{text:'BIT103'}}),status(){},showPlan:p=>shown.push(p),
    window:{ReBotWritingWorkflow:{assertCurrent(){}},dispatchEvent:event=>events.push(event)},CustomEvent:class{},
    twin:{request:async()=>{await deferred;return {program_plan:{}};}}});
  vm.runInContext('var requestEpoch=0,plan=null,ghostEpoch=0;'+source.slice(a,b),context);
  const running=context.previewProgram();context.requestEpoch++;finish();await assert.rejects(running,/已作废/);
  assert.deepEqual(shown,[]);assert.deepEqual(events,[]);
});
test('both library writing templates open plane settings without loading an archived motion program',async()=>{
  const actions=new Map(),events=[],loaded=[],saved={id:'saved-1',saved_at:0,program:{name:'我的程序',steps:[{type:'wait',seconds:0}]}};
  const nodes={'wb-program-library':{value:''},'wb-program-saved':{replaceChildren(...options){this.options=options;}}};
  const context=vm.createContext({window:{dispatchEvent:event=>events.push(event.detail.text)},
    CustomEvent:class{constructor(type,options){Object.assign(this,{type,...options});}},
    $:id=>nodes[id],action:(id,out,fn)=>actions.set(id,fn),status(){},loadProgram:program=>loaded.push(program),
    api:async()=>({programs:[saved]}),Option:class{constructor(text,value){Object.assign(this,{text,value});}}});
  const selection=source.slice(source.indexOf('  function selectWriting('),source.indexOf("  action('wb-writing-view'"));
  const library=source.slice(source.indexOf('  async function programs()'),source.indexOf('  function showPlan('));
  vm.runInContext('var library=[];'+selection+library,context);
  await context.programs();assert.equal(nodes['wb-program-saved'].options[0].value,'saved-1');
  for(const text of ['BIT103','国庆快乐']){nodes['wb-program-library'].value='writing:'+text;await actions.get('wb-program-load')();}
  assert.deepEqual(events,['BIT103','国庆快乐']);assert.deepEqual(loaded,[]);
  nodes['wb-program-library'].value='saved-1';await actions.get('wb-program-load')();assert.equal(loaded[0],saved.program);
});
