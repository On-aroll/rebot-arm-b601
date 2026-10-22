// Run the page's actual program coordinator against inert controller methods.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const source=fs.readFileSync(require.resolve('../public/js/twin-workbench.js'),'utf8');
const start=source.indexOf('  async function executeProgram()');
const end=source.indexOf("  action('wb-program-run'",start);
assert.ok(start>=0&&end>start);
function fixture({rejectPreview=false,deferPreview=false}={}){
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
  const context=vm.createContext({twin,document:{hidden:false},editorProgram:()=>({steps:[]}),status(){},showPlan(){}});
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
