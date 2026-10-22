// Run the actual tuning UI against inert transport and storage; no ROS or CAN.
const {test}=require('node:test'),assert=require('node:assert/strict'),vm=require('node:vm'),fs=require('node:fs');
const source=fs.readFileSync(require.resolve('../public/js/twin-algorithm.js'),'utf8');
function page(storage=new Map()){
  const els=new Map(),events=new Map(),requests=[];
  const html=fs.readFileSync(require.resolve('../public/index.html'),'utf8');
  function el(id){
    assert.ok(html.includes(`id="${id}"`),`missing actual DOM ${id}`);
    if(!els.has(id)){const handlers=new Map();els.set(id,{value:'',disabled:false,hidden:false,dataset:{},textContent:'',
      addEventListener(type,fn){if(!handlers.has(type))handlers.set(type,[]);handlers.get(type).push(fn);},
      replaceChildren(...opts){this.options=opts;this.value=opts[0].value;},
      async fire(type){for(const fn of handlers.get(type)||[])await fn({target:this});},
      async input(value){this.value=value;await this.fire('input');},async click(){await this.fire('click');}});
    }return els.get(id);
  }
  el('ctrl-engine').value='external';el('ctrl-rate').value='10';el('ctrl-output').value='joint_velocity';el('ctrl-params').value='{}';
  const twin={fresh:()=>true,owned:()=>true,jointSpeed:()=>.1,state:()=>({actual:Array(6).fill(0),motion_profile:{selected:'ruckig'}}),
    async request(op,args){requests.push({op,args});return {check_id:'checked',message:'ok'};}};
  const window={ReBotTwin:twin,addEventListener:(name,fn)=>events.set(name,fn)};
  vm.runInNewContext(source,{window,document:{getElementById:el},localStorage:{getItem:k=>storage.get(k),setItem:(k,v)=>storage.set(k,v)},
    Option:class{constructor(text,value){this.text=text;this.value=value;}},console,setTimeout});
  return {el,events,storage,requests,async select(engine){el('ctrl-engine').value=engine;await el('ctrl-engine').fire('change');},params:()=>JSON.parse(el('ctrl-params').value)};
}
test('P PD PID have independent drafts, per-axis gains and persistent named profiles',async()=>{
  const p=page();await p.select('pid');await p.el('ctrl-kp1').input('1.25');await p.el('ctrl-ki1').input('.05');
  assert.equal(p.params().kp[0],1.25);assert.equal(p.params().ki[0],.05);
  await p.select('pd');assert.equal(p.params().kp[0],.6);assert.equal(p.el('ctrl-ki1').disabled,true);
  await p.el('ctrl-kp1').input('.9');await p.select('pid');assert.equal(p.params().kp[0],1.25);assert.equal(p.params().ki[0],.05);
  await p.el('ctrl-name').input('PID test A');await p.el('ctrl-save').click();
  const reloaded=page(p.storage);assert.equal(reloaded.el('ctrl-engine').value,'pid');assert.equal(reloaded.params().kp[0],1.25);
  await reloaded.select('p');assert.equal(reloaded.el('ctrl-kd1').disabled,true);
  await reloaded.select('pd');assert.equal(reloaded.params().kp[0],.9);
  assert.equal(p.requests.length,0);assert.equal(reloaded.requests.length,0);
});
test('editing gains and global motion configuration invalidates checked references',async()=>{
  const p=page();await p.select('pd');await p.el('ctrl-current').click();await p.el('ctrl-check').click();
  assert.equal(p.el('ctrl-start').disabled,false);assert.equal(p.requests[0].args.config.engine,'pd');assert.equal(p.requests[0].args.config.endpoint,'');
  await p.el('ctrl-kp2').input('.7');assert.equal(p.el('ctrl-start').disabled,true);
  await p.el('ctrl-check').click();p.events.get('rebot-motion-profile-changed')();assert.equal(p.el('ctrl-start').disabled,true);
  assert.ok(p.requests.every(c=>c.op==='controller_check'));
  p.events.get('rebot-twin-state')({detail:{extensions:{controller:{active:true,status:'running',cycles:1}}}});
  assert.equal(p.el('ctrl-engine').disabled,true);assert.equal(p.el('ctrl-kp1').disabled,true);assert.equal(p.el('ctrl-default').disabled,true);
});

test('external service parameters survive switching to built-in controllers without gain injection',async()=>{
  const p=page();await p.el('ctrl-params').input('{"model":"custom","gain":0.9}');
  await p.select('pid');await p.select('external');
  assert.deepEqual(p.params(),{model:'custom',gain:.9});
  await p.select('pd');await p.select('external');
  assert.deepEqual(p.params(),{model:'custom',gain:.9});
});

test('advanced controllers have actual independent parameter forms and no PID field injection',async()=>{
  const p=page();await p.select('ladrc');
  assert.equal(p.el('ctrl-gains').hidden,true);assert.equal(p.el('ctrl-ladrc').hidden,false);
  await p.el('ctrl-bandwidth').input('.8');assert.equal(p.params().control_bandwidth,.8);
  assert.equal(p.params().kp,undefined);
  await p.select('mpc');await p.el('ctrl-horizon').input('16');assert.equal(p.params().horizon,16);
  assert.equal(p.el('ctrl-mpc').hidden,false);assert.equal(p.params().control_bandwidth,undefined);
  await p.select('scheduled_pid');await p.el('ctrl-schedule-ratio').input('2');await p.el('ctrl-ki1').input('.04');
  assert.equal(p.params().schedule_ratio,2);assert.equal(p.el('ctrl-ki1').disabled,false);
  await p.select('ladrc');assert.equal(p.params().control_bandwidth,.8);
  await p.select('mpc');assert.equal(p.params().horizon,16);await p.el('ctrl-current').click();await p.el('ctrl-check').click();
  assert.equal(p.requests[0].args.config.engine,'mpc');assert.equal(p.params().kp,undefined);
  p.events.get('rebot-twin-state')({detail:{extensions:{controller:{active:true,status:'running',cycles:1}}}});
  assert.equal(p.el('ctrl-horizon').disabled,true);assert.equal(p.el('ctrl-bandwidth').disabled,true);
});

test('check results cannot be exported as a new running experiment or display old run metrics',async()=>{
  const p=page();await p.select('pid');await p.el('ctrl-current').click();await p.el('ctrl-check').click();await p.el('ctrl-start').click();
  p.events.get('rebot-twin-state')({detail:{extensions:{controller:{active:false,status:'checked',engine:'pid',name:'PID 初始参数',cycles:0,tracking:{samples:10,rmse_deg:[1,1,1,1,1,1]}}}}});
  await p.el('ctrl-record').click();assert.match(p.el('ctrl-status').textContent,/等待本次控制器产生运行样本/);
  assert.match(p.el('ctrl-tracking').textContent,/等待新鲜运行样本/);
});
