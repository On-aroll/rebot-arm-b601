// Exercise the actual page controller with an inert ROS transport. No network/CAN.
const {test}=require('node:test');
const assert=require('node:assert/strict');
const vm=require('node:vm');
const fs=require('node:fs');
const input=require('../public/js/twin-input.js');
const Display=require('../public/js/twin-feedback-display.js');
const source=fs.readFileSync(require.resolve('../public/js/twin-console.js'),'utf8');
function page({connected=true,storage=new Map(),pads=[]}={}) {
  const elements=new Map(), subscriptions=new Map(), windowEvents=new Map(), documentEvents=new Map();
  const commands=[],calls=[],ghosts=[];
  let now=100,animation;
  const timers=new Map();
  function el(id) {
    if (!elements.has(id)) {
      const handlers=new Map();
      elements.set(id,{id,value:'0',checked:false,dataset:{},textContent:'',disabled:false,
        hidden:false,readOnly:false,classList:{add(){},remove(){}},
        closest:()=>null,focus(){document.activeElement=this;},append(){},checkValidity:()=>true,
        setAttribute(k,v){this[k]=v;},replaceChildren(...opts){this.value=(opts.find(o=>o.selected)||opts[0]).value;},
        addEventListener(type,fn){handlers.set(type,[...(handlers.get(type)||[]),fn]);},
        async dispatchEvent(e){for(const fn of handlers.get(e.type)||[]) await fn(e);},
        async click(){await this.dispatchEvent({type:'click'});}});
    }
    return elements.get(id);
  }
  const tabs=['point','keyboard','gamepad','tcp'].map(m=>{const e=el(m);e.dataset.twinMode=m;return e;});
  const panels=['point','keyboard','gamepad'].map(m=>{const e=el('panel-'+m);e.dataset.twinPanel=m;return e;});
  el('ros-control-enable').closest=()=>({});
  el('twin-orientation').value='free';
  el('twin-joint-speed').value='.01';
  el('twin-gripper-speed').value='.5';
  el('twin-manual-override').value='100';
  el('twin-auto-manual-recovery').checked=true;
  const client={connected,namespace:'rebotarm',advertise(){},publish(topic,msg){commands.push(JSON.parse(msg.data));},
    subscribe(topic,type,fn){subscriptions.set(topic,fn);},addEventListener(name,fn){windowEvents.set('client-'+name,fn);},
    async callService(name,type,args,options){
      calls.push({name,options});
      if(name.endsWith('/acquire')) return {success:true,message:JSON.stringify({session:'lease'})};
      return {success:true,message:JSON.stringify({id:'reset-1',status:'queued',message:'等待重置'})};
    }};
  const sim={stopMotion(){},setGhostVisible(v){ghosts.push(v);},getAngles:()=>({}),setAngles(){},setFeedbackTarget(){ghosts.push(true);}};
  const document={activeElement:null,hidden:false,body:{classList:{toggle:()=>false}},
    getElementById:el,addEventListener(name,fn){documentEvents.set(name,fn);},querySelectorAll(selector){
      if(selector==='[data-twin-mode]')return tabs;
      if(selector==='[data-twin-panel]')return panels;
      return [];
    }};
  const window={reBotRos:client,reBotRosUI:{target:'hardware',canTwinControl:()=>true,prepareTwinControl(){}},
    localStorage:{getItem:key=>storage.get(key),setItem:(key,value)=>storage.set(key,value)},
    ReBotTwinInput:input,ReBotFeedbackDisplay:Display,reBotSim:sim,reBotControlMode:{is:()=>true},
    dispatchEvent(event){windowEvents.get(event.type)?.(event);},
    addEventListener(name,fn){windowEvents.set(name,fn);}};
  const context={window,document,navigator:{getGamepads:()=>pads},performance:{now:()=>now},console,
    setTimeout,clearTimeout,setInterval(fn,ms){timers.set(ms,fn);return ms;},Event:class{constructor(type){this.type=type;}},
    CustomEvent:class{constructor(type,args={}){this.type=type;this.detail=args.detail;}},
    Option:class{constructor(text,value,d,selected){Object.assign(this,{text,value,selected});}},
    requestAnimationFrame(fn){animation=fn;}};
  vm.runInNewContext(source,context);
  const state={actual:[0,0,0,0,0,0],pose:[.1,.2,.3,0,0,0],feedback_valid:true,owned:false,
    extensions:{gripper:{available:true,fresh:true,speed_max:1}},message:'只读',hardware_phase:'READONLY',control_prepared:true,
    control_settings:{mode:'small',max_speed_rad_s:.02,remaining_s:null}};
  function feedback(extra={}) {Object.assign(state,extra);subscriptions.get('/rebotarm/twin/state')({data:JSON.stringify(state)});}
  return {el,client,commands,calls,ghosts,feedback,tabs,windowEvents,window,storage,document,documentEvents,
    frame(){now+=60;timers.get(20)?.();animation(now);},
    inputTick(ms=20){now+=ms;assert.equal(typeof timers.get(20),'function');timers.get(20)();},
    connectionTick(ms=50){now+=ms;assert.equal(typeof timers.get(50),'function');timers.get(50)();},
    event(result){subscriptions.get('/rebotarm/twin/events')({data:JSON.stringify(result)});}};
}
test('keyboard quick speed stays local; TCP drop uses automatic point speed and the guarded executor',async()=>{
  const p=page();p.feedback({control_settings:{mode:'full',max_speed_rad_s:1}});
  await p.tabs[1].click();const key=(code,repeat=false)=>p.windowEvents.get('keydown')({code,repeat,preventDefault(){}});
  key('BracketRight');key('BracketRight',true);
  assert.equal(p.el('twin-manual-override').value,'110');assert.equal(p.commands.length,0);
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;await p.el('twin-acquire').click();
  p.feedback({owned:true,mode:'HOLD',hardware_phase:'ACTIVE'});
  key('Space');key('KeyW');p.inputTick();const fast=p.commands.at(-1);
  assert.equal(fast.op,'jog');assert.ok(Math.abs(fast.twist[0]-.0275)<1e-12);assert.equal(fast.speed,.1);
  key('ShiftLeft');p.inputTick();assert.ok(Math.abs(p.commands.at(-1).twist[0]-.006875)<1e-12);
  assert.equal(p.commands.at(-1).speed,fast.speed);
  p.windowEvents.get('keyup')({code:'ShiftLeft'});p.inputTick();assert.equal(p.commands.at(-1).twist[0],fast.twist[0]);
  p.document.activeElement={closest:()=>({})};key('BracketRight');assert.equal(p.el('twin-manual-override').value,'110');
  p.document.activeElement=null;await p.tabs[3].click();
  p.el('twin-joint-speed').value='.005';p.el('twin-linear-speed').value='.5';
  const moving=p.window.ReBotTwin.drag([.15,.2,.3,0,0,0]);
  assert.equal(p.commands.at(-1).op,'hold');await acknowledge(p);
  const drag=p.commands.at(-1);assert.equal(drag.op,'preview');assert.equal(drag.speed,.3);
  assert.deepEqual(drag.pose,[.15,.2,.3,0,0,0]);assert.equal(drag.linear,undefined);
  await acknowledge(p,{plan:dragPlan});assert.equal(p.commands.at(-1).op,'goal');
  assert.equal(p.commands.at(-1).plan_id,dragPlan.id);await acknowledge(p);await moving;
  assert.equal(p.el('twin-joint-speed').value,'.005');assert.equal(p.el('twin-linear-speed').value,'.5');
  assert.equal(p.commands.filter(c=>c.op==='tcp_target').length,0);
});
const dragPlan={id:'drop-plan',joints:[0,0,0,0,0,0],duration:1.5};
async function acknowledge(p,extra={}) {p.event({id:p.commands.at(-1).id,ok:true,...extra});await Promise.resolve();}
async function dragPage(settings={mode:'full',max_speed_rad_s:1}) {
  const p=page();p.feedback({control_settings:settings});await p.tabs[3].click();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;await p.el('twin-acquire').click();
  p.feedback({owned:true,mode:'HOLD',hardware_phase:'ACTIVE'});return p;
}
test('TCP automatic speed respects small/device caps and snapshots the orientation constraint',async()=>{
  for(const cap of [.02,.12]) {
    const p=await dragPage({mode:cap===.02?'small':'full',min_speed_rad_s:.005,max_speed_rad_s:cap});
    p.el('twin-drag-orientation').checked=true;
    const moving=p.window.ReBotTwin.drag([.12,.2,.3,1,2,3]);await acknowledge(p);
    assert.equal(p.commands.at(-1).speed,cap);assert.equal(p.commands.at(-1).orientation,true);
    await acknowledge(p,{plan:dragPlan});await acknowledge(p);await moving;
  }
});
test('TCP rejected path never starts a goal',async()=>{
  const p=await dragPage(),moving=p.window.ReBotTwin.drag([.7,.2,.3,0,0,0]);
  const rejected=assert.rejects(moving,/路径超限/);await acknowledge(p);
  p.event({id:p.commands.at(-1).id,ok:false,message:'路径超限'});await rejected;
  assert.equal(p.commands.filter(c=>c.op==='goal').length,0);
});
test('TCP stop, input switch, hidden page and superseding gesture discard a late plan',async()=>{
  for(const cancel of [p=>p.window.ReBotTwin.hold(),p=>p.tabs[1].click(),p=>{p.document.hidden=true;},null]) {
    const p=await dragPage();let current=true;
    const moving=p.window.ReBotTwin.drag([.12,.2,.3,0,0,0],{check(){if(!current)throw Error('新目标已覆盖');}});
    const rejected=assert.rejects(moving,/取消|覆盖/);await acknowledge(p);
    const preview=p.commands.at(-1);if(cancel)await cancel(p);else current=false;
    p.event({id:preview.id,ok:true,plan:dragPlan});await rejected;
    assert.equal(p.commands.filter(c=>c.op==='goal').length,0);
  }
});
test('TCP cannot start without ownership or fresh feedback',async()=>{
  const p=page();await assert.rejects(p.window.ReBotTwin.drag([.1,.2,.3,0,0,0]),/取消/);
  const active=await dragPage();active.feedback({feedback_valid:false});
  await assert.rejects(active.window.ReBotTwin.drag([.1,.2,.3,0,0,0]),/过期/);
  assert.equal(active.commands.filter(c=>c.op==='preview').length,0);
});
test('gamepad X/Y update slider once per press without enabling or changing A/B mapping',async()=>{
  const pad={mapping:'standard',connected:true,index:0,id:'test',axes:[0,0,0,0],buttons:Array.from({length:17},()=>({value:0}))};
  const p=page({pads:[pad]});p.feedback({control_settings:{mode:'full',max_speed_rad_s:1}});await p.tabs[2].click();
  p.connectionTick();pad.buttons[3].value=1;p.connectionTick();p.connectionTick();
  assert.equal(p.el('twin-manual-override').value,'110');assert.equal(p.commands.length,0);assert.equal(p.calls.length,0);
  pad.buttons[3].value=0;p.connectionTick();pad.buttons[2].value=1;p.connectionTick();
  assert.equal(p.el('twin-manual-override').value,'100');
});
test('motion profile switch waits for acknowledgement, invalidates plans and cannot enable',async()=>{
  const p=page();p.feedback({control_prepared:false,motion_profile:{selected:'ruckig',available:['ruckig','quintic']},control_settings:{mode:'full',max_speed_rad_s:1}});
  assert.equal(p.el('twin-motion-profile').disabled,false);
  let invalidated=0;p.window.addEventListener('rebot-motion-profile-changed',()=>invalidated++);
  p.el('twin-motion-profile').value='quintic';
  const switching=p.el('twin-motion-profile').dispatchEvent({type:'change'});
  const command=p.commands.at(-1);
  assert.equal(command.op,'motion_profile');assert.equal(command.profile,'quintic');
  assert.equal(p.el('twin-motion-profile').disabled,true);
  await assert.rejects(p.window.ReBotTwin.prepareProgram(),/正在切换/);
  assert.equal(p.calls.length,0);
  p.event({id:command.id,ok:true,motion_profile:'quintic'});await switching;
  assert.equal(p.el('twin-motion-profile').value,'quintic');assert.equal(invalidated,1);
  assert.equal(p.commands.length,1);assert.equal(p.calls.length,0);
  p.feedback({control_prepared:true});assert.equal(p.el('twin-motion-profile').disabled,true);
});
test('rejected profile switch restores the controller selection',async()=>{
  const p=page();p.feedback({control_prepared:false,motion_profile:{selected:'ruckig'},control_settings:{mode:'full',max_speed_rad_s:1}});
  p.el('twin-motion-profile').value='quintic';
  const switching=p.el('twin-motion-profile').dispatchEvent({type:'change'});
  p.event({id:p.commands.at(-1).id,ok:false,message:'控制权已占用'});await switching;
  assert.equal(p.el('twin-motion-profile').value,'ruckig');
  assert.equal(p.calls.length,0);
});

test('MIT driver edits stay inert until ack, block preparation races and retain actual backend display',async()=>{
  const p=page(),driver={selected:'csp',parameters:{kp:[50,150,150,50,50,50],kd:[3,10,10,5,4,4]}};
  const motion={selected:'ruckig',backend:'CSP',driver,profiles:{ruckig:{acceleration_scale:1,jerk_scale:1}}};
  p.feedback({control_prepared:false,motion_profile:motion,control_settings:{mode:'full',max_speed_rad_s:1}});
  p.el('drive-backend').value='mit';await p.el('drive-backend').dispatchEvent({type:'change'});
  assert.equal(p.commands.length,0);assert.equal(p.el('drive-mit-gains').hidden,false);
  assert.match(p.el('drive-status').textContent,/已生效：CSP.*未应用/);
  p.el('drive-kp1').value='40';const applying=p.el('drive-apply').click(),cmd=p.commands.at(-1);
  assert.equal(cmd.op,'drive_config');assert.equal(cmd.parameters.kp[0],40);
  await assert.rejects(p.window.ReBotTwin.prepareProgram(),/正在切换/);assert.equal(p.calls.length,0);
  driver.selected='mit';driver.parameters.kp[0]=40;motion.backend='MIT';
  p.event({id:cmd.id,ok:true,motion_settings:motion});await applying;
  assert.match(p.el('drive-status').textContent,/已生效：MIT/);assert.equal(p.calls.length,0);
  p.feedback({control_prepared:true});assert.equal(p.el('drive-apply').disabled,true);
});
test('reset clears ownership, held keys, approvals and old pose; success reloads measured pose',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true});
  await p.tabs[1].click();
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  p.frame();assert.ok(p.commands.some(c=>c.op==='jog'));
  const count=p.commands.length;
  await p.el('twin-reset-control').click();
  assert.equal(p.window.reBotTwinActive,false);
  assert.equal(p.el('twin-onsite').checked,false);
  assert.equal(p.el('ros-control-enable').checked,false);
  assert.equal(p.el('twin-go').disabled,true);
  assert.equal(p.el('twin-reset-control').disabled,true);
  assert.equal(p.el('twin-x').value,'');
  assert.equal(p.tabs[0]['aria-pressed'],'true');
  p.frame();assert.equal(p.commands.length,count);
  assert.equal(p.calls.filter(c=>c.name.endsWith('/reset')).length,1);
  assert.equal(p.calls.at(-1).options.retryOnTimeout,false);
  p.feedback({owned:false,control_prepared:false,reset_status:{id:'reset-1',status:'succeeded',message:'已重置'}});
  assert.equal(p.el('twin-x').value,'100.0');
  assert.equal(p.el('twin-reset-control').disabled,false);
  assert.equal(p.el('twin-reset-status').textContent,'已重置');
  p.frame();assert.equal(p.commands.length,count);
});
test('healthy visible feedback keeps heartbeats independent of animation frames',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();
  for(let i=0;i<24;i++){
    p.feedback({owned:true,hardware_phase:'ACTIVE',mode:'HOLD'});
    p.connectionTick(); // Deliberately never invoke the animation callback.
  }
  const beats=p.commands.filter(c=>c.op==='heartbeat');
  assert.ok(beats.length>=7);
  assert.ok(p.commands.every(c=>c.op==='heartbeat'));
  assert.equal(p.calls.filter(c=>c.name.endsWith('/stop')).length,0);
});
test('manual input uses the 20ms timer without rendering and never replays missed samples',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true,mode:'HOLD'});
  await p.tabs[1].click();
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  for(let i=0;i<10;i++)p.inputTick(); // No animation frames at all.
  assert.equal(p.commands.filter(c=>c.op==='jog').length,10);
  p.inputTick(80); // A delayed timer samples once, without catch-up bursts.
  assert.equal(p.commands.filter(c=>c.op==='jog').length,11);
  p.windowEvents.get('keyup')({code:'Space'});
  assert.equal(p.commands.at(-1).op,'hold');
  const count=p.commands.length;
  for(let i=0;i<5;i++)p.inputTick();
  assert.equal(p.commands.length,count);
});
test('hold recovery updates the old error without acquiring or enabling',()=>{
  const p=page();
  const settings={mode:'full',max_speed_rad_s:1,continuous:true,remaining_s:null};
  p.feedback({hardware_phase:'HOLDING',mode:'HOLD',message:'通信异常：反馈未确认',
    communication_hold:{feedback_confirmed:false,reason:'J2 电流反馈过期'},control_settings:settings});
  assert.match(p.el('twin-message').textContent,/反馈未确认/);
  assert.match(p.el('twin-message').textContent,/J2 电流反馈过期/);
  assert.doesNotMatch(p.el('twin-session-remaining').textContent,/尚未出力/);
  p.feedback({message:'J2 电流反馈过期；反馈已恢复，重新接管后可提交新动作。',
    communication_hold:{feedback_confirmed:true,reason:'J2 电流反馈过期'}});
  assert.match(p.el('twin-message').textContent,/反馈已恢复/);
  assert.doesNotMatch(p.el('twin-message').textContent,/反馈未确认/);
  assert.match(p.el('twin-session-remaining').textContent,/冻结/);
  p.connectionTick(501);
  assert.match(p.el('twin-power-state').textContent,/反馈未确认/);
  assert.equal(p.calls.length,0);assert.equal(p.commands.length,0);
});
test('read-only feedback retry blocks preparation and displays recovery without a lease',()=>{
  const p=page();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  p.feedback({hardware_phase:'FEEDBACK_WAIT',control_prepared:false,feedback_valid:false,
    message:'只读反馈恢复中：J1查询超时；禁止使能'});
  assert.equal(p.el('twin-enable-hardware').disabled,true);
  assert.equal(p.el('twin-acquire').disabled,true);
  assert.match(p.el('twin-message').textContent,/只读反馈恢复中/);
  assert.match(p.el('twin-power-state').textContent,/只读反馈重试/);
  p.feedback({hardware_phase:'READONLY',feedback_valid:true,message:'只读反馈已恢复；未使能，旧控制已作废'});
  assert.match(p.el('twin-message').textContent,/只读反馈已恢复/);
  assert.equal(p.el('twin-enable-hardware').disabled,false);
  assert.equal(p.el('twin-acquire').disabled,true); // Explicit preparation still required.
  assert.equal(p.calls.length,0);assert.equal(p.commands.length,0);
});
test('missing ROS state stops through the communication timer without rendering',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true});
  p.connectionTick(501);
  assert.match(p.commands.find(c=>c.op==='release').reason,/未收到 ROS 控制状态/);
  const count=p.commands.length;p.connectionTick();
  assert.equal(p.commands.length,count);
  assert.equal(p.calls.filter(c=>c.name.endsWith('/stop')).length,1);
});
test('fresh ROS state with stale motor feedback reports the motor cause',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();
  p.feedback({owned:true,feedback_valid:false,feedback_age_ms:650});p.connectionTick();
  assert.match(p.commands.find(c=>c.op==='release').reason,/控制器报告实机反馈过期/);
  assert.ok(!p.commands.some(c=>c.op==='heartbeat'));
});
test('hidden pages do not renew the control heartbeat',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true});
  p.document.hidden=true;p.connectionTick(200);
  assert.ok(!p.commands.some(c=>c.op==='heartbeat'));
});
test('communication timer preserves motion request priority until acknowledgement',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true});
  const result=p.window.ReBotTwin.request('power_hold');
  const command=p.commands.at(-1);
  for(let i=0;i<5;i++){p.feedback();p.connectionTick();}
  assert.equal(p.commands.length,1);assert.equal(command.op,'power_hold');
  p.event({id:command.id,ok:true,message:'保持已请求'});await result;
  p.connectionTick();assert.equal(p.commands.at(-1).op,'heartbeat');
});
test('program preparation combines prepare and acquire without motor startup',async()=>{
  const p=page();p.feedback({control_prepared:false,control_settings:{mode:'full',max_speed_rad_s:1,continuous:true}});
  p.el('twin-onsite').checked=true;
  await p.window.ReBotTwin.prepareProgram();
  assert.deepEqual(p.calls.map(c=>c.name),['/rebotarm/enable','/rebotarm/twin/acquire']);
  assert.equal(p.el('ros-control-enable').checked,true);
  assert.equal(p.window.ReBotTwin.owned(),true);
  assert.equal(p.commands.length,0); // No power_hold or motion before program validation.
});
test('program preparation needs onsite confirmation and respects cancellation',async()=>{
  const p=page();p.feedback({control_prepared:false,control_settings:{mode:'full',max_speed_rad_s:1}});
  await assert.rejects(p.window.ReBotTwin.prepareProgram(),/现场确认/);
  assert.equal(p.calls.length,0);
  p.el('twin-onsite').checked=true;
  let cancelled=false;
  const original=p.client.callService;
  p.client.callService=async(...args)=>{const result=await original(...args);cancelled=true;return result;};
  await assert.rejects(p.window.ReBotTwin.prepareProgram(()=>{if(cancelled)throw new Error('已取消');}),/已取消/);
  assert.deepEqual(p.calls.map(c=>c.name),['/rebotarm/enable']);
  assert.equal(p.window.ReBotTwin.owned(),false);
});
test('cancelled program acquisition releases a late lease',async()=>{
  const p=page();p.feedback({control_settings:{mode:'full',max_speed_rad_s:1}});
  p.el('twin-onsite').checked=true;
  let cancelled=false;
  const original=p.client.callService;
  p.client.callService=async(...args)=>{const result=await original(...args);cancelled=true;return result;};
  await assert.rejects(p.window.ReBotTwin.prepareProgram(()=>{if(cancelled)throw new Error('已取消');}),/已取消/);
  assert.equal(p.commands.at(-1).op,'suspend');
  assert.equal(p.window.ReBotTwin.owned(),false);
});
test('direction release sends fresh neutral samples until acknowledged; Space release immediately holds',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true,mode:'HOLD',input_seq:-1});
  await p.tabs[1].click();
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  p.frame();assert.equal(p.commands.at(-1).op,'jog');
  p.windowEvents.get('keyup')({code:'KeyR'});p.frame();
  const neutral=p.commands.at(-1);
  assert.equal(neutral.op,'jog');assert.ok(neutral.twist.every(x=>x===0));
  p.feedback({mode:'HOLD',input_seq:neutral.seq-1});p.frame();
  assert.equal(p.commands.at(-1).op,'jog'); // Old HOLD cannot acknowledge braking.
  p.feedback({mode:'HOLD',input_seq:neutral.seq});
  const count=p.commands.filter(c=>c.op==='jog').length;p.frame();
  assert.equal(p.commands.filter(c=>c.op==='jog').length,count);
  p.windowEvents.get('keydown')({code:'KeyR',preventDefault(){}});p.frame();
  assert.equal(p.commands.at(-1).op,'jog');
  p.windowEvents.get('keyup')({code:'Space'});
  assert.equal(p.commands.at(-1).op,'hold');
  p.frame();assert.notEqual(p.commands.at(-1).op,'jog');
});
test('keyboard shows lease, editing, Space and accepted-input states without silently swallowing input',async()=>{
  const p=page();p.feedback();await p.tabs[1].click();p.frame();
  assert.match(p.el('twin-keyboard-status').textContent,/尚未接管/);
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true,mode:'HOLD'});
  p.document.activeElement={closest:()=>({})};p.documentEvents.get('focusin')();p.frame();
  assert.match(p.el('twin-keyboard-status').textContent,/正在编辑/);
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  p.frame();assert.ok(!p.commands.some(c=>c.op==='jog'));
  await p.el('twin-keyboard-focus').click();p.frame();
  assert.match(p.el('twin-keyboard-status').textContent,/等待空格/);
  p.windowEvents.get('keydown')({code:'Space',preventDefault(){}});p.frame();
  assert.match(p.el('twin-keyboard-status').textContent,/空格已识别/);
  p.windowEvents.get('keydown')({code:'KeyR',preventDefault(){}});p.frame();
  assert.equal(p.commands.at(-1).op,'jog');
  p.feedback({mode:'JOG'});p.frame();assert.match(p.el('twin-keyboard-status').textContent,/已接收/);
});
test('controller rejection stays visible and held OS repeats cannot restart keyboard motion',async()=>{
  const p=page();p.feedback();p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true,mode:'HOLD'});await p.tabs[1].click();
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  p.frame();const before=p.commands.filter(c=>c.op==='jog').length;
  p.event({ok:false,message:'J2 重力超限'});p.frame();
  assert.match(p.el('twin-keyboard-status').textContent,/控制器拒绝：J2 重力超限/);
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,repeat:true,preventDefault(){}});
  p.frame();assert.equal(p.commands.filter(c=>c.op==='jog').length,before);
  for(const code of ['Space','KeyR']){p.windowEvents.get('keyup')({code});p.windowEvents.get('keydown')({code,repeat:false,preventDefault(){}});}
  p.frame();assert.equal(p.commands.filter(c=>c.op==='jog').length,before+1);
});
test('gamepad center brakes while RB is held; RB release bypasses the ramp',async()=>{
  const pad={index:0,id:'inert',connected:true,mapping:'standard',axes:[0,0,0,0],buttons:Array.from({length:17},()=>({value:0}))};
  const p=page({pads:[pad]});p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true,mode:'HOLD'});
  await p.tabs[2].click();p.frame(); // Neutral guard.
  pad.buttons[5].value=1;pad.axes[1]=-.58;p.frame();
  assert.equal(p.commands.at(-1).op,'jog');
  assert.ok(p.commands.at(-1).twist[0]>0);
  pad.axes[1]=0;p.frame();
  assert.equal(p.commands.at(-1).op,'jog');assert.ok(p.commands.at(-1).twist.every(x=>x===0));
  pad.buttons[5].value=0;p.frame();assert.equal(p.commands.at(-1).op,'hold');
});
test('delayed preview cannot bring back a cleared target after reset',async()=>{
  const p=page();p.feedback();
  const preview=p.el('twin-preview').click();
  assert.ok(p.commands.some(c=>c.op==='preview'));
  await p.el('twin-reset-control').click();await preview;
  p.event({id:p.commands[0].id,ok:true,plan:{id:'old',joints:[0,0,0,0,0,0]}});
  assert.equal(p.el('twin-go').disabled,true);
  assert.ok(!p.ghosts.includes(true));
});
test('offline reset never claims hardware recovered or sends a command',async()=>{
  const p=page({connected:false});
  await p.el('twin-reset-control').click();
  assert.equal(p.calls.length,0);assert.equal(p.commands.length,0);
  assert.match(p.el('twin-reset-status').textContent,/未确认实机/);
  assert.equal(p.el('twin-reset-status').dataset.error,'true');
  assert.equal(p.el('twin-reset-control').disabled,false);
});
test('lost connection during reset is visible and does not strand the button',async()=>{
  const p=page();p.feedback();await p.el('twin-reset-control').click();
  p.client.connected=false;p.windowEvents.get('client-status')({detail:{state:'closed'}});
  assert.equal(p.el('twin-reset-control').disabled,false);
  assert.match(p.el('twin-reset-status').textContent,/结果未确认/);
});
test('full mode shows continuous power only after startup, without a countdown',()=>{
  const p=page();
  const control_settings={mode:'full',continuous:true,session_limit_s:null,remaining_s:null,max_speed_rad_s:.1};
  p.feedback({control_settings});
  assert.match(p.el('twin-session-remaining').textContent,/尚未出力/);
  p.feedback({hardware_phase:'ACTIVE'});
  assert.match(p.el('twin-session-remaining').textContent,/持续使能保持/);
  assert.match(p.el('twin-range-help').textContent,/无固定使能时长/);
  assert.doesNotMatch(p.el('twin-session-remaining').textContent,/尚未出力|剩余|30/);
});
test('full defaults are five times the previous defaults and small stays slow',()=>{
  const p=page();
  p.feedback({control_settings:{mode:'full',continuous:true,max_speed_rad_s:.1}});
  assert.equal(p.el('twin-joint-speed').value,'0.1');
  assert.equal(p.el('twin-linear-speed').value,'25');
  assert.equal(p.el('twin-angular-speed').value,'10');
  p.feedback({control_settings:{mode:'small',max_speed_rad_s:.02}});
  assert.equal(p.el('twin-joint-speed').value,'0.01');
  assert.equal(p.el('twin-linear-speed').value,'1');
  assert.equal(p.el('twin-angular-speed').value,'1');
});
test('numeric speed follows backend bounds, preserves edits and accepts non-preset values',async()=>{
  const p=page();
  const control_settings={mode:'full',min_speed_rad_s:.005,max_speed_rad_s:.9,command_period_ms:76.5};
  p.feedback({control_settings});
  assert.equal(p.el('twin-joint-speed').max,'0.9');
  assert.equal(p.el('twin-joint-speed').value,'0.1');
  p.el('twin-joint-speed').value='0.175';
  await p.el('twin-joint-speed').dispatchEvent({type:'input'});
  assert.match(p.el('twin-joint-degrees').textContent,/10.0/);
  p.feedback();assert.equal(p.el('twin-joint-speed').value,'0.175');
  assert.match(p.el('twin-control-timing').textContent,/77 ms/);
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true});
  await p.tabs[1].click();
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  p.frame();assert.equal(p.commands.at(-1).op,'jog');assert.equal(p.commands.at(-1).speed,.175);
  p.feedback({mode:'JOG'});assert.equal(p.el('twin-joint-speed').disabled,true);
  p.feedback({mode:'HOLD'});assert.equal(p.el('twin-joint-speed').disabled,false);
});
test('official and device caps are distinguished and full mode removes the hidden 20 percent reduction',()=>{
  const p=page();p.feedback({control_settings:{mode:'full',min_speed_rad_s:.005,max_speed_rad_s:1.,
    official_speed_max_rad_s:1.5,acceleration_rad_s2:4.,jerk_rad_s3:30.,manual_speed_fraction:1.,
    torque_limits_nm:[36,36,36,14,14,14],current_limits_a:[57,57,57,16,16,16],gravity_limits_nm:[11,11,11,5,5,5]}});
  assert.equal(p.el('twin-joint-speed').max,'1');
  assert.match(p.el('twin-limit-summary').textContent,/官方网页关节速度上限 1.5/);
  assert.match(p.el('twin-limit-summary').textContent,/当前生效上限 1 rad/);
  assert.match(p.el('twin-limit-summary').textContent,/额定 11 \/ 5/);
  assert.doesNotMatch(p.el('twin-speed-help').textContent,/20%/);
  assert.match(p.el('twin-braking-hint').textContent,/0.03 秒/);
});
test('blank, non-finite and excessive numeric speeds never publish a jog',async()=>{
  for(const value of ['', 'NaN', 'Infinity','0','-1','0.901']) {
    const p=page();p.feedback({control_settings:{mode:'full',min_speed_rad_s:.005,max_speed_rad_s:.9}});
    p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
    await p.el('twin-acquire').click();p.feedback({owned:true});
    await p.tabs[1].click();p.el('twin-joint-speed').value=value;
    for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
    p.frame();assert.ok(!p.commands.some(c=>c.op==='jog'));
    assert.match(p.el('twin-message').textContent,/0.005–0.9/);
  }
});
test('blur stops immediately and preserves a reason across cleanup and reload',async()=>{
  const p=page();p.feedback();
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true});
  p.windowEvents.get('blur')();
  const notice=p.commands.find(c=>c.source===undefined && c.reason?.includes('页面失去焦点') && !c.op);
  assert.ok(notice?.stamp_ms && notice.session==='lease');
  assert.match(p.commands.find(c=>c.op==='release').reason,/页面失去焦点/);
  assert.ok(p.calls.some(c=>c.name.endsWith('/stop')));
  p.feedback({owned:false,control_prepared:false,cleanup_ok:true,hardware_phase:'READONLY'});
  assert.match(p.el('twin-stop-reason').textContent,/页面失去焦点/);
  const reloaded=page({storage:p.storage});
  assert.match(reloaded.el('twin-stop-reason').textContent,/页面失去焦点/);
  assert.equal(reloaded.commands.length,0);
});
test('full feedback timeout and blur suspend without issuing disable; explicit stop still disables',async()=>{
  for(const cause of ['timeout','blur']){
    const p=page();p.feedback({control_settings:{mode:'full',max_speed_rad_s:1,continuous:true}});
    p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
    await p.el('twin-acquire').click();p.feedback({owned:true,hardware_phase:'ACTIVE'});
    if(cause==='timeout')p.connectionTick(501);else p.windowEvents.get('blur')();
    assert.ok(p.commands.some(c=>c.op==='suspend'));
    assert.ok(p.calls.some(c=>c.name.endsWith('/suspend')));
    assert.ok(!p.commands.some(c=>c.op==='release') && !p.calls.some(c=>c.name.endsWith('/stop')));
    assert.equal(p.window.ReBotTwin.owned(),false);
    await p.el('twin-stop').click();assert.ok(p.calls.some(c=>c.name.endsWith('/stop')));
  }
});
test('J7 uses its own bounded speed in the same jog stream, independently of slow arm speed',async()=>{
  const p=page();p.feedback({control_settings:{mode:'full',min_speed_rad_s:.005,max_speed_rad_s:.37,continuous:true}});
  p.el('twin-joint-speed').value='.37';await p.el('twin-joint-speed').dispatchEvent({type:'input'});
  p.el('twin-linear-speed').value='40';await p.el('twin-linear-speed').dispatchEvent({type:'input'});
  p.el('twin-angular-speed').value='20';await p.el('twin-angular-speed').dispatchEvent({type:'input'});
  assert.equal(Number(p.el('twin-joint-speed').value),.37);
  assert.equal(Number(p.el('twin-linear-speed').value),40);
  assert.equal(Number(p.el('twin-angular-speed').value),20);
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true});await p.tabs[1].click();
  for(const code of ['Space','KeyQ'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  p.frame();const jog=p.commands.find(c=>c.op==='jog');
  assert.equal(jog.gripper_velocity,.5);assert.deepEqual(jog.twist,[0,0,0,0,0,0]);
  p.el('twin-joint-speed').value='.01';p.el('twin-gripper-speed').value='.7';
  p.inputTick();assert.equal(p.commands.at(-1).gripper_velocity,.7);
  assert.equal(p.commands.at(-1).speed,.01);
  p.feedback({extensions:{gripper:{available:true,fresh:true,speed_max:.2}}});
  p.inputTick();assert.equal(p.commands.at(-1).gripper_velocity,.2);
  p.windowEvents.get('keyup')({code:'Space'});
  assert.equal(p.commands.at(-1).op,'hold');
});

test('a typed feedback hold automatically reacquires manual readiness without enable or replay',async()=>{
  const p=page();p.feedback({control_settings:{mode:'full',max_speed_rad_s:1,continuous:true}});
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true,hardware_phase:'ACTIVE',mode:'HOLD'});
  await p.tabs[1].click();
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  p.inputTick();assert.equal(p.commands.at(-1).op,'jog');
  p.connectionTick(250); // Initial acquire race window has elapsed.
  const hold={since:123,automatic_manual_recovery:true,feedback_confirmed:true,reason:'J1 电流反馈过期'};
  p.feedback({owned:false,hardware_phase:'HOLDING',mode:'HOLD',communication_hold:hold});
  const priorJog=p.commands.filter(c=>c.op==='jog').length;
  for(let i=0;i<12;i++){p.feedback();p.connectionTick();await Promise.resolve();}
  assert.equal(p.calls.filter(c=>c.name.endsWith('/acquire')).length,2);
  assert.ok(p.calls.every(c=>c.name.endsWith('/acquire')));
  assert.equal(p.commands.filter(c=>c.op==='jog').length,priorJog);
  p.feedback({owned:true,hardware_phase:'ACTIVE',communication_hold:null});
  await new Promise(resolve=>setTimeout(resolve,60));
  p.inputTick();p.inputTick();
  assert.equal(p.commands.filter(c=>c.op==='jog').length,priorJog);
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,repeat:true,preventDefault(){}});
  p.inputTick();assert.equal(p.commands.filter(c=>c.op==='jog').length,priorJog);
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,repeat:false,preventDefault(){}});
  p.inputTick();assert.equal(p.commands.filter(c=>c.op==='jog').length,priorJog+1);
});

test('program activity disarms manual automatic recovery',async()=>{
  const p=page();p.feedback({control_settings:{mode:'full',max_speed_rad_s:1,continuous:true}});
  p.el('twin-onsite').checked=p.el('ros-control-enable').checked=true;
  await p.el('twin-acquire').click();p.feedback({owned:true,hardware_phase:'ACTIVE',mode:'HOLD'});
  await p.tabs[1].click();
  for(const code of ['Space','KeyR'])p.windowEvents.get('keydown')({code,preventDefault(){}});
  p.inputTick();p.connectionTick(250);
  const action=p.window.ReBotTwin.request('program_pause');
  p.event({id:p.commands.at(-1).id,ok:true});await action;
  p.feedback({owned:false,hardware_phase:'HOLDING',communication_hold:{since:123,automatic_manual_recovery:true,feedback_confirmed:true}});
  for(let i=0;i<14;i++){p.feedback();p.connectionTick();}
  assert.equal(p.calls.filter(c=>c.name.endsWith('/acquire')).length,1);
});
test('trajectory tuning preserves drafts, waits for ack and never prepares hardware',async()=>{
  const p=page(), motion={selected:'ruckig',available:['ruckig','quintic'],profiles:{ruckig:{acceleration_scale:1,jerk_scale:1},quintic:{acceleration_scale:.5,jerk_scale:.4}},effective_acceleration:4,effective_jerk:30};
  p.feedback({control_prepared:false,motion_profile:motion,control_settings:{mode:'full',max_speed_rad_s:1}});
  p.el('motion-acceleration').value='35';p.el('motion-jerk').value='25';p.feedback();
  assert.equal(p.el('motion-acceleration').value,'35');
  const applying=p.el('motion-apply').click(), c=p.commands.at(-1);
  assert.deepEqual(c.parameters,{acceleration_scale:.35,jerk_scale:.25});
  assert.equal(p.el('motion-apply').disabled,true);assert.equal(p.calls.length,0);
  p.event({id:c.id,ok:true,motion_profile:'ruckig',motion_settings:{...motion,profiles:{...motion.profiles,ruckig:c.parameters},effective_acceleration:1.4,effective_jerk:7.5}});await applying;
  assert.match(p.el('motion-effective').textContent,/1.400.*7.500/);
  p.el('motion-acceleration').value='101';await p.el('motion-apply').click();assert.equal(p.commands.length,1);
  p.feedback({control_prepared:true});assert.equal(p.el('motion-apply').disabled,true);
});

test('one manual action requires fresh feedback and onsite confirmation before any enable',async()=>{
  const p=page();await p.el('twin-start-manual').click();
  assert.equal(p.calls.length,0);assert.equal(p.commands.length,0);
  p.feedback({control_settings:{mode:'full',max_speed_rad_s:1}});
  await p.el('twin-start-manual').click();
  assert.equal(p.calls.length,0);assert.equal(p.commands.length,0);
  assert.match(p.el('twin-message').textContent,/现场确认/);
  p.el('twin-onsite').checked=true;p.connectionTick(550);
  await p.el('twin-start-manual').click();
  assert.equal(p.calls.length,0);assert.ok(!p.commands.some(c=>c.op==='power_hold'));
});
test('manual enable prepares and acquires once, only power_hold can follow the explicit click',async()=>{
  const p=page();p.feedback({control_prepared:false,control_settings:{mode:'full',max_speed_rad_s:1}});
  assert.equal(p.calls.length,0);assert.equal(p.commands.length,0);
  p.el('twin-onsite').checked=true;
  const starting=p.el('twin-start-manual').click();
  await p.el('twin-start-manual').click();
  await assert.rejects(p.window.ReBotTwin.prepareProgram(),/已有控制准备/);
  for(let i=0;i<8&&!p.commands.some(c=>c.op==='power_hold');i++)await Promise.resolve();
  const command=p.commands.find(c=>c.op==='power_hold');assert.ok(command);
  assert.deepEqual(p.calls.map(c=>c.name),['/rebotarm/enable','/rebotarm/twin/acquire']);
  p.event({id:command.id,ok:true,message:'使能保持已请求'});await starting;
  assert.equal(p.commands.filter(c=>c.op==='power_hold').length,1);
  assert.ok(!p.commands.some(c=>['joint_goal','pose_goal','jog','program_start'].includes(c.op)));
});
test('pause during manual preparation cancels the late response before acquisition or power_hold',async()=>{
  const p=page();p.feedback({control_prepared:false,control_settings:{mode:'full',max_speed_rad_s:1}});
  p.el('twin-onsite').checked=true;
  const original=p.client.callService;let resolveEnable;
  p.client.callService=async(...args)=>{
    const result=await original(...args);
    if(args[0]==='/rebotarm/enable')await new Promise(resolve=>{resolveEnable=resolve;});
    return result;
  };
  const starting=p.el('twin-start-manual').click();
  for(let i=0;i<8&&!resolveEnable;i++)await Promise.resolve();
  assert.ok(resolveEnable);await p.el('twin-suspend').click();
  resolveEnable();await starting;
  assert.ok(!p.calls.some(c=>c.name.endsWith('/acquire')));
  assert.ok(!p.commands.some(c=>c.op==='power_hold'));
});
test('disconnect button is unavailable while real motors are active or holding',()=>{
  const p=page();p.feedback({hardware_phase:'ACTIVE'});
  assert.equal(p.el('twin-connect').disabled,true);
  p.feedback({hardware_phase:'HOLDING'});assert.equal(p.el('twin-connect').disabled,true);
  p.feedback({hardware_phase:'READONLY'});assert.equal(p.el('twin-connect').disabled,false);
});
