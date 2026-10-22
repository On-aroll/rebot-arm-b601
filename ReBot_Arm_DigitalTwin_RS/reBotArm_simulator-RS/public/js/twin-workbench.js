(function () {
  'use strict';
  const $ = id => document.getElementById(id), twin = window.ReBotTwin;
  if (!twin || !$('workbench-tabs')) return;
  let config = {profiles:[]}, contracts, library = [], plan = null, proposal = null, observation = null;
  let lastState = null, recording = null, ghostEpoch = 0, requestEpoch = 0, drag = null, dragFlight = null, simTrial = null, watchedRun = '';
  const history = [];
  let writingSegments = [], programBusy = false, submittedRun = '';
  const writingColors = ['#ffc857','#50e3c2','#70b7ff','#ff8fbd'];
  function writingPath(program) {
    writingSegments=[];let previous=null;
    program.steps.forEach((s,index)=>{
      const match=/^([国庆快乐])·\d+·书写$/.exec(s.label||'');
      if(match&&s.type==='pose'&&previous&&finiteVector(s.pose,6)&&finiteVector(previous,6)){
        writingSegments.push({a:previous.slice(0,3),b:s.pose.slice(0,3),index,color:writingColors['国庆快乐'.indexOf(match[1])]});
      }
      previous=s.type==='pose'?s.pose:null;
    });
    $('wb-writing-preview').hidden=!writingSegments.length;
    $('wb-writing-svg').replaceChildren();
    if(writingSegments.length){
      const points=writingSegments.flatMap(s=>[s.a,s.b]),xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);
      const minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys);
      const scale=Math.min(880/Math.max(.001,maxX-minX),160/Math.max(.001,maxY-minY));
      const x=p=>480+(p[0]-(minX+maxX)/2)*scale,y=p=>120-(p[1]-(minY+maxY)/2)*scale;
      writingSegments.forEach(s=>{
        const line=document.createElementNS('http://www.w3.org/2000/svg','line');
        for(const [key,value] of Object.entries({x1:x(s.a),y1:y(s.a),x2:x(s.b),y2:y(s.b),stroke:s.color,'stroke-width':6,'stroke-linecap':'round'}))line.setAttribute(key,String(value));
        $('wb-writing-svg').append(line);s.element=line;
      });
    }
    window.reBotSim?.setWritingPath(writingSegments);
  }
  function writingProgress(index) {
    for(const s of writingSegments)s.element.setAttribute('stroke',s.index<index?s.color:s.index===index?'#ffffff':'#3b4960');
  }
  const status = (id,text,error=false) => { $(id).textContent=text; $(id).dataset.error=String(error); };
  const json = v => JSON.stringify(v,null,2);
  const angleMap = q => Object.fromEntries(q.map((v,i)=>[`joint${i+1}`,v]));
  const finiteVector = (v,n) => Array.isArray(v) && v.length===n && v.every(x=>typeof x==='number'&&Number.isFinite(x));
  function tab(name,scroll=false) {
    document.querySelectorAll('[data-workbench-panel]').forEach(e=>{e.hidden=e.dataset.workbenchPanel!==name;});
    document.querySelectorAll('[data-workbench-tab]').forEach(e=>{const active=e.dataset.workbenchTab===name;e.classList.toggle('is-active',active);e.setAttribute('aria-selected',String(active));e.setAttribute('aria-pressed',String(active));});
    window.dispatchEvent(new CustomEvent('rebot-workbench-changed',{detail:name}));
    if (scroll) ($('hub-title') || $('workbench-tabs')).scrollIntoView({behavior:'smooth',block:'start'});
  }
  document.querySelectorAll('[data-workbench-tab]').forEach(e=>e.addEventListener('click',()=>tab(e.dataset.workbenchTab)));
  document.querySelectorAll('[data-open-workbench]').forEach(e=>e.addEventListener('click',()=>tab(e.dataset.openWorkbench,true)));
  async function api(route,data) {
    const response = await fetch(`/api/workbench/${route}`,{method:data===undefined?'GET':'POST',headers:data===undefined?{}:{'Content-Type':'application/json'},body:data===undefined?undefined:JSON.stringify(data),signal:AbortSignal.timeout(35000)});
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error||'工作区请求失败');
    return result;
  }
  function action(id,out,fn) {
    $(id).addEventListener('click',async()=>{
      const button=$(id);if(button.disabled)return;button.disabled=true;
      try { await fn(); } catch(e) { status(out,e.message,true); }
      finally { button.disabled=id==='wb-program-run'&&(programBusy||Boolean(submittedRun)); }
    });
  }
  function download(value,name) {
    const url=URL.createObjectURL(new Blob([json(value)],{type:'application/json'})),a=document.createElement('a');
    a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
  }
  async function readFile(input) {
    const file=input.files?.[0];if(!file)throw new Error('未选择文件');
    if(file.size>2*1024*1024)throw new Error('JSON文件需小于2MB');
    return JSON.parse(await file.text());
  }
  function sample(steps) {return {schema_version:'1.0',robot_model:'b601_rs',base_frame:'base_link',calibration_id:lastState?.extensions?.calibration_id||'rs-identity-v1',units:{position:'m',angle:'rad',time:'s'},name:$('wb-program-name').value||'我的动作程序',steps};}
  function convertTrajectory(raw) {
    const t=raw.trajectory||raw;
    if(!Array.isArray(t.joint_names)||!Array.isArray(t.points))return raw;
    const indices=Array.from({length:6},(_,i)=>t.joint_names.indexOf(`joint${i+1}`));
    if(t.joint_names.length!==6||indices.some(i=>i<0))throw new Error('JointTrajectory需要joint1–joint6，恰好六轴；J7请用gripper步骤');
    let previous=0;
    const steps=t.points.map((point,i)=>{
      if(!finiteVector(point.positions,6))throw new Error(`轨迹第${i+1}点positions无效`);
      const stamp=point.time_from_start;
      const time=typeof stamp==='number'?stamp:Number(stamp?.sec)+Number(stamp?.nanosec||0)/1e9;
      if(!Number.isFinite(time)||time<0||i>0&&time<=previous)throw new Error('time_from_start必须递增，单位秒');
      const duration=Math.max(.05,time-previous);previous=time;
      return {type:'joint',joints:indices.map(j=>point.positions[j]),duration,speed:twin.jointSpeed(),label:`ROS关键帧${i+1}`};
    });
    return sample(steps);
  }
  function loadProgram(raw,message='程序已载入；点击「使能并执行程序」即可运行，检查自动完成。') {
    const program=convertTrajectory(raw.program||raw);
    if(program.schema_version!=='1.0'||!Array.isArray(program.steps)||!program.steps.length||program.steps.length>250)throw new Error('需要1–250步的1.0版动作程序');
    if(programBusy){requestEpoch++;if(twin.owned())void twin.stop('程序已更换，取消上次执行准备');}
    plan=null;watchedRun='';ghostEpoch++;$('wb-program-json').value=json(program);$('wb-program-name').value=program.name||'导入程序';
    status('wb-program-status',message);$('wb-program-state').textContent=`${program.steps.length}步 · 已载入`;
    $('wb-program-steps').replaceChildren();writingPath(program);return program;
  }
  function editorProgram() {const p=convertTrajectory(JSON.parse($('wb-program-json').value));p.name=$('wb-program-name').value;return p;}
  $('wb-program-json').addEventListener('input',()=>{if(programBusy){requestEpoch++;if(twin.owned())void twin.stop('程序已修改，取消执行准备');}plan=null;ghostEpoch++;writingPath({steps:[]});status('wb-program-status','程序已修改，下次执行时自动检查');});
  $('wb-program-import').addEventListener('change',async e=>{try{loadProgram(await readFile(e.target));}catch(err){status('wb-program-status',err.message,true);}finally{e.target.value='';}});
  action('wb-program-example','wb-program-status',()=>loadProgram(contracts?.program||sample([{type:'wait',seconds:1,label:'等待一秒（不移动）'}])));
  action('wb-program-writing','wb-program-status',async()=>{
    const response=await fetch('/programs/guoqing_kuaile.json',{cache:'no-store'});
    if(!response.ok)throw new Error('书写示例未生成，请运行 scripts/generate_guoqing_program.py');
    loadProgram(await response.json(),'已载入26笔「国庆快乐」；点击「使能并执行程序」开始空中书写。');
  });
  action('wb-writing-view','wb-program-status',()=>{$('hub-monitor').open=true;window.reBotSim?.focusWritingPath();$('viewport').scrollIntoView({behavior:'smooth',block:'center'});});
  action('wb-program-export','wb-program-status',()=>download(editorProgram(),'b601-motion-program.json'));
  function pickValue(id){
    const text=$(id).value.trim();
    if(!text)throw new Error('请填写目标坐标、姿态及空夹/电流标定值');
    const value=Number(text);if(!Number.isFinite(value))throw new Error('夹取参数需为有限数值');return value;
  }
  action('wb-pick-capture','wb-pick-status',()=>{
    if(!twin.fresh()||!finiteVector(lastState?.pose,6))throw new Error('需要新鲜末端反馈');
    ['x','y','z','roll','pitch','yaw'].forEach((key,i)=>{$(`wb-pick-${key}`).value=(lastState.pose[i]*(i<3?1000:180/Math.PI)).toFixed(3);});
    status('wb-pick-status','已填写当前模型末端位置；请将目标改为实际夹取位置。');
  });
  action('wb-grasp-capture-empty','wb-pick-status',()=>{
    const g=lastState?.extensions?.gripper;
    if(!twin.fresh()||!g?.fresh||!Number.isFinite(g.position)||!Number.isFinite(g.velocity)||Math.abs(g.velocity)>.01)throw new Error('需要新鲜且静止的J7反馈');
    if(!$('wb-grasp-empty-confirm').checked)throw new Error('请先确认当前空夹且已闭合');
    $('wb-grasp-empty_position').value=g.position.toFixed(4);$('wb-grasp-empty-confirm').checked=false;
    status('wb-pick-status',`已记录${twin.hardware?'实机':'Fake'}空夹位置 ${g.position.toFixed(4)} rad；接触/中止电流仍须用实测数据填写。`);
  });
  action('wb-pick-build','wb-pick-status',()=>{
    if(['running','paused'].includes(lastState?.extensions?.program?.status))throw new Error('请先取消现有程序再生成新任务');
    const keys=['empty_position','empty_tolerance','min_gap','contact_current_a','abort_current_a','stall_velocity','min_travel','dwell','hold_time','timeout'];
    const profile=Object.fromEntries(keys.map(key=>[key,pickValue(`wb-grasp-${key}`)]));
    const pose=['x','y','z','roll','pitch','yaw'].map((key,i)=>pickValue(`wb-pick-${key}`)*(i<3?.001:Math.PI/180));
    const program=window.ReBotFixedPick.build({pose,profile,approach:pickValue('wb-pick-approach')/1000,
      openPosition:pickValue('wb-pick-open'),graspSpeed:pickValue('wb-pick-speed'),jointSpeed:twin.jointSpeed(),
      calibrationId:lastState?.extensions?.calibration_id||'rs-identity-v1'});
    loadProgram(program,'已生成指定点夹取程序；执行前检查路径、TCP与现场空间。夹持判定后原位保持。');
    status('wb-pick-status',`已生成 ${program.steps.length} 步，未发送运动；使用下方检查/执行按钮。`);
  });
  async function programs() {library=(await api('programs')).programs;$('wb-program-library').replaceChildren(new Option('选择本机程序',''),...library.map(p=>new Option(`${p.program.name||'程序'} · ${new Date(p.saved_at).toLocaleString()}`,p.id)));}
  action('wb-program-save','wb-program-status',async()=>{await api('programs',editorProgram());await programs();status('wb-program-status','已保存到本机程序库');});
  action('wb-program-load','wb-program-status',()=>{const p=library.find(p=>p.id===$('wb-program-library').value);if(!p)throw new Error('请选择程序');loadProgram(p.program);});
  function showPlan(p) {
    writingPath(p.program);
    plan=p;watchedRun='';ghostEpoch++;$('wb-program-steps').replaceChildren(...p.steps.map((s,i)=>{
      const li=document.createElement('li');li.textContent=`${i+1} · ${s.label} / ${s.type} · ${s.duration.toFixed(2)}秒${s.retimed?'（已按限速延长）':''}`;return li;
    }));
    $('wb-program-state').textContent=`${p.steps.length}步 · 预计${p.duration.toFixed(1)}秒`;
    status('wb-program-status','整套路径检查通过；预览60秒有效，起点变化须重算。');
  }
  action('wb-program-preview','wb-program-status',async()=>{status('wb-program-status','正在检查每一步逆解、路径和时长…');showPlan((await twin.request('program_preview',{program:editorProgram()})).program_plan);});
  function playGhost(p) {
    const epoch=++ghostEpoch,started=performance.now();let offset=0;
    const timeline=p.steps.map(s=>{const row={...s,offset};offset+=s.duration;return row;});
    const frame=now=>{
      if(epoch!==ghostEpoch)return;
      const elapsed=(now-started)/1000;
      const step=timeline.find(s=>elapsed<s.offset+s.duration);
      if(!step){writingProgress(p.steps.length);window.reBotSim?.setGhostVisible(false);return;}
      writingProgress(step.index);
      if(step.joints){const u=Math.min(1,Math.max(0,(elapsed-step.offset)/step.duration)),b=u*u*u*(10+u*(-15+6*u));window.reBotSim?.setFeedbackTarget(angleMap(step.start.map((q,i)=>q+(step.joints[i]-q)*b)));}
      requestAnimationFrame(frame);
    };requestAnimationFrame(frame);status('wb-program-status','正在播放目标虚影；实色模型保持真实反馈，本操作不发送运动。');
  }
  action('wb-program-visual','wb-program-status',()=>{if(!plan)throw new Error('先检查整套路径');$('hub-monitor').open=true;playGhost(plan);$('viewport').scrollIntoView({behavior:'smooth',block:'center'});});
  async function executeProgram() {
    if(programBusy||submittedRun||['running','paused'].includes(twin.state()?.extensions?.program?.status))throw new Error('已有程序正在执行或暂停，请先取消');
    if(simTrial)throw new Error('请先结束当前仿真试运行');
    const program=editorProgram(),epoch=++requestEpoch;
    const check=()=>{if(epoch!==requestEpoch||document.hidden)throw new Error('执行准备已取消');};
    programBusy=true;
    try {
      status('wb-program-status','正在自动准备控制…');
      await twin.prepareProgram(check);check();
      status('wb-program-status','正在自动检查程序…');
      const p=(await twin.request('program_preview',{program})).program_plan;check();
      showPlan(p);watchedRun=p.id;
      submittedRun=p.id;await twin.request('program_start',{plan_id:p.id});check();plan=null;ghostEpoch++;
      status('wb-program-status',`已提交${twin.hardware?'实机':'仿真'}执行，电机自动启动，进度由控制器反馈。`);
    } catch(error) {
      submittedRun='';
      if(epoch===requestEpoch&&twin.owned())await twin.stop('程序执行准备失败，已停止并释放控制');
      throw error;
    } finally {programBusy=false;}
  }
  action('wb-program-run','wb-program-status',executeProgram);
  action('wb-program-pause','wb-program-status',async()=>{if(simTrial){await simTrial.command('program_pause');return;}await twin.request('program_pause');});
  action('wb-program-resume','wb-program-status',async()=>{status('wb-program-status','正在重新检查剩余路径…');if(simTrial){await simTrial.command('program_resume');return;}await twin.request('program_resume');});
  action('wb-program-cancel','wb-program-status',async()=>{requestEpoch++;ghostEpoch++;plan=null;submittedRun='';if(simTrial)await simTrial.stop();else if(programBusy)await twin.stop('已取消程序执行准备');else await twin.request('program_cancel');status('wb-program-status','程序已取消');});

  // A separate rosbridge client is hard-wired to the Fake namespace. It cannot
  // address /rebotarm, acquire a real lease, or overwrite the solid model.
  async function simulationTrial(program) {
    if(twin.owned())throw new Error('请先停止并释放当前控制，再试运行仿真');
    if(simTrial)throw new Error('已有仿真试运行');
    const epoch=++requestEpoch;
    status('wb-program-status','正在连接安全仿真 /rebotarm_rs…');
    const started=await fetch('/api/quick-start',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
    const launch=await started.json();if(!started.ok||launch.ok===false)throw new Error(launch.error||launch.message||'仿真启动失败');
    if(epoch!==requestEpoch)return;
    const c=new window.ReBotRosClient({url:window.reBotRos.url,namespace:'rebotarm_rs'});c.autoReconnect=false;
    const base='/rebotarm_rs/twin';let session='',seq=0,state=null,received=0,closed=false,beats=0,runId='',pending=new Map();
    let resolveReady;const ready=new Promise(resolve=>{resolveReady=resolve;});
    const send=(op,args={})=>{if(!c.connected)throw new Error('仿真连接已断开');c.advertise(base+'/command','std_msgs/msg/String');c.publish(base+'/command',{data:JSON.stringify({op,session,seq:++seq,stamp_ms:Date.now(),...args})});};
    const request=(op,args={})=>new Promise((resolve,reject)=>{const id=`sim-${++seq}`,timer=setTimeout(()=>{pending.delete(id);reject(new Error('仿真控制器响应超时'));},35000);pending.set(id,{resolve,reject,timer,op});try{send(op,{...args,id});}catch(e){clearTimeout(timer);pending.delete(id);reject(e);}});
    const service=name=>c.callService(base+'/'+name,'std_srvs/srv/Trigger',{}, {timeoutMs:5000,retryOnTimeout:false});
    const stop=async()=>{
      if(closed)return;closed=true;clearInterval(beats);requestEpoch++;
      try{if(session&&c.connected)await service('stop');}catch(_){}finally{
        for(const p of pending.values()){clearTimeout(p.timer);p.reject(new Error('仿真试运行已结束'));}pending.clear();c.disconnect();simTrial=null;window.reBotSim?.setGhostVisible(false);
      }
    };
    simTrial={stop,command:request};
    c.subscribe(base+'/events','std_msgs/msg/String',msg=>{const r=JSON.parse(msg.data),p=pending.get(r.id);if(p){clearTimeout(p.timer);pending.delete(r.id);r.ok?p.resolve(r):p.reject(new Error(r.message));}},{throttleRate:1});
    c.subscribe(base+'/state','std_msgs/msg/String',msg=>{
      state=JSON.parse(msg.data);received=performance.now();resolveReady();
      if(session&&state.actual){const angles=angleMap(state.actual);if(state.extensions?.gripper?.fresh)angles.gripper=state.extensions.gripper.position/5*.0715;window.reBotSim?.setFeedbackTarget(angles);}
      const run=state.extensions?.program;if(session&&run?.status&&run.id===runId){showRun(run,'仿真');if(['completed','failed','cancelled'].includes(run.status))void stop();}
    },{throttleRate:50});
    c.addEventListener('status',event=>{if(['closed','error'].includes(event.detail.state)&&!closed){status('wb-program-status','仿真连接中断，试运行已取消',true);void stop();}});
    c.connect();
    try {
      await Promise.race([ready,new Promise((_,reject)=>setTimeout(()=>reject(new Error('等待仿真反馈超时')),12000))]);
      if(closed||epoch!==requestEpoch)throw new Error('仿真启动已取消');
      if(state.owned)throw new Error('安全仿真被其它页面占用');
      const compiled=(await request('program_preview',{program})).program_plan;
      runId=compiled.id;
      const lease=await service('acquire');if(!lease.success)throw new Error(lease.message);session=JSON.parse(lease.message).session;
      beats=setInterval(()=>{
        if(closed)return;
        if(document.hidden||performance.now()-received>500||state.feedback_valid===false){status('wb-program-status','仿真反馈或页面失活，已停止',true);void stop();return;}
        try{if(!pending.size||[...pending.values()].every(p=>p.op.endsWith('_preview')||p.op==='program_resume'))send('heartbeat');}catch(_){void stop();}
      },150);
      if(compiled.steps.some(s=>s.type==='gripper'))await request('gripper_authorize',{enabled:true});
      await request('program_start',{plan_id:compiled.id});
      status('wb-program-status','安全仿真试运行中；目标虚影显示仿真反馈，实色保留当前连接反馈。');
    }catch(e){await stop();throw e;}
  }
  action('wb-program-sim','wb-program-status',()=>simulationTrial(editorProgram()));
  function showRun(run,prefix='') {
    if(run.id===submittedRun&&['completed','failed','cancelled'].includes(run.status))submittedRun='';
    writingProgress(run.index||0);
    const labels={running:'执行中',paused:'已暂停',completed:'已完成',cancelled:'已取消',failed:'失败'};
    $('wb-program-progress').max=run.count||1;$('wb-program-progress').value=run.index||0;
    $('wb-program-state').textContent=`${prefix}${labels[run.status]||run.status} · ${run.index||0}/${run.count||0}`;
    status('wb-program-status',run.message||'',run.status==='failed');
  }

  function recordKeyframe(force=false) {
    if(!recording)return;
    if(!twin.fresh())return;
    const s=twin.state(),q=s.actual,g=s.extensions?.gripper;
    if(!finiteVector(q,6))return;
    const at=performance.now(),previous=recording.frames.at(-1);
    if(!force&&previous&&(at-previous.at<200||Math.max(...q.map((v,i)=>Math.abs(v-previous.q[i])))<.02&&Math.abs((g?.position||0)-(previous.gripper||0))<.04))return;
    if(recording.frames.length>=120){status('wb-teach-status','已达120个反馈关键帧，请结束生成程序');return;}
    recording.frames.push({q:[...q],at,gripper:g?.fresh?g.position:null});
    status('wb-teach-status',`录制中 · ${recording.frames.length}个反馈关键帧 · ${((at-recording.at)/1000).toFixed(1)}秒`);
  }
  action('wb-teach-start','wb-teach-status',()=>{if(!twin.fresh())throw new Error('需要新鲜反馈才能录制');recording={at:performance.now(),frames:[]};recordKeyframe(true);status('wb-teach-status','正在录制实际反馈；可切换手动控制操作。');});
  action('wb-teach-keyframe','wb-teach-status',()=>{if(!recording)throw new Error('请先开始录制');if(!twin.fresh())throw new Error('反馈已过期');recordKeyframe(true);});
  action('wb-teach-stop','wb-teach-status',()=>{
    if(!recording)throw new Error('尚未录制');recordKeyframe(true);const r=recording;recording=null;const steps=[];let previous=r.at,gprev=null;
    r.frames.forEach((f,i)=>{const step={type:'joint',joints:f.q,speed:twin.jointSpeed(),label:`示教关键帧${i+1}`};if($('wb-teach-preserve-time').checked)step.duration=Math.max(.05,(f.at-previous)/1000);steps.push(step);previous=f.at;if(f.gripper!==null&&(gprev===null||Math.abs(gprev-f.gripper)>.04||i===r.frames.length-1&&Math.abs(gprev-f.gripper)>.0015)){steps.push({type:'gripper',position:Math.max(0,Math.min(5,f.gripper)),speed:Math.min(twin.jointSpeed(),lastState?.extensions?.gripper?.speed_max||.1),label:`夹爪关键帧${i+1}`});gprev=f.gripper;}});
    loadProgram(sample(steps));status('wb-teach-status',`已生成${steps.length}步；回放按速度重新定时，请先检查整套路径。`);
  });

  action('wb-support-capture','wb-support-status',async()=>{
    const r=await twin.request('support_capture',{name:$('wb-support-name').value,support_note:$('wb-support-note').value,verified:$('wb-support-verified').checked});
    $('wb-support-verified').checked=false;status('wb-support-status',r.message);
  });
  async function supportPreview(){const r=await twin.request('support_preview',{speed:twin.jointSpeed()});showPlan(r.program_plan);window.reBotSim?.setFeedbackTarget(angleMap(r.program_plan.steps[0].joints));const tolerance=r.program_plan.support.arrival_tolerance_deg;status('wb-support-status',`已选择：${r.program_plan.support.name}；预计${r.program_plan.duration.toFixed(1)}秒，保留J1、J7不动。${tolerance?`J2–J6 每轴到位偏差 ≤ ${tolerance}°。`:''}`);return r.program_plan;}
  action('wb-support-preview','wb-support-status',supportPreview);
  action('wb-support-return','wb-support-status',async()=>{
    if(!twin.owned()||!twin.fresh())throw new Error('请先准备并接管控制');
    const p=await supportPreview();watchedRun=p.id;await twin.request('program_start',{plan_id:p.id,release_after:$('wb-support-release').checked});plan=null;
    status('wb-support-status',`正在返回「${p.support.name}」；${$('wb-support-release').checked?'到位后按本次选择撤力':'到位后继续保持'}`);
  });
  document.querySelectorAll('[data-twin-mode]').forEach(e=>e.addEventListener('click',()=>{
    const enabled=e.dataset.twinMode==='tcp';window.reBotSim?.setTwinDrag(enabled);drag=null;
    if(enabled)$('hub-monitor').open=true;
    status('wb-drag-status',enabled?'拖动绿点选目标，松手后自动到点；远处较快、接近时减速。移动平面与当前视角平行。':'拖拽未启用');
  }));
  async function finishTcpDrag(target) {
    const epoch=requestEpoch;
    const check=()=>{if(epoch!==requestEpoch || document.hidden || document.body.dataset.hubView!=='tcp')throw new Error('TCP目标已取消');};
    try {
      status('wb-drag-status','正在检查拖动目标与路径…');
      const result=await twin.drag(target.pose,{orientation:target.orientation,check});check();
      dragFlight={seen:false};
      status('wb-drag-status',`自动到点中 · 预计 ${result.plan.duration.toFixed(1)} 秒 · 接近目标减速，停稳后保持`);
    } catch(error) { if(epoch===requestEpoch)status('wb-drag-status',error.message,true); }
  }
  window.addEventListener('rebot-twin-drag',e=>{
    if(e.detail.phase==='cancel'){drag=null;status('wb-drag-status','已取消拖动目标');return;}
    if(e.detail.phase==='end'){
      const target=drag;drag=null;if(!target)return;
      if(Math.hypot(...target.pose.slice(0,3).map((v,i)=>v-target.origin[i]))<.0005){status('wb-drag-status','目标未移动，请拖动绿点后松手');return;}
      void finishTcpDrag(target);return;
    }
    if(!twin.owned()||!twin.fresh()){status('wb-drag-status','先准备并接管控制；当前只显示反馈',true);return;}
    const p=e.detail.position;if(!p||![p.x,p.y,p.z].every(Number.isFinite))return;
    if(e.detail.phase==='start'){
      requestEpoch++;dragFlight=null;twin.hold('正在选择新的TCP目标，保持当前位置');
      drag={pose:[p.x,p.y,p.z,...twin.state().pose.slice(3)],origin:[p.x,p.y,p.z],orientation:$('twin-drag-orientation').checked};
    } else if(drag){drag.pose.splice(0,3,p.x,p.y,p.z);status('wb-drag-status',`目标 ${(p.x*1000).toFixed(1)} / ${(p.y*1000).toFixed(1)} / ${(p.z*1000).toFixed(1)} mm · 松手自动到点`);}
  });

  function fillProfiles() {
    for(const [id,kind,placeholder] of [['wb-connector-list',null,'新增连接'],['wb-vision-profile','vision','选择视觉服务'],['wb-planner-profile','planner','选择规划服务'],['wb-chat-profile','llm','选择大模型']]){
      const previous=$(id).value,rows=config.profiles.filter(p=>!kind||p.kind===kind);$(id).replaceChildren(new Option(placeholder,''),...rows.map(p=>new Option(`${p.name} · ${p.model||p.kind}`,p.id)));
      if(rows.some(p=>p.id===previous))$(id).value=previous;else if(kind&&rows.length===1)$(id).value=rows[0].id;
    }
    $('wb-calibration').value=json(config.calibration);
  }
  function profileForm(p={}) {
    for(const [id,key] of [['name','name'],['kind','kind'],['protocol','protocol'],['url','endpoint'],['health','health_url'],['model','model']])$(`wb-connector-${id}`).value=p[key]||(key==='kind'?'vision':key==='protocol'?'http-json':'');
    $('wb-connector-key').value='';$('wb-connector-clear-key').checked=false;
    status('wb-connector-status',p.has_key?'此配置已保存密钥；留空可保留同一地址的密钥。':'填写服务地址后保存。支持导入连接配置和下载接口示例。');
  }
  function formProfile(){return {schema_version:'1.0',id:$('wb-connector-list').value||undefined,name:$('wb-connector-name').value,kind:$('wb-connector-kind').value,protocol:$('wb-connector-protocol').value,endpoint:$('wb-connector-url').value.trim(),health_url:$('wb-connector-health').value.trim(),model:$('wb-connector-model').value.trim(),api_key:$('wb-connector-key').value,clear_key:$('wb-connector-clear-key').checked};}
  $('wb-connector-list').addEventListener('change',()=>profileForm(config.profiles.find(p=>p.id===$('wb-connector-list').value)));
  $('wb-connector-kind').addEventListener('change',()=>{if($('wb-connector-kind').value!=='llm')$('wb-connector-protocol').value='http-json';});
  action('wb-connector-save','wb-connector-status',async()=>{const raw=formProfile();config=await api('connector',raw);fillProfiles();const p=config.profiles.find(p=>p.id===raw.id)||config.profiles.at(-1);$('wb-connector-list').value=p.id;profileForm(p);status('wb-connector-status','连接已保存。可测试服务或发送一次实际输入。');});
  action('wb-connector-test','wb-connector-status',async()=>{const r=await api('test',{id:$('wb-connector-list').value});status('wb-connector-status',r.message);});
  $('wb-connector-import').addEventListener('change',async e=>{try{const p=await readFile(e.target);$('wb-connector-list').value='';profileForm(p);if(p.api_key)$('wb-connector-key').value=p.api_key;status('wb-connector-status','配置已填入表单，请核对地址后保存。');}catch(err){status('wb-connector-status',err.message,true);}finally{e.target.value='';}});
  action('wb-connector-export','wb-connector-status',()=>{const p=formProfile();delete p.api_key;delete p.clear_key;download(p,'b601-connector.json');status('wb-connector-status','已导出连接配置（密钥不导出）。');});
  action('wb-contract-download','wb-global-status',async()=>download(contracts||await api('contracts'),'b601-interface-contracts.json'));
  action('wb-calibration-save','wb-vision-status',async()=>{config=await api('calibration',JSON.parse($('wb-calibration').value));fillProfiles();observation=null;$('wb-objects').replaceChildren();status('wb-vision-status','相机标定已保存；请重新识别。');});
  function showObjects(result) {
    if(!result||result.schema_version!=='1.0'||!Number.isFinite(result.stamp_ms)||!Array.isArray(result.objects)||result.objects.length>200)throw new Error('识别结果需要1.0版本、stamp_ms和objects数组');
    observation=result;$('wb-objects').replaceChildren();
    for(const object of result.objects){
      if(typeof object.label!=='string'||!Number.isFinite(object.confidence)||object.confidence<0||object.confidence>1||object.position&&!finiteVector(object.position,3))throw new Error('识别结果类别/置信度/位置无效');
      const row=document.createElement('div');row.className='wb-object';const text=document.createElement('span');text.textContent=`${object.label} · ${(object.confidence*100).toFixed(1)}% · ${object.position?`${object.frame_id} [${object.position.map(v=>v.toFixed(3)).join(', ')}] m`:'仅二维结果，无可用三维坐标'}`;row.append(text);
      if(object.frame_id==='base_link'&&object.position){const b=document.createElement('button');b.type='button';b.textContent='填入坐标目标';b.addEventListener('click',()=>{if(Math.abs(Date.now()-result.stamp_ms)>5000){status('wb-vision-status','观测已过期，请重新识别',true);return;}['x','y','z'].forEach((name,i)=>{const input=$(`twin-${name}`);input.value=(object.position[i]*1000).toFixed(1);input.dispatchEvent(new Event('input',{bubbles:true}));});tab('control',true);document.querySelector('[data-twin-mode="point"]')?.click();twin.message('识别位置已填入；先计算预览再执行。');});row.append(b);}
      $('wb-objects').append(row);
    }
    status('wb-vision-status',`${result.objects.length}个对象 · ${new Date(result.stamp_ms).toLocaleTimeString()} · ${Math.abs(Date.now()-result.stamp_ms)>5000?'已过期，仅查看':'观测5秒内可用于规划'}`);
  }
  action('wb-vision-run','wb-vision-status',async()=>{
    if(!$('wb-vision-consent').checked)throw new Error('请确认允许向所选服务发送本帧图像');
    const frame=await window.ReBotTwinCamera.capture();status('wb-vision-status','识别请求中…');const r=await api('vision',{id:$('wb-vision-profile').value,frame,allow_image:true});showObjects(r.observation);
  });
  $('wb-vision-import').addEventListener('change',async e=>{try{const r=await readFile(e.target);showObjects(r.observation||r);}catch(err){observation=null;status('wb-vision-status',err.message,true);}finally{e.target.value='';}});
  function robotContext(){if(!twin.fresh())throw new Error('先连接新鲜机器人反馈，作为规划起点');const s=twin.state();return {actual:s.actual,pose:s.pose,stamp_ms:s.stamp_ms,calibration_id:s.extensions?.calibration_id||'rs-identity-v1',speed_max_rad_s:s.control_settings?.max_speed_rad_s||.3,requested_speed_rad_s:twin.jointSpeed()};}
  action('wb-vision-clear','wb-vision-status',()=>{observation=null;$('wb-objects').replaceChildren();status('wb-vision-status','当前观测已清除');});
  async function contextObservation(){if($('wb-vision-refresh').checked){if(!$('wb-vision-consent').checked)throw new Error('刷新识别需要先确认发送相机帧');const frame=await window.ReBotTwinCamera.capture(),r=await api('vision',{id:$('wb-vision-profile').value,allow_image:true,frame});showObjects(r.observation);}return freshObservation();}
  function freshObservation(){if(!observation)return null;if(Math.abs(Date.now()-observation.stamp_ms)>5000)throw new Error('已有观测已过期，请重新识别；或清除观测后只按机器人状态规划');return observation;}
  action('wb-plan-run','wb-plan-status',async()=>{const epoch=requestEpoch;status('wb-plan-status','等待规划服务…');const result=await api('plan',{id:$('wb-planner-profile').value,task:$('wb-planner-task').value,observation:await contextObservation(),robot:robotContext()});if(epoch!==requestEpoch)throw new Error('控制状态已改变，旧规划结果已作废');loadProgram(result.program,'规划已载入，尚未执行；请检查整套路径。');status('wb-plan-status',result.explanation||'动作提案已载入程序工作区');tab('programs',true);});
  function chatMessage(role,text){const p=document.createElement('p');p.className=`wb-${role}`;p.textContent=text;$('wb-chat-messages').append(p);p.scrollIntoView({block:'nearest'});}
  action('wb-chat-send','wb-chat-status',async()=>{
    const task=$('wb-chat-input').value.trim();if(!task)throw new Error('请描述任务');const epoch=requestEpoch;
    const payload={id:$('wb-chat-profile').value,task,observation:await contextObservation(),robot:robotContext(),history:history.slice(-8)};
    if($('wb-chat-image').checked){payload.frame=await window.ReBotTwinCamera.capture();payload.allow_image=true;}
    chatMessage('user',task);status('wb-chat-status','模型正在生成提案…');proposal=null;
    const result=await api('chat',payload);if(epoch!==requestEpoch)throw new Error('控制状态已改变，旧模型提案已作废');
    history.push({role:'user',content:task},{role:'assistant',content:result.explanation});if(history.length>16)history.splice(0,history.length-16);
    proposal=result.program;chatMessage('assistant',result.explanation);$('wb-chat-input').value='';
    status('wb-chat-status',proposal?`已生成${proposal.steps.length}步提案；点击载入后检查路径。`:'模型未提供动作程序；可继续补充信息。');
  });
  action('wb-chat-load','wb-chat-status',()=>{if(!proposal)throw new Error('本次还没有有效动作程序');loadProgram(proposal,'大模型动作提案已载入；请检查整套路径。');tab('programs',true);});

  let supportKey='';
  window.addEventListener('rebot-twin-state',event=>{
    lastState=event.detail;const ext=lastState.extensions||{},g=ext.gripper||{},run=ext.program;
    const grasp=g.grasp||{},reading=(v,d=3)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(d):'—';
    const graspLabels={closing:'闭合检测',contact:'接触确认中',holding:'电机保持判定通过',failed:'未通过',cancelled:'已中断'};
    status('wb-grasp-status',`J7 ${g.fresh?'反馈新鲜':'反馈未确认'} · ${reading(g.position)} rad · ${reading(g.velocity)} rad/s · ${reading(g.current_a)} A${grasp.status?` · ${graspLabels[grasp.status]||grasp.status}：${grasp.message}`:''}`,grasp.status==='failed');
    if(recording)recordKeyframe();
    if(run?.status&&!simTrial&&(run.id===watchedRun||['running','paused'].includes(run.status))){watchedRun=run.id;showRun(run);}
    const supports=ext.support_poses||[],key=supports.map(p=>p.id).join(',');
    if(key!==supportKey){supportKey=key;$('wb-support-list').replaceChildren(...(supports.length?supports.map(p=>new Option(p.name,p.id)):[new Option('尚未记录支撑姿态','')]));}
    $('wb-program-run').disabled=programBusy||Boolean(submittedRun)||!twin.fresh()||Boolean(simTrial)||['running','paused'].includes(run?.status);
    $('wb-program-pause').disabled=!simTrial&&run?.status!=='running';
    $('wb-program-resume').disabled=!simTrial&&run?.status!=='paused';
    $('wb-support-return').disabled=!twin.owned()||!twin.fresh()||!supports.length;
    // Acquiring/releasing stops local animations and may hide the marker.
    // Reconcile the visual mode, without acquiring or sending any target.
    window.reBotSim?.setTwinDrag(document.body.dataset.hubView === 'tcp');
    if(dragFlight){
      if(lastState.mode==='GOAL')dragFlight.seen=true;
      else if(dragFlight.seen || !twin.owned()){status('wb-drag-status',lastState.message||'到点运动已结束，请查看反馈');dragFlight=null;}
    }
    if(!twin.owned())drag=null;
  });
  function clearTransient(){requestEpoch++;ghostEpoch++;drag=null;dragFlight=null;plan=null;watchedRun='';submittedRun='';proposal=null;$('wb-support-release').checked=false;if(simTrial)void simTrial.stop();}
  for(const id of ['twin-stop','twin-reset-control','twin-release'])$(id)?.addEventListener('click',clearTransient);
  window.addEventListener('rebot-twin-suspended',clearTransient);
  window.addEventListener('rebot-motion-profile-changed',clearTransient);
  window.addEventListener('keydown',e=>{if(e.code==='Escape')clearTransient();});
  window.addEventListener('blur',()=>{drag=null;requestEpoch++;if(simTrial)void simTrial.stop();});
  window.addEventListener('pagehide',clearTransient);
  tab('control');
  Promise.all([api('config'),api('contracts')]).then(([cfg,c])=>{config=cfg;contracts=c;fillProfiles();loadProgram(c.program);status('wb-global-status','程序库、算法连接与对话接口已就绪；真实动作共用顶部的接管和停止入口。');return programs();}).catch(e=>status('wb-global-status',`工作区加载失败：${e.message}`,true));
})();
