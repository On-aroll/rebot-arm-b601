(function () {
  'use strict';
  const $ = id => document.getElementById(id), twin = window.ReBotTwin;
  const SCHEMA = 'b601-controller/1.0';
  let checkId = null, checking = false, epoch = 0, state = null, loadedEngine = 'external', runConfig = null;
  let library = [], drafts = {};
  const pidFamily = engine => ['p','pd','pid','scheduled_pid'].includes(engine);
  const integralEngine = engine => ['pid','scheduled_pid'].includes(engine);
  const advancedFields = {'ctrl-schedule-ratio':'schedule_ratio','ctrl-schedule-error':'schedule_error_rad','ctrl-bandwidth':'control_bandwidth','ctrl-observer':'observer_bandwidth','ctrl-b0':'input_gain','ctrl-disturbance':'disturbance_limit','ctrl-horizon':'horizon','ctrl-position-weight':'position_weight','ctrl-velocity-weight':'velocity_weight','ctrl-delta-weight':'delta_weight'};
  const defaults = engine => ({schema:SCHEMA,engine,name:engine==='external'?'我的控制算法':`${engine.toUpperCase()} 初始参数`,
    endpoint:engine==='external'?'http://127.0.0.1:8089/step':'',rate_hz:10,output:'joint_velocity',
    parameters:engine==='external'?{}:engine==='ladrc'?{control_bandwidth:.6,observer_bandwidth:3,input_gain:1,disturbance_limit:.03}:engine==='mpc'?{horizon:12,position_weight:1,velocity_weight:.05,delta_weight:.1}:{kp:Array(6).fill(.6),ki:Array(6).fill(integralEngine(engine)?.02:0),kd:Array(6).fill(engine==='p'?0:.03),derivative_filter_s:.1,integral_limit_rad_s:.03,...(engine==='scheduled_pid'?{schedule_ratio:1.5,schedule_error_rad:.1}:{})}});
  function libraryOptions(){
    $('ctrl-library').replaceChildren(new Option('选择参数档案…',''),...library.map((c,i)=>new Option(`${(c.engine||'external').toUpperCase()} · ${c.name}`,String(i))));
  }
  function syncGains(){
    const engine=$('ctrl-engine').value, builtin=engine!=='external';
    $('ctrl-gains').hidden=!pidFamily(engine); $('ctrl-endpoint-row').hidden=builtin;
    for(const name of ['scheduled','ladrc','mpc'])$(`ctrl-${name}`).hidden=engine!==(name==='scheduled'?'scheduled_pid':name);
    const hints={scheduled_pid:'误差大时平滑增大Kp，趋近目标时恢复近端增益；Ki/Kd仍按逐轴参数。',ladrc:'一阶离散ESO估计总扰动，使用平滑器实际速度参考；带宽过大会放大噪声。停止/反馈断档清空观测器。',mpc:'每次求解有限时域二次优化，采用首个速度参考；预测位置使用保守约束，不替代执行器路径检查。'};
    $('ctrl-engine-hint').textContent=hints[engine]||(builtin?'主机位置误差→速度参考，经过上方共用运动配置；不写电机内部PID。每次启动清空积分与滤波历史。':'连接本机计算服务，可接入有实际模型与实现的算法。');
    if(!builtin)return;
    const p={...defaults(engine).parameters,...JSON.parse($('ctrl-params').value)};
    for(const [id,key] of Object.entries(advancedFields))if(p[key]!==undefined)$(id).value=p[key];
    if(!pidFamily(engine))return;
    $('ctrl-derivative-filter').value=p.derivative_filter_s; $('ctrl-integral-limit').value=p.integral_limit_rad_s;
    for(const g of ['kp','ki','kd'])for(let i=1;i<=6;i++){
      const el=$(`ctrl-${g}${i}`);el.value=Array.isArray(p[g])?p[g][i-1]:(p[g]??0);
      el.disabled=Boolean(state?.active||checking||(g==='ki'&&!integralEngine(engine))||(g==='kd'&&engine==='p'));
    }
  }
  function rememberDraft(){try{drafts[loadedEngine]=config(loadedEngine);localStorage.setItem('b601-controller-drafts',JSON.stringify(drafts));}catch(_){} }

  const tell = (text, error=false) => { $('ctrl-status').textContent=text; $('ctrl-status').dataset.error=String(error); };
  function config(engine=$('ctrl-engine').value) {
    return {schema:SCHEMA,engine,name:$('ctrl-name').value,endpoint:engine==='external'?$('ctrl-endpoint').value:'',
      rate_hz:Number($('ctrl-rate').value),output:$('ctrl-output').value,parameters:{...defaults(engine).parameters,...JSON.parse($('ctrl-params').value)}};
  }
  function load(c) {
    if(!['external','p','pd','pid','scheduled_pid','ladrc','mpc'].includes(c.engine||'external'))throw new Error('不支持此控制器');
    if(c.schema!==SCHEMA||!['joint_position','joint_velocity'].includes(c.output)||!c.parameters||typeof c.parameters!=='object')throw new Error('需要b601-controller/1.0控制器配置');
    loadedEngine=c.engine||'external';$('ctrl-engine').value=loadedEngine;
    $('ctrl-name').value=c.name||''; $('ctrl-endpoint').value=c.endpoint||''; $('ctrl-rate').value=c.rate_hz;
    $('ctrl-output').value=c.output; $('ctrl-params').value=JSON.stringify(c.parameters,null,2); invalidate();syncGains();
  }
  function invalidate(){epoch++;checkId=null;controls();}
  window.addEventListener('rebot-motion-profile-changed',invalidate);
  function controls(){
    const active=state?.active;
    $('ctrl-start').disabled=!checkId||!twin.owned()||!twin.fresh()||active||checking;
    $('ctrl-stop').disabled=!active||!twin.owned();
    $('ctrl-check').disabled=active||checking||!twin.fresh();
    for(const id of ['ctrl-engine','ctrl-library','ctrl-default','ctrl-save','ctrl-name','ctrl-endpoint','ctrl-rate','ctrl-output','ctrl-params','ctrl-import','ctrl-current','ctrl-from-preview',...Array.from({length:6},(_,i)=>`ctrl-j${i+1}`)])$(id).disabled=Boolean(active||checking);
    if($('ctrl-engine').value!=='external')$('ctrl-output').disabled=true;
    $('ctrl-derivative-filter').disabled=Boolean(active||checking||$('ctrl-engine').value==='p');
    $('ctrl-integral-limit').disabled=Boolean(active||checking||!integralEngine($('ctrl-engine').value));
    for(const id of Object.keys(advancedFields))$(id).disabled=Boolean(active||checking);
    for(const g of ['kp','ki','kd'])for(let i=1;i<=6;i++)$(`ctrl-${g}${i}`).disabled=Boolean(active||checking||(g==='ki'&&!integralEngine($('ctrl-engine').value))||(g==='kd'&&$('ctrl-engine').value==='p'));
  }
  function target(){return Array.from({length:6},(_,i)=>{const value=$(`ctrl-j${i+1}`).value;if(!value.trim()||!Number.isFinite(Number(value)))throw new Error('请先填入六个期望关节角');return Number(value)*Math.PI/180;});}
  function fill(q){if(!q?.length)throw new Error('没有可用关节目标');q.forEach((v,i)=>{$(`ctrl-j${i+1}`).value=(v*180/Math.PI).toFixed(9);});invalidate();}
  function download(value,name){const blob=new Blob([JSON.stringify(value,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  function action(id,fn){$(id).addEventListener('click',async()=>{try{await fn();}catch(e){tell(e.message,true);}finally{controls();}});}
  for(const id of ['ctrl-name','ctrl-endpoint','ctrl-rate','ctrl-output','ctrl-params','twin-joint-speed',...Array.from({length:6},(_,i)=>`ctrl-j${i+1}`)])$(id).addEventListener('input',invalidate);
  action('ctrl-current',()=>{if(!twin.fresh())throw new Error('请先连接新鲜反馈');const s=twin.state(),limits=s.control_settings?.joint_limits_rad;fill(s.actual.map((q,i)=>limits?Math.max(limits[i][0],Math.min(limits[i][1],q)):q));tell('已填入当前反馈（边界噪声按名义范围取值）；试算不发送运动。');});
  action('ctrl-from-preview',()=>{const q=twin.previewJoints();if(!q)throw new Error('请先在手动控制输入XYZ并计算预览');fill(q);tell('已载入坐标预览的关节目标；运动将由你接入的算法持续计算。');});
  $('ctrl-engine').addEventListener('change',()=>{const next=$('ctrl-engine').value;rememberDraft();load(drafts[next]||defaults(next));tell('已载入该控制器独立参数，尚未启动；请重新试算。');});
  $('ctrl-library').addEventListener('change',()=>{if($('ctrl-library').value==='')return;rememberDraft();load(library[Number($('ctrl-library').value)]);});
  $('ctrl-params').addEventListener('input',()=>{try{syncGains();rememberDraft();}catch(e){tell('参数JSON尚未完整',true);}});
  for(const g of ['kp','ki','kd'])for(let i=1;i<=6;i++)$(`ctrl-${g}${i}`).addEventListener('input',()=>{
    try{const p=JSON.parse($('ctrl-params').value),values=Array.isArray(p[g])?p[g]:Array(6).fill(p[g]||0);values[i-1]=Number($(`ctrl-${g}${i}`).value);p[g]=values;$('ctrl-params').value=JSON.stringify(p,null,2);invalidate();rememberDraft();}catch(e){tell(e.message,true);}
  });
  for(const [id,key] of [['ctrl-derivative-filter','derivative_filter_s'],['ctrl-integral-limit','integral_limit_rad_s']])$(id).addEventListener('input',()=>{
    invalidate();try{const p=JSON.parse($('ctrl-params').value);p[key]=Number($(id).value);$('ctrl-params').value=JSON.stringify(p,null,2);rememberDraft();}catch(e){tell(e.message,true);}
  });
  for(const [id,key] of Object.entries(advancedFields))$(id).addEventListener('input',()=>{
    invalidate();try{const p=JSON.parse($('ctrl-params').value);p[key]=Number($(id).value);$('ctrl-params').value=JSON.stringify(p,null,2);rememberDraft();}catch(e){tell(e.message,true);}
  });
  action('ctrl-default',()=>{load(defaults($('ctrl-engine').value));tell('已恢复该算法初始参数；尚未应用，需重新试算。');});
  action('ctrl-save',()=>{
    const c=config();if(!c.name.trim())throw new Error('请给参数档案起一个名称');
    const index=library.findIndex(v=>(v.engine||'external')===c.engine&&v.name===c.name);
    if(index<0){if(library.length>=64)throw new Error('最多保存64份档案，请导出备份');library.push(c);}else library[index]=c;
    rememberDraft();localStorage.setItem('b601-controller-library',JSON.stringify(library));localStorage.setItem('b601-control-algorithm',JSON.stringify(c));libraryOptions();tell('已保存参数档案；可改名另存一组增益。尚未启动算法。');
  });
  action('ctrl-export',()=>download(config(),'b601-controller.json'));
  $('ctrl-import').addEventListener('change',async e=>{try{const file=e.target.files?.[0];if(!file)return;if(file.size>32768)throw new Error('配置超过32KB');load(JSON.parse(await file.text()));tell('控制器配置已载入，未启动；请试算。');}catch(err){tell(err.message,true);}finally{e.target.value='';}});
  action('ctrl-contract',()=>download({schema:SCHEMA,config:config(),request:{schema:SCHEMA,request_id:'echo-this',dry_run:true,robot_model:'b601_rs',namespace:'rebotarm_rs',calibration_id:'rs-identity-v1',units:{position:'rad',velocity:'rad/s',time:'s'},state:{position:[0,0,0,0,0,0],velocity:[0,0,0,0,0,0],tcp_pose:[0,0,0,0,0,0],feedback_age_ms:0,feedback_stamp_s:0},reference:{position:[0,0,0,0,0,0]},period_s:.1,elapsed_s:0,limits:{position:'six [lower,upper] rad pairs',velocity:.1},output:config().output,parameters:config().parameters},response:{schema:SCHEMA,request_id:'echo-this',output:config().output,values:[0,0,0,0,0,0]}},'b601-controller-protocol.json'));
  action('ctrl-check',async()=>{
    const token=++epoch;checkId=null;runConfig=null;checking=true;controls();tell('正在读取反馈并调用控制算法；不发送运动…');
    try{const r=await twin.request('controller_check',{config:config(),target:target(),speed:twin.jointSpeed()});if(token!==epoch)return;checkId=r.check_id;tell(r.message);}
    finally{checking=false;}
  });
  action('ctrl-start',async()=>{if(!checkId)throw new Error('请先完成单步试算');const id=checkId;checkId=null;const snapshot={config:config(),motion:twin.state().motion_profile,target_rad:target(),speed_rad_s:twin.jointSpeed()};await twin.request('controller_start',{check_id:id});runConfig=snapshot;tell('闭环控制已启动，运行状态以控制器反馈为准');});
  action('ctrl-record',()=>{if(!runConfig||!state?.cycles||state.engine!==runConfig.config.engine||state.name!==runConfig.config.name)throw new Error('请等待本次控制器产生运行样本，再导出记录');download({schema:'b601-tuning-run/1.0',exported_at:new Date().toISOString(),...runConfig,result:state,metric_definition:'六轴实测反馈相对固定期望位置的误差；非电机力矩与轨迹跟踪精度验收'},'b601-tuning-run.json');});
  action('ctrl-stop',async()=>{invalidate();await twin.request('controller_stop');tell('控制算法已停止，保持当前位置');});
  for(const id of ['twin-stop','twin-reset-control','twin-release'])$(id).addEventListener('click',invalidate);
  window.addEventListener('keydown',e=>{if(e.code==='Escape')invalidate();});
  window.addEventListener('rebot-twin-state',event=>{
    const s=event.detail;state=s.extensions?.controller;
    $('ctrl-target').textContent=`当前目标：/${s.namespace} · ${s.hardware_phase||s.feedback_source} · ${s.owned?'已接管':'仅观察'}`;
    if(state){const tracking=['running','stopped','error'].includes(state.status)?state.tracking:null;
      $('ctrl-tracking').textContent=tracking?`目标误差RMSE · °：${tracking.rmse_deg.map(v=>v.toFixed(3)).join(' / ')} · ${tracking.samples}份反馈样本 · 输出限幅 ${tracking.saturation_samples??'外部算法未报告'} 次（包含趋近过程）`:'目标误差：等待新鲜运行样本';
      const diag=['running','stopped','error'].includes(state.status)?state.diagnostics:null;
      $('ctrl-diagnostics').textContent=diag?.disturbance_rad_s?`ESO扰动估计 · rad/s：${diag.disturbance_rad_s.map(v=>v.toFixed(5)).join(' / ')}`:diag?.effective_kp?`当前逐轴Kp：${diag.effective_kp.map(v=>v.toFixed(3)).join(' / ')}`:diag?.iterations?`MPC：${diag.iterations}次迭代 · 投影残差 ${Number(diag.projected_residual).toExponential(2)} · 关节积分预测模型`:'算法诊断：等待运行样本';
      const labels={idle:'未运行',checking:'试算中',checked:'试算通过',running:'运行中',stopped:'已停止',error:'故障保持'};
      $('ctrl-metrics').textContent=`${labels[state.status]||state.status} · 完成${state.cycles}次计算 · 响应 ${state.latency_ms??'—'}ms · 请求 ${state.requested_hz??'—'}Hz / 参考更新 ${state.applied_hz??'—'}Hz（实测，非硬实时）`;
      const format=v=>v?v.map(x=>Number(x).toFixed(5)).join('  '):'—';$('ctrl-requested').textContent=format(state.requested);$('ctrl-applied').textContent=format(state.applied);
      if(state.status==='error')tell(state.message,true);
      if(state.status==='stopped'){checkId=null;tell(state.message);}
    }else{$('ctrl-metrics').textContent='当前驱动未提供控制算法接口，请更新并重新连接';checkId=null;}
    controls();
  });
  try{const saved=localStorage.getItem('b601-control-algorithm');if(saved)load(JSON.parse(saved));}catch(e){tell(`未载入保存的配置：${e.message}`,true);}
  try{library=JSON.parse(localStorage.getItem('b601-controller-library')||'[]');if(!Array.isArray(library)||library.length>64)library=[];drafts=JSON.parse(localStorage.getItem('b601-controller-drafts')||'{}');}catch(_){library=[];drafts={};}
  libraryOptions();controls();
})();
