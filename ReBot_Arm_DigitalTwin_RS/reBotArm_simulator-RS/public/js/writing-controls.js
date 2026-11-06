(function(){
  'use strict';
  const $=id=>document.getElementById(id),geometry=window.ReBotWritingPlane,twin=window.ReBotTwin;
  if(!geometry||!twin||!$('writing-settings'))return;
  const key='rebot-writing-settings-v1',numeric=['tilt-x','tilt-y','tilt-z','spin','offset-x','offset-y','offset-z','width','height','lift'];
  let preferences={},captured=null,fixed=null,templates=null,plane=null,draft=null;
  let recordedHere=false,capturedAt='',approved=null,confirmed='',checked=false,busy=false,generating=false,editing=false;
  try{preferences=JSON.parse(localStorage.getItem(key)||'{}');if(!preferences||typeof preferences!=='object'||Array.isArray(preferences))preferences={};}catch(_){}
  const status=(text,error=false)=>{$('writing-status').textContent=text;$('writing-status').dataset.error=String(error);};
  const signature=p=>JSON.stringify([p?.schema_version,p?.robot_model,p?.base_frame,p?.calibration_id,p?.units,p?.steps,p?.writing_plane,p?.writing_setup]);
  function assertCurrent(program,requireCheck=false){
    if(!recordedHere||!confirmed||signature(program)!==confirmed)throw new Error('书写设置或程序已改变，请重新确认平面并生成程序');
    if(requireCheck&&!checked)throw new Error('请先检查本次书写路径，再执行或播放虚影');
  }
  function forget(){approved=null;confirmed='';checked=false;}
  function updateFlow(){
    const step=!recordedHere?1:!draft?2:!confirmed?3:!checked?4:5;
    for(let i=1;i<=5;i++){$('writing-step-'+i).dataset.state=i<step?'done':i===step?'current':'pending';$('writing-step-'+i).setAttribute('aria-current',i===step?'step':'false');}
    $('writing-flow-state').textContent=busy?'正在处理，请等待结果。':[
      '', '下一步：手动摆到书写起点，再记录末端位姿。','下一步：选择参考轴、偏角与尺寸，查看法线和平面。',
      '下一步：核对原点、字的右方/上方及抬笔侧，再确认此平面。','平面已确认并固定；下一步：检查整套书写路径。',
      twin.hardware?'路径检查已通过；当前实机书写仍待工具与纸面标定，可播放虚影。':'路径检查已通过；可播放虚影，或执行已确认的仿真程序。'][step];
    $('writing-generate').disabled=!draft||!recordedHere||busy;
    $('writing-check').disabled=!confirmed||busy||!twin.fresh();
    $('writing-visual').disabled=!checked||busy;
    $('writing-execute').disabled=!checked||busy||!twin.fresh()||Boolean(twin.hardware&&approved?.writing_plane?.preview_only);
    $('writing-execute-note').textContent=twin.hardware?
      '实际笔尖、工具碰撞体与纸面尚未标定，当前实机书写仅预览；确认平面不等于工具标定。检查保留限位、重力和机器人自碰撞，执行前还会按实时起点重新检查。':
      '此处执行当前连接的仿真程序；执行前按实时起点重新检查。改动设置后必须重新确认、重新检查，不会续跑旧路径。';
    $('writing-capture-note').textContent=captured?`${recordedHere?'本次记录':'保存的规划，需重新记录'} · ${capturedAt} · TCP (${captured.slice(0,3).map(x=>(x*1000).toFixed(1)).join(', ')}) mm；姿态 (${captured.slice(3).map(x=>(x*180/Math.PI).toFixed(1)).join(', ')})°`:'尚未记录本次书写位置。';
  }
  window.ReBotWritingWorkflow={assertCurrent,active:()=>editing,invalidate(){forget();updateFlow();},uncheck(program){if(confirmed&&signature(program)===confirmed){checked=false;updateFlow();}}};
  function number(name){const value=$('writing-'+name).value;if(!value.trim()||!Number.isFinite(Number(value)))throw new Error('请输入有限的书写尺寸和偏角');return Number(value);}
  function settings(){return {axis:$('writing-axis').value,tilt_deg:['x','y','z'].map(a=>number('tilt-'+a)),spin_deg:number('spin'),
    retract_sign:Number($('writing-retract').value),offset_m:['x','y','z'].map(a=>number('offset-'+a)/1000)};}
  function values(){return {...settings(),width_mm:number('width'),height_mm:number('height'),lift_mm:number('lift')};}
  function fill(value){
    $('writing-axis').value=value?.axis||'';$('writing-retract').value=String(value?.retract_sign===-1?-1:1);
    for(const [i,a]of ['x','y','z'].entries()){$('writing-tilt-'+a).value=String(value?.tilt_deg?.[i]??0);$('writing-offset-'+a).value=String((value?.offset_m?.[i]??0)*1000);}
    $('writing-spin').value=String(value?.spin_deg??0);
    for(const [field,fallback]of [['width',90],['height',20],['lift',5]])$('writing-'+field).value=String(value?.[field+'_mm']??fallback);
    $('writing-width').max=$('writing-text').value==='BIT103'?'100':'300';
  }
  const ns='http://www.w3.org/2000/svg';
  function node(tag,attrs,text){const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,String(v));if(text!==undefined)e.textContent=text;return e;}
  const project=v=>[.82*(v[0]-v[1]),.38*(v[0]+v[1])-v[2]];
  function diagram(){
    const svg=$('writing-direction-svg');svg.replaceChildren();
    svg.append(node('text',{x:20,y:25,fill:'#cfdfed','font-size':13},'腕部正轴 · 固定基座斜视'),node('text',{x:320,y:25,fill:'#cfdfed','font-size':13},'书写平面与抬笔 · 非实际距离'));
    if(!plane){svg.append(node('text',{x:80,y:145,fill:'#8da5ba','font-size':16},'记录末端位姿并选择法线轴后显示方向'));return;}
    function arrow(anchor,vector,length,color,label,dashed=false){
      const p=project(vector),tip=[anchor[0]+p[0]*length,anchor[1]+p[1]*length],dx=tip[0]-anchor[0],dy=tip[1]-anchor[1],size=Math.hypot(dx,dy);
      if(size<3){svg.append(node('text',{x:anchor[0]+8,y:anchor[1]-8,fill:color,'font-size':13},(vector[0]+vector[1]+.76*vector[2]>0?'⊙ ':'⊗ ')+label));return;}
      svg.append(node('line',{x1:anchor[0],y1:anchor[1],x2:tip[0],y2:tip[1],stroke:color,'stroke-width':3,...(dashed?{'stroke-dasharray':'6 4'}:{})}));
      const a=[tip[0]-dx/size*10,tip[1]-dy/size*10],perp=[-dy/size*4,dx/size*4];
      svg.append(node('polygon',{points:`${tip} ${a[0]+perp[0]},${a[1]+perp[1]} ${a[0]-perp[0]},${a[1]-perp[1]}`,fill:color}),
        node('text',{x:tip[0]+5,y:tip[1]-5,fill:color,'font-size':13},label));
    }
    const origin=[425,165],corners=[[-1,-1],[1,-1],[1,1],[-1,1]].map(([a,b])=>{
      const p=project(plane.u.map((x,i)=>x*a+plane.v[i]*b));return [origin[0]+p[0]*46,origin[1]+p[1]*46];});
    svg.append(node('polygon',{points:corners.map(p=>p.join(',')).join(' '),fill:'#244943',stroke:'#548378','fill-opacity':.5}));
    ['X','Y','Z'].forEach((axis,i)=>arrow([135,165],plane.wrist_axes[i],85,['#ff6868','#65d89e','#71b4ff'][i],'+'+axis));
    const axis=plane.configuration.axis,reference=plane.wrist_axes['XYZ'.indexOf(axis[1])].map(v=>v*(axis[0]==='+'?1:-1));
    arrow(origin,reference,46,'#a7b4c7','参考 '+axis,true);
    arrow(origin,plane.u,70,'#5adfd5','R 字右');arrow(origin,plane.v,70,'#ac9aff','U 字上');
    arrow(origin,plane.normal,80,'#ffd170','+N 法线');arrow(origin,plane.retract_direction,112,'#ff91c1','抬笔',true);
    svg.append(node('text',{x:25,y:270,fill:'#8da5ba','font-size':12},'⊙ 朝向你 / ⊗ 背向你；落笔方向与粉色抬笔箭头相反。'));
  }
  function render(invalidate=false){
    plane=null;draft=null;
    if(invalidate){editing=true;forget();window.ReBotWritingEditor?.invalidate();}
    for(const [id,text]of [['wb-program-bit103','BIT103'],['wb-program-writing','国庆快乐']])$(id)?.setAttribute('aria-pressed',String($('writing-text').value===text));
    try{
      const value=values();preferences[$('writing-text').value]=value;
      try{localStorage.setItem(key,JSON.stringify(preferences));}catch(_){}
      if(captured&&fixed&&value.axis){
        plane=geometry.create(captured,fixed,value);
        if(templates&&recordedHere)draft=geometry.build(templates[$('writing-text').value],plane,{text:$('writing-text').value,width:value.width_mm/1000,height:value.height_mm/1000,lift:value.lift_mm/1000,speed:Math.min(.1,Math.max(.005,twin.jointSpeed()))});
        const fmt=v=>v.map(x=>(Math.abs(x)<.0005?0:x).toFixed(3)).join(', ');
        $('writing-direction-note').textContent=`N=${value.axis}腕部轴经偏转后，基座向量 (${fmt(plane.normal)})；抬笔 (${fmt(plane.retract_direction)})。平面原点 (${plane.origin.map(x=>(x*1000).toFixed(1)).join(', ')}) mm，已冻结。文字按 +N 一侧观察为正向。`;
      }else $('writing-direction-note').textContent=captured?'已记录末端位姿，请选择带正负号的法线轴。':'尚未记录末端位姿；偏角设置可以先保存，当前不生成运动目标。';
      if(invalidate)status('设置已保存在本机，旧确认和路径检查已作废。核对方向后请重新确认平面。');
    }catch(error){status(error.message,true);}
    diagram();updateFlow();
  }
  function select(text){
    if(!['BIT103','国庆快乐'].includes(text))return;
    $('writing-text').value=text;fill(preferences[text]);$('writing-settings').open=true;render(true);
    $('writing-settings').scrollIntoView({behavior:'smooth',block:'start'});
  }
  for(const name of [...numeric,'axis','retract'])$('writing-'+name).addEventListener('input',()=>render(true));
  $('writing-text').addEventListener('change',()=>select($('writing-text').value));
  window.addEventListener('rebot-writing-select',event=>select(event.detail.text));
  $('writing-capture').addEventListener('click',()=>{
    const state=twin.state();
    if(['running','paused'].includes(state?.extensions?.program?.status)){status('请先结束当前程序，再记录新的书写平面。',true);return;}
    if(!twin.fresh()||state?.feedback_valid!==true||!Array.isArray(state.pose)||state.pose.length!==6||!state.pose.every(Number.isFinite)){status('需要新鲜末端反馈才能记录平面；不会启动电机。',true);return;}
    captured=state.pose.slice();recordedHere=true;capturedAt=new Date().toLocaleTimeString();render(true);status('已记录当前末端位姿。请调整参考轴、偏角、文字大小和抬笔方向，再确认平面。');
  });
  $('writing-generate').addEventListener('click',()=>{
    try{
      if(busy||!draft||!recordedHere)throw new Error('请先记录本次末端位姿，选择法线轴并填写有效尺寸');
      if(['running','paused'].includes(twin.state()?.extensions?.program?.status))throw new Error('请先结束当前程序，再生成新程序');
      const text=$('writing-text').value,program=draft;
      generating=true;
      try{window.ReBotWritingEditor.load(program,'书写平面已确认并固定。下一步检查路径、播放虚影；实际工具标定未完成，不执行实机书写。');}finally{generating=false;}
      approved=program;confirmed=signature(program);checked=false;updateFlow();
      status(`已确认 ${text} 的平面并生成 ${program.steps.length} 步；宽 ${number('width')} mm，高 ${number('height')} mm。未发送运动。`);
    }catch(error){status(error.message,true);}
  });
  window.addEventListener('rebot-program-loaded',event=>{
    const program=event.detail.program,p=program.writing_plane,s=program.writing_setup;
    if(!generating){editing=Boolean(s);forget();recordedHere=false;capturedAt='来自保存的程序';}
    if(!p?.configuration||!s||!Array.isArray(p.captured_pose)){updateFlow();return;}
    $('writing-text').value=s.text;captured=p.captured_pose.slice();fixed=p.fixed_transform;
    fill({...p.configuration,width_mm:s.width*1000,height_mm:s.height*1000,lift_mm:s.lift*1000});render(false);
  });
  window.addEventListener('rebot-program-checked',event=>{if(confirmed&&signature(event.detail.program)===confirmed){checked=true;updateFlow();}});
  window.addEventListener('rebot-motion-profile-changed',()=>{checked=false;updateFlow();});
  $('twin-target')?.addEventListener('change',()=>{recordedHere=false;forget();window.ReBotWritingEditor?.invalidate();updateFlow();status('控制对象已改变，请重新记录本次书写位置。');});
  window.addEventListener('rebot-twin-state',()=>updateFlow());
  async function perform(method,requireCheck){
    if(busy)return;
    try{assertCurrent(approved,requireCheck);busy=true;updateFlow();await window.ReBotWritingEditor[method]();status(method==='preview'?'路径检查通过，可播放预览虚影；执行前仍会重新检查。':method==='visual'?'正在播放预览虚影，未发送运动。':'程序已提交，进度以控制器反馈为准。');}
    catch(error){if(method==='preview')checked=false;status(error.message,true);}
    finally{busy=false;updateFlow();}
  }
  $('writing-check').addEventListener('click',()=>perform('preview',false));
  $('writing-visual').addEventListener('click',()=>perform('visual',true));
  $('writing-execute').addEventListener('click',()=>perform('execute',true));
  fill(preferences[$('writing-text').value]);render(false);
  Promise.all([fetch('/api/urdf',{cache:'no-store'}).then(r=>{if(!r.ok)throw new Error('腕部模型读取失败');return r.text();}),
    fetch('/programs/writing-strokes.json',{cache:'no-store'}).then(r=>{if(!r.ok)throw new Error('书写笔画读取失败');return r.json();})]).then(([urdf,data])=>{
      const xml=new DOMParser().parseFromString(urdf,'application/xml'),joint=xml.querySelector('joint[name="j_gripper_end"]');
      if(joint?.getAttribute('type')!=='fixed'||joint.querySelector('parent')?.getAttribute('link')!=='link6'||joint.querySelector('child')?.getAttribute('link')!=='gripper_end')throw new Error('腕部与TCP的模型关系不匹配，未猜测坐标轴');
      const origin=joint.querySelector('origin'),parse=name=>origin.getAttribute(name).trim().split(/\s+/).map(Number);
      fixed={xyz:parse('xyz'),rpy:parse('rpy')};templates=data.templates;render(false);
    }).catch(error=>status(error.message,true));
})();
