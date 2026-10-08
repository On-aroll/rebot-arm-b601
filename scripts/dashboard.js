'use strict';
const $ = id => document.getElementById(id);
const all = selector => [...document.querySelectorAll(selector)];
const state = {data:null,online:false,busy:false,stream:null,initialized:false,selected:'writing',document:null,editorToken:0,samples:[],lastJob:'',camera:{azimuth:140,elevation:-25,distance:1.15},drag:null,cameraBusy:false,cameraPending:null};
const labels = {running:'执行中',paused:'已暂停',succeeded:'已完成',cancelled:'已取消',failed:'执行失败'};
const presets = {
 writing:{title:'一笔三维书写',url:'/writing-program',action:'print_demo',description:'沿连续三维路径书写「国庆快乐」，显示物理仿真中末端实际走过的轨迹。'},
 grasp:{title:'抓取与放置',url:'/program-example',action:'grasp_demo',description:'接近、抓取、搬运并放置物体，通过物体位置断言判断任务完成。'},
 circle:{title:'末端画圆',action:'circle_demo',description:'内置循环轨迹。点击「保持 / 暂停」结束循环并保持当前位置。'},
 joints:{title:'关节运动',action:'joints_demo',description:'内置六轴和夹爪循环运动。点击「保持 / 暂停」结束循环。'}
};
let toastTimer;
function log(message) {
 const li=document.createElement('li'), time=document.createElement('time'), span=document.createElement('span');
 time.textContent=new Date().toLocaleTimeString('zh-CN',{hour12:false});span.textContent=message;li.append(time,span);$('eventLog').prepend(li);
 while($('eventLog').children.length>60)$('eventLog').lastChild.remove();$('eventSummary').textContent=message;
}
function notice(message,error=false){$('notice').textContent=message;$('notice').className='notice'+(error?' error':'');$('notice').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('notice').hidden=true,6000);log(message);}
async function api(path,body){
 const response=await fetch(path,{method:body===undefined?'GET':'POST',headers:body===undefined?{}:{'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),signal:AbortSignal.timeout(body===undefined?2500:30000)});
 const result=await response.json();if(!response.ok||result.error)throw Error(result.error||`HTTP ${response.status}`);return result;
}
const post=(action,fields={})=>api('/command',{action,...fields});
function active(){return ['running','paused'].includes(state.data?.job?.status);}
function occupied(){return active()||state.data?.playing||state.data?.manual_motion;}
function gate(){
 const ready=state.online&&!state.busy,free=ready&&!occupied();
 all('.manual,.preset-run').forEach(e=>e.disabled=!free);
 $('resetScene').disabled=!free;$('runProgram').disabled=!free||!state.document;
 $('holdButton').disabled=!ready||!occupied()||state.data?.job?.status==='paused';
 $('resumeTask').disabled=!ready||state.data?.job?.status!=='paused';$('cancelTask').disabled=!ready||!active();
 all('[data-action^="camera_"]').forEach(e=>e.disabled=!ready);$('exportJob').disabled=!state.online||!state.data?.job;
 $('controlOwner').textContent=!state.online?'反馈不可用':state.busy?'正在提交':active()?'动作程序':state.data?.playing?'循环演示':state.data?.manual_motion?'手动运动':'可手动操作';
}
async function perform(fn,message){if(state.busy)return;state.busy=true;gate();try{await fn();if(message)notice(message);}catch(e){notice(e.message,true);}finally{state.busy=false;await pollOnce();gate();}}
function page(name){all('.page').forEach(e=>e.classList.toggle('active',e.id==='page-'+name));all('.main-nav [data-page]').forEach(e=>e.classList.toggle('active',e.dataset.page===name));if(name==='workspace')drawChart();window.scrollTo({top:0,behavior:'instant'});}
function tab(name){all('[data-tab]').forEach(e=>e.setAttribute('aria-selected',String(e.dataset.tab===name)));all('.control-pane').forEach(e=>e.classList.toggle('active',e.id==='pane-'+name));}
for(let i=0;i<6;i++){
 const row=document.createElement('div');row.className='joint-control';
 row.innerHTML=`<div class="joint-name"><b>J${i+1}</b><span id="actual${i}">—</span></div><button class="manual" data-jog="${i},-1" aria-label="J${i+1} 减小" disabled>−</button><input class="manual" id="q${i}" aria-label="J${i+1} 目标角度" type="number" step="any" required disabled><span class="unit">°</span><button class="manual" data-jog="${i},1" aria-label="J${i+1} 增大" disabled>+</button>`;
 $('jointControls').append(row);
 const tr=document.createElement('tr');tr.innerHTML=`<td>J${i+1}</td><td id="feedback${i}">—</td><td id="target${i}">—</td><td id="error${i}">—</td>`;$('feedbackRows').append(tr);
}
function syncTCP(){if(!state.data)return;state.data.xyz.forEach((v,i)=>$('tcp'+'XYZ'[i]).value=(v*1000).toFixed(1));$('tcpResult').className='muted small';$('tcpResult').textContent='已读取当前末端位置；执行前会重新求解和检查路径。';}
function syncDrafts(){if(!state.data)return;state.data.q_deg.forEach((v,i)=>$('q'+i).value=v.toFixed(3));$('finger').value=(state.data.finger_target_m*1000).toFixed(1);fingerLabel();syncTCP();}
function fingerLabel(){$('fingerLabel').textContent=Number($('finger').value).toFixed(1)+' mm';}
function numeric(id){const e=$(id);if(!Number.isFinite(e.valueAsNumber)||(e.min!==''&&e.valueAsNumber<Number(e.min))||(e.max!==''&&e.valueAsNumber>Number(e.max))){e.reportValidity();throw Error('请填写有效且在限位内的数值');}return e.valueAsNumber;}
const speed=()=>Number($('manualSpeed').value);
function stepsList(id,steps,done=false){$(id).replaceChildren();steps.forEach((s,i)=>{const li=document.createElement('li');li.className=done?'done':'';li.textContent=s.name||`${i+1}. ${s.op||'步骤'}`;const small=document.createElement('small');small.textContent=done?`${Number(s.elapsed_s||0).toFixed(1)} s`:s.op+(s.seconds!==undefined?` · ${s.seconds} s`:'');li.append(small);$(id).append(li);});}
function renderTask(job){
 if(!job){$('taskName').textContent='尚无动作任务';$('taskStatus').textContent='空闲';$('stepCount').textContent='0 / 0 步';$('taskProgress').style.width='0%';$('stepName').textContent='选择一个预置任务，或载入自己的程序。';$('completedSteps').replaceChildren();$('taskError').textContent='';$('jobId').textContent='';state.lastJob='';return;}
 $('taskName').textContent=job.name;$('taskStatus').textContent=labels[job.status]||job.status;$('taskStatus').className='tag '+job.status;
 const count=job.completed_steps.length,partial=active()?Math.min(.99,job.step_elapsed_s/Math.max(.001,job.step_duration_s)):0;
 $('stepCount').textContent=`${count} / ${job.step_count} 步`;$('taskProgress').style.width=(100*Math.min(1,(count+partial)/job.step_count))+'%';
 $('stepName').textContent=`${active()?'当前':'最近'}：${job.step_name}`;stepsList('completedSteps',job.completed_steps.slice(-4),true);$('taskError').textContent=job.error||'';$('jobId').textContent='任务 '+job.id;
 const key=job.id+job.status;if(key!==state.lastJob){log(`${job.name} · ${labels[job.status]||job.status}`);state.lastJob=key;}
}
function drawChart(){
 const canvas=$('errorChart'),w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;
 const ratio=window.devicePixelRatio||1;canvas.width=w*ratio;canvas.height=h*ratio;const ctx=canvas.getContext('2d');ctx.scale(ratio,ratio);
 const now=performance.now()/1000;state.samples=state.samples.filter(p=>now-p.t<=30);const scale=Math.max(.2,...state.samples.map(p=>p.v))*1.15;
 ctx.strokeStyle='#2c3f4c';ctx.lineWidth=1;for(let i=0;i<3;i++){ctx.beginPath();ctx.moveTo(0,4+i*(h-8)/2);ctx.lineTo(w,4+i*(h-8)/2);ctx.stroke();}
 ctx.strokeStyle='#65c9d1';ctx.lineWidth=1.7;ctx.beginPath();state.samples.forEach((p,i)=>{const x=w*(1-(now-p.t)/30),y=h-4-(h-8)*p.v/scale;if(!i||p.t-state.samples[i-1].t>1.5)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.stroke();$('chartScale').textContent=scale.toFixed(2)+'°';
}
function connection(ok,message){if(state.online!==ok)log(ok?'仿真反馈已连接':'仿真反馈中断，运动控件已禁用');state.online=ok;$('connection').className='pill'+(ok?'':' offline');$('connection').replaceChildren();const dot=document.createElement('i');$('connection').append(dot,document.createTextNode(ok?'仿真已连接':'反馈不可用'));$('sceneOffline').hidden=ok;$('deviceConnection').textContent=ok?'已连接 · MuJoCo':message||'连接中断';gate();}
let polling=false;
async function pollOnce(){
 if(polling)return;polling=true;
 try{const data=await api('/state');const age=Math.max(0,Date.now()/1000-data.feedback_time_unix_s);if(!Number.isFinite(age)||age>2)throw Error('反馈已过期');
 state.data=data;if(state.stream!==data.stream_id){state.stream=data.stream_id;state.initialized=false;state.samples=[];state.lastJob='';log('已连接仿真会话 '+data.stream_id.slice(0,8));}
 data.q_deg.forEach((v,i)=>{const input=$('q'+i);input.min=data.lower_deg[i];input.max=data.upper_deg[i];$('actual'+i).textContent=v.toFixed(1)+'°';$('feedback'+i).textContent=v.toFixed(2);$('target'+i).textContent=data.target_deg[i].toFixed(2);$('error'+i).textContent=(data.target_deg[i]-v).toFixed(2);});
 if(!state.initialized){syncDrafts();if(active())tab('task');state.initialized=true;}
 if(!state.drag&&!state.cameraBusy&&!state.cameraPending&&data.camera)state.camera={...data.camera};
 $('viewLabel').textContent=data.camera?.ghost_robot?'三维书写视角':'机械臂视角';$('feedbackSource').textContent='MuJoCo · 实际状态';$('tcpReadout').textContent=data.xyz.map(v=>(v*1000).toFixed(1)).join(' / ');$('simTime').textContent=data.sim_time.toFixed(1)+' s';$('traceLength').textContent=(data.job?.trace_length_m||0).toFixed(2)+' m';$('fingerActual').textContent=(data.finger_m*1000).toFixed(1);$('feedbackAge').textContent=Math.round(age*1000)+' ms';
 const error=Math.max(...data.q_deg.map((v,i)=>Math.abs(data.target_deg[i]-v)));$('maxError').textContent='最大偏差 '+error.toFixed(2)+'°';state.samples.push({t:performance.now()/1000,v:error});renderTask(data.job);connection(true);drawChart();
 }catch(e){connection(false,e.message);$('feedbackAge').textContent='不可用';drawChart();}finally{polling=false;}
}
function parseEditor(){
 try{const d=JSON.parse($('programText').value);if(d.version!==1||!Array.isArray(d.steps)||!d.steps.length)throw Error('需要 version: 1 和非空 steps 数组');if(d.steps.some(s=>!s||typeof s!=='object'||typeof s.op!=='string'))throw Error('每个步骤需要 op 操作名称');state.document=d;$('editorTitle').textContent=d.name||'未命名程序';$('programParseError').textContent='';$('programStepCount').textContent=d.steps.length+' 步';stepsList('programSteps',d.steps);}
 catch(e){state.document=null;$('programParseError').textContent=e.message;$('programStepCount').textContent='待检查';$('programSteps').replaceChildren();}gate();
}
async function loadPreset(key){
 const token=++state.editorToken,p=presets[key];state.selected=key;state.document=null;$('programText').value='';$('programText').disabled=true;$('editorTitle').textContent=p.title;$('programDescription').textContent=p.description;$('runPreset').dataset.preset=key;all('[data-load]').forEach(e=>e.classList.toggle('selected',e.dataset.load===key));$('programParseError').textContent='';$('programSteps').replaceChildren();$('programStepCount').textContent=p.url?'加载中':'循环';$('editorMode').textContent=p.url?'JSON':'内置程序';$('downloadProgram').disabled=$('formatProgram').disabled=!p.url;gate();
 if(!p.url){$('programText').value=p.description+'\n\n使用右侧「复位并运行原始预置」启动。';return;}
 try{const d=await api(p.url);if(token!==state.editorToken)return;$('programText').disabled=false;$('programText').value=JSON.stringify(d,null,2);parseEditor();}catch(e){if(token===state.editorToken){$('programParseError').textContent=e.message;notice('程序加载失败：'+e.message,true);}}
}
function download(value,name){const url=URL.createObjectURL(new Blob([JSON.stringify(value,null,2)],{type:'application/json'}));const a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
async function runPreset(key){await perform(async()=>{await post('reset');await post(key==='writing'?'camera_writing':'camera_robot');await post(presets[key].action);page('workspace');tab('task');},'已启动：'+presets[key].title);}
document.addEventListener('click',e=>{
 const b=e.target.closest('button');if(!b||b.disabled)return;
 if(b.dataset.page)page(b.dataset.page);
 else if(b.dataset.tab)tab(b.dataset.tab);
 else if(b.dataset.load)loadPreset(b.dataset.load);
 else if(b.dataset.preset)runPreset(b.dataset.preset);
 else if(b.dataset.action)perform(()=>post(b.dataset.action),({pause:'已保持 / 暂停',program_resume:'任务已继续',program_cancel:'任务已取消'})[b.dataset.action]);
 else if(b.dataset.jog)perform(async()=>{const [i,sign]=b.dataset.jog.split(',').map(Number),q=[...state.data.q_deg];q[i]+=sign*Number($('jogStep').value);if(q[i]<state.data.lower_deg[i]||q[i]>state.data.upper_deg[i])throw Error('点动目标超出关节限位');await post('joints',{q_deg:q,finger_m:state.data.finger_target_m,speed_rad_s:speed()});q.forEach((v,k)=>$('q'+k).value=v.toFixed(3));},'点动指令已提交');
});
$('syncDrafts').onclick=syncDrafts;$('syncTCP').onclick=syncTCP;$('finger').oninput=fingerLabel;
$('applyJoints').onclick=()=>perform(()=>post('joints',{q_deg:Array.from({length:6},(_,i)=>numeric('q'+i)),finger_m:numeric('finger')/1000,speed_rad_s:speed()}),'关节目标已提交');
function tcpAction(action){return perform(async()=>{
 try{const result=await post(action,{xyz:[...'XYZ'].map(c=>numeric('tcp'+c)/1000),orientation_mode:$('tcpMode').value,speed_rad_s:speed()});
 $('tcpResult').className='muted small';$('tcpResult').textContent=action==='ik_check'?`检查通过，未发送运动。位置残差 ${result.position_error_mm.toFixed(3)} mm；关节解 [${result.q_deg.map(v=>v.toFixed(1)).join(', ')}]°。${result.orientation_mode==='position'?'执行时允许夹爪朝向改变。':'保持当前朝向。'}`:'末端目标已提交，请观察实际反馈。';
 }catch(e){$('tcpResult').className='small failed';$('tcpResult').textContent=e.message;throw e;}
 },action==='ik'?'末端目标已提交':undefined);}
$('checkTCP').onclick=()=>tcpAction('ik_check');$('applyTCP').onclick=()=>tcpAction('ik');
['tcpX','tcpY','tcpZ','tcpMode'].forEach(id=>$(id).addEventListener('input',()=>{$('tcpResult').className='muted small';$('tcpResult').textContent='目标已修改，请重新检查；执行时仍会重新求解和检查路径。';}));
$('resetScene').onclick=()=>perform(async()=>{await post('reset');state.initialized=false;},'仿真场景已复位');
$('runProgram').onclick=()=>{parseEditor();if(state.document)perform(async()=>{await api('/program',state.document);page('workspace');tab('task');},'程序已提交');};
let editTimer;$('programText').oninput=()=>{state.document=null;$('programDescription').textContent='当前编辑的 JSON 程序；步骤预览随内容更新，执行前由服务端完整预检。';gate();clearTimeout(editTimer);editTimer=setTimeout(parseEditor,200);};
$('formatProgram').onclick=()=>{parseEditor();if(state.document)$('programText').value=JSON.stringify(state.document,null,2);};
$('downloadProgram').onclick=()=>{parseEditor();if(state.document)download(state.document,'rebot-program.json');};
$('importProgram').onclick=()=>$('programFile').click();
$('programFile').onchange=async()=>{const f=$('programFile').files[0];if(!f)return;try{if(f.size>=65536)throw Error('程序文件需小于 64 KB');++state.editorToken;$('programText').value=await f.text();$('programText').disabled=false;$('editorTitle').textContent=f.name;$('editorMode').textContent='JSON';$('downloadProgram').disabled=$('formatProgram').disabled=false;$('programDescription').textContent='导入程序从当前场景执行。右侧复位按钮仍运行所选原始预置。';parseEditor();}catch(e){notice(e.message,true);}finally{$('programFile').value='';}};
$('exportJob').onclick=()=>perform(async()=>download(await api('/jobs/'+state.data.job.id),'rebot-run-'+state.data.job.id+'.json'));
$('toggleEvents').onclick=()=>{$('eventLog').hidden=!$('eventLog').hidden;$('toggleEvents').setAttribute('aria-expanded',String(!$('eventLog').hidden));};
async function flushCamera(){if(state.cameraBusy||!state.cameraPending)return;state.cameraBusy=true;try{while(state.cameraPending){const camera=state.cameraPending;state.cameraPending=null;await post('camera',camera);}}catch(e){state.cameraPending=null;notice('视角更新失败：'+e.message,true);}finally{state.cameraBusy=false;}}
function cameraChanged(){state.cameraPending={azimuth:state.camera.azimuth,elevation:state.camera.elevation,distance:state.camera.distance};flushCamera();}
$('scene').onpointerdown=e=>{if(!state.online||state.busy||e.button!==0)return;state.drag={x:e.clientX,y:e.clientY};$('scene').setPointerCapture(e.pointerId);};
$('scene').onpointermove=e=>{if(!state.drag)return;state.camera.azimuth-=(e.clientX-state.drag.x)*.4;state.camera.elevation=Math.max(-80,Math.min(-5,state.camera.elevation+(e.clientY-state.drag.y)*.3));state.drag={x:e.clientX,y:e.clientY};cameraChanged();};
$('scene').onpointerup=$('scene').onpointercancel=()=>state.drag=null;
$('scene').addEventListener('wheel',e=>{e.preventDefault();if(!state.online||state.busy)return;state.camera.distance=Math.max(.35,Math.min(2.2,state.camera.distance*Math.exp(e.deltaY*.001)));cameraChanged();},{passive:false});
window.addEventListener('resize',drawChart);
function frame(){$('scene').src='/frame.jpg?t='+Date.now();}$('scene').onload=()=>setTimeout(frame,100);$('scene').onerror=()=>setTimeout(frame,1000);
async function poll(){await pollOnce();setTimeout(poll,400);}gate();frame();poll();loadPreset('writing');
