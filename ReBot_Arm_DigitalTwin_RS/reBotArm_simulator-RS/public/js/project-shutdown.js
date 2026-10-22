(function(){
  'use strict';
  const dialog=document.getElementById('project-exit-dialog');
  if(!dialog)return;
  const confirm=document.getElementById('project-exit-confirm'),submit=document.getElementById('project-exit-submit');
  const cancel=document.getElementById('project-exit-cancel'),status=document.getElementById('project-exit-status');
  const labels={hardware:'实机只读驱动',text:'文本代理',mcp:'MCP接口',simulation:'仿真与模型进程',bridge:'ROS联动接口'};
  let busy=false,polling=false;
  const channel=typeof BroadcastChannel==='function'?new BroadcastChannel('rebot-project-exit'):null;
  function completed(){
    // A separate static document releases old WebGL loops, cameras and timers.
    const html='<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>项目已结束</title><style>body{margin:0;background:#101c21;color:#e3f1ed;font:16px/1.8 system-ui;display:grid;min-height:100vh;place-items:center}main{max-width:580px;padding:32px}h1{font-size:28px;color:#91e4c5}p{color:#aec6c8}</style><main><h1>✓ 项目已结束</h1><p>后台服务已停止，网页服务正在退出。</p><p>可以直接关闭此页面。下次使用，双击桌面<br><strong>ReBot Arm B601-RS 项目启动台</strong>。</p></main></html>';
    location.replace(URL.createObjectURL(new Blob([html],{type:'text/html'})));
  }
  if(channel)channel.onmessage=event=>{if(event.data==='completed')completed();};
  async function api(route,body){
    const response=await fetch('/api/startup/shutdown/'+route,{method:body?'POST':'GET',
      headers:body?{'Content-Type':'application/json'}:{},body:body?JSON.stringify(body):undefined,
      cache:'no-store',signal:AbortSignal.timeout(8000)});
    const value=await response.json();
    if(!response.ok||!value.ok)throw new Error(value.error||'退出请求未确认');return value;
  }
  function show(job){
    status.dataset.error=String(job.state==='error');
    status.textContent=(job.steps||[]).map(s=>'✓ '+labels[s.kind]+'已结束').concat(job.message||'').join('\n');
  }
  async function follow(){
    if(polling)return;polling=true;
    try {
      const {job}=await api('status');show(job);
      if(job.state==='ready'){
        await api('finish',{id:job.id});
        channel?.postMessage('completed');completed();return;
      }
      if(job.state==='error'||job.state==='idle'){busy=false;cancel.disabled=false;submit.disabled=!confirm.checked;return;}
      setTimeout(()=>{polling=false;void follow();},400);
    }catch(error){
      busy=false;cancel.disabled=false;submit.disabled=!confirm.checked;
      status.dataset.error='true';status.textContent='退出尚未确认：'+error.message+'。可重新检查；不要把连接断开视为退出成功。';
    }finally{polling=false;}
  }
  document.querySelectorAll('[data-project-exit]').forEach(button=>button.addEventListener('click',async()=>{
    if(!dialog.open)dialog.showModal();
    if(busy)return;
    confirm.checked=false;submit.disabled=true;status.textContent='先让机械臂有支撑并失能，再确认结束项目。关闭标签页不会执行此操作。';
    status.dataset.error='false';
    try{const {job}=await api('status');if(['running','ready','exiting'].includes(job.state)){busy=true;cancel.disabled=true;void follow();}else if(job.state==='error')show(job);}
    catch(error){status.dataset.error='true';status.textContent='无法读取退出状态：'+error.message;}
  }));
  confirm.addEventListener('change',()=>{submit.disabled=busy||!confirm.checked;});
  cancel.addEventListener('click',()=>{if(!busy)dialog.close();});
  dialog.addEventListener('cancel',event=>{if(busy)event.preventDefault();});
  submit.addEventListener('click',async()=>{
    if(busy||!confirm.checked)return;
    busy=true;submit.disabled=cancel.disabled=true;status.dataset.error='false';status.textContent='正在检查退出条件…';
    try{const {job}=await api('start',{confirmed:true});show(job);void follow();}
    catch(error){busy=false;cancel.disabled=false;submit.disabled=!confirm.checked;status.dataset.error='true';status.textContent=error.message;}
  });
})();
