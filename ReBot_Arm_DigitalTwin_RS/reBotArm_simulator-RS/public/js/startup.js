(function () {
  'use strict';
  const $=id=>document.getElementById(id), buttons=[...document.querySelectorAll('[data-start]')];
  const mode='hardware';
  let snapshot=null, polling=false;
  function message(text,error=false) { $('startup-message').textContent=text;$('startup-message').dataset.error=String(error); }
  const steps=()=>['ros','bridge','can','hardware'];
  function prerequisites(key,s) {
    if (key==='ros') return true;
    if (!s.ros.ready) return false;
    if (key==='bridge') return true;
    if (!s.bridge.ready) return false;
    if (key==='hardware') return s.can.ready;
    if (key==='mcp') return s.simulation.ready;
    if (key==='text') return s.mcp.ready;
    return true;
  }
  function render() {
    document.querySelectorAll('[data-for]').forEach(e=>{e.hidden=e.dataset.for!==mode;});
    if (!snapshot) return;
    const s=snapshot.services;
    const completed=Object.values(s).filter(service=>service.job && service.job.state!=='running').sort((a,b)=>a.job.startedAt.localeCompare(b.job.startedAt)).at(-1);
    if(completed && !Object.values(s).some(service=>service.job?.state==='running')) message(completed.job.message,completed.job.state==='error');
    for (const button of buttons) {
      const key=button.dataset.start, service=s[key], article=button.closest('[data-service]'), job=service.job;
      const running=job?.state==='running', error=job?.state==='error' && !service.ready;
      article.dataset.state=running?'running':service.ready?'ready':error?'error':'pending';
      article.querySelector('.badge').textContent=running?'处理中':service.ready?'已就绪':error?'需处理':key==='ros'?'待检查':'未就绪';
      const info=key==='can' && service.ready?'CAN 已启用 · 1 Mbps · '+({'ERROR-ACTIVE':'总线正常','ERROR-WARNING':'总线错误警告','ERROR-PASSIVE':'总线错误较多'}[service.state] || service.state):key==='bridge' && service.running && !service.ready?'联动接口进程已启动，正在等待连接确认。':service.message;
      article.querySelector('.service-message').textContent=running || error ? job.message : info;
      button.disabled=running || !prerequisites(key,s);
      if (service.ready) button.textContent=key==='ros'?'重新检查':'检查 / 复用';
      else button.textContent=running?'请稍候…':({ros:'检查环境',bridge:'启动联动接口',can:'启用 CAN',simulation:'启动仿真',hardware:'启动只读反馈',mcp:'启动 MCP',text:'启动文本代理'})[key];
    }
    const required=steps(), done=required.filter(key=>s[key].ready).length, ready=done===required.length;
    $('startup-progress').textContent=`${done} / ${required.length} 步就绪`;
    $('startup-next').textContent=ready?'可以进入操作工作台':'按编号完成尚未就绪的步骤';
    $('startup-enter').setAttribute('aria-disabled',String(!ready));
    $('startup-enter-title').textContent=ready?'真实机械臂反馈已就绪，进入工作台操作':'服务准备完成后，进入真实机械臂工作台';
    $('startup-root').textContent=snapshot.root;
  }
  async function api(route,method='GET') {
    const response=await fetch(`/api/startup/${route}`,{method,cache:'no-store',signal:AbortSignal.timeout(6000)});
    const value=await response.json();if(!response.ok||!value.ok)throw new Error(value.error||`HTTP ${response.status}`);return value;
  }
  async function refresh() {
    if (polling) return;polling=true;
    try { snapshot=await api('status');render(); }
    catch(error) { snapshot=null;message(`无法连接启动服务：${error.message}。如网页服务已关闭，请双击桌面启动台。`,true);$('startup-enter').setAttribute('aria-disabled','true');buttons.forEach(b=>{b.disabled=true;}); }
    finally { polling=false; }
  }
  buttons.forEach(button=>button.addEventListener('click',async()=>{
    if(button.disabled)return;button.disabled=true;
    try {const value=await api(`start/${button.dataset.start}`,'POST');message(value.job?.message||'已发起启动步骤。');await refresh();}
    catch(error){message(error.message,true);render();}
  }));
  document.querySelectorAll('[data-service] details').forEach(details=>details.addEventListener('toggle',async()=>{
    if(!details.open)return;try{const value=await api(`logs/${details.closest('[data-service]').dataset.service}`);details.querySelector('pre').textContent=value.log;}catch(error){details.querySelector('pre').textContent=error.message;}
  }));
  $('startup-refresh').addEventListener('click',refresh);
  $('startup-enter').addEventListener('click',event=>{
    if($('startup-enter').getAttribute('aria-disabled')==='true'){event.preventDefault();message('请先完成上方必需步骤。',true);return;}
    try {localStorage.setItem('rebotarm.rs.ros.target','hardware');sessionStorage.setItem('rebotarm.startup.connect','hardware');}catch(_){}
  });
  render();void refresh();setInterval(()=>{if(!document.hidden)void refresh();},1500);
})();
