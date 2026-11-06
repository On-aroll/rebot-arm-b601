(function () {
  'use strict';
  const $=id=>document.getElementById(id), buttons=[...document.querySelectorAll('[data-start]')];
  const mode='hardware';
  const maintenanceUrl='/#hub-reset-section';
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
    const needsRecovery=s.hardware.needsRecovery===true;
    const feedback=snapshot.feedback?.hardware;
    const bundle=snapshot.bundle, allRunning=bundle?.state==='running';
    const completed=Object.values(s).filter(service=>service.job && service.job.state!=='running').sort((a,b)=>a.job.startedAt.localeCompare(b.job.startedAt)).at(-1);
    if(bundle)message(bundle.state==='error' && needsRecovery
      ? '实机反馈需要恢复，请使用第 4 步的维护入口。' : bundle.message,bundle.state==='error');
    else if(completed && !Object.values(s).some(service=>service.job?.state==='running')) message(completed.job.message,completed.job.state==='error');
    $('startup-all').disabled=allRunning||Object.values(s).some(service=>service.job?.state==='running');
    $('startup-all').textContent=allRunning?'正在检查并启动…':'一键检查并启动实机与相机';
    for (const button of buttons) {
      const key=button.dataset.start, service=s[key], article=button.closest('[data-service]'), job=service.job;
      const running=job?.state==='running', error=job?.state==='error' && !service.ready;
      const hardwareRecovery=key==='hardware' && needsRecovery && !running;
      article.dataset.state=running?'running':hardwareRecovery||error?'error':service.ready?'ready':'pending';
      article.querySelector('.badge').textContent=running?'处理中':service.needsRecovery?'需恢复':service.ready?'已就绪':error?'需处理':key==='ros'?'待检查':'未就绪';
      let info=service.message;
      if (key==='can' && service.ready) {
        const busState={'ERROR-ACTIVE':'总线正常','ERROR-WARNING':'总线错误警告','ERROR-PASSIVE':'总线错误较多'}[service.state];
        info=['CAN 已启用','1 Mbps',busState||service.state,service.usb_port && `USB ${service.usb_port}`].filter(Boolean).join(' · ');
      } else if (key==='bridge' && service.running && !service.ready) {
        info='联动接口进程已启动，正在等待连接确认。';
      }
      article.querySelector('.service-message').textContent=hardwareRecovery
        ? [service.message, feedback?.fresh && feedback.message].filter(Boolean).join(' · ')
        : running || error ? job.message : info;
      button.disabled=allRunning || running || hardwareRecovery || !prerequisites(key,s);
      if (hardwareRecovery) button.textContent='需先恢复';
      else if (service.ready) button.textContent=key==='ros'?'重新检查':'检查 / 复用';
      else button.textContent=running?'请稍候…':({ros:'检查环境',bridge:'启动联动接口',can:'启用 CAN',simulation:'启动仿真',hardware:'启动只读反馈',camera:'启动相机',mcp:'启动 MCP',text:'启动文本代理'})[key];
    }
    const required=steps(), done=required.filter(key=>s[key].ready).length, ready=done===required.length;
    $('startup-progress').textContent=`${done} / ${required.length} 步就绪`;
    $('startup-next').textContent=needsRecovery?'打开「重置与维护」处理实机反馈':ready?'可以进入操作工作台':'点击一键启动完成检查';
    $('startup-recovery').hidden=!needsRecovery;
    $('startup-enter').setAttribute('aria-disabled',String(!ready && !needsRecovery));
    $('startup-enter').setAttribute('href',needsRecovery?maintenanceUrl:'/');
    $('startup-enter').textContent=needsRecovery?'进入重置与维护 →':'进入操作工作台 →';
    $('startup-enter-title').textContent=needsRecovery?'实机反馈需要恢复':ready?'真实机械臂反馈已就绪，进入工作台操作':'服务准备完成后，进入真实机械臂工作台';
    $('startup-enter-hint').textContent=needsRecovery?'进入维护页查看状态。机械臂可靠支撑后，由你点击恢复按钮。':'坐标、键盘、手柄、TCP、程序、调参和画面，都在工作台左侧工具栏。';
    $('startup-root').textContent=snapshot.root;
  }
  async function api(route,method='GET') {
    const response=await fetch(`/api/startup/${route}`,{method,cache:'no-store',signal:AbortSignal.timeout(6000)});
    const value=await response.json();if(!response.ok||!value.ok)throw new Error(value.error||`HTTP ${response.status}`);return value;
  }
  async function refresh() {
    if (polling) return;polling=true;
    try { snapshot=await api('status');render(); }
    catch(error) { snapshot=null;message(`无法连接启动服务：${error.message}。如网页服务已关闭，请双击桌面启动台。`,true);$('startup-enter').setAttribute('aria-disabled','true');$('startup-recovery').hidden=true;buttons.forEach(b=>{b.disabled=true;}); }
    finally { polling=false; }
  }
  buttons.forEach(button=>button.addEventListener('click',async()=>{
    if(button.disabled)return;button.disabled=true;
    try {const value=await api(`start/${button.dataset.start}`,'POST');message(value.job?.message||'已发起启动步骤。');await refresh();}
    catch(error){message(error.message,true);render();}
  }));
  $('startup-all').addEventListener('click',async()=>{
    if($('startup-all').disabled)return;$('startup-all').disabled=true;
    try{const value=await api('start/all','POST');message(value.job.message);await refresh();}
    catch(error){message(error.message,true);$('startup-all').disabled=false;}
  });
  document.querySelectorAll('[data-service] details').forEach(details=>details.addEventListener('toggle',async()=>{
    if(!details.open)return;try{const value=await api(`logs/${details.closest('[data-service]').dataset.service}`);details.querySelector('pre').textContent=value.log;}catch(error){details.querySelector('pre').textContent=error.message;}
  }));
  $('startup-refresh').addEventListener('click',refresh);
  function selectHardware(connect) {
    try {
      localStorage.setItem('rebotarm.rs.ros.target','hardware');
      if (connect) sessionStorage.setItem('rebotarm.startup.connect','hardware');
      else sessionStorage.removeItem('rebotarm.startup.connect');
    } catch (_) {}
  }
  $('startup-recovery').addEventListener('click',()=>selectHardware(false));
  $('startup-enter').addEventListener('click',event=>{
    if($('startup-enter').getAttribute('aria-disabled')==='true'){event.preventDefault();message('请先完成上方必需步骤。',true);return;}
    selectHardware(!snapshot.services.hardware.needsRecovery);
  });
  render();void refresh();setInterval(()=>{if(!document.hidden)void refresh();},1500);
})();
