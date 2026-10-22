(function () {
  'use strict';
  const $ = id => document.getElementById(id), twin = window.ReBotTwin;
  if (!twin || !$('operation-hub')) return;
  const text = (id, value) => { if ($(id).textContent !== value) $(id).textContent = value; };
  const scroll = id => $(id).scrollIntoView({behavior:'smooth',block:'start'});
  const openWorkbench = name => document.querySelector(`[data-workbench-tab="${name}"]`).click();
  const workspaceForms = document.querySelector('.workspace-forms');
  const twinDeck = document.querySelector('.twin-control-deck');
  const projectWorkspace = $('project-mode-workspace');
  const projectPanels = {
    motorbridge: $('motorbridge-mode-panel'),
    ros: $('ros-mode-panel')
  };
  const monitor = $('hub-monitor');
  const motionSection = $('hub-motion-section');
  const sessionPanel = document.querySelector('.twin-session-panel');
  if (motionSection && sessionPanel) motionSection.querySelector('h2').after(sessionPanel);
  const motorGrid = twinDeck?.querySelector('.wb-tools-grid');
  if (motionSection && motorGrid) {
    const details = document.createElement('details'), summary = document.createElement('summary');
    details.className = 'wb-card motor-diagnostics';
    summary.textContent = '七轴参数与跟随诊断';
    details.append(summary, motorGrid);motionSection.append(details);
  }
  const toolbar = $('feature-toolbar'), orderKey = 'rebot-feature-order-v1';
  const entries = Array.from(toolbar.querySelectorAll('button'));
  entries.forEach(button => {
    const attr = Array.from(button.attributes).find(a => a.name.startsWith('data-'));
    button.dataset.toolbarKey = `${attr.name}:${attr.value}`;
    button.draggable = true;
    button.title = '拖动调整顺序；Alt + ↑ / ↓ 移动入口';
    const handle = document.createElement('span');handle.className='toolbar-grip';handle.textContent='⠿';handle.setAttribute('aria-hidden','true');button.append(handle);
  });
  try {
    const saved = JSON.parse(localStorage.getItem(orderKey));
    if (Array.isArray(saved)) {
      const remaining = new Map(entries.map(button => [button.dataset.toolbarKey, button]));
      saved.forEach(key => {if(remaining.has(key)){toolbar.append(remaining.get(key));remaining.delete(key);}});
      remaining.forEach(button => toolbar.append(button));
    }
  } catch (_) {}
  const saveOrder = () => {try{localStorage.setItem(orderKey,JSON.stringify(Array.from(toolbar.querySelectorAll('button'),b=>b.dataset.toolbarKey)));}catch(_){};};
  let dragged = null, pointerDrag = null, suppressClickUntil = 0;
  const entryAt = event => event.target.closest('button[data-toolbar-key]');
  // Embedded browsers may omit HTML drag/drop. Dragging the grip uses pointer
  // capture; button bodies retain ordinary navigation and native HTML dragging.
  toolbar.addEventListener('pointerdown',event=>{
    if(event.button!==0||!event.target.closest('.toolbar-grip'))return;
    const button=entryAt(event);if(!button)return;
    pointerDrag={button,id:event.pointerId,y:event.clientY,moved:false};
    toolbar.setPointerCapture(event.pointerId);event.preventDefault();
  });
  toolbar.addEventListener('pointermove',event=>{
    const p=pointerDrag;if(!p||p.id!==event.pointerId)return;
    if(!p.moved&&Math.abs(event.clientY-p.y)<6)return;
    p.moved=true;p.button.classList.add('toolbar-dragging');
    const target=document.elementFromPoint(event.clientX,event.clientY)?.closest('button[data-toolbar-key]');
    if(!target||target===p.button||!toolbar.contains(target))return;
    const box=target.getBoundingClientRect();
    toolbar.insertBefore(p.button,event.clientY>box.top+box.height/2?target.nextSibling:target);
  });
  const finishPointer=event=>{
    const p=pointerDrag;if(!p||p.id!==event.pointerId)return;
    pointerDrag=null;p.button.classList.remove('toolbar-dragging');
    if(toolbar.hasPointerCapture(event.pointerId))toolbar.releasePointerCapture(event.pointerId);
    if(p.moved){saveOrder();suppressClickUntil=Date.now()+250;}
  };
  for(const name of ['pointerup','pointercancel','lostpointercapture'])toolbar.addEventListener(name,finishPointer);
  toolbar.addEventListener('click',event=>{
    if(Date.now()<suppressClickUntil){event.preventDefault();event.stopImmediatePropagation();}
  },true);
  toolbar.addEventListener('dragstart', event => {
    if(pointerDrag){event.preventDefault();return;}
    dragged=entryAt(event);if(!dragged)return;
    dragged.classList.add('toolbar-dragging');event.dataTransfer.effectAllowed='move';event.dataTransfer.setData('text/plain',dragged.dataset.toolbarKey);
  });
  toolbar.addEventListener('dragover', event => {if(dragged){event.preventDefault();event.dataTransfer.dropEffect='move';}});
  toolbar.addEventListener('drop', event => {
    const target=entryAt(event);if(!dragged||!target||target===dragged)return;
    event.preventDefault();
    const after=event.clientY>target.getBoundingClientRect().top+target.getBoundingClientRect().height/2;
    toolbar.insertBefore(dragged,after?target.nextSibling:target);saveOrder();
  });
  toolbar.addEventListener('dragend',()=>{if(dragged)suppressClickUntil=Date.now()+250;dragged?.classList.remove('toolbar-dragging');dragged=null;});
  toolbar.addEventListener('keydown',event=>{
    if(!event.altKey||!['ArrowUp','ArrowDown'].includes(event.key))return;
    const button=entryAt(event),buttons=Array.from(toolbar.querySelectorAll('button')),index=buttons.indexOf(button);
    const target=buttons[index+(event.key==='ArrowUp'?-1:1)];if(!target)return;
    event.preventDefault();toolbar.insertBefore(button,event.key==='ArrowUp'?target:target.nextSibling);button.focus();saveOrder();
  });
  const projectMeta = {
    motorbridge: {
      eyebrow: 'MOTORBRIDGE DIRECT',
      title: 'MotorBridge 直连',
      description: '原始逐轴实机控制项目：连接、扫描、明确使能后，可单独拖动 J1–J7。单轴实时跟随只保留最新目标，80 ms 采样且同一时刻只发送一个请求。',
      local: '逐轴目标 · 实机直连'
    },
    ros: {
      eyebrow: 'ROSBRIDGE SIMULATION',
      title: 'ROSBridge 仿真控制台',
      description: 'ROS 2、仿真模型、任务程序和路径预览项目；仿真与任务功能在这里独立使用，默认不会连接、使能或运动真实机械臂。',
      local: 'ROS 2 · 仿真 · 任务'
    }
  };
  // Keep the legacy control-mode DOM and event bindings intact, but present
  // each original project as a first-class work area in the main workspace.
  // Moving the existing nodes avoids duplicating IDs or creating a second
  // command path for MotorBridge and ROSBridge.
  if (projectWorkspace) {
    Object.values(projectPanels).forEach(panel => panel && projectWorkspace.appendChild(panel));
  }
  const titles={point:'坐标到点',keyboard:'键盘 / 按钮',gamepad:'手柄控制',tcp:'拖拽末端 TCP',programs:'动作程序',teach:'示教录制',camera:'相机画面',settings:'速度与平滑',support:'支撑姿态',controller:'控制算法与调参',integrations:'视觉与任务规划',chat:'大模型对话',system:'连接与服务',prepare:'分步准备与接管',maintenance:'重置与维护'};
  function setProjectHeader(mode) {
    const meta = projectMeta[mode];
    if (!meta) return;
    $('operation-hub').dataset.projectMode = mode;
    text('hub-project-eyebrow', meta.eyebrow);
    text('hub-title', meta.title);
    text('hub-project-description', meta.description);
    text('hub-project-local', meta.local);
  }
  function activateProject(mode, button) {
    // The project entries are navigation only. Showing the existing control
    // panel must not connect, enable, scan, or submit a motion command.
    if (document.body.classList.contains('twin-advanced-open')) $('twin-advanced').click();
    document.body.classList.remove('toolbar-open');
    document.body.dataset.hubView = mode;
    $('toolbar-toggle')?.setAttribute('aria-expanded','false');
    window.reBotControlMode?.set(mode);
    window.reBotSim?.setTwinDrag(false);
    setProjectHeader(mode);
    if (projectWorkspace) projectWorkspace.hidden = false;
    if (workspaceForms) workspaceForms.hidden = true;
    if (twinDeck) twinDeck.hidden = true;
    document.querySelectorAll('[data-workbench-panel]').forEach(panel => { panel.hidden = true; });
    if ($('hub-motion-section')) $('hub-motion-section').hidden = true;
    Object.entries(projectPanels).forEach(([panelMode, panel]) => { if (panel) panel.hidden = panelMode !== mode; });
    document.querySelectorAll('.hub-features button').forEach(entry => entry.setAttribute('aria-current',String(entry===button)));
    projectWorkspace?.scrollIntoView({behavior:'smooth',block:'start'});
  }
  document.querySelectorAll('[data-project-mode]').forEach(button => button.addEventListener('click', () => {
    activateProject(button.dataset.projectMode, button);
  }));
  function selectView(name, button) {
    if (document.body.classList.contains('twin-advanced-open')) $('twin-advanced').click();
    if (projectWorkspace) projectWorkspace.hidden = true;
    $('operation-hub').removeAttribute('data-project-mode');
    Object.values(projectPanels).forEach(panel => { if (panel) panel.hidden = true; });
    document.body.dataset.hubView = name;
    const manual = ['point','keyboard','gamepad','tcp'].includes(name);
    const panelName = manual ? 'control' : name;
    const panel = document.querySelector(`[data-workbench-panel="${panelName}"]`);
    const needsMotion = manual || ['programs','teach','support','controller'].includes(name);
    document.querySelectorAll('[data-workbench-panel]').forEach(e=>{e.hidden=e!==panel;});
    if (motionSection) {
      motionSection.hidden = !needsMotion;
      if (needsMotion && panel) panel.append(motionSection);
    }
    if (sessionPanel) sessionPanel.hidden = !manual && !['teach','controller'].includes(name);
    if (workspaceForms) workspaceForms.hidden = name === 'camera';
    if (twinDeck) twinDeck.hidden = !manual;
    text('hub-project-eyebrow', 'ROBOT WORKSPACE');
    text('hub-project-description', '');text('hub-project-local', '');
    $('hub-title').textContent=titles[name] || name;
    window.reBotSim?.setTwinDrag(name === 'tcp');
    document.querySelectorAll('.hub-features button').forEach(e=>e.setAttribute('aria-current',String(e===button)));
    document.body.classList.remove('toolbar-open');
    $('toolbar-toggle').setAttribute('aria-expanded','false');
  }
  const activeMode = () => document.querySelector('[data-twin-mode][aria-pressed="true"]')?.dataset.twinMode || 'point';
  $('toolbar-toggle').addEventListener('click',()=>{
    const open=document.body.classList.toggle('toolbar-open');$('toolbar-toggle').setAttribute('aria-expanded',String(open));
  });
  document.querySelector('[data-hub-advanced]').addEventListener('click',()=>$('twin-advanced').click());
  function showMonitor(kind) {
    monitor.open = true;
    scroll(kind === 'camera' ? 'twin-camera-title' : 'viewport');
  }
  document.querySelectorAll('[data-hub-scroll]').forEach(button => button.addEventListener('click', () => scroll(button.dataset.hubScroll)));
  document.querySelectorAll('[data-hub-monitor]').forEach(button => button.addEventListener('click', () => {
    if(button.dataset.hubMonitor==='camera')selectView('camera',button);
    showMonitor(button.dataset.hubMonitor);
  }));
  document.querySelectorAll('[data-hub-mode]').forEach(button => button.addEventListener('click', () => {
    openWorkbench('control');
    document.querySelector(`[data-twin-mode="${button.dataset.hubMode}"]`).click();
    selectView(button.dataset.hubMode,button);
    if (button.dataset.hubMode === 'tcp') showMonitor('model');
    else scroll('hub-title');
  }));
  document.querySelector('[data-hub-teach]').addEventListener('click', () => {
    openWorkbench('teach'); scroll('hub-teach-section'); $('hub-teach-section').focus({preventScroll:true});
    selectView('teach',document.querySelector('[data-hub-teach]'));
  });
  document.querySelector('[data-hub-system]').addEventListener('click', () => {
    selectView('system',document.querySelector('[data-hub-system]'));scroll('hub-title');
  });
  document.querySelectorAll('[data-hub-system-section]').forEach(button=>button.addEventListener('click',()=>{
    selectView($(button.dataset.hubSystemSection).dataset.workbenchPanel,button);scroll('hub-title');
  }));
  window.addEventListener('rebot-workbench-changed', event => {
    const name=event.detail==='control'?activeMode():event.detail;
    selectView(name,name==='teach'?document.querySelector('[data-hub-teach]'):document.querySelector(`.hub-features [data-${event.detail==='control'?'hub-mode':'open-workbench'}="${name}"]`));
  });
  document.querySelectorAll('[data-twin-mode]').forEach(button=>button.addEventListener('click',()=>selectView(button.dataset.twinMode,document.querySelector(`[data-hub-mode="${button.dataset.twinMode}"]`))));
  const initialProject = window.reBotControlMode?.get() || 'ros';
  activateProject(initialProject, document.querySelector(`[data-project-mode="${initialProject}"]`));
  $('hub-hardware-advanced').hidden = !twin.hardware;
  $('twin-power-hold').hidden = !twin.hardware;
  text('hub-manual-help', twin.hardware
    ? '现场确认后，一次点击完成完整控制准备、接管和使能保持。执行程序可直接进入「动作程序」。'
    : '连接完成后点这里，从左侧选择操作方式。执行程序可直接进入「动作程序」。');

  function render() {
    const state = twin.state(), connected = window.reBotRos.connected, fresh = twin.fresh();
    let title, detail;
    if (!connected) {
      title = '下一步：连接机器人';
      detail = twin.hardware ? '连接只读取真实反馈，不使能电机。完整启动步骤见左侧「项目启动台」。' : '完整启动步骤见左侧「项目启动台」；也可使用连接按钮准备仿真。';
    } else if (!fresh) {
      title = '正在等待机器人反馈';
      detail = state?.hardware_phase === 'HOLDING' ? '控制器保留原目标，当前反馈未确认；反馈恢复后需明确重新接管。' : '暂时不能使能或运动。请查看下方提示，或打开「连接与服务」检查服务状态。';
    } else if (state?.recovery_required || ['FAULT','RECOVERY','STOPPING','RESETTING'].includes(state?.hardware_phase)) {
      title = '控制器尚未就绪';
      detail = state.message || '请在「连接与服务」查看控制状态。';
    } else if (state?.owned && !twin.owned()) {
      title = '其他页面正在控制'; detail = '请回到取得控制权的页面操作，或在那里暂停并释放控制权。';
    } else if (twin.owned()) {
      title = twin.hardware && state.hardware_phase === 'STARTING' ? '正在使能，请等待确认' : '本页已接管，从左侧选择功能';
      detail = state?.mode === 'GOAL' ? '正在执行目标。需要中止时，使用顶部「暂停并保持」。' : '键盘按住空格，手柄按住 RB / R1 才移动；程序在「动作程序」中单独启动。';
    } else if (twin.hardware && !$('twin-onsite').checked) {
      title = '下一步：勾选现场确认'; detail = '随后点击「使能并进入手动控制」；执行程序则直接打开「动作程序」。';
    } else {
      title = state?.hardware_phase === 'HOLDING' ? '已暂停，等待重新接管' : '已连接，选择手动或程序';
      detail = state?.hardware_phase === 'HOLDING' ? '旧动作不会自动继续。点击手动控制按钮可重新接管，程序需明确再次执行。' : '手动：点击上方的手动控制按钮。程序：打开「动作程序」，导入后点击「使能并执行程序」。';
    }
    text('hub-next-title', title); text('hub-next-detail', detail);
    $('operation-hub').dataset.connected = String(fresh);
    text('hub-ros-status', fresh ? '反馈正常' : connected ? '等待反馈' : '未连接');
    text('hub-last-message', $('twin-message').textContent);
    $('hub-last-message').dataset.error = $('twin-message').dataset.error || 'false';
  }
  async function refreshStatus() {
    const button = $('hub-refresh-status');
    if (button.disabled) return;
    button.disabled = true;
    try {
      const response = await fetch('/api/hardware/status', {signal:AbortSignal.timeout(4000),cache:'no-store'});
      const result = await response.json();
      if (!response.ok || !result.ok) throw new Error(result.error || `HTTP ${response.status}`);
      text('hub-web-status', '正常');
      text('hub-driver-status', result.running ? '运行中（不代表已使能）' : '未启动');
      text('hub-system-message', '检查完成。服务运行不代表反馈有效或电机已使能；当前反馈以上方状态为准。');
    } catch (error) {
      text('hub-driver-status', '未能确认');
      text('hub-system-message', `服务检查失败：${error.message}。可双击桌面「ReBot Arm B601-RS 一键启动」重新打开工作台。`);
    } finally { button.disabled = false; render(); }
  }
  $('hub-refresh-status').addEventListener('click', refreshStatus);
  render(); setInterval(render,250);
  void refreshStatus();
  // Consume only the explicit startup-page entry; ordinary visits remain inert.
  try {
    const connectTarget=sessionStorage.getItem('rebotarm.startup.connect');
    sessionStorage.removeItem('rebotarm.startup.connect');
    if(connectTarget===(twin.hardware?'hardware':'simulation') && !window.reBotRos.connected) $('twin-connect').click();
  } catch (_) {}
})();
