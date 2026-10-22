(function () {
  'use strict';
  const $ = (id) => document.getElementById(id);
  const client = window.reBotRos;
  const rosUI = window.reBotRosUI;
  const input = window.ReBotTwinInput;
  if (!client || !rosUI || !input) return;
  const manualRecovery = new input.ManualRecovery();
  const speedShortcut = new input.SpeedShortcut();
  const hardware = rosUI.target === 'hardware';
  const base = `/${client.namespace}/twin`;
  const fields = ['x','y','z','roll','pitch','yaw'].map((name) => $(`twin-${name}`));
  const pressed = new Set();
  const pending = new Map();
  let session = '', mode = 'point', sequence = 0, requestId = 0;
  let feedback = null, receivedAt = 0, plan = null, acquiring = false, acquireEpoch = 0;
  let jogActive = false, pointer = null, gamepadIndex = null, gamepadReady = false, neutralSequence = null;
  let lastSampleAt = 0, lastHeartbeatAt = 0, planRevision = 0, goalBusy = false, acquiredAt = 0, previewBusy = false, dragRevision = 0;
  const display = new window.ReBotFeedbackDisplay();
  let stateIntervals = [], lastStateAt = 0, frameCount = 0, fpsAt = 0, fps = 0;
  let lastControllerMessage = '', lastControllerPhase = '', stopping = false;
  let controlRange = 'small', changingRange = false, renderedRange = '', powerBusy = false;
  let resetting = false, resetJob = '', seenReset = '', resetAwaitingPose = false;
  let controlEpoch = 0;
  let manualError = '', gripperProbeRequested = false;
  let shownGripperSpeedMax = null;
  let changingMotionProfile = false, motionDraftKey = null, driveDraftKey = null, manualStarting = false, connectBusy = false, preparingControl = false;
  // Keep interactive input aligned with the full-mode CAN reference target.
  // The controller still owns the safety/watchdog decisions; this only reduces
  // the size of the browser-side target steps.
  const MANUAL_INPUT_PERIOD_MS = 20;
  const stopStorageKey = `rebot-twin-last-stop:${base}`;
  function showStopRequest(notice) {
    $('twin-stop-reason').textContent = notice?.reason
      ? `历史停止记录（非实时状态）${new Date(notice.stamp_ms).toLocaleTimeString()}：${notice.reason}` : '';
  }
  try { showStopRequest(JSON.parse(window.localStorage?.getItem(stopStorageKey) || 'null')); } catch (_) {}
  function recordStopRequest(reason) {
    const notice = {reason:String(reason).slice(0,160),stamp_ms:Date.now()};
    showStopRequest(notice);
    try { window.localStorage?.setItem(stopStorageKey,JSON.stringify(notice)); } catch (_) {}
    if (session && client.connected) {
      try {
        client.advertise(`${base}/stop_notices`,'std_msgs/msg/String');
        client.publish(`${base}/stop_notices`,{data:JSON.stringify({...notice,session})});
      } catch (_) {} // Diagnostic delivery never delays the actual stop.
    }
  }
  const editing = () => Boolean(document.activeElement?.closest('input,textarea,select,[contenteditable=true],dialog[open]'));
  const tell = (message, error = false) => { $('twin-message').textContent = message; $('twin-message').dataset.error = String(error); };
  const validJoints = (values) => Array.isArray(values) && values.length === 6 && values.every(Number.isFinite);
  const angleMap = (values) => Object.fromEntries(values.map((v,i) => [`joint${i+1}`,v]));
  function jointSpeed() {
    const field = $('twin-joint-speed'), value = Number(field.value);
    if (!field.value.trim() || !Number.isFinite(value) || value < Number(field.min) || value > Number(field.max)) {
      throw new Error(`关节速度请输入 ${field.min}–${field.max} rad/s`);
    }
    return value;
  }
  function showJointSpeed() {
    try {
      const speed = jointSpeed();
      $('twin-joint-degrees').textContent = `${(speed*180/Math.PI).toFixed(1)} °/s`;
      const acceleration = feedback?.control_settings?.acceleration_rad_s2;
      const fraction = feedback?.control_settings?.manual_speed_fraction ?? .8;
      if (feedback?.motion_profile?.selected === 'ruckig') {
        $('twin-braking-hint').textContent = 'Ruckig按速度、加速度和jerk约束减速，边界提前制动；减速期间仍会移动。松空格 / RB 立即请求保持。';
        return;
      }
      $('twin-braking-hint').textContent = acceleration
        ? `普通回中减速理论上界 ${(speed*fraction/acceleration).toFixed(2)} 秒 / ${(Math.pow(speed*fraction,2)/(2*acceleration)*180/Math.PI).toFixed(1)}°，边界可能提前减速。松空格 / RB 立即请求保持。`
        : '';
    }
    catch (_) { $('twin-joint-degrees').textContent = '数值超出允许范围'; }
  }

  function publish(op, args = {}) {
    if (!client.connected) throw new Error('ROS 未连接');
    client.advertise(`${base}/command`, 'std_msgs/msg/String');
    client.publish(`${base}/command`, { data:JSON.stringify({ op, session, seq:++sequence, stamp_ms:Date.now(), ...args }) });
  }
  function request(op, args = {}) {
    if (op.startsWith('program_') || op==='goal' || op.endsWith('_start')) manualRecovery.cancel();
    if (resetting) return Promise.reject(new Error('正在重置，旧请求已作废'));
    const id = `twin-${++requestId}`;
    return new Promise((resolve,reject) => {
      const timer = setTimeout(() => { pending.delete(id); reject(new Error('控制器未确认请求；请检查连接和控制器版本')); },op.endsWith('_preview') || op==='program_resume' ? 35000 : 4000);
      pending.set(id, { resolve, reject, timer, op, sentAt:performance.now() });
      try { publish(op, { ...args,id }); } catch (error) { clearTimeout(timer); pending.delete(id); reject(error); }
    });
  }
  function clearInput() {
    pressed.clear(); pointer = null; jogActive = false;
    neutralSequence = null;
    gamepadReady = false;
    speedShortcut.reset();
    document.querySelectorAll('[data-axis].is-held').forEach((button) => button.classList.remove('is-held'));
  }
  function invalidatePlan() {
    planRevision++;
    plan = null; $('twin-go').disabled = true;
    $('twin-preview-info').textContent = '先预览再执行。虚影是目标，实色模型始终显示反馈。';
    $('twin-position-only-retry').hidden = true;
    window.reBotSim?.setGhostVisible(false);
  }
  function showControlRange(settings) {
    if (!hardware || !settings || !['small','full'].includes(settings.mode)) return;
    controlRange = settings.mode;
    const recovering = Boolean(feedback?.recovery_required);
    if (!changingRange) $('twin-control-range').value = recovering ? '' : controlRange;
    $('twin-range-lock').hidden = !recovering;
    $('twin-range-lock').textContent = recovering
      ? '完整控制功能仍在；当前小幅、完整均暂停。须先核查停止与参数恢复，恢复确认后才能选择范围。'
      : '';
    $('twin-enable-hardware').textContent = recovering ? '准备控制（待恢复核查）' : controlRange === 'full' ? '准备完整控制（不使能）' : '准备小幅控制（不使能）';
    $('twin-range-help').textContent = recovering
      ? '两种范围：小幅试机每轴 0.5° / 15 秒；完整控制为 J1–J6 模型限位内 / 持续保持。当前恢复未确认，两种范围均不可执行。'
      : controlRange === 'full'
      ? `完整控制：关节速度可输入 ${settings.min_speed_rad_s ?? .005}–${settings.max_speed_rad_s} rad/s；J1–J6 模型限位内持续保持，无固定使能时长。通信超时暂停并保持；明确失能/重置或电气、机械保护异常时结束。保留力矩、电流及自碰撞校验；J7 与键盘、手柄共用接管与速度设置。`
      : '小幅模式：每次接管各轴 ≤ 0.5°、最高 0.02 rad/s，首次使能起最多 15 秒。J7 暂不参与。';
    $('twin-session-remaining').textContent = feedback?.recovery_required
      ? '已锁定：停止/参数恢复待核查；仅保留反馈观察'
      : feedback?.hardware_phase === 'HOLDING' ? '已冻结原位置目标 · 等待明确重新接管；请先确认当前反馈'
      : settings.continuous && feedback?.hardware_phase === 'ACTIVE' ? '持续使能保持 · 通信超时冻结目标；需重新接管'
      : settings.remaining_s == null ? '尚未出力 · 接管后可点「启动电机保持」，也可直接执行轨迹或手动控制' : `本次出力剩余 ${Math.ceil(settings.remaining_s)} 秒`;
    $('twin-limit-summary').textContent = settings.official_speed_max_rad_s
      ? `官方网页关节速度上限 ${settings.official_speed_max_rad_s} rad/s；结合电机内限速，当前生效上限 ${settings.max_speed_rad_s} rad/s。\n官方轨迹加速度 ${settings.acceleration_rad_s2} rad/s²，坐标轨迹 jerk 上限 ${settings.jerk_rad_s3} rad/s³。\nJ1–J3 / J4–J6：驱动峰值力矩 ${settings.torque_limits_nm[0]} / ${settings.torque_limits_nm[3]} N·m，电流运行上限 ${settings.current_limits_a[0]} / ${settings.current_limits_a[3]} A；普通动作持续重力按官方额定 ${settings.gravity_limits_nm[0]} / ${settings.gravity_limits_nm[3]} N·m 检查。峰值不表示可持续输出。${settings.support_recovery ? `\n支撑恢复第 ${settings.support_recovery.stage}/${settings.support_recovery.stages} 段：${settings.support_recovery.exception}；当前阶段模型预检包络 ${settings.support_recovery.gravity_limits_nm.join('/')} N·m。` : ''}\n关节范围沿用官方 RS 模型及 J1 ±150°。通信超时、跟踪误差和现场确认是本工程保护，官方没有为本 CSP 链路提供替代值。`
      : '小幅试机保留原试验范围。使用官方运行参数请选择完整控制；完整控制无需先进行小幅试机。';
    $('twin-control-timing').textContent = settings.command_period_ms == null
      ? settings.poll_duration_ms == null ? '目标更新周期：运动启动后显示；单一 CAN 控制器交错轮询'
        : `只读反馈查询：${settings.poll_duration_ms.toFixed(0)} ms / 轮 · 运动参考目标周期 ${settings.reference_period_target_ms ?? 90} ms，运动后显示实测值`
      : `目标更新周期：${settings.command_period_ms.toFixed(0)} ms · 曲线在 CAN 查询间隙连续取样，反馈保持独立时间戳`;
    if (settings.command_timeout_ms != null) $('twin-control-timing').textContent += ` · 命令期限 ${settings.command_timeout_ms} ms`;
    const timing=settings.poll_timing;
    if (Number.isFinite(timing?.max_reply_ms) && Number.isFinite(timing?.max_between_queries_ms)) $('twin-control-timing').textContent += ` · 近${timing.window}次查询：回复最长 ${timing.max_reply_ms.toFixed(0)} ms / 查询间空档最长 ${timing.max_between_queries_ms.toFixed(0)} ms`;
    const rangeKey = `${controlRange}:${settings.min_speed_rad_s}:${settings.max_speed_rad_s}`;
    if (renderedRange === rangeKey) return;
    renderedRange = rangeKey;
    invalidatePlan();
    Object.assign($('twin-joint-speed'),{min:String(settings.min_speed_rad_s ?? .005),max:String(settings.max_speed_rad_s),step:'any',value:String(Math.min(controlRange === 'full' ? .1 : .01,settings.max_speed_rad_s))});
    $('twin-speed-help').textContent = `允许 ${$('twin-joint-speed').min}–${settings.max_speed_rad_s} rad/s，可直接输入${settings.official_speed_max_rad_s ? `；官方网页上限 ${settings.official_speed_max_rad_s}，当前受设备内限速约束` : ''}。${settings.manual_speed_fraction === 1 ? '完整模式使用输入的关节速度上限。' : '手动指令留 20% 余量。'}加速度、姿态与跟随能力仍会限制速度。先暂停再改值；J7速度也受自身设备上限约束。`;
    showJointSpeed();
    const ranges = controlRange === 'full' ? [['linear',25,.5,40,.5,'mm/s'],['angular',10,.5,20,.5,'°/s']]
      : [['linear',1,.5,3,.5,'mm/s'],['angular',1,.5,3,.5,'°/s']];
    for (const [name,value,min,max,step,unit] of ranges) {
      Object.assign($(`twin-${name}-speed`),{min:String(min),max:String(max),step:String(step),value:String(value)});
      $(`twin-${name}-value`).textContent = `${value} ${unit}`;
    }
    showManualSpeed();
  }
  function hold(message = '已保持当前位置') {
    dragRevision++;
    const wasJogging = jogActive;
    clearInput();
    if (session && client.connected) {
      try { publish('hold'); } catch (_) {}
      if (wasJogging) invalidatePlan();
    }
    tell(message);
  }
  function forgetSession() {
    dragRevision++;
    session = ''; window.reBotTwinActive = false;
    clearInput(); invalidatePlan();
  }
  const suspendAll = (message='暂停并保持，等待重新接管') => stopAll(message,true);
  async function stopAll(message = '用户明确请求失能撤力', keepPower = false) {
    manualRecovery.cancel();
    keepPower = keepPower && hardware && controlRange === 'full';
    stopping = hardware && !keepPower;
    controlEpoch++; acquireEpoch++;
    if (session || acquiring) recordStopRequest(message);
    if (session && client.connected) { try { publish(keepPower?'suspend':'release',{reason:message}); } catch (_) {} }
    forgetSession();
    window.dispatchEvent(new CustomEvent('rebot-twin-suspended'));
    window.reBotSim?.stopMotion();
    if (client.connected) {
      try {
        const result = await client.callService(`${base}/${keepPower?'suspend':'stop'}`, 'std_srvs/srv/Trigger', {}, { timeoutMs:2000,retryOnTimeout:false });
        if (!result.success) throw new Error(result.message);
        tell(keepPower ? '已请求冻结原位置目标；不自动续跑，反馈恢复后重新接管。' : hardware ? '失能请求已提交，正在核对参数恢复…' : message);
      } catch (error) { tell(`停止确认未收到：${error.message}。实机结果未确认，请查看当前状态。`,true); }
    } else tell(keepPower?'连接已断开；已禁止本页运动，控制器将超时冻结目标，实机反馈未确认。':'连接已断开；停止结果未确认，请观察实机状态。',true);
    updateControls();
  }

  function resetLocalControls() {
    manualRecovery.cancel();
    manualError = '';
    controlEpoch++; acquireEpoch++;
    forgetSession();
    stopping = false;
    gamepadIndex = null;
    lastSampleAt = lastHeartbeatAt = acquiredAt = 0;
    renderedRange = ''; // restore conservative speed defaults for the selected range
    $('twin-manual-override').value='100';
    for (const wait of pending.values()) { clearTimeout(wait.timer); wait.reject(new Error('已重置，旧请求作废')); }
    pending.clear();
    $('twin-onsite').checked = false;
    $('ros-control-enable').checked = false;
    $('ros-control-enable').dispatchEvent(new Event('change'));
    rosUI.resetTwinControl?.();
    $('twin-orientation').value = 'free'; syncOrientation();
    mode = 'point';
    document.querySelectorAll('[data-twin-mode]').forEach((item) => item.setAttribute('aria-pressed',String(item.dataset.twinMode === mode)));
    document.querySelectorAll('[data-twin-panel]').forEach((panel) => { panel.hidden = panel.dataset.twinPanel !== mode; });
    fields.forEach((field) => { field.value = ''; });
    for (const id of ['twin-position-error','twin-rotation-error','twin-tracking-error']) $(id).textContent = '—';
    window.reBotSim?.stopMotion();
    resetAwaitingPose = true;
  }
  function showReset(status) {
    const key = `${status.id}:${status.status}`;
    if (key === seenReset) return;
    seenReset = key;
    const busy = ['queued','running'].includes(status.status);
    resetting = busy;
    $('twin-reset-status').textContent = status.message;
    $('twin-reset-status').dataset.error = String(status.status === 'failed');
    tell(status.message,status.status === 'failed');
  }
  $('twin-reset-control').addEventListener('click',async () => {
    if (resetting) return;
    resetting = true; resetJob = ''; seenReset = '';
    resetLocalControls(); updateControls();
    if (!client.connected) {
      resetting = false;
      $('twin-reset-status').textContent = '页面已重置；ROS 未连接，未确认实机停止或参数恢复。请先连接反馈并核查状态；缺少原始快照时，反复重置不能解除恢复锁。';
      $('twin-reset-status').dataset.error = 'true';
      tell($('twin-reset-status').textContent,true); updateControls(); return;
    }
    $('twin-reset-status').textContent = '已清空本页输入，正在请求控制器停止并重置…';
    try {
      const result = await client.callService(`${base}/reset`, 'std_srvs/srv/Trigger', {}, {timeoutMs:3000,retryOnTimeout:false});
      if (!result.success) throw new Error(result.message);
      const status = JSON.parse(result.message);
      resetJob = status.id;
      // The worker may finish before the service response reaches the page.
      showReset(feedback?.reset_status?.id === resetJob ? feedback.reset_status : status);
    } catch (error) {
      resetting = false;
      $('twin-reset-status').textContent = `本页已重置；控制器重置未确认：${error.message}。请先核查停止与恢复记录；缺少原始快照时，反复重置不能解除恢复锁。`;
      $('twin-reset-status').dataset.error = 'true';
      tell($('twin-reset-status').textContent,true);
    }
    updateControls();
  });

  $('twin-target').value = rosUI.target;
  $('twin-target').addEventListener('change', () => {
    if (session || client.connected || acquiring) { $('twin-target').value = rosUI.target; tell('请先释放控制并在连接按钮断开 ROS，再切换对象',true); return; }
    const old = $('ros-target'); old.value = $('twin-target').value; old.dispatchEvent(new Event('change'));
  });
  $('twin-view-title').textContent = hardware ? '机械臂 · 实机反馈孪生' : '机械臂 · 仿真反馈';
  $('twin-hardware-gate').hidden = !hardware;
  if (hardware) {
    const control = $('ros-control-enable').closest('label');
    $('twin-control-lock-host').append(control);
    $('twin-connect').textContent = '连接实机反馈';
    $('twin-service-status').textContent = '连接不会使能机械臂';
    $('twin-enable-hardware').textContent = '准备小幅控制（不使能）';
    tell('先连接实机反馈。反馈就绪后，勾选现场确认，再选择手动控制或动作程序。');
    Object.assign($('twin-joint-speed'),{min:'.005',max:'.02',value:'.01',step:'any'});
    showJointSpeed();
    for (const [name,value,min,max,step,unit] of [['linear',1,.5,3,.5,'mm/s'],['angular',1,.5,3,.5,'°/s']]) {
      const field = $(`twin-${name}-speed`); Object.assign(field,{min:String(min),max:String(max),step:String(step),value:String(value)});
      $(`twin-${name}-value`).textContent = `${value} ${unit}`;
    }
    for (const id of ['ros-safe-home','ros-open-gripper','ros-close-gripper','ros-gravity-start','ros-gravity-stop','ros-gravity-status-query']) $(id).disabled = true;
  }
  $('twin-advanced').addEventListener('click', () => {
    const expanded = document.body.classList.toggle('twin-advanced-open');
    $('twin-advanced').setAttribute('aria-expanded',String(expanded));
  });
  $('twin-reset-view').addEventListener('click',() => $('reset-camera').click());
  $('twin-connect').addEventListener('click', async () => {
    if (connectBusy) return;
    if (client.connected) {
      await stopAll('断开 ROS，停止并释放控制'); client.disconnect();
      if (hardware) {
        try { await fetch('/api/hardware/disconnect',{method:'POST'}); }
        catch (error) { tell(`反馈服务停止请求失败：${error.message}`,true); }
      }
      return;
    }
    connectBusy = true; $('twin-connect').disabled = true;
    try {
      window.reBotControlMode?.set('ros');
      if (!hardware) {
        tell('正在启动安全仿真与 ROS 桥接…');
        const response = await fetch('/api/quick-start', { method:'POST',headers:{Accept:'application/json'} });
        const result = await response.json();
        if (!response.ok || !result.ok) throw new Error(result.error || '仿真启动失败');
      } else {
        tell('正在启动实机只读反馈；不会使能电机…');
        const response = await fetch('/api/hardware/connect',{method:'POST'});
        const result = await response.json();
        if (!response.ok || !result.ok) throw new Error(result.error || '实机反馈服务启动失败');
      }
      const url = $('ros-url');
      if (!url.value.trim()) url.value = `${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.hostname || '127.0.0.1'}:9090`;
      $('ros-connect').click();
      tell('正在连接；等待统一控制器与关节反馈…');
    } catch (error) { tell(error.message,true); }
    finally { connectBusy = false; updateControls(); }
  });
  function hardwareAllowed() {
    return !hardware || ($('twin-onsite').checked && $('ros-control-enable').checked);
  }
  $('twin-control-range').addEventListener('change', async () => {
    if (resetting) return;
    const epoch = controlEpoch;
    const selected = $('twin-control-range').value;
    changingRange = true; invalidatePlan(); updateControls();
    try {
      const result = await request('control_mode',{mode:selected});
      if (epoch !== controlEpoch) return;
      if (feedback) feedback.control_settings = result.control_settings;
      showControlRange(result.control_settings);
      tell(`${selected === 'full' ? '完整' : '小幅'}控制已选定，尚未使能。核对现场确认项后，准备并接管控制。`);
    } catch (error) { if (epoch === controlEpoch) tell(error.message,true); }
    finally { changingRange = false; $('twin-control-range').value = controlRange; updateControls(); }
  });
  async function applyMotionProfile(parameters) {
    if (resetting || changingMotionProfile) return;
    const epoch = controlEpoch, profile = $('twin-motion-profile').value;
    changingMotionProfile = true;invalidatePlan();updateControls();
    try {
      const result = await request('motion_profile',parameters ? {profile,parameters} : {profile});
      if (epoch !== controlEpoch) return;
      if (feedback?.motion_profile) feedback.motion_profile = result.motion_settings || {...feedback.motion_profile,selected:result.motion_profile};
      motionDraftKey = null;
      window.dispatchEvent(new CustomEvent('rebot-motion-profile-changed'));
      tell('共用运动配置已生效；请重新检查路径。尚未使能或运动。');
    } catch (error) { if (epoch === controlEpoch) tell(error.message,true); }
    finally { changingMotionProfile = false;updateControls(); }
  }
  $('twin-motion-profile').addEventListener('change', () => applyMotionProfile());
  $('motion-apply').addEventListener('click', () => {
    const a = Number($('motion-acceleration').value), j = Number($('motion-jerk').value);
    if (![a,j].every(v => Number.isFinite(v) && v >= 5 && v <= 100)) {tell('参数比例须在5–100%之间',true);return;}
    return applyMotionProfile({acceleration_scale:a/100,jerk_scale:j/100});
  });
  $('motion-default').addEventListener('click', () => applyMotionProfile({acceleration_scale:1,jerk_scale:1}));
  $('motion-soft-draft').addEventListener('click', () => {
    $('motion-acceleration').value='25';$('motion-jerk').value='20';
    tell('已填入柔和试调草稿，尚未应用。停止并释放控制后，点击“应用并保存参数”；此配置共用于六轴运动，程序speed保持原值。');
  });
  const mitDefaults={kp:[50,150,150,50,50,50],kd:[3,10,10,5,4,4]};
  function driveParams(){return Object.fromEntries(['kp','kd'].map(g=>[g,Array.from({length:6},(_,i)=>Number($(`drive-${g}${i+1}`).value))]));}
  $('drive-backend').addEventListener('change',()=>{$('drive-mit-gains').hidden=$('drive-backend').value!=='mit';updateControls();});
  $('drive-default').addEventListener('click',()=>{for(const g of ['kp','kd'])mitDefaults[g].forEach((v,i)=>{$(`drive-${g}${i+1}`).value=v;});updateControls();});
  for(const g of ['kp','kd'])for(let i=1;i<=6;i++)$(`drive-${g}${i}`).addEventListener('input',updateControls);
  $('drive-apply').addEventListener('click',async()=>{
    if(resetting||changingMotionProfile)return;
    const epoch=controlEpoch,driver=$('drive-backend').value,parameters=driveParams();
    if(['kp','kd'].some(g=>parameters[g].some((v,i)=>!Number.isFinite(v)||v<=0||v>mitDefaults[g][i]))){tell('MIT增益须大于0且不超过锁定官方逐轴初值',true);return;}
    changingMotionProfile=true;invalidatePlan();updateControls();
    try{
      const result=await request('drive_config',{driver,parameters});
      if(epoch!==controlEpoch)return;
      if(feedback?.motion_profile)feedback.motion_profile=result.motion_settings;
      driveDraftKey=null;window.dispatchEvent(new CustomEvent('rebot-motion-profile-changed'));
      tell('驱动与MIT参数已保存；尚未使能，请重新检查路径。');
    }catch(e){if(epoch===controlEpoch)tell(e.message,true);}
    finally{changingMotionProfile=false;updateControls();}
  });
  $('twin-enable-hardware').addEventListener('click', () => {
    if (changingMotionProfile) return;
    if (!hardwareAllowed()) { tell('请先完成现场确认，并手动打开「允许网页向 RS 真机发控制」',true); return; }
    $('ros-enable').click();
  });
  async function enablePowerHold(guard = () => {}) {
    guard();
    if (resetting || powerBusy || !session || !hardwareAllowed()) throw new Error('尚未取得控制权或现场确认已撤回');
    const epoch = controlEpoch;
    powerBusy = true; clearInput(); invalidatePlan(); updateControls();
    try { const result = await request('power_hold'); guard(); if (epoch === controlEpoch) tell(result.message); }
    finally { powerBusy = false; updateControls(); }
  }
  $('twin-power-hold').addEventListener('click', () => enablePowerHold().catch(error=>tell(error.message,true)));
  async function acquireControl(guard = () => {}) {
    guard();
    if (session) return;
    if (resetting || acquiring || changingMotionProfile) throw new Error('控制准备尚未结束');
    if (changingRange || !hardwareAllowed() || !rosUI.canTwinControl()) throw new Error(hardware ? '请先核对现场确认项并点击「准备控制」，再接管。' : '等待新鲜反馈和仿真驱动就绪后，再接管控制。');
    acquiring = true; const epoch = ++acquireEpoch;
    updateControls();
    try {
      rosUI.prepareTwinControl();
      const result = await client.callService(`${base}/acquire`, 'std_srvs/srv/Trigger', {}, { timeoutMs:4000,retryOnTimeout:false });
      if (!result.success) throw new Error(result.message);
      const token = JSON.parse(result.message).session;
      try { guard(); } catch (error) { publish(hardware&&controlRange==='full'?'suspend':'release',{session:token}); throw error; }
      if (epoch !== acquireEpoch || document.hidden || !hardwareAllowed()) {
        publish(hardware&&controlRange==='full'?'suspend':'release', { session:token });
        return;
      }
      session = token; acquiredAt = performance.now(); window.reBotTwinActive = true; lastHeartbeatAt = 0;
      invalidatePlan(); clearInput();
      tell(feedback?.hardware_phase==='HOLDING'?'正在重新接管冻结的保持状态，旧程序不会续跑。':hardware ? (controlRange === 'full' ? '已接管完整控制，电机尚未使能。可点「启动电机保持」；坐标先预览，键盘按住空格，手柄按住 RB / R1。' : '已接管，电机尚未使能。可点「启动电机保持」或发起运动；各轴本次范围 0.5°，出力最多 15 秒。') : '已接管控制。坐标先计算预览；键盘按住空格，手柄按住 RB / R1。');
    } finally { acquiring = false; updateControls(); }
  }
  $('twin-acquire').addEventListener('click', () => acquireControl().catch(error=>tell(error.message,true)));
  async function prepareProgramControl(guard = () => {}) {
    if (preparingControl) throw new Error('已有控制准备正在进行，请等待或暂停');
    preparingControl = true;
    try { await prepareControl(guard); }
    finally { preparingControl = false; }
  }
  async function prepareControl(guard) {
    const epoch = controlEpoch;
    const check = () => {
      guard();
      if (epoch !== controlEpoch || resetting || document.hidden) throw new Error('执行准备已取消');
      if (changingMotionProfile) throw new Error('运动方案正在切换，请稍后执行');
      if (!client.connected) throw new Error('请先连接机器人反馈');
      if (hardware && !$('twin-onsite').checked) throw new Error('请先勾选顶部现场确认；本次确认后可直接执行程序');
    };
    check();
    if (!receivedAt || performance.now()-receivedAt > 500 || feedback?.feedback_valid === false) throw new Error('等待新鲜机器人反馈');
    if (feedback?.recovery_required || ['FAULT','RECOVERY','STOPPING'].includes(feedback?.hardware_phase)) throw new Error('上次控制尚未恢复，请查看当前状态');
    if (feedback?.owned && !session) throw new Error('其他页面正在控制机械臂');
    if (session) {
      if (hardware && controlRange !== 'full') throw new Error('当前是小幅试机会话，请先停止，再执行完整程序');
      return;
    }
    // Explicit program execution or manual enable authorizes web control.
    // Connecting, loading a program and switching panels never call this.
    if (hardware) {
      $('ros-control-enable').checked = true;
      $('ros-control-enable').dispatchEvent(new Event('change'));
      if (controlRange !== 'full') {
        const result = await request('control_mode',{mode:'full'});check();
        if (feedback) feedback.control_settings = result.control_settings;
        showControlRange(result.control_settings);
      }
      if (!feedback?.control_prepared) {
        const result = await client.callService(`/${client.namespace}/enable`,'std_srvs/srv/Trigger',{}, {timeoutMs:4000,retryOnTimeout:false});
        check();if (!result.success) throw new Error(result.message);
      }
    }
    const until = performance.now()+5000;
    while (!rosUI.canTwinControl() || !receivedAt || performance.now()-receivedAt>500 || feedback?.feedback_valid===false) {
      check();if (performance.now()>until) throw new Error('控制准备未确认，请查看机器人状态');
      await new Promise(resolve=>setTimeout(resolve,50));
    }
    const wasHolding=feedback?.hardware_phase==='HOLDING';
    check();await acquireControl(check);check();
    while(wasHolding && feedback?.hardware_phase==='HOLDING') {
      check();if(performance.now()>until)throw new Error('保持状态重新接管尚未确认');
      await new Promise(resolve=>setTimeout(resolve,50));
    }
    if (!session) throw new Error('未取得控制权，执行已取消');
  }
  $('twin-start-manual').addEventListener('click', async () => {
    if (manualStarting || powerBusy || session) return;
    const epoch = controlEpoch;
    const check = () => {
      if (epoch !== controlEpoch || document.hidden) throw new Error('手动控制准备已取消');
      if (hardware && !$('twin-onsite').checked) throw new Error('请先勾选现场确认');
    };
    manualStarting = true; updateControls();
    try {
      await prepareProgramControl(check); check();
      if (hardware && feedback?.hardware_phase === 'READONLY') await enablePowerHold(check);
      check();
      manualRecovery.arm();
      tell(hardware ? '使能保持已请求。请选择下方手动方式；键盘按住空格、手柄按住 RB / R1 才移动。' : '已接管仿真。请选择下方手动方式开始操作。');
    } catch (error) {
      if (epoch === controlEpoch) {
        if (session) await suspendAll('手动控制准备未完成，暂停并保持');
        tell(error.message,true);
      }
    } finally { manualStarting = false; updateControls(); }
  });
  $('twin-release').addEventListener('click', () => void suspendAll());
  $('twin-suspend')?.addEventListener('click', () => void suspendAll());
  $('twin-stop').addEventListener('click', () => void stopAll());
  $('twin-hold').addEventListener('click', () => { manualRecovery.cancel();hold(); invalidatePlan(); });
  for (const id of ['twin-onsite','ros-control-enable']) $(id).addEventListener('change', () => {
    if (!hardwareAllowed()) manualRecovery.cancel();
    if (session && !hardwareAllowed()) void suspendAll('实机控制确认已撤回，暂停并保持');
  });

  document.querySelectorAll('[data-twin-mode]').forEach((button) => button.addEventListener('click', () => {
    if (session) hold('已切换输入方式，保持当前位置');
    mode = button.dataset.twinMode; gamepadIndex = null; gamepadReady = false;
    manualRecovery.cancel();manualError = ''; clearInput();
    document.querySelectorAll('[data-twin-mode]').forEach((item) => item.setAttribute('aria-pressed',String(item === button)));
    document.querySelectorAll('[data-twin-panel]').forEach((panel) => { panel.hidden = panel.dataset.twinPanel !== mode; });
    showManualSpeed();
  }));
  fields.forEach((field) => field.addEventListener('input',invalidatePlan));
  function syncOrientation() {
    const constraint = $('twin-orientation').value;
    fields.slice(3).forEach((field) => { field.readOnly = constraint !== 'rpy'; });
    $('twin-orientation-help').textContent = constraint === 'free'
      ? '仅约束 XYZ，朝向由逆解选择；先观察目标虚影再执行。'
      : constraint === 'keep' ? '保持计算时的实测朝向；靠近关节限位时，部分平移方向可能无法保持该姿态。'
        : '同时约束 XYZ 和输入的 Roll / Pitch / Yaw；不会自动放宽姿态要求。';
  }
  $('twin-orientation').addEventListener('change',() => { syncOrientation(); invalidatePlan(); });
  syncOrientation();
  $('twin-position-only-retry').addEventListener('click',() => {
    $('twin-orientation').value = 'free'; syncOrientation(); invalidatePlan(); $('twin-preview').click();
  });
  $('twin-joint-speed').addEventListener('input',() => { invalidatePlan(); showJointSpeed(); });
  $('twin-joint-speed').addEventListener('change',() => { invalidatePlan(); showJointSpeed(); });
  $('twin-fill-pose').addEventListener('click', () => {
    if (!feedback?.pose || performance.now()-receivedAt > 500) { tell('请先连接并等待反馈',true); return; }
    fields.forEach((field,i) => { field.value = (feedback.pose[i] * (i<3 ? 1000 : 180/Math.PI)).toFixed(1); });
    invalidatePlan();
  });
  $('twin-preview').addEventListener('click', async () => {
    if (resetting) return;
    previewBusy = true; $('twin-preview').disabled = true; invalidatePlan();
    const revision = planRevision;
    try {
      if (!feedback || performance.now()-receivedAt > 500) throw new Error('请先连接并等待新鲜反馈');
      if (fields.some((field) => !field.value.trim() || !field.checkValidity())) throw new Error('请填写有效的位置和姿态数值');
      const pose = fields.map((field,i) => Number(field.value) * (i<3 ? 0.001 : Math.PI/180));
      const orientation = $('twin-orientation').value;
      if (orientation === 'keep') pose.splice(3,3,...feedback.pose.slice(3));
      const result = await request('preview', { pose,orientation:orientation !== 'free',speed:jointSpeed() });
      if (revision !== planRevision) return;
      plan = result.plan;
      if (orientation === 'free' && plan.solved_pose) fields.slice(3).forEach((field,i) => { field.value = (plan.solved_pose[i+3]*180/Math.PI).toFixed(1); });
      window.reBotSim?.setFeedbackTarget(angleMap(plan.joints));
      $('hub-monitor').open = true;
      const delta = Math.max(...plan.joints.map((value,i) => Math.abs(value-plan.start[i])))*180/Math.PI;
      const poseInfo = plan.solved_pose ? ` · 目标 RPY ${plan.solved_pose.slice(3).map((value) => (value*180/Math.PI).toFixed(1)).join(' / ')}°` : '';
      $('twin-preview-info').textContent = `预计 ${plan.duration.toFixed(1)} 秒 · 最大关节变化 ${delta.toFixed(1)}° · IK 位置残差 ${(Math.hypot(...plan.residual.slice(0,3))*1000).toFixed(2)} mm${poseInfo} · 预览 30 秒内有效`;
      tell('预览已生成。确认虚影位置和周围空间后，点击「执行预览轨迹」。');
    } catch (error) { if (revision === planRevision) { tell(error.message,true); $('twin-position-only-retry').hidden = !error.diagnostic?.position_only_possible; } }
    finally { previewBusy = false; updateControls(); }
  });
  $('twin-go').addEventListener('click', async () => {
    if (resetting || !session || !plan || goalBusy) return;
    const epoch = controlEpoch;
    const id = plan.id; goalBusy = true; $('twin-go').disabled = true;
    try { const result = await request('goal',{plan_id:id}); if (epoch === controlEpoch) { tell(result.message); plan = null; } }
    catch (error) { if (epoch === controlEpoch) { tell(error.message,true); invalidatePlan(); } }
    finally { goalBusy = false; updateControls(); }
  });

  window.addEventListener('keydown',(event) => {
    if (event.code === 'Escape') { event.preventDefault(); void stopAll(); return; }
    if (mode !== 'keyboard' || editing() || event.ctrlKey || event.altKey || event.metaKey) return;
    if (['BracketLeft','BracketRight'].includes(event.code)) {
      event.preventDefault();if(!event.repeat)adjustManualSpeed(event.code==='BracketRight'?10:-10);return;
    }
    if (!session || !(event.code === 'Space' || input.keys[event.code] || ['KeyQ','KeyE','ShiftLeft','ShiftRight'].includes(event.code))) return;
    event.preventDefault();
    // Rejection or focus changes clear keys. OS repeats must not restart motion.
    if (event.repeat && !pressed.has(event.code)) return;
    manualError = ''; pressed.add(event.code);
    if (['ShiftLeft','ShiftRight'].includes(event.code)) showManualSpeed();
  });
  window.addEventListener('keyup',(event) => {
    pressed.delete(event.code);
    if (['ShiftLeft','ShiftRight'].includes(event.code)) showManualSpeed();
    if (mode === 'keyboard' && event.code === 'Space' && jogActive) hold('空格已松开：保持当前位置');
  });
  document.addEventListener('focusin',() => {
    if (editing() && mode === 'keyboard') {
      if (jogActive) hold('编辑期间暂停键盘控制');
      else clearInput();
    }
  });
  $('twin-keyboard-focus').addEventListener('click',() => {
    $('twin-keyboard-focus').focus();
    if (jogActive) hold('已暂停，请重新按住空格和方向键');
    else clearInput();
  });
  for (const button of document.querySelectorAll('#twin-jog-buttons [data-axis],#twin-tcp-rotation [data-axis]')) {
    button.addEventListener('pointerdown',(event) => {
      if (!session || !['keyboard','tcp'].includes(mode) || event.button !== 0) return;
      event.preventDefault();
      if(mode==='tcp')hold('切换到TCP姿态微调，旧拖拽目标已取消');
      clearInput(); manualError = '';
      pointer = { axis:Number(button.dataset.axis),sign:Number(button.dataset.sign),id:event.pointerId };
      button.classList.add('is-held'); button.setPointerCapture(event.pointerId);
    });
    const release = (event) => { if (pointer?.id === event.pointerId) hold('按钮已松开：保持当前位置'); };
    for (const name of ['pointerup','pointercancel','lostpointercapture']) button.addEventListener(name,release);
  }
  window.addEventListener('blur',() => { manualRecovery.cancel(); if (session || acquiring) void suspendAll('页面失去焦点，暂停并保持；返回后请重新接管'); });
  document.addEventListener('visibilitychange',() => { if(document.hidden)manualRecovery.cancel();if (document.hidden && (session || acquiring)) void suspendAll('页面已隐藏，暂停并保持'); });
  window.addEventListener('pagehide',() => {
    manualRecovery.cancel();
    acquireEpoch++;
    if (session) recordStopRequest('页面关闭或离开，暂停并保持');
    if (session && client.connected) { try { publish(hardware&&controlRange==='full'?'suspend':'release',{reason:'页面关闭或离开，暂停并保持'}); } catch (_) {} }
    forgetSession();
  });
  window.addEventListener('gamepaddisconnected',(event) => {
    manualRecovery.cancel();
    if (mode === 'gamepad' && event.gamepad.index === gamepadIndex) { if (session) void suspendAll('手柄断开，暂停并保持'); gamepadIndex = null; }
  });
  for (const name of ['rebot-leader-starting','rebot-control-mode-change']) window.addEventListener(name,() => { manualRecovery.cancel();if (session) void suspendAll('控制入口已改变，暂停并保持'); });
  for (const id of ['ros-disable','ros-safe-home','ros-gravity-start']) $(id).addEventListener('click',() => { if (session) void stopAll('请在释放完成后操作高级控制'); },true);

  client.subscribe(`${base}/state`,'std_msgs/msg/String',(message) => {
    try {
      const state = JSON.parse(message.data);
      if (!validJoints(state.actual) || !Array.isArray(state.pose) || !state.pose.every(Number.isFinite)) return;
      const firstFeedback = !receivedAt;
      feedback = state; receivedAt = performance.now();
      if(hardware && !gripperProbeRequested && !state.extensions?.gripper?.available && state.hardware_phase==='READONLY' && !state.owned) {
        gripperProbeRequested=true;void request('gripper_probe').catch(error=>tell(error.message,true));
      }
      showMotors(state);
      window.dispatchEvent(new CustomEvent('rebot-twin-state',{detail:state}));
      $('twin-smoothing-status').textContent = state.jog_smoothing?.enabled
        ? state.motion_profile?.selected === 'ruckig' ? 'Ruckig已接入 · 连续S曲线 / 速度、加速度与jerk约束' : '原方案平滑 · 五次到点 / 渐进加速 / 回中减速'
        : '控制器未确认手动平滑，请重新连接新版服务';
      if (jogActive && neutralSequence !== null && state.mode === 'HOLD' && state.input_seq >= neutralSequence) {
        jogActive = false; neutralSequence = null;
      }
      if (state.reset_status && (state.reset_status.id === resetJob || ['queued','running'].includes(state.reset_status.status))) {
        if (!resetJob || state.reset_status.id !== resetJob) { resetJob = state.reset_status.id; resetLocalControls(); }
        showReset(state.reset_status);
      }
      if (resetAwaitingPose && resetJob && state.reset_status?.id === resetJob &&
          ['succeeded','failed'].includes(state.reset_status.status) && state.feedback_valid !== false) {
        fields.forEach((field,i) => { field.value = (state.pose[i] * (i<3 ? 1000 : 180/Math.PI)).toFixed(1); });
        resetAwaitingPose = false;
      }
      if (firstFeedback) tell('反馈已连接，可只读观察；接管后可使用坐标、键盘或手柄控制。');
      if (lastStateAt) stateIntervals = [...stateIntervals.slice(-31),receivedAt-lastStateAt];
      lastStateAt = receivedAt;
      const renderTo = angleMap(state.actual);
      if (state.extensions?.gripper?.fresh && Number.isFinite(state.extensions.gripper.position)) renderTo.gripper = Math.max(0,Math.min(5,state.extensions.gripper.position))/5*.0715;
      if (state.feedback_valid === false) display.reset();
      else display.push(renderTo,receivedAt,Math.ceil((Number(state.control_settings?.poll_duration_ms)||30)/20)*20+20);
      window.reBotTwinDisplay = window.reBotControlMode?.is('ros') !== false;
      if (session && !state.owned && performance.now()-acquiredAt > 200) { manualRecovery.lost(state); forgetSession(); window.dispatchEvent(new CustomEvent('rebot-twin-suspended')); tell(state.message,true); }
      if (!session && manualRecovery.awaitingHold) manualRecovery.lost(state);
      // A 20 Hz status must not immediately erase preview/validation results.
      if (session && !jogActive && state.message !== lastControllerMessage) tell(state.message);
      if (!resetting && state.recovery_required && state.message !== lastControllerMessage) tell(state.message,true);
      if (state.hardware_phase === 'HOLDING' && state.message !== lastControllerMessage) {
        const reason=state.communication_hold?.reason;
        tell(reason && !state.message.includes(reason) ? `${state.message}（触发原因：${reason}）` : state.message,true);
      }
      if ((['FEEDBACK_WAIT','FAULT'].includes(state.hardware_phase) || lastControllerPhase==='FEEDBACK_WAIT') && state.message !== lastControllerMessage) tell(state.message,true);
      lastControllerMessage = state.message;
      lastControllerPhase = state.hardware_phase;
      if (stopping && !state.owned && !state.control_prepared && state.hardware_phase === 'READONLY' && state.cleanup_ok !== false) {
        stopping = false; tell(state.cleanup_ok === true ? '已停止并失能；已核对停稳，临时参数已恢复。' : '已停止，仅保留只读反馈；未启动电机出力。');
      }
      if (state.goal) window.reBotSim?.setFeedbackTarget(angleMap(state.goal));
      $('twin-tracking-error').textContent = `${Number(state.joint_error_deg).toFixed(2)}°`;
      const positionError = state.position_error_mm ?? state.last_result?.position_error_mm;
      const rotationError = state.rotation_error_deg ?? state.last_result?.rotation_error_deg;
      $('twin-position-error').textContent = positionError == null ? '—' : `${positionError.toFixed(2)} mm`;
      $('twin-rotation-error').textContent = rotationError == null || state.last_result?.orientation_constrained === false ? '—' : `${rotationError.toFixed(2)}°`;
      updateControls();
    } catch (error) { console.warn('[twin state]',error); }
  },{throttleRate:50});
  client.subscribe(`${base}/events`,'std_msgs/msg/String',(message) => {
    try {
      const result = JSON.parse(message.data), wait = pending.get(result.id);
      if (wait) {
        clearTimeout(wait.timer); pending.delete(result.id);
        if (result.ok) wait.resolve(result);
        else { const error = new Error(result.message); error.diagnostic = result.diagnostic; wait.reject(error); }
      }
      else if (!result.ok && session) { clearInput(); manualError = result.message; tell(result.message,true); }
    } catch (_) {}
  },{throttleRate:1});
  client.addEventListener('status',(event) => {
    if (event.detail.state === 'closed' || event.detail.state === 'error') {
      if (resetting) {
        resetting = false;
        $('twin-reset-status').textContent = '重置期间连接中断，实机结果未确认；重新连接反馈后查看重置结果。';
        $('twin-reset-status').dataset.error = 'true';
      }
      manualRecovery.cancel();gripperProbeRequested=false; acquireEpoch++; forgetSession(); receivedAt = 0; display.reset(); lastStateAt = 0; stateIntervals = []; window.reBotTwinDisplay = false;
      for (const wait of pending.values()) { clearTimeout(wait.timer); wait.reject(new Error('ROS 连接已断开')); } pending.clear();
      tell('ROS 已断开；不会自动恢复运动，重连后请重新接管。',true);
    }
    updateControls();
  });

  function updateControls() {
    const fresh = client.connected && receivedAt && performance.now()-receivedAt < 500 && feedback?.feedback_valid !== false;
    showControlRange(feedback?.control_settings);
    for(const id of ['twin-joint-speed','twin-linear-speed','twin-angular-speed']) $(id).disabled = resetting || acquiring || Boolean(session && ['GOAL','JOG'].includes(feedback?.mode));
    $('twin-reset-control').textContent = resetting ? '↻ 正在重置…' : '↻ 重置控制状态';
    $('twin-reset-control').disabled = resetting;
    $('twin-connection').textContent = fresh ? (hardware ? '实机反馈已连接' : '仿真反馈已连接') : (client.connected ? '等待统一控制器反馈' : 'ROS 未连接');
    $('twin-connection').dataset.ok = String(Boolean(fresh));
    $('twin-acquire').disabled = changingRange || changingMotionProfile || acquiring || !!session || !fresh || Boolean(feedback?.owned) || Boolean(hardware && (!hardwareAllowed() || !feedback?.control_prepared));
    $('twin-release').disabled = !session;
    $('twin-go').disabled = goalBusy || !session || !plan || !fresh || feedback?.mode === 'GOAL';
    $('twin-preview').disabled = changingRange || changingMotionProfile || previewBusy || Boolean(feedback?.recovery_required) || feedback?.mode === 'GOAL' || Boolean(feedback?.owned && !session && fresh);
    $('twin-target').disabled = client.connected || acquiring || connectBusy || client.socket?.readyState === 0;
    $('twin-connect').textContent = client.connected ? '断开 ROS' : hardware ? '连接实机反馈' : '启动并连接仿真';
    const disconnectLocked = hardware && client.connected && ['ACTIVE','HOLDING','STARTING','STOPPING','RESETTING'].includes(feedback?.hardware_phase);
    $('twin-connect').disabled = connectBusy || manualStarting || disconnectLocked || client.socket?.readyState === 0;
    $('twin-connect').title = disconnectLocked ? '机械臂正在出力或处理停止；暂停请用顶部「暂停并保持」。支撑后明确失能，再断开服务。' : '';
    $('twin-start-manual').disabled = manualStarting || powerBusy || resetting || changingRange || changingMotionProfile || !fresh || !!session || Boolean(feedback?.owned) || Boolean(feedback?.recovery_required) || Boolean(hardware && (!$('twin-onsite').checked || !['READONLY','HOLDING'].includes(feedback?.hardware_phase)));
    $('twin-start-manual').textContent = manualStarting ? '正在准备控制…' : session ? '本页已接管控制' : hardware ? (feedback?.hardware_phase === 'HOLDING' ? '重新接管手动控制' : '使能并进入手动控制') : '开始仿真控制';
    const phases = {CONNECTING:'连接中',FEEDBACK_WAIT:'只读反馈重试 · 禁止使能',RESETTING:'正在重置 · 不使能、不回零',READONLY:'只读观察 · 未发起使能',RECOVERY:'仅观察 · 停止/参数恢复待核查',STARTING:'正在准备驱动',ACTIVE:`${feedback?.motion_profile?.backend || 'CSP'} ${controlRange === 'full' ? '完整' : '小幅'}控制中`,HOLDING:'通信异常保持 · 禁止自动续跑',STOPPING:'正在失能与恢复参数',FAULT:'停止检查未通过'};
    $('twin-service-status').textContent = client.connected ? (fresh ? (hardware ? (phases[feedback.hardware_phase] || '实机反馈就绪') : '已就绪 · 接管前可只读观察') : '等待统一控制器反馈') : hardware ? '连接不会使能机械臂' : '首次启动约需数秒';
    $('twin-enable-hardware').disabled = changingRange || changingMotionProfile || !fresh || !hardwareAllowed() || Boolean(feedback?.control_prepared) || feedback?.hardware_phase !== 'READONLY';
    $('twin-power-hold').disabled = powerBusy || !session || !fresh || !hardwareAllowed() || feedback?.hardware_phase !== 'READONLY';
    const power = {FEEDBACK_WAIT:'只读反馈重试 · 禁止使能',RESETTING:'正在失能并重置',READONLY:feedback?.control_prepared ? (session ? '已接管，尚未使能' : '已准备，尚未使能') : '未使能 · 只读',RECOVERY:'恢复未确认 · 禁止使能',STARTING:'正在配置并使能',ACTIVE:'CSP 已启动 · 使能已下发',HOLDING:fresh && feedback?.communication_hold?.feedback_confirmed?'原位置保持 · 反馈恢复，需重新接管':'保留原位置目标 · 反馈未确认',STOPPING:'正在失能与核对',FAULT:'异常 · 请检查停止结果'};
    $('twin-power-state').textContent = `电机：${fresh || ['HOLDING','FEEDBACK_WAIT'].includes(feedback?.hardware_phase) ? (power[feedback?.hardware_phase] || '等待状态') : '未获取新鲜状态'}`;
    $('twin-power-state').dataset.ok = String(Boolean(fresh && feedback?.hardware_phase === 'ACTIVE'));
    $('twin-power-state').dataset.warning = String(Boolean(fresh && feedback?.recovery_required));
    $('twin-control-range').disabled = !fresh || changingRange || changingMotionProfile || acquiring || !!session || Boolean(feedback?.owned) || Boolean(feedback?.control_prepared) || feedback?.hardware_phase !== 'READONLY' || !feedback?.control_settings;
    $('twin-motion-profile').disabled = !fresh || changingMotionProfile || !!session || Boolean(feedback?.owned) || Boolean(feedback?.control_prepared) || !feedback?.motion_profile || Boolean(hardware && (feedback.hardware_phase !== 'READONLY' || controlRange !== 'full'));
    if (!changingMotionProfile && feedback?.motion_profile) $('twin-motion-profile').value = feedback.motion_profile.selected;
    for (const id of ['motion-apply','motion-default','motion-acceleration','motion-jerk']) $(id).disabled = $('twin-motion-profile').disabled || !feedback?.motion_profile?.profiles;
    const motion = feedback?.motion_profile, params = motion?.profiles?.[motion.selected];
    const draftKey = params ? JSON.stringify([motion.selected, params]) : null;
    if (!changingMotionProfile && params && motionDraftKey !== draftKey) {
      $('motion-acceleration').value = params.acceleration_scale*100;
      $('motion-jerk').value = params.jerk_scale*100;
      motionDraftKey = draftKey;
    }
    $('motion-effective').textContent = params ? `已生效：加速度 ${Number(motion.effective_acceleration).toFixed(3)} rad/s² · jerk ${Number(motion.effective_jerk).toFixed(3)} rad/s³ · 参数按方案保存；不改变速度档位` : '等待支持调参的控制器；尚未应用参数';
    const drive=motion?.driver,driveKey=drive?JSON.stringify([drive.selected,drive.parameters]):null;
    if(!changingMotionProfile&&drive&&driveKey!==driveDraftKey){
      $('drive-backend').value=drive.selected;
      for(const g of ['kp','kd'])drive.parameters[g].forEach((v,i)=>{$(`drive-${g}${i+1}`).value=v;});
      driveDraftKey=driveKey;
    }
    const driveDisabled=$('twin-motion-profile').disabled||!drive;
    for(const id of ['drive-backend','drive-apply','drive-default',...['kp','kd'].flatMap(g=>Array.from({length:6},(_,i)=>`drive-${g}${i+1}`))])$(id).disabled=driveDisabled;
    $('drive-mit-gains').hidden=$('drive-backend').value!=='mit';
    const driveDirty=drive&&(drive.selected!==$('drive-backend').value||JSON.stringify(drive.parameters)!==JSON.stringify(driveParams()));
    $('drive-status').textContent=drive?`已生效：${drive.selected.toUpperCase()} · ${driveDirty?'表单有未应用修改 · ':''}MIT实机验收待完成；重连默认CSP`:'当前仿真不模拟MIT电机动力学';
    $('twin-motion-profile-status').textContent = feedback?.motion_profile
      ? `${feedback.motion_profile.selected === 'ruckig' ? '当前：Ruckig连续S曲线' : '当前：原方案'} · ${motion.backend||'仿真'} · 方案切换后须重新检查路径`
      : '等待新版控制器报告运动方案';
    showJointSpeed();
    $('twin-owner-state').textContent = session ? ({GOAL:'到点运动中',JOG:'手动控制中',HOLD:'已接管 · 保持'}[feedback?.mode] || '已接管') : feedback?.owned && fresh ? '其他控制页占用中' : '仅观察';
    $('twin-owner-state').dataset.ok = String(!!session);
    $('twin-feedback-source').textContent = fresh ? (feedback.feedback_source || (hardware ? '/rebotarm 编码器' : '/rebotarm_rs 仿真驱动')) : '等待新鲜反馈';
    if (resetting) for (const id of ['twin-acquire','twin-go','twin-preview','twin-target','twin-enable-hardware','twin-power-hold','twin-control-range','twin-motion-profile']) $(id).disabled = true;
  }
  function showKeyboardStatus() {
    if (mode !== 'keyboard') return;
    let message;
    const fresh = client.connected && receivedAt && performance.now()-receivedAt < 500 && feedback?.feedback_valid !== false;
    if (!fresh) message = '未收到新鲜反馈：请先连接控制器';
    else if (!session) message = feedback?.message ? `尚未接管：${feedback.message}；请准备并接管控制` : '尚未接管：请准备并接管控制';
    else if (manualError) message = `控制器拒绝：${manualError}。松开按键后再试，或先调整姿态。`;
    else if (editing()) message = '正在编辑输入框，键盘控制已暂停；点击上方按钮后再按键';
    else if (feedback?.mode === 'GOAL') message = '坐标轨迹执行中；先点「暂停」再用键盘';
    else if (feedback?.hardware_phase === 'STARTING') message = '正在配置并使能：等待控制器确认';
    else if (pointer) message = '已识别方向按钮，等待 / 接收控制器反馈';
    else if (!pressed.has('Space')) message = '等待空格：按住空格，再按 W / S、A / D 或 R / F';
    else if (!input.keyboard(pressed).some(value => Math.abs(value) > .001) && !input.keyboardGripper(pressed)) message = '空格已识别：继续按方向键；松方向键平滑减速，松空格保持';
    else message = feedback?.mode === 'JOG' ? '控制器已接收手动输入；实色模型显示实机反馈' : '方向键已识别，等待控制器接受；若被拒绝会显示原因';
    $('twin-keyboard-status').textContent = message;
    $('twin-keyboard-status').dataset.error = String(Boolean(manualError) || !fresh);
  }
  function sampleGamepad() {
    const blocked = {values:[0,0,0,0,0,0],enabled:false};
    let pads = [];
    try {
      if (!navigator.getGamepads) throw new Error('当前浏览器不支持 Gamepad API');
      pads = [...navigator.getGamepads()].filter((pad) => pad && pad.connected !== false);
    } catch (_) {
      window.ReBotTwinGuides?.showPad(null);
      $('twin-gamepad-name').textContent = '当前浏览器无法读取游戏手柄';
      $('twin-gamepad-state').textContent = '请使用支持 Gamepad API 的浏览器';
      return blocked;
    }
    let pad = pads.find((item) => item.index === gamepadIndex);
    if (!pad && pads.length && gamepadIndex === null) { pad = pads[0]; gamepadIndex = pad.index; gamepadReady = false; }
    window.ReBotTwinGuides?.showPad(pad);
    if (!pad) { $('twin-gamepad-name').textContent = '等待手柄 · 按任意键唤醒'; $('twin-gamepad-state').textContent = '未连接'; return blocked; }
    $('twin-gamepad-name').textContent = pad.id;
    if (pad.mapping !== 'standard') { $('twin-gamepad-state').textContent = '非标准映射，暂不可控制'; return blocked; }
    const sample = input.gamepad(pad);
    const adjusting=Number(pad.buttons[2]?.value)>.5||Number(pad.buttons[3]?.value)>.5;
    if (sample.neutral && !adjusting) gamepadReady = true;
    const speedChange=speedShortcut.sample(pad,gamepadReady);
    if(speedChange)adjustManualSpeed(speedChange*10);
    $('twin-gamepad-state').textContent = !gamepadReady ? '先松开摇杆和 RB' : sample.enabled ? 'RB 已按住' : '就绪 · 按住 RB';
    const values = gamepadReady ? sample.values : [0,0,0,0,0,0];
    showPadAction(pad,sample);
    $('twin-gamepad-values').textContent = values.map((v,i) => `${['X','Y','Z','R','P','Y'][i]} ${v.toFixed(2)}`).join(' · ');
    return {values,gripper:gamepadReady?sample.gripper:0,enabled:gamepadReady && sample.enabled,neutral:sample.neutral};
  }
  $('twin-render-quality').addEventListener('change',() => window.reBotSim?.setRenderQuality($('twin-render-quality').value));
  $('twin-display-mode').addEventListener('change',() => {
    display.reset();
    if (feedback?.feedback_valid !== false && receivedAt && performance.now()-receivedAt < 500) display.push(angleMap(feedback.actual),performance.now());
  });
  function showPadAction(pad,sample) {
    const out=$('twin-gamepad-action');if(!out)return;
    const button=i=>Number(pad.buttons[i]?.value)||0;
    const g=feedback?.extensions?.gripper||{};
    const gripSpeed=currentManualSpeeds().gripper;
    const scale=button(4)>.5?.25:1;
    const actions=[];
    if(button(4)>.5)actions.push('LB 精细：当前速度 × 25%');
    if(button(2)>.5||button(3)>.5)actions.push('X减速 / Y加速，每次10%');
    if(button(6)>.16||button(7)>.16)actions.push(`LT/RT 绕基座X轴滚转 ${sample.values[3]>0?'+':'−'}${Math.abs(sample.values[3]*Number($('twin-angular-speed').value)).toFixed(2)}°/s`);
    if(button(0)>.5||button(1)>.5)actions.push(`J7 ${button(0)>button(1)?'打开':'闭合'} · ${(gripSpeed*scale).toFixed(2)} rad/s`);
    const permission=!sample.enabled?'按住 RB 才会执行':!session?'尚未接管':!gamepadReady?'先松开所有按键':feedback?.hardware_phase!=='ACTIVE'?'等待使能保持':'手动控制就绪';
    const gripState=g.error?`J7异常：${g.error}`:!g.fresh?'J7反馈未就绪':g.active?'J7已使能': 'J7待命，首次A/B开合时自动使能';
    out.textContent=`${permission} · ${actions.join(' · ')||'LB是按住慢速；RB+LT/RT滚转；RB+A/B开合'} · ${gripState}`;
  }
  async function recoverManual() {
    manualRecovery.begin(performance.now());
    const epoch=controlEpoch;
    const guard=()=>{if(epoch!==controlEpoch||document.hidden||!hardwareAllowed()||!$('twin-auto-manual-recovery').checked)throw new Error('自动恢复已取消');};
    try {
      // Only take over the already-powered hold. Never prepare/power_hold here.
      await acquireControl(guard);guard();
      const deadline=performance.now()+3000;
      while(session&&feedback?.hardware_phase==='HOLDING') {
        guard();if(performance.now()>deadline)throw new Error('保持接管未确认');
        await new Promise(resolve=>setTimeout(resolve,50));
      }
      if(!session||feedback?.hardware_phase!=='ACTIVE')throw new Error(feedback?.message||'保持接管未通过');
      manualRecovery.finish(true);
      tell('通信恢复，已恢复手动就绪；松开后重新按住空格 / RB，旧动作已清空。');
    } catch(error) {
      manualRecovery.finish(false);
      $('twin-recovery-state').textContent=`恢复检查 ${manualRecovery.attempts}/3 未通过：${error.message}`;
    }
  }
  $('twin-auto-manual-recovery').addEventListener('change',()=>{if(!$('twin-auto-manual-recovery').checked)manualRecovery.cancel();});
  $('twin-gripper-speed').addEventListener('input',()=>{$('twin-gripper-speed-value').textContent=Number($('twin-gripper-speed').value).toFixed(2)+' rad/s';showManualSpeed();});
  function pollConnection(now) {
    if (mode === 'gamepad' && !session && !document.hidden) sampleGamepad();
    const timing = $('twin-display-timing');
    if (timing) timing.textContent = receivedAt && now-receivedAt < 500
      ? `近${stateIntervals.length}帧状态间隔 ${stateIntervals.length ? Math.round(stateIntervals.reduce((a,b)=>a+b,0)/stateIntervals.length) : '—'} ms / 最长 ${stateIntervals.length ? Math.round(Math.max(...stateIntervals)) : '—'} ms · CAN查询一轮 ${Number.isFinite(feedback?.control_settings?.poll_duration_ms) ? feedback.control_settings.poll_duration_ms.toFixed(0) : '—'} ms · 画面 ${fps} fps`
      : '无新鲜反馈 · 模型停止更新';
    const age = receivedAt ? now-receivedAt + Number(feedback?.feedback_age_ms || 0) : Infinity;
    $('twin-feedback-age').textContent = Number.isFinite(age) ? `${Math.round(age)} ms` : '—';
    if (client.connected && receivedAt && (now-receivedAt > 500 || feedback?.feedback_valid === false)) {
      window.reBotTwinDisplay = false; updateControls();
      if (session) void suspendAll(now-receivedAt > 500
        ? `网页连续 ${Math.round(now-receivedAt)} ms 未收到 ROS 控制状态，请求暂停并保持`
        : `控制器报告实机反馈过期（${Math.round(Number(feedback?.feedback_age_ms || 0))} ms），请求暂停并保持`);
    }
    if (manualRecovery.pending && feedback) {
      const neutral=mode==='gamepad'?Boolean(sampleGamepad()?.neutral):pressed.size===0&&!pointer;
      const allowed=hardware&&controlRange==='full'&&hardwareAllowed()&&!document.hidden&&$('twin-auto-manual-recovery').checked&&['keyboard','gamepad','tcp'].includes(mode);
      const fresh=client.connected&&now-receivedAt<500&&feedback.feedback_valid===true;
      if(manualRecovery.ready(feedback,now,{fresh,neutral,allowed}))void recoverManual();
      if(manualRecovery.pending&&!manualRecovery.busy)$('twin-recovery-state').textContent=manualRecovery.attempts>=3?'恢复检查未通过，请查看原因后手动接管。':!neutral?'等待松开空格 / RB 和全部摇杆；恢复不会续跑旧动作。':'正在采集反馈，连续正常500ms后自动恢复手动就绪。';
    } else if (!manualRecovery.busy) $('twin-recovery-state').textContent='通信短暂过期时保持原目标；反馈恢复且输入松开后自动恢复手动就绪。程序需明确继续。';
    // Control liveness must not depend on WebGL animation frames. A suspended
    // render loop can coexist with fresh socket messages and visible controls.
    // This timer still shares the page event loop: a genuinely stalled/hidden
    // page loses its lease through the unchanged controller watchdog.
    // Commands use a depth-one ROS queue. Do not overwrite an unaccepted
    // preview with a heartbeat; a fresh compiling state confirms receipt.
    if (session && client.connected && !document.hidden && now-lastHeartbeatAt >= 150 &&
        (!pending.size || [...pending.values()].every(p=>
          ['program_preview','support_preview','program_resume'].includes(p.op) &&
          feedback?.extensions?.program_compiling === true && receivedAt >= p.sentAt))) {
      try { publish('heartbeat'); lastHeartbeatAt = now; }
      catch (error) { forgetSession(); tell(error.message,true); }
    }
  }
  function sampleManualInput(now) {
    if (!(session && client.connected && !document.hidden)) return;
    if(manualRecovery.busy)return;
    if(manualRecovery.waitingActive){if(feedback?.hardware_phase!=='ACTIVE')return;manualRecovery.waitingActive=false;clearInput();return;}
    const pad = mode === 'gamepad' ? sampleGamepad() : null;
    let values = pad ? pad.values : mode === 'keyboard' && !editing() ? input.keyboard(pressed) : [0,0,0,0,0,0];
    let gripper=pad?.gripper || (mode==='keyboard' && !editing() ? input.keyboardGripper(pressed):0);
    if (pointer && ['keyboard','tcp'].includes(mode)) { values = [0,0,0,0,0,0]; if(pointer.axis===6)gripper=pointer.sign;else values[pointer.axis] = pointer.sign; }
    const moving = values.some((value) => Math.abs(value)>0.001) || Math.abs(gripper)>.001;
    const held = pad ? pad.enabled : !editing() && ((mode==='keyboard' && pressed.has('Space')) || pointer);
    try {
      if ((moving || (jogActive && held)) && feedback?.mode !== 'GOAL') {
        if (!jogActive) invalidatePlan();
        if (moving && held) manualRecovery.arm();
        const speeds=currentManualSpeeds();
        const twist = input.twist(values,speeds.linear/1000,speeds.angular*Math.PI/180);
        publish('jog',{twist,speed:speeds.joint,gripper_velocity:gripper*speeds.gripper});
        if (moving) neutralSequence = null;
        else if (neutralSequence === null) neutralSequence = sequence;
        jogActive = true; lastHeartbeatAt = now;
      } else if (jogActive) hold('控制已松开：保持当前位置');
    } catch (error) { forgetSession(); tell(error.message,true); }
  }
  function frame(now) {
    showKeyboardStatus();
    frameCount++;
    if (now-fpsAt >= 1000) { fps = Math.round(frameCount*1000/(now-fpsAt)); frameCount=0;fpsAt=now; }
    const angles = display.sample(now,$('twin-display-mode')?.value === 'raw');
    if (window.reBotTwinDisplay && angles && feedback?.feedback_valid !== false && window.reBotSim) {
      window.reBotSim.setAngles(angles,{source:'ros',forceFeedback:true,preserveGhost:true});
    }
    requestAnimationFrame(frame);
  }
  function showManualSpeed() {
    showJointSpeed();
    for(const name of ['linear','angular']) $(`twin-${name}-value`).textContent=`${Number($(`twin-${name}-speed`).value).toFixed(1)} ${name==='linear'?'mm/s':'°/s'}`;
    $('twin-speed-summary').textContent=`坐标到点、键盘、手柄及姿态微调：关节 ≤ ${Number($('twin-joint-speed').value).toFixed(3)} rad/s · 平移 ${$('twin-linear-speed').value} mm/s · 旋转 ${$('twin-angular-speed').value}°/s。TCP拖动选点自动调速，程序各步speed独立。`;
    let speeds;
    try { speeds=currentManualSpeeds(); } catch (_) { $('twin-manual-effective').textContent='请先输入有效的基础速度';return; }
    $('twin-manual-override-value').textContent=`${$('twin-manual-override').value}%`;
    $('twin-manual-effective').textContent=`键盘/手柄当前：平移 ${speeds.linear.toFixed(1)} mm/s · 旋转 ${speeds.angular.toFixed(1)}°/s · 关节上限 ${speeds.joint.toFixed(3)} rad/s · 夹爪上限 ${speeds.gripper.toFixed(2)} rad/s。关节硬上限保持页面设定；Shift / LB 按住微调，手柄摇杆轻推更慢。`;
  }
  function currentManualSpeeds() {
    const base={joint:jointSpeed(),linear:Number($('twin-linear-speed').value),angular:Number($('twin-angular-speed').value),gripper:Number($('twin-gripper-speed').value)};
    const limits={minimum:Number($('twin-joint-speed').min),joint:Number($('twin-joint-speed').max),linear:Number($('twin-linear-speed').max),angular:Number($('twin-angular-speed').max),gripper:feedback?.extensions?.gripper?.speed_max||0};
    return input.manualSpeeds(base,limits,['keyboard','gamepad'].includes(mode)?Number($('twin-manual-override').value):100,mode==='keyboard'&&(pressed.has('ShiftLeft')||pressed.has('ShiftRight')));
  }
  function adjustManualSpeed(delta) {
    const field=$('twin-manual-override');field.value=String(Math.max(10,Math.min(200,Number(field.value)+delta)));showManualSpeed();
  }
  $('twin-manual-override').addEventListener('input',showManualSpeed);
  for(const name of ['joint','linear','angular']) $(`twin-${name}-speed`).addEventListener('input',()=>{ invalidatePlan();showManualSpeed(); });
  function showMotors(state) {
    const g=state.extensions?.gripper||{},settings=state.control_settings||{};
    if(g.speed_max>0){const field=$('twin-gripper-speed');field.max=String(g.speed_max);field.value=String(Math.min(Number(field.value),g.speed_max));$('twin-gripper-speed-value').textContent=Number(field.value).toFixed(2)+' rad/s';}
    if(shownGripperSpeedMax!==g.speed_max){shownGripperSpeedMax=g.speed_max;showManualSpeed();}
    $('twin-gripper-time').textContent=g.speed_max>0?`J7设备上限 ${g.speed_max.toFixed(2)} rad/s；按此上限走满0–5rad至少约 ${(5/g.speed_max).toFixed(1)} 秒，另需加减速。这里是电机角度，尚未标定为夹口毫米。`:'等待J7反馈以确认实际限速；小幅模式不开放夹爪。';
    for(let i=1;i<=7;i++) {
      const m=state.motor_parameters?.[i];
      const q=i===7?g.position:state.actual[i-1];
      $(`twin-motor-q${i}`).textContent=Number.isFinite(q)?q.toFixed(3):'—';
      $(`twin-motor-limit${i}`).textContent=i===7?'0.00–5.00':m?`${m.lower.toFixed(2)}–${m.upper.toFixed(2)}`:'—';
      const cap=i===7?g.speed_max:settings.max_speed_rad_s;
      $(`twin-motor-speed${i}`).textContent=Number.isFinite(cap)&&cap>0?cap.toFixed(3):'—';
      const d=settings.motor_diagnostics?.[i],fmt=(value,digits=2)=>Number.isFinite(value)?value.toFixed(digits):'—';
      $(`twin-motor-current${i}`).textContent=i===7?'—':`${fmt(d?.current_a)} / ${fmt(settings.current_limits_a?.[i-1],0)}`;
      $(`twin-motor-torque${i}`).textContent=i===7?'—':fmt(settings.torque_limits_nm?.[i-1],0);
      $(`twin-motor-error${i}`).textContent=i===7?'—':`${fmt(d?.following_error_deg)}° / ${fmt(d?.position_age_ms,0)} ms`;
      $(`twin-motor-gains${i}`).textContent=i===7?'—':['0x701e','0x701f','0x7020','0x702b'].map(k=>Number.isFinite(d?.csp_gains?.[k])?String(d.csp_gains[k]):'—').join(' / ');
      $(`twin-motor-status${i}`).textContent=i===7?(g.recovery_required?'恢复待核查':g.fresh?(g.active?'CSP保持 / 运动':'只读'):'反馈未确认'):`模式 ${state.drive_modes?.[i]??'—'}`;
    }
    $('twin-gripper-status').textContent=g.error?`J7：${g.error}`:g.fresh?'J7已接入：空格 + Q/E 或 RB + A/B 开合，松开保持。':'J7：等待新鲜反馈；开合控制暂不可用。';
  }
  // Pending motion requests keep priority on the depth-one ROS command queue;
  // pollConnection waits for their acknowledgement before sending a heartbeat.
  setInterval(() => pollConnection(performance.now()),50);
  // requestAnimationFrame is intentionally reserved for rendering. Browsers
  // commonly run it at 60 Hz, which turns a 20 ms threshold into an uneven
  // 30 Hz stream. A dedicated timer gives manual input a stable ~50 Hz cadence
  // matching the full-mode reference target without owning the CAN bus.
  setInterval(() => sampleManualInput(performance.now()), MANUAL_INPUT_PERIOD_MS);
  updateControls(); requestAnimationFrame(frame);
  async function dragTo(target, {orientation=$('twin-drag-orientation')?.checked === true, check=()=>{}} = {}) {
    if (!validJoints(target)) throw new Error('TCP目标需要六个有效坐标');
    if (goalBusy || previewBusy || manualRecovery.busy || manualRecovery.waitingActive) throw new Error('正在检查目标或准备控制，请稍后重新拖动');
    const epoch=controlEpoch, revision=++dragRevision;
    const guard=()=>{
      check();
      if (epoch!==controlEpoch || revision!==dragRevision || resetting || document.hidden || mode!=='tcp' || !session || !hardwareAllowed()) throw new Error('TCP到点已取消');
      if (!client.connected || !receivedAt || performance.now()-receivedAt>=500 || feedback?.feedback_valid===false) throw new Error('反馈已过期，TCP目标未继续执行');
    };
    guard();
    const settings=feedback?.control_settings;
    const maximum=settings?.max_speed_rad_s ?? (hardware ? NaN : .3);
    const minimum=settings?.min_speed_rad_s ?? (hardware ? .005 : .05);
    if (!Number.isFinite(maximum) || !Number.isFinite(minimum) || maximum<minimum || minimum<=0 || minimum>.3) throw new Error('等待控制器确认有效的TCP到点速度范围');
    // A drop is an explicit point command. Reuse the checked IK/trajectory
    // executor, independent of the keyboard's deliberately slow tool speeds.
    const speed=Math.max(minimum,Math.min(.3,maximum)), pose=[...target];
    manualRecovery.cancel();clearInput();invalidatePlan();goalBusy=true;updateControls();
    try {
      await request('hold');guard();
      const preview=await request('preview',{pose,orientation,speed});guard();
      window.reBotSim?.setFeedbackTarget(angleMap(preview.plan.joints));
      const result=await request('goal',{plan_id:preview.plan.id});guard();
      tell(`TCP自动到点中 · 预计 ${preview.plan.duration.toFixed(1)} 秒 · 接近目标减速，停稳后保持`);
      return {...result,plan:preview.plan};
    } finally { goalBusy=false;updateControls(); }
  }
  window.ReBotTwin = {
    request, hold, stop:suspendAll, jointSpeed, prepareProgram:prepareProgramControl,
    previewJoints:() => plan?.joints ? [...plan.joints] : null,
    state:() => feedback,
    owned:() => Boolean(session),
    hardware,
    fresh:() => Boolean(client.connected && receivedAt && performance.now()-receivedAt < 500 && feedback?.feedback_valid !== false),
    drag:dragTo,
    message:tell,
  };
})();
