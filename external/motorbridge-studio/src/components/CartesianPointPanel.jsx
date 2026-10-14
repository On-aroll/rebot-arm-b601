import React from 'react';

async function request(path, body) {
  const response = await fetch(path, {
    method: body ? 'POST' : 'GET',
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
    signal: AbortSignal.timeout(30000),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

export function CartesianPointPanel({ mode = 'offline', feedback, previewReady, virtualEnabled, locked, active, moving, now,
  onPreview, onBusy, onExecuted, onPrepareHardwarePreview, onExecuteHardware, hardwareConfirmed = false, hardwareError,
  onStop, resetKey, readiness, currentXyz }) {
  const offline = mode === 'offline';
  const [xyz, setXyz] = React.useState(['200', '150', '200']);
  const [orientation, setOrientation] = React.useState('position');
  const [speed, setSpeed] = React.useState('0.15');
  const [hardwareSpeed, setHardwareSpeed] = React.useState('0.02');
  const [timeoutMode, setTimeoutMode] = React.useState('auto');
  const [timeoutSeconds, setTimeoutSeconds] = React.useState('5');
  const [feedforwardMode,setFeedforwardMode] = React.useState('none');
  const [plan, setPlan] = React.useState(null);
  const [submitted, setSubmitted] = React.useState(false);
  const [pending, setPending] = React.useState(false);
  const [message, setMessage] = React.useState('输入 XYZ，先预演，再执行。');
  const [capabilities, setCapabilities] = React.useState(null);
  const running = React.useRef(false);
  const motionOptions = capabilities?.hardware_motion_options;
  const currentSpeed = offline ? speed : hardwareSpeed;
  React.useEffect(() => {
    let alive = true;
    request('/point-capabilities').then(value => { if (alive) {
      setCapabilities(value);
      if (value.hardware_motion_options) {
        setHardwareSpeed(String(value.hardware_motion_options.default_speed_rad_s));
        setTimeoutSeconds(String(value.hardware_motion_options.default_timeout_s));
      }
    } })
      .catch(() => { if (alive) setCapabilities(null); });
    return () => { alive = false; };
  }, []);
  React.useEffect(() => {
    setPlan(null); setSubmitted(false); onPreview(null);
    setMessage(offline ? '从虚拟电机当前反馈开始预演，无需手动同步姿态。' : '预演读取实机当前姿态；确认执行后交给 Seeed SDK，模型跟随真实反馈。');
  }, [mode, resetKey]);
  const occupied = pending || locked || active || moving;
  const expired = plan && now >= plan.expiresAt;
  const changed = plan && Array.isArray(feedback?.q_deg) && feedback.q_deg.some((q, i) => Math.abs(q - plan.start_deg[i]) > .5);
  const invalidPlan = !plan || expired || changed || !previewReady;
  const actualError = plan && currentXyz?.length === 3
    ? Math.hypot(...currentXyz.map((v, i) => v - plan.xyz_mm[i])) : null;
  const job = feedback?.job;
  const executing = ['starting', 'running', 'cancelling'].includes(job?.status);
  const hardwareReason = capabilities?.hardware_reason || '实机同步能力尚未确认，不能下发。';

  function invalidate() {
    setPlan(null); setSubmitted(false); onPreview(null);
    setMessage('坐标或设置已变化，请重新预演。');
  }
  async function preview() {
    if (running.current || occupied || !previewReady) return;
    running.current = true; setPending(true); onBusy(true);
    try {
      if (xyz.some(value => !value.trim() || !Number.isFinite(Number(value)))) throw new Error('请完整填写 X、Y、Z，单位为毫米。');
      setPlan(null); setSubmitted(false); onPreview(null);
      setMessage('正在从当前反馈计算关节目标并检查模型路径…');
      const body = { xyz_mm: xyz.map(Number), orientation_mode: orientation, speed_rad_s: Number(currentSpeed) };
      let result;
      if (offline) result = await request('/ros-control/action', { ...body, action: 'preview' });
      else {
        body.hardware_motion = { speed_rad_s: Number(hardwareSpeed), timeout_mode: timeoutMode,
          feedforward_mode:feedforwardMode,
          ...(timeoutMode === 'manual' ? { timeout_s: Number(timeoutSeconds) } : {}) };
        const measured = await onPrepareHardwarePreview();
        result = await request('/command', { ...body, ...measured, action: 'point_plan', target: 'simulation', execution_source: 'hardware-preview' });
      }
      const next = { ...result, expiresAt: Date.now() + result.expires_in_s * 1000 };
      setPlan(next); onPreview(next);
      setMessage('预演通过。蓝线是预计路径，橙色标记是目标点；当前机械臂尚未运动。');
    } catch (error) { setMessage(error.message); setPlan(null); onPreview(null); }
    finally { running.current = false; setPending(false); onBusy(false); }
  }
  async function execute() {
    if (running.current || occupied || invalidPlan || submitted || (offline ? !virtualEnabled : !hardwareConfirmed || !onExecuteHardware)) return;
    running.current = true; setPending(true); onBusy(true);
    try {
      const result = offline ? await request('/ros-control/action', { action: 'execute', plan_id: plan.plan_id }) : await onExecuteHardware(plan.plan_id);
      setSubmitted(true); onExecuted(result);
      setMessage(offline ? '虚拟动作已提交，三维模型持续跟随执行反馈。' : '实机任务已提交；模型跟随执行器读取的真实关节反馈。');
    } catch (error) { setMessage(error.message); setPlan(null); onPreview(null); }
    finally { running.current = false; setPending(false); onBusy(false); }
  }

  return <section className="card glass cartesianPanel" aria-label="三维目标点">
    <div className="cartesianHeading"><div><h3>{offline ? '2' : '4'} · XYZ：预演 → 执行</h3>
      <p>mm · 基座坐标系 · Z 向上 · 自动求六轴关节角，夹爪不参与。</p></div>
      <button disabled={occupied || !currentXyz} onClick={() => { setXyz(currentXyz.map(v => v.toFixed(2))); invalidate(); }}>填入当前坐标</button>
    </div>
    <div className="cartesianInputs">
      {['X', 'Y', 'Z'].map((axis, index) => <label key={axis} htmlFor={`point-${axis}`}>{axis}（mm）
        <input id={`point-${axis}`} type="number" step="1" value={xyz[index]} disabled={occupied}
          onChange={e => { setXyz(prev => prev.map((v, i) => i === index ? e.target.value : v)); invalidate(); }} /></label>)}
    </div>
    <details className="pointOptions" open={!offline}><summary>运动设置 · {orientation === 'hold' ? '保持朝向' : '自动朝向'} · {currentSpeed} rad/s{!offline && ` · ${timeoutMode === 'auto' ? '自动时限' : `${timeoutSeconds} 秒时限`}`}</summary><div className="cartesianInputs">
      <label htmlFor="point-orientation">末端朝向<select id="point-orientation" value={orientation} disabled={occupied}
        onChange={e => { setOrientation(e.target.value); invalidate(); }}>
        <option value="position">自动调整朝向（只指定位置）</option><option value="hold">保持当前朝向</option>
      </select></label>
      <label htmlFor="point-speed">{offline ? '虚拟关节速度（rad/s）' : '实机关节速度上限（rad/s）'}<input id="point-speed" type="number"
        min={offline ? .05 : motionOptions?.speed_min_rad_s ?? .005} max={offline ? .30 : motionOptions?.speed_max_rad_s ?? .15}
        step={offline ? .05 : .005} value={currentSpeed} disabled={occupied}
        onChange={e => { if (offline) setSpeed(e.target.value); else setHardwareSpeed(e.target.value); invalidate(); }} /></label>
      {!offline && <label htmlFor="point-timeout-mode">实机任务时限<select id="point-timeout-mode" value={timeoutMode} disabled={occupied}
        onChange={e => { setTimeoutMode(e.target.value); invalidate(); }}>
        <option value="auto">自动：轨迹用时 + 5 秒余量</option><option value="manual">手动指定秒数</option>
      </select></label>}
      {!offline && timeoutMode === 'manual' && <label htmlFor="point-timeout">任务时限（秒）<input id="point-timeout" type="number"
        min={motionOptions?.timeout_min_s ?? 5} max={motionOptions?.timeout_max_s ?? 120} step="1" value={timeoutSeconds} disabled={occupied}
        onChange={e => { setTimeoutSeconds(e.target.value); invalidate(); }} /></label>}
      {!offline && <label htmlFor="point-feedforward">轨迹控制方案<select id="point-feedforward" value={feedforwardMode} disabled={occupied}
        onChange={e=>{setFeedforwardMode(e.target.value);invalidate();}}>
        <option value="none">原方案：Kp=3 / Kd=1 / 前馈=0</option>
        <option value="seeed-rs-gravity-v1">Seeed 官方：Kp=3 / Kd=1 + 重力前馈</option>
      </select></label>}
    </div></details>
    {!offline && <p className="consoleNote">以上设置只用于本次任务。Kp=3、Kd=1、加速度与 jerk 沿用已有配置；自动时限最多 120 秒。官方方案随实测姿态计算重力前馈，不覆盖保存的前馈草稿或设备参数。修改后须重新预演，现场确认后再执行。</p>}
    <div className="consoleActions">
      <button disabled={occupied || !previewReady} onClick={preview}>{pending ? '处理中…' : '计算并预演'}</button>
      {offline ? <button className="primary" disabled={occupied || invalidPlan || submitted || !virtualEnabled} onClick={execute}>执行虚拟动作</button>
        : <button className="primary" disabled={occupied || invalidPlan || submitted || !hardwareConfirmed || !onExecuteHardware} onClick={execute}>确认实机执行</button>}
      <button className="danger" disabled={!executing && !(pending && !offline)} onClick={onStop}>{offline ? '停止当前任务' : '立即停止实机任务'}</button>
    </div>
    {!previewReady && <p className="consoleNote">{offline ? '等待完整、新鲜的虚拟电机反馈；请检查虚拟服务。' : '请先连接实机，取得完整、新鲜的六轴位置。'}</p>}
    {offline && !virtualEnabled && <p className="consoleNote">可以先预演；执行前点击上方“使能虚拟电机”。</p>}
    {!offline && !hardwareConfirmed && !executing && <p className="consoleNote">预演通过后，勾选上方一次现场确认即可点击“确认实机执行”。</p>}
    {!offline && <p className="consoleNote">使能本身不会保持姿态。{feedforwardMode==='none'?'本次使用 Kp=3、Kd=1、前馈=0。':'本次复用 Seeed 官方 MIT 轨迹回调，使用 Kp=3、Kd=1 和随姿态变化的重力前馈。'}首帧目标为实测当前位置；停止不自动回零，沿用已有撤力流程。</p>}
    <p className="consoleNote">预演检查模型内的关节路径与碰撞，末端不保证走直线；不验证真实承重或现场障碍。</p>
    {plan && <div className="pointPlanSummary">
      <span>{offline ? '预计运动' : Number.isFinite(plan.hardware_preflight?.hardware_seconds) ? '实机预计运动' : '模型预演估计'} <strong>{plan.seconds.toFixed(1)} s</strong></span>
      {!offline && Number.isFinite(plan.hardware_preflight?.task_timeout_s) && <span>任务时限 <strong>{plan.hardware_preflight.task_timeout_s} s</strong></span>}
      <span>求解残差 <strong>{plan.position_error_mm.toFixed(3)} mm</strong></span>
      <span>当前距目标 <strong>{actualError?.toFixed(2) ?? '—'} mm</strong></span>
      <details><summary>查看六轴计算结果</summary><div className="consoleJoints">{plan.q_deg.map((q, i) => <span key={i}>J{i + 1}<strong>{q.toFixed(2)}°</strong></span>)}</div></details>
    </div>}
    {plan && !submitted && (expired || changed) && <p role="alert">{expired ? '预演已超过 60 秒' : '反馈起点已经变化'}，请重新预演。</p>}
    {plan?.hardware_preflight && <div className="consoleNote"><strong>实机执行检查</strong>
      <p>{plan.hardware_preflight.execution_ready ? '当前试验参数的模型检查通过；执行前仍会复读实机起点。' : '路径预演已通过，当前实机试验参数有以下阻挡：'}</p>
      {plan.hardware_preflight.execution_reasons?.map(reason => <p key={reason}>{reason}</p>)}
      {plan.hardware_preflight.static_balance_check?.feedforward_mode==='seeed-rs-gravity-v1' && <p>已选择官方重力前馈。模型通过仅说明标称补偿与包络相符；首次应在可靠支撑下验收小幅运动和停止，再执行较长轨迹。起始保持使用原始实测角度，不要求手托或把支撑姿态设成零位。</p>}
      <details><summary>本次实机试验参数与重力估算</summary><pre>{JSON.stringify(plan.hardware_preflight,null,2)}</pre></details>
    </div>}
    <p role="status" className="consoleMessage">{submitted && job?.status === 'succeeded' ? `${offline ? '虚拟动作完成' : '实机反馈已到位，任务已结束并撤力'}，当前末端误差 ${actualError?.toFixed(2) ?? '—'} mm。`
      : submitted && job?.status === 'failed' ? `执行失败：${job.error}` : submitted && job?.status === 'cancelled' ? '动作已停止，请确认现场状态。' : message}</p>
    {hardwareError && !offline && hardwareError !== message && <p role="alert">{hardwareError}</p>}
    <details className="pointReadiness"><summary>{offline ? '控制点与离线验证说明' : '实机执行状态与控制点说明'}</summary>
      {!offline && <><p>{hardwareReason}</p>{readiness?.reasons && <ul>{readiness.reasons.map(reason => <li key={reason}>{reason}</li>)}</ul>}</>}
      <p>控制点为 gripper_end，未计入额外笔尖或打印头长度。预演不发送运动指令；当前坐标根据执行反馈计算。</p>
    </details>
  </section>;
}
