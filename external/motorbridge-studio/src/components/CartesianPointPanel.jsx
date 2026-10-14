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
  onPreview, onBusy, onExecuted, onPrepareHardwarePreview, onStop, resetKey, readiness, currentXyz }) {
  const offline = mode === 'offline';
  const [xyz, setXyz] = React.useState(['200', '150', '200']);
  const [orientation, setOrientation] = React.useState('position');
  const [speed, setSpeed] = React.useState('0.15');
  const [plan, setPlan] = React.useState(null);
  const [submitted, setSubmitted] = React.useState(false);
  const [pending, setPending] = React.useState(false);
  const [message, setMessage] = React.useState('输入 XYZ，先预演，再执行。');
  const [capabilities, setCapabilities] = React.useState(null);
  const running = React.useRef(false);
  React.useEffect(() => {
    let alive = true;
    request('/point-capabilities').then(value => { if (alive) setCapabilities(value); })
      .catch(() => { if (alive) setCapabilities(null); });
    return () => { alive = false; };
  }, []);
  React.useEffect(() => {
    setPlan(null); setSubmitted(false); onPreview(null);
    setMessage(offline ? '从虚拟电机当前反馈开始预演，无需手动同步姿态。' : '预演前读取实机当前姿态；真实轨迹执行仍待验收。');
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
      const body = { xyz_mm: xyz.map(Number), orientation_mode: orientation, speed_rad_s: Number(speed) };
      let result;
      if (offline) result = await request('/ros-control/action', { ...body, action: 'preview' });
      else {
        await onPrepareHardwarePreview();
        result = await request('/command', { ...body, action: 'point_plan', target: 'simulation', execution_source: 'hardware-preview' });
      }
      const next = { ...result, expiresAt: Date.now() + result.expires_in_s * 1000 };
      setPlan(next); onPreview(next);
      setMessage('预演通过。蓝线是预计路径，橙色标记是目标点；当前机械臂尚未运动。');
    } catch (error) { setMessage(error.message); setPlan(null); onPreview(null); }
    finally { running.current = false; setPending(false); onBusy(false); }
  }
  async function execute() {
    if (!offline || running.current || occupied || invalidPlan || submitted || !virtualEnabled) return;
    running.current = true; setPending(true); onBusy(true);
    try {
      const result = await request('/ros-control/action', { action: 'execute', plan_id: plan.plan_id });
      setSubmitted(true); onExecuted(result);
      setMessage('虚拟动作已提交，三维模型持续跟随执行反馈。');
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
    <details className="pointOptions"><summary>运动设置 · {orientation === 'hold' ? '保持朝向' : '自动朝向'} · {speed} rad/s</summary><div className="cartesianInputs">
      <label htmlFor="point-orientation">末端朝向<select id="point-orientation" value={orientation} disabled={occupied}
        onChange={e => { setOrientation(e.target.value); invalidate(); }}>
        <option value="position">自动调整朝向（只指定位置）</option><option value="hold">保持当前朝向</option>
      </select></label>
      <label htmlFor="point-speed">预演关节速度（rad/s）<input id="point-speed" type="number" min="0.05" max="0.30" step="0.05" value={speed} disabled={occupied}
        onChange={e => { setSpeed(e.target.value); invalidate(); }} /></label>
    </div></details>
    <div className="consoleActions">
      <button disabled={occupied || !previewReady} onClick={preview}>{pending ? '处理中…' : '计算并预演'}</button>
      {offline ? <button className="primary" disabled={occupied || invalidPlan || submitted || !virtualEnabled} onClick={execute}>执行虚拟动作</button>
        : <button disabled title={hardwareReason}>确认实机执行（待验收）</button>}
      {offline && <button className="danger" disabled={!executing} onClick={onStop}>停止当前任务</button>}
    </div>
    {!previewReady && <p className="consoleNote">{offline ? '等待完整、新鲜的虚拟电机反馈；请检查虚拟服务。' : '请先连接实机，取得完整、新鲜的六轴位置。'}</p>}
    {offline && !virtualEnabled && <p className="consoleNote">可以先预演；执行前点击上方“使能虚拟电机”。</p>}
    <p className="consoleNote">预演检查模型内的关节路径与碰撞，末端不保证走直线；不验证真实承重或现场障碍。</p>
    {plan && <div className="pointPlanSummary">
      <span>预计运动 <strong>{plan.seconds.toFixed(1)} s</strong></span>
      <span>求解残差 <strong>{plan.position_error_mm.toFixed(3)} mm</strong></span>
      <span>当前距目标 <strong>{actualError?.toFixed(2) ?? '—'} mm</strong></span>
      <details><summary>查看六轴计算结果</summary><div className="consoleJoints">{plan.q_deg.map((q, i) => <span key={i}>J{i + 1}<strong>{q.toFixed(2)}°</strong></span>)}</div></details>
    </div>}
    {plan && !submitted && (expired || changed) && <p role="alert">{expired ? '预演已超过 60 秒' : '反馈起点已经变化'}，请重新预演。</p>}
    <p role="status" className="consoleMessage">{submitted && job?.status === 'succeeded' ? `虚拟动作完成，当前末端误差 ${actualError?.toFixed(2) ?? '—'} mm。`
      : submitted && job?.status === 'failed' ? `虚拟执行失败：${job.error}` : submitted && job?.status === 'cancelled' ? '虚拟动作已停止。' : message}</p>
    <details className="pointReadiness"><summary>{offline ? '控制点与离线验证说明' : '实机执行状态与控制点说明'}</summary>
      {!offline && <><p>{hardwareReason}</p>{readiness?.reasons && <ul>{readiness.reasons.map(reason => <li key={reason}>{reason}</li>)}</ul>}</>}
      <p>控制点为 gripper_end，未计入额外笔尖或打印头长度。预演不发送运动指令；当前坐标根据执行反馈计算。</p>
    </details>
  </section>;
}
