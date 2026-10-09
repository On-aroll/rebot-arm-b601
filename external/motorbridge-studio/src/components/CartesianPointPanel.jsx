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

export function CartesianPointPanel({ state, locked, active, moving, now, onPreview, onBusy, onExecuted, resetKey, readiness }) {
  const [xyz, setXyz] = React.useState(['200', '150', '200']);
  const [orientation, setOrientation] = React.useState('position');
  const [speed, setSpeed] = React.useState('0.15');
  const [plan, setPlan] = React.useState(null);
  const [jobId, setJobId] = React.useState(null);
  const [pending, setPending] = React.useState(false);
  const [message, setMessage] = React.useState('先计算预览，确认路径后执行。');
  const [capabilities, setCapabilities] = React.useState(null);
  const running = React.useRef(false);
  React.useEffect(() => {
    let alive = true;
    request('/point-capabilities').then(value => { if (alive) setCapabilities(value); })
      .catch(() => { if (alive) setCapabilities(null); });
    return () => { alive = false; };
  }, []);
  React.useEffect(() => { setPlan(null); setJobId(null); setMessage('仿真姿态就绪，请输入坐标并计算预览。'); }, [resetKey]);
  const occupied = pending || locked || active || moving;
  const expired = plan && now >= plan.expiresAt;
  const changed = plan && Array.isArray(state?.q_deg) && state.q_deg.some((q, i) => Math.abs(q - plan.start_deg[i]) > .5);
  const invalidPlan = !plan || expired || changed;
  const actualError = plan && state?.xyz?.length === 3
    ? Math.hypot(...state.xyz.map((v, i) => v * 1000 - plan.xyz_mm[i])) : null;
  const ownJob = state?.job?.id === jobId ? state.job : null;
  const hardwareReason = capabilities?.hardware_reason || '实机同步能力尚未确认，不能下发；请确认后端已更新。';

  function invalidate() {
    setPlan(null); setJobId(null); onPreview(null);
    setMessage('坐标或设置已变化，请重新计算。');
  }
  async function perform(action) {
    if (running.current || occupied) return;
    if (action === 'point_execute' && invalidPlan) return;
    running.current = true; setPending(true); onBusy(true);
    try {
      if (action === 'point_plan') {
        if (xyz.some(value => !value.trim() || !Number.isFinite(Number(value)))) throw new Error('请完整填写 X、Y、Z，单位为毫米。');
        setPlan(null); setJobId(null); onPreview(null);
        setMessage('正在计算关节目标并检查模型内的碰撞…');
        const result = await request('/command', {
          action, target: 'simulation', xyz_mm: xyz.map(Number),
          orientation_mode: orientation, speed_rad_s: Number(speed),
        });
        const next = { ...result, expiresAt: Date.now() + result.expires_in_s * 1000 };
        setPlan(next); onPreview(next);
        setMessage('预览通过。蓝线是末端预计路径，橙色标记是目标点。');
      } else {
        const result = await request('/command', { action, target: 'simulation', plan_id: plan.plan_id });
        setJobId(result.job.id); onExecuted();
        setMessage('仿真到点已开始。实机未下发。可在本区停止。');
      }
    } catch (error) {
      setMessage(error.message);
      setPlan(null); onPreview(null);
    } finally { running.current = false; setPending(false); onBusy(false); }
  }
  async function cancel() {
    if (running.current) return;
    running.current = true; setPending(true); onBusy(true);
    try {
      await request('/command', { action: 'program_cancel' });
      setMessage('仿真到点已取消，模型保持当前姿态。');
      setPlan(null); onPreview(null);
    } catch (error) { setMessage(error.message); }
    finally { running.current = false; setPending(false); onBusy(false); }
  }

  return <section className="card glass cartesianPanel" aria-label="三维目标点">
    <div className="cartesianHeading"><div><h3>三维目标点 <small>输入坐标，自动计算六轴位置</small></h3>
      <p>单位 mm · 机械臂底座坐标系 base_link · Z 向上 · 控制点为夹爪末端 gripper_end，尚未计入额外笔尖或打印头长度。</p></div>
      <button disabled={occupied || !state?.xyz} onClick={() => { setXyz(state.xyz.map(v => (v * 1000).toFixed(2))); invalidate(); }}>填入当前仿真坐标</button>
    </div>
    <div className="cartesianInputs">
      {['X', 'Y', 'Z'].map((axis, index) => <label key={axis} htmlFor={`point-${axis}`}>{axis}（mm）
        <input id={`point-${axis}`} type="number" step="1" value={xyz[index]} disabled={occupied}
          onChange={e => { setXyz(prev => prev.map((v, i) => i === index ? e.target.value : v)); invalidate(); }} /></label>)}
      <label htmlFor="point-orientation">末端朝向<select id="point-orientation" value={orientation} disabled={occupied}
        onChange={e => { setOrientation(e.target.value); invalidate(); }}>
        <option value="position">自动调整朝向（只指定位置）</option><option value="hold">保持当前朝向</option>
      </select></label>
      <label htmlFor="point-speed">仿真关节速度（rad/s）<input id="point-speed" type="number" min="0.05" max="0.30" step="0.05" value={speed} disabled={occupied}
        onChange={e => { setSpeed(e.target.value); invalidate(); }} /></label>
    </div>
    <div className="consoleActions">
      <button disabled={occupied} onClick={() => perform('point_plan')}>{pending ? '处理中…' : '计算并预览'}</button>
      <button className="primary" disabled={occupied || invalidPlan || Boolean(jobId)} onClick={() => perform('point_execute')}>执行仿真到点</button>
      <button disabled title={hardwareReason}>确认实机执行（待单轴验收）</button>
      <button disabled={pending || !ownJob || !['running', 'paused'].includes(ownJob.status)} onClick={cancel}>停止当前任务</button>
    </div>
    <p className="consoleNote">到点采用平滑关节插值，末端不保证沿直线移动；预览仅检查模型中的碰撞，不能识别实物周围障碍。</p>
    {plan && <div className="pointPlanSummary">
      <span>预计运动 <strong>{plan.seconds.toFixed(1)} s</strong></span>
      <span>求解残差 <strong>{plan.position_error_mm.toFixed(3)} mm</strong></span>
      <span>当前仿真距目标 <strong>{actualError?.toFixed(2) ?? '—'} mm</strong></span>
      <div className="consoleJoints">{plan.q_deg.map((q, i) => <span key={i}>J{i + 1}<strong>{q.toFixed(2)}°</strong></span>)}</div>
    </div>}
    {plan && !jobId && (expired || changed) && <p role="alert">{expired ? '预览已超过 60 秒' : '仿真起点已经变化'}，请重新计算。</p>}
    <p role="status" className="consoleMessage">{ownJob?.status === 'succeeded' ? `仿真任务完成，当前末端误差 ${actualError?.toFixed(2) ?? '—'} mm。实机未下发。`
      : ownJob?.status === 'failed' ? `到点失败：${ownJob.error}` : message}</p>
    {readiness?.reasons && <details className="pointReadiness"><summary>实机执行还差什么</summary><ul>{readiness.reasons.map(reason=><li key={reason}>{reason}</li>)}</ul></details>}
    <p className="pointHardwareReason">{hardwareReason} 连接与同步不需要填写全部工程参数；实机执行前会列出当次试验边界及确认项。</p>
  </section>;
}
