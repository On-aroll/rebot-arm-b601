import React from 'react';
import { initialFeedbackReport, emptyObservations, observeFeedback, OBSERVATION_SPAN_DEG } from '../lib/initialStateCheck';
import { START_BOUNDARY_TOLERANCE_DEG } from '../hooks/useMotorGateway';

export function InitialStateCheck({ gateway, mapping, limits, now, blocked, onBusyChange }) {
  const [report, setReport] = React.useState(null);
  const [observations, setObservations] = React.useState([]);
  const [phase, setPhase] = React.useState('idle');
  const [message, setMessage] = React.useState('日常启动读取当前位置即可；手动观察只在首次核对或对应关系有疑问时使用。');
  const [reviewed, setReviewed] = React.useState(false);
  const [reviewedAt, setReviewedAt] = React.useState(null);
  const epoch = React.useRef(0);
  const online = gateway.status === 'online';
  const checking = phase === 'checking', observing = phase === 'observing';
  const powerPending = gateway.powerPending != null || ['checking', 'sending'].includes(gateway.powerGroup?.status);
  const enabledRequest = Object.values(gateway.powerRequests || {}).some(row => row.op === 'enable');
  const context = JSON.stringify([gateway.endpoint, online, mapping, limits]);
  const previousContext = React.useRef(context);
  const powerRecord = JSON.stringify(gateway.powerRequests || {});
  const previousPowerRecord = React.useRef(powerRecord);
  const manualEligible = report?.positionsOk && report?.startOk && !blocked && !enabledRequest && !powerPending && online;
  const changedAxes = observations.filter(row => row.spanDeg >= OBSERVATION_SPAN_DEG && row.samples > 1).map(row => row.id);
  const feedbackLive = report && report.rows.every(row => {
    const reading = gateway.motors?.[row.id], age = now - reading?.positionUpdatedAt;
    return !reading?.error && Number.isFinite(reading?.mechPosRad) && age >= 0 && age < 3000;
  });

  React.useEffect(() => { onBusyChange(checking || observing); }, [checking, observing, onBusyChange]);
  React.useEffect(() => () => { ++epoch.current; onBusyChange(false); }, [onBusyChange]);
  React.useEffect(() => {
    if (previousContext.current === context) return;
    previousContext.current = context; ++epoch.current;
    setReport(null); setObservations([]); setReviewed(false); setReviewedAt(null); setPhase('idle');
    setMessage('连接、映射或模型限位已变化，请重新检测当前状态。');
  }, [context]);
  React.useEffect(() => {
    const powerChanged = previousPowerRecord.current !== powerRecord;
    previousPowerRecord.current = powerRecord;
    if (powerChanged || powerPending || !feedbackLive) { setReviewed(false); setReviewedAt(null); }
    if ((checking && (powerPending || powerChanged || blocked)) ||
        (observing && (powerPending || powerChanged || enabledRequest || blocked || !feedbackLive))) {
      ++epoch.current; setPhase('interrupted');
      setMessage('检测或观察已中断：反馈过期、出现电源操作或其他任务。请重新检测，未完成的记录不能用于核对。');
      return;
    }
    if (observing) setObservations(previous => observeFeedback(previous, gateway.motors, now));
  }, [checking, observing, gateway.motors, now, blocked, powerPending, powerRecord, enabledRequest, feedbackLive]);

  async function check() {
    if (!online || blocked || checking || observing || powerPending) return;
    const ticket = ++epoch.current;
    setPhase('checking'); setReport(null); setObservations([]); setReviewed(false); setReviewedAt(null);
    setMessage('正在读取七轴位置和运行模式；不发送使能、失能、运动或零位写入。');
    try {
      const readings = await gateway.refresh({ waitForIdle: true });
      if (ticket !== epoch.current) return;
      if (!readings) throw Error('未取得完整查询，请检查连接；没有自动重试或发送运动。');
      const result = initialFeedbackReport(readings, mapping, limits, Date.now(), true, START_BOUNDARY_TOLERANCE_DEG);
      setReport(result); setObservations(emptyObservations(result)); setPhase('ready');
      setMessage(result.positionsOk && result.modesOk && result.startOk
        ? '当前反馈、模式和模型起点检查通过；这份记录不代表零位、真实承重或停止试验已验收。'
        : '当前状态仍有待处理项，见七轴结果；原始角度没有裁剪或改零位。');
    } catch (e) {
      if (ticket === epoch.current) { setPhase('idle'); setMessage(e.message); }
    }
  }
  function beginObservation() {
    if (!manualEligible || !feedbackLive) return;
    // Start at the current readings, not the earlier detection snapshot.
    const baseline = initialFeedbackReport(gateway.motors, mapping, limits, Date.now(), online, START_BOUNDARY_TOLERANCE_DEG);
    if (!baseline.positionsOk || !baseline.startOk) {
      setMessage('当前位置反馈或模型起点范围已变化，请重新检测。');
      return;
    }
    setObservations(emptyObservations(baseline)); setReviewed(false); setReviewedAt(null); setPhase('observing');
    setMessage('正在记录七轴变化，请观察实物与三维模型的对应；不自动判断机械零位或限位。');
  }
  function finish() {
    setPhase('recorded'); setReviewed(false); setReviewedAt(null);
    setMessage('手动变化已记录。请对照三维模型和实物；未观察到变化的轴不自动算通过。');
  }
  function download() {
    const record = { schemaVersion: 1, report, observations, phase, reviewed, reviewedAt, changedAxes,
      manualReview: reviewed ? 'operator-confirmed-visual-consistency-for-observed-axes' : 'not-confirmed',
      mappingAtCapture: mapping, hardwareExecutionAllowed: false, hardwareParametersWritten: false };
    const url = URL.createObjectURL(new Blob([JSON.stringify(record, null, 2)], { type: 'application/json' }));
    const link = document.createElement('a'); link.href = url; link.download = 'rebot-initial-state-check.json'; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }
  return <section className="card glass initialStateCheck" aria-label="初始状态检测">
    <div className="deviceHeading"><div><h3>3 · 初始状态检测</h3><p>读取七轴当前位置，保留既有方向、零位修正和限位；当前位置作为起点。</p></div>
      <button className="primary" disabled={!online || blocked || checking || observing || powerPending} onClick={check}>{checking ? '正在读取七轴…' : '一键检测当前状态'}</button></div>
    <p role="status">{message}</p>
    {!online && <p className="consoleNote">先连接实机只读反馈，再点击检测；不需要每次重新校准。</p>}
    {report && <><small>检测快照：{new Date(report.capturedAt).toLocaleTimeString()} · 以下检查结果属于该次读取</small><div className="initialCheckSummary"><span>位置反馈：{report.rows.filter(row => row.fresh).length}/7</span>
      <span>六轴模式：{report.rows.slice(0,6).every(row => row.modeOk) ? 'MIT' : '待处理'}</span>
      <span>模型起点：{report.startOk ? '检查通过' : '待处理'}</span><span>实机轨迹：待验收</span></div>
      {!feedbackLive && <p role="alert">当前反馈已不新鲜，以上仅是检测时的记录。</p>}
      <details><summary>七轴角度、起点范围与已有核对记录</summary><div className="mappingTableScroll"><table className="compareTable"><thead><tr><th>轴</th><th>原始角度</th><th>模型角度</th><th>起点范围</th><th>模式</th><th>人工核对记录</th></tr></thead><tbody>
        {report.rows.map(row => <tr key={row.id}><td>J{row.id}</td><td>{row.rawDeg == null ? '未读到' : `${row.rawDeg.toFixed(2)}°`}</td><td>{row.modelDeg == null ? '—' : `${row.modelDeg.toFixed(2)}°`}</td>
          <td>{row.id === 7 ? '夹爪独立核对' : row.nominal ? '名义范围内' : row.tolerated ? '在已约定的 1° 起点容差内' : '待处理'}</td>
          <td>{row.mode === 0 ? 'MIT' : row.mode == null ? '未取得新鲜模式' : `模式 ${row.mode}`}</td><td>{row.id === 7 ? '不套用六轴标定' : row.previouslyConfirmed ? '已有软件核对记录' : '未记录'}</td></tr>)}
      </tbody></table></div><p className="consoleNote">已有记录来自人工确认，不是自动校准证明；检测不独立验证设备身份。</p></details>
      <details className="initialManual"><summary>可选：手动变化观察（首次核对时使用）</summary>
        <p>先确认真实电机已撤力、机械支撑可靠。只对能安全改变姿态的轴小幅手动移动，不用来回大幅晃动，不用手托承重关节；不能安全手动的轴可以跳过。检测不会自动失能，发送失能成功也不能代替现场确认已撤力。</p>
        <div className="consoleActions">{observing ? <button onClick={finish}>结束手动记录</button>
          : <button disabled={!manualEligible || !feedbackLive || phase === 'interrupted'} onClick={beginObservation}>已确认撤力与支撑，开始记录</button>}</div>
        {enabledRequest && <p role="alert">本会话仍有使能发送记录；请先失能并现场确认撤力，不能按使能记录推断真实状态。</p>}
        <div className="machineAxisStatus" aria-label="手动变化记录">{observations.map(row => <span key={row.id}>J{row.id}<strong>Δ {row.deltaDeg.toFixed(2)}° · 跨度 {row.spanDeg.toFixed(2)}°</strong>
          <small>{changedAxes.includes(row.id) ? '变化已记录，方向需观察核对' : '未观察到明显变化'}</small></span>)}</div>
        {phase === 'recorded' && <label><input type="checkbox" checked={reviewed} disabled={!feedbackLive || powerPending || enabledRequest || blocked || changedAxes.length === 0} onChange={e => { setReviewed(e.target.checked); setReviewedAt(e.target.checked ? Date.now() : null); }} />我已观察到上述有变化的轴，模型与实物姿态和方向一致（一次确认）。</label>}
        <p className="consoleNote">只记录反馈与人工观察；不据此自动改方向、零位或限位。静止姿态、手动变化都不能自动确定机械零位或完整可动范围。</p>
      </details>
      <button disabled={checking || observing} onClick={download}>导出检测记录</button>
      <p className="consoleNote">下一步仍需有支撑的小幅动作、承重与停止验收，之后才能开放 XYZ 实机执行。检测通过不自动使能或启动运动。</p></>}
  </section>;
}
