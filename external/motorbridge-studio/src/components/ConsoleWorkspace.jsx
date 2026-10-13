import React from 'react';
import { ArmUrdfViewer } from './ArmUrdfViewer';
import { HardwareMappingPanel } from './HardwareMappingPanel';
import { loadMapping, mapHardwareFeedback } from '../lib/hardwareMapping';
import { DeviceConnectionPanel } from './DeviceConnectionPanel';
import { CartesianPointPanel } from './CartesianPointPanel';
import { MotorModuleCard } from './MotorModuleCard';
import { useMotorGateway, MACHINE_IDS, MACHINE_MODELS } from '../hooks/useMotorGateway';
import { useSoftwareConfig } from '../hooks/useSoftwareConfig';
import { feedbackJoints, feedbackProblem, toStudioPoint } from '../lib/workspaceFeedback';
import '../styles/console-workspace.css';

const STATUS = { running: '正在执行', paused: '已暂停', succeeded: '执行完成', cancelled: '已取消', failed: '执行失败' };
const GATEWAY_STATUS = { disconnected: '未连接', connecting: '连接中', online: '已连接', offline: '断开' };

// 任务清单：两个都已在仿真侧验收过，动作名对应 rebot_sim.py 的 /command。
const TASKS = {
  writing: { label: '国庆快乐 · 连续立体书写', action: 'print_demo', note: '6 阶段一笔轨迹 · 需先回到初始姿态' },
  grasp: { label: '抓取并放置 · 往返搬运', action: 'grasp_demo', note: '13 步：接近、抓取、抬升、放置、抓回原位，含空抓判定' },
};

// 官方 config/rebotarm_rs.yaml：motor_id N ↔ joint N。
const AXES = [
  { id: 1, label: 'J1 底座' }, { id: 2, label: 'J2 大臂' }, { id: 3, label: 'J3 小臂' },
  { id: 4, label: 'J4 腕转' }, { id: 5, label: 'J5 腕俯' }, { id: 6, label: 'J6 末端' },
  { id: 7, label: 'J7 夹爪' },
];

async function api(path, body) {
  const response = await fetch(path, {
    method: body ? 'POST' : 'GET',
    headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined,
    signal: AbortSignal.timeout(5000),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

function formatAge(age) {
  if (age == null) return '—';
  if (age < 60) return `${age.toFixed(1)} 秒前`;
  if (age < 3600) return `${Math.round(age / 60)} 分钟前`;
  if (age < 86400) return `${(age / 3600).toFixed(1)} 小时前`;
  return `${Math.round(age / 86400)} 天前`;
}

// 仿真静止时关节角会停在 -1e-9 量级，直接 toFixed 会显示成 "-0.0°"。
function formatDeg(value, digits = 1) {
  if (value == null || !Number.isFinite(value)) return '—';
  const text = value.toFixed(digits);
  return Number(text) === 0 ? (0).toFixed(digits) : text;
}

// 手柄只负责“看得见输入”，下发仍被禁用：见手柄区的说明。
function useGamepad() {
  const [pads, setPads] = React.useState([]);
  const [axes, setAxes] = React.useState([]);
  const [buttons, setButtons] = React.useState([]);
  React.useEffect(() => {
    let alive = true;
    const scan = () => {
      if (!alive) return;
      const list = Array.from((navigator.getGamepads && navigator.getGamepads()) || []).filter(Boolean);
      setPads(list.map((pad) => ({ index: pad.index, id: pad.id })));
      if (list.length) {
        setAxes(list[0].axes.map((value) => (Math.abs(value) < 0.08 ? 0 : Number(value.toFixed(3)))));
        setButtons(list[0].buttons.map((button) => button.pressed));
      } else {
        setAxes([]); setButtons([]);
      }
    };
    scan();
    const timer = setInterval(scan, 150);
    window.addEventListener('gamepadconnected', scan);
    window.addEventListener('gamepaddisconnected', scan);
    return () => {
      alive = false; clearInterval(timer);
      window.removeEventListener('gamepadconnected', scan);
      window.removeEventListener('gamepaddisconnected', scan);
    };
  }, []);
  return { pads, axes, buttons };
}

export function ConsoleWorkspace() {
  const requestedHardwareView = React.useRef(new URL(window.location.href).searchParams.get('pose') === 'hardware').current;
  const [state, setState] = React.useState(null);
  const [mapping, setMapping] = React.useState(loadMapping);
  const [displaySource, setDisplaySource] = React.useState(requestedHardwareView ? 'hardware' : 'simulation');
  const lastHardwareJoints = React.useRef({});
  const [pointPlan, setPointPlan] = React.useState(null);
  const [syncing, setSyncing] = React.useState(false);
  const [synchronization, setSynchronization] = React.useState(null);
  const [syncVersion, setSyncVersion] = React.useState(0);
  const [hardwareReadiness, setHardwareReadiness] = React.useState(null);
  const [machine, setMachine] = React.useState(null);
  const [document, setDocument] = React.useState(null);
  const [networkError, setNetworkError] = React.useState('');
  const [message, setMessage] = React.useState('');
  const [busy, setBusy] = React.useState(false);
  const [task, setTask] = React.useState('writing');
  const [cameraView, setCameraView] = React.useState('overview');
  const [resetViewSeq, setResetViewSeq] = React.useState(0);
  const [ghost, setGhost] = React.useState(false);
  const [showPlan, setShowPlan] = React.useState(true);
  const [showTrail, setShowTrail] = React.useState(true);
  const [now, setNow] = React.useState(Date.now());
  const commandBusy = React.useRef(false);
  const config = useSoftwareConfig();
  const [expandedAxis, setExpandedAxis] = React.useState(() => {
    try { const id = Number(localStorage.getItem('b601-console-expanded-axis')); return id >= 1 && id <= 7 ? id : null; } catch { return null; }
  });
  function toggleAxis(id) {
    const next = expandedAxis === id ? null : id; setExpandedAxis(next);
    try { localStorage.setItem('b601-console-expanded-axis', String(next || '')); } catch { /* display preference only */ }
  }
  async function exportConfig() {
    try {
      const result = await config.action({ action: 'export' });
      const url = URL.createObjectURL(new Blob([JSON.stringify(result.backup, null, 2)], { type: 'application/json' }));
      const link = window.document.createElement('a'); link.href = url; link.download = 'b601-software-config.json'; link.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
    } catch (e) { setMessage(e.message); }
  }
  async function importConfig(event) {
    const file = event.target.files?.[0]; if (!file) return;
    try {
      if (file.size > 60000) throw new Error('备份超过 60 KB，请使用本地历史恢复流程');
      await config.action({ action: 'import', backup: JSON.parse(await file.text()) });
      setMessage('备份已导入草稿；批准记录未替换，请逐轴审核');
    } catch (e) { setMessage(e.message); }
    event.target.value = '';
  }
  async function archiveSnapshot(id) {
    const motor = gateway.motors[id]; if (!motor) return;
    const units = { mechPosRad:'rad', runMode:'enum', limitSpd:'rad/s', limitPpSpd:'rad/s', locKp:'firmware', limitCur:'A' };
    const fields = Object.fromEntries(Object.entries(units).map(([key, unit]) => {
      const ok = Number.isFinite(motor[key]) && Number.isFinite(motor.fieldTimes?.[key]);
      return [key, { unit, value: ok ? motor[key] : null, measured_at: ok ? motor.fieldTimes[key] / 1000 : null, status: ok ? 'ok' : 'unknown' }];
    }));
    try {
      await config.action({ action:'snapshot', snapshot:{ schema_version:1, axis_id:id, model:motor.model,
        source:'authorized-browser-read', measured_at:motor.updatedAt / 1000, control_mode:motor.runMode ?? null,
        review_status:'collected', identity_status:'unverified', device_id:null, fields } });
      setMessage(`J${id} 现有读数已归档；没有发起额外查询，也未批准配置`);
    } catch (e) { setMessage(e.message); }
  }
  const gamepad = useGamepad();
  const gateway = useMotorGateway({ intervalMs: 2500 });

  React.useEffect(() => {
    let disposed = false;
    let simTimer; let machineTimer; let readinessTimer;
    const pollReadiness = async () => {
      try { const next=await api('/hardware-readiness'); if (!disposed) setHardwareReadiness(next); }
      catch { if (!disposed) setHardwareReadiness(null); }
      finally { if (!disposed) readinessTimer=setTimeout(pollReadiness,3000); }
    };
    const poll = async () => {
      try {
        const next = await api('/studio-state');
        if (!disposed) { setState(next); setNetworkError(''); }
      } catch (error) {
        if (!disposed) setNetworkError(`仿真服务连接中断：${error.message}`);
      } finally {
        if (!disposed) simTimer = setTimeout(poll, 160);
      }
    };
    // 网关在线时以它为准；这条文件通道是 rebot-twin-sync 的后备来源。
    const pollMachine = async () => {
      try {
        const next = await api('/machine-state');
        if (!disposed) setMachine(next);
      } catch (error) {
        if (!disposed) setMachine({ present: false, reason: 'unreachable', error: error.message });
      } finally {
        if (!disposed) machineTimer = setTimeout(pollMachine, 500);
      }
    };
    poll(); pollMachine(); pollReadiness();
    const clock = setInterval(() => setNow(Date.now()), 400);
    api('/writing-program').then((data) => { if (!disposed) setDocument(data); })
      .catch((error) => { if (!disposed) setMessage(`程序读取失败：${error.message}`); });
    return () => { disposed = true; clearTimeout(simTimer); clearTimeout(machineTimer); clearTimeout(readinessTimer); clearInterval(clock); };
  }, []);

  const problem = networkError || feedbackProblem(state, now);
  const job = state?.job;
  const active = job?.status === 'running' || job?.status === 'paused';
  const moving = Boolean(state?.manual_motion || state?.playing);
  const locked = Boolean(problem || busy || gateway.powerPending != null);
  const jointTargets = React.useMemo(() => feedbackJoints(state), [state]);
  const feedbackTrace = React.useMemo(() => ({ points: (state?.trace_xyz || []).map(toStudioPoint) }), [state]);
  const plannedPath = React.useMemo(() => {
    // 计划路径来自书写程序的 trace_path 步骤；换任务时必须跟着消失，
    // 否则会在抓取任务上叠一层“国庆快乐”的字形。
    if (!showPlan) return [];
    if (pointPlan) return pointPlan.path_xyz.map(toStudioPoint);
    if (task !== 'writing' || !document) return [];
    return (document.steps.find((step) => step.op === 'trace_path')?.points || []).map(toStudioPoint);
  }, [document, showPlan, task, pointPlan]);

  // 实机读数优先走网关，其次才是 twin-sync 发布的文件。
  const gatewayOnline = gateway.status === 'online';
  const mapped = React.useMemo(() => mapHardwareFeedback(gateway.motors, mapping, state?.limits_deg, now, gatewayOnline), [gateway.motors, mapping, state?.limits_deg, now, gatewayOnline]);
  React.useEffect(() => { if (mapped.joints) lastHardwareJoints.current = mapped.joints; }, [mapped.joints]);
  const measuredView = mapped.joints || lastHardwareJoints.current;
  const machineLive = Boolean(machine?.present && !machine?.stale);
  const machineAge = machine?.present ? now / 1000 - (machine.report_time_unix_s || 0) : null;
  const realDeg = React.useMemo(() => {
    return mapped.rows.map(row => row.fresh ? row.modelDeg : null);
  }, [mapped.rows]);
  const realSource = gatewayOnline ? `网关 9002 · ${(gateway.updatedAt ? formatAge((now - gateway.updatedAt) / 1000) : '等待读取')}`
    : (machineLive ? `twin-sync 文件 · ${formatAge(machineAge)}` : '未接入');
  const simDeg = state?.q_deg || [];
  const readCount = gatewayOnline ? MACHINE_IDS.filter((id) => gateway.motors[id]?.mechPosDeg != null).length : 0;

  async function synchronizeHardwarePose() {
    if (syncing || active || moving) return;
    setSyncing(true); setSynchronization(null);
    try {
      const readings = await gateway.refresh();
      if (!readings) throw new Error('状态查询正在进行，请稍后再点同步');
      const result = await api('/command', {action:'sync_hardware_pose',motors:readings,mapping});
      setSynchronization(result); setSyncVersion(v=>v+1); setPointPlan(null);
      setState(await api('/studio-state')); setDisplaySource('simulation');
    } catch(error) { setSynchronization({ok:false,error:error.message}); }
    finally { setSyncing(false); }
  }

  async function command(action) {
    if (commandBusy.current || problem) return;
    commandBusy.current = true;
    setBusy(true); setMessage('');
    try {
      if (action === 'start') {
        setDisplaySource('simulation');
        setPointPlan(null);
        await api('/command', { action: 'reset' });
        await api('/command', { action: TASKS[task].action });
        setMessage(`已提交「${TASKS[task].label}」，模型跟随 MuJoCo 实际反馈。`);
      } else {
        if (action === 'reset') setDisplaySource('simulation');
        await api('/command', { action });
      }
      setState(await api('/studio-state'));
    } catch (error) { setMessage(`操作失败：${error.message}`); }
    finally { commandBusy.current = false; setBusy(false); }
  }

  function download() {
    const blob = new Blob([JSON.stringify({
      name: TASKS[task].label, source: 'mujoco-simulation-feedback',
      frame: 'RS URDF base_link, metres, Z-up', exported_at: new Date().toISOString(),
      job, trace_xyz: state?.trace_xyz || [], trace_styles: state?.trace_styles || [],
      machine: { via_gateway: gatewayOnline ? gateway.motors : null, via_file: machine },
      program: document,
    }, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = window.document.createElement('a');
    link.href = url; link.download = `虚实联调-${job?.id || 'preview'}.json`; link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  }

  const stepProgress = job ? Math.min(100, ((job.completed_steps?.length || 0) / job.step_count) * 100) : 0;

  return (
    <section className="consoleWorkspace" aria-label="虚实联调控制台">
      <div className="consoleHeading card glass">
        <div><span className="consoleEyebrow">reBot B601-RS · 虚实联调控制台</span>
          <h2>机械臂工作台 <small>连接 → 同步姿态 → 输入坐标 → 预演 → 确认执行</small></h2>
          <p>沿用已确认的静态姿态映射。实机反馈、仿真预演和计划路径分别显示。</p></div>
        <div className="consoleBadges">
          <span className={`badge ${problem ? 'err' : 'ok'}`}>{problem ? '仿真反馈异常' : '仿真在线'}</span>
          <span className={`badge ${gatewayOnline ? 'ok' : (gateway.status === 'offline' ? 'err' : '')}`}>
            {gatewayOnline ? `网关 ${readCount}/7 轴` : `网关：${GATEWAY_STATUS[gateway.status] || gateway.status}`}
          </span>
          <span className="badge">逐轴使能 / 失能 · 轨迹锁定</span>
        </div>
        <a href="#motor-control-panel">实机电机 · 使能 / 失能</a>
      </div>
      <DeviceConnectionPanel readiness={hardwareReadiness} gateway={gateway} now={now} onSync={synchronizeHardwarePose} syncing={syncing} synchronization={synchronization} blocked={active||moving||busy||Boolean(problem)||gateway.powerPending!=null} />
      <div className="consoleStatus" role="status"><strong>逐轴电源可手动操作 · XYZ 实机执行锁定</strong><span>{hardwareReadiness?.next_step || '正在读取现场验收条件；实机执行保持锁定'}</span></div>
      {config.error && <div className="consoleAlert" role="alert">{config.error}；仿真仍可用，配置不可自动恢复默认值。</div>}
      {problem && <div className="consoleAlert" role="alert">{problem}。画面保留最后一次仿真反馈。</div>}
      <div className="consolePrimary"><div className="consoleModelColumn">
        <div className="consoleSources"><button aria-pressed={displaySource === 'simulation'} onClick={() => setDisplaySource('simulation')}>MuJoCo 仿真（预测）</button><button aria-pressed={displaySource === 'hardware'} onClick={() => setDisplaySource('hardware')}>实机反馈（只读）</button></div>
      {displaySource === 'hardware' && (!mapped.complete || !mapped.withinLimits) && <div className="consoleAlert" role="alert">
        {!mapped.complete ? '实机反馈不完整或已过期，三维画面停止更新。' : '存在超出模型限位的读数，画面按映射结果原样展示，没有裁剪；当前姿态不允许用于实机规划。'}
      </div>}
      <div className="card glass consoleStage">
        <div className="consoleSceneToolbar"><strong>运动与轨迹</strong><div>
          <button onClick={() => { setCameraView('writing'); setResetViewSeq((n) => n + 1); }}>正视文字</button>
          <button onClick={() => { setCameraView('overview'); setResetViewSeq((n) => n + 1); }}>机械臂全景</button>
        </div></div>
        <div className="consoleViewport" aria-label="RS 机械臂实时三维场景">
          {displaySource === 'hardware' && !Object.keys(measuredView).length ? <div className="hardwarePoseLoading">等待读取完整的六轴实机姿态…<br /><small>不会用零位代替尚未取得的反馈。</small></div> : <ArmUrdfViewer profile="workspace_rs" jointTargets={displaySource === 'hardware' ? measuredView : jointTargets}
            displayMeasuredOutOfRange={displaySource === 'hardware'} simMode="trajectory"
            feedbackDriven feedbackTrace={displaySource === 'hardware' ? { points: [] } : feedbackTrace} plannedPath={displaySource === 'hardware' ? [] : plannedPath}
            waypointMarkers={displaySource !== 'hardware' && pointPlan ? [{ id: 'cartesian-target', ...toStudioPoint(pointPlan.xyz_mm.map(v => v / 1000)), label: '目标点', color: 0xf29d38 }] : []}
            trailColor="#ef6948" trailVisible={showTrail} cameraView={cameraView}
            robotOpacity={ghost ? 0.2 : 1} resetViewSeq={resetViewSeq} />}
          <div className="consoleSceneCaption">{displaySource === 'hardware' ? (mapped.complete ? '实机反馈姿态 · 只读预览' : '实机反馈过期 · 保留最后姿态') : (STATUS[job?.status] || '准备就绪')}<br />
            <small>{displaySource === 'hardware' ? (mapped.confirmed ? '已人工核对映射；运动验收尚未完成' : '官方基准映射尚未核对；夹爪开合未知') : '拖动旋转 · 滚轮缩放 · 关节角取仿真实际反馈'}</small></div>
        </div>
        <div className="consoleLegend">
          <label><input type="checkbox" checked={showPlan} disabled={!pointPlan && !document} onChange={(e) => setShowPlan(e.target.checked)} /><i className="plan" />计划路径</label>
          <label><input type="checkbox" checked={showTrail} onChange={(e) => setShowTrail(e.target.checked)} /><i className="actual" />实际仿真轨迹</label>
          <label><input type="checkbox" checked={ghost} onChange={(e) => setGhost(e.target.checked)} />淡化机械臂</label>
        </div>
      </div>

      </div><div className="consolePointColumn">
      <CartesianPointPanel readiness={hardwareReadiness} state={state} resetKey={syncVersion} locked={locked||syncing} active={active} moving={moving} now={now}
        onPreview={(plan) => { setPointPlan(plan); if (plan) { setDisplaySource('simulation'); setCameraView('overview'); setShowPlan(true); setGhost(false); } }} onBusy={(value) => { commandBusy.current = value; setBusy(value); }}
        onExecuted={() => { setDisplaySource('simulation'); setCameraView('overview'); setGhost(false); setShowPlan(true); }} />

      </div></div>
      <div id="motor-control-panel" className="consoleModulesHead">
        <h3>实机七轴 · 使能 / 失能 <small>J1–J7 独立电机</small></h3>
        <span>实机读数来源：{realSource}
          {gateway.cycleMs ? ` · 每轮 ${gateway.cycleMs} ms` : ''}</span>
      </div>
      <div className="consoleModules">
        {AXES.map((axis) => <MotorModuleCard key={axis.id} id={axis.id} label={axis.label}
          motor={gateway.motors?.[axis.id]} status={gateway.status} gateway={gateway}
          powerBlocked={active || moving || busy || syncing || hardwareReadiness?.power_trial?.busy}
          limits={axis.id <= 6 ? state?.limits_deg?.[axis.id - 1] : null}
          plannedDeg={axis.id <= 6 ? state?.target_deg?.[axis.id - 1] : null} config={config} expanded={expandedAxis === axis.id} onToggle={() => toggleAxis(axis.id)} />)}
      </div>

      <details className="consoleAdvanced card glass"><summary>已有仿真任务 · 写字 / 抓取 / 状态与导出</summary>
      <div className="consoleRow4">
        <section className="card glass consoleCard"><h3>任务执行 <small>仿真</small></h3>
          <div className="consoleRow"><select value={task} disabled={active || locked} onChange={(e) => setTask(e.target.value)}>
            {Object.entries(TASKS).map(([key, value]) => <option key={key} value={key}>{value.label}</option>)}
          </select></div>
          <p className="consoleNote">{TASKS[task].note}<br />程序将路径解算成六轴目标，无需逐个填写电机位置。此按钮当前仅运行仿真。</p>
          <button className="primary consoleStart" disabled={locked || active || moving || !document} onClick={() => command('start')}>
            {busy ? '正在处理…' : '重置并开始执行'}</button>
          <div className="consoleActions">
            <button disabled={locked || job?.status !== 'running'} onClick={() => command('program_pause')}>暂停</button>
            <button disabled={locked || job?.status !== 'paused'} onClick={() => command('program_resume')}>继续</button>
            <button disabled={locked || !active} onClick={() => command('program_cancel')}>取消任务</button>
          </div>
          <button disabled={locked || active} onClick={() => command('reset')}>重置仿真</button>
          {message && <p role="status" className="consoleMessage">{message}</p>}
        </section>

        <section className="card glass consoleCard"><h3>{STATUS[job?.status] || '等待执行'}</h3>
          <p>{job?.step_name || '已载入动作程序'}</p>
          <progress max="100" value={stepProgress} aria-label="已完成步骤" />
          <div className="consoleStats"><span>完成步骤<strong>{job?.completed_steps?.length || 0} / {job?.step_count || 6}</strong></span>
            <span>轨迹长度<strong>{(job?.trace_length_m || 0).toFixed(3)} m</strong></span>
            <span>轨迹采样<strong>{state?.trace_xyz?.length || 0} 点</strong></span>
            <span>仿真时间<strong>{(state?.sim_time || 0).toFixed(1)} s</strong></span></div>
          {job?.error && <p role="alert">{job.error}</p>}
          <button disabled={!state?.trace_xyz?.length} onClick={download}>导出轨迹与执行记录</button>
        </section>

        <section className="card glass consoleCard"><h3>独立状态差值 <small>度 · 非轨迹跟踪误差</small></h3>
          <table className="compareTable"><thead><tr><th>轴</th><th>实机换算</th><th>仿真</th><th>差值</th></tr></thead><tbody>
            {AXES.slice(0, 6).map((axis, i) => {
              const real = realDeg[i];
              const sim = simDeg[i];
              const delta = real != null && sim != null ? real - sim : null;
              return <tr key={axis.id}><td>{axis.label.split(' ')[0]}</td>
                <td className={real == null ? 'off' : ''}>{real == null ? '—' : `${formatDeg(real)}°`}</td>
                <td>{sim == null ? '—' : `${formatDeg(sim)}°`}</td>
                <td className={`delta ${delta != null && Math.abs(delta) > 2 ? 'warn' : ''}`}>{delta == null ? '—' : `${delta > 0.05 ? '+' : ''}${formatDeg(delta)}°`}</td></tr>;
            })}
          </tbody></table>
          <p className="consoleScope">实机列使用上方方向与零位换算，原始多圈角不归一化。此处比较的是实机反馈和独立仿真姿态；切换显示来源不会让任何电机移动。</p>
        </section>

        <section className="card glass consoleCard"><h3>仿真末端反馈 <small>mm</small></h3>
          <div className="consoleTcp">{['X', 'Y', 'Z'].map((axis, i) => <span key={axis}>{axis}<strong>{state?.xyz ? (state.xyz[i] * 1000).toFixed(1) : '—'}</strong></span>)}</div>
          <div className="consoleJoints">{(state?.q_deg || Array(6).fill(null)).map((angle, i) => <span key={i}>J{i + 1}<strong>{angle == null ? '—' : `${formatDeg(angle)}°`}</strong></span>)}</div>
          <p className="consoleScope">数据来源：MuJoCo 物理仿真<br />显示角度与轨迹使用实际反馈，不使用目标角度。</p>
        </section>
      </div>

      </details>
      <details className="card glass consoleAdvanced"><summary>高级调试 · 关节映射、软件备份与只读网关</summary>
        <details><summary>关节映射（首次验收/高级调试）</summary>
          <HardwareMappingPanel profile={mapping} onChange={setMapping} mapped={mapped} source={displaySource}
            onSource={(source) => { setDisplaySource(source); setCameraView('overview'); setGhost(false); }} />
          <p>显示映射仍保留原浏览器记录；软件批准配置独立保存，不会自动覆盖映射，也不把静态确认升级为动态验收。</p>
        </details>
        <h4>参数来源与备份状态</h4>
        <p>真实设备参数：{config.provenance?.device_parameters?.snapshot_count ? `${config.provenance.device_parameters.snapshot_count} 份局部快照，身份与完整备份未核验` : config.provenance?.device_parameters?.status === 'not-collected' ? '未采集，未验证备份' : '状态不可用，不能判断备份'}。来源：明确归档的只读查询或历史文件；不能证明设备当前值。</p>
        <p>待验证 MIT 配置：J1–J6 的 Kp=3、Kd=1、前馈=0 来自用户本次要求；不是设备读回或物理验收。既有真实调定值未被覆盖，B1 合成数据不用于实机。</p>
        <p>人工批准配置：{config.provenance?.approved_profile?.status === 'software-approved' ? `已有 ${Object.keys(config.provenance.approved_profile.axis_versions).length} 轴的软件批准记录` : config.provenance?.approved_profile?.status === 'not-approved' ? '尚无批准记录' : '状态不可用'}。来源为人工逐次软件审核，不证明已写入设备或已现场验证。</p>
        <h4>软件配置保护</h4><p>版本 {config.state?.revision ?? '未知'} · 设备身份未核验 · 真实控制始终锁定。导出不含设备序列号或快照；导入仅恢复草稿。</p>
        <div className="motorActions"><button disabled={config.busy} onClick={config.reload}>刷新软件配置</button><button disabled={!config.state || config.busy} onClick={exportConfig}>导出软件备份</button>
          <label>导入为草稿 <input type="file" accept="application/json,.json" disabled={!config.state || config.busy} onChange={importConfig} /></label></div>
        <h4>读数归档与通信记录</h4><p>连接和姿态同步入口已移到顶部。以下归档只保存已读取的数据，不改变电机。</p>
        <div className="motorActions">{MACHINE_IDS.map(id => <button key={id} disabled={!gateway.motors[id]?.updatedAt || !config.state || config.busy} onClick={() => archiveSnapshot(id)}>归档 J{id} 现有读数</button>)}</div>
        {gateway.log.length > 0 && <ul className="gwLog">
          {gateway.log.slice(-6).reverse().map((entry) => (
            <li key={entry.t} className={entry.level}>{new Date(entry.t).toLocaleTimeString()} {entry.text}</li>
          ))}
        </ul>}
      </details>

      <details className="consoleAdvanced card glass"><summary>手柄输入监视（只读）与接入边界</summary>
      <div className="consoleGrid">
        <section className="card glass consoleCard"><h3>手柄遥操作 <small>未接入</small></h3>
          <div className="padState"><span className={`badge ${gamepad.pads.length ? 'ok' : ''}`}>{gamepad.pads.length ? '已检测到手柄' : '未检测到手柄'}</span>
            <small className="consoleScope">{gamepad.pads[0]?.id || '按任意键唤醒手柄后自动识别'}</small></div>
          {gamepad.axes.length > 0 && <div className="padAxes">{gamepad.axes.slice(0, 6).map((value, i) => (
            <div className="padAxis" key={i}><span>轴 {i}</span>
              <span className="padBar"><i style={value >= 0 ? { left: '50%', width: `${value * 50}%` } : { right: '50%', width: `${-value * 50}%` }} /></span>
              <b>{value.toFixed(2)}</b></div>
          ))}</div>}
          {gamepad.buttons.length > 0 && <div className="padButtons">{gamepad.buttons.map((pressed, i) => (
            <span key={i} className={pressed ? 'on' : ''}>{i}</span>
          ))}</div>}
          <button disabled title="第 5 步接入：末端增量 → IK → 可达性预检 → 执行层">启用手柄控制末端</button>
          <p className="consoleNote">后端解算已就绪（<code className="consoleCmd">scripts/rebot_core.py: ik() / check_path()</code>），
            缺的是执行层与手柄映射：摇杆 → 末端 XYZ 增量 → 自动求各关节运动。</p>
        </section>

        <section className="card glass consoleCard"><h3>接入边界</h3><p>J1–J6 保留 MIT，夹爪单独核实。软件批准不授予实机权限；本地服务没有 CAN 执行实现。旧控制页恢复已审计的只读连接；真实写入仍锁定。</p><p>MIT 的 Kp、Kd、前馈及轨迹速度/加速度/jerk 须独立验证；PP/CSP 寄存器不提供 MIT 限速证明。现场动态映射、TCP、出力和停止行为未验收。</p></section>
      </div>
      </details>
    </section>
  );
}
