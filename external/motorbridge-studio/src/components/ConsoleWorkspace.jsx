import React from 'react';
import { ArmUrdfViewer } from './ArmUrdfViewer';
import { HardwareMappingPanel } from './HardwareMappingPanel';
import { loadMapping, mapHardwareFeedback } from '../lib/hardwareMapping';
import { CartesianPointPanel } from './CartesianPointPanel';
import { MotorModuleCard } from './MotorModuleCard';
import { useMotorGateway, MACHINE_IDS, MACHINE_MODELS } from '../hooks/useMotorGateway';
import { positionSpeedState, positionMotionRefusal } from '../lib/motorSettings';
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
  { id: 7, label: '夹爪' },
];

/**
 * 长按 1.5 秒才能打开的总开关。
 * 目的很单纯：不让一次误点就让七个电机同时有力矩。
 */
function ArmSwitch({ armed, onArm, onDisarm, online }) {
  const [progress, setProgress] = React.useState(0);
  const timerRef = React.useRef(null);
  const stop = React.useCallback(() => {
    clearInterval(timerRef.current);
    timerRef.current = null;
    setProgress(0);
  }, []);
  const start = () => {
    if (armed || !online) return;
    const began = Date.now();
    timerRef.current = setInterval(() => {
      const ratio = Math.min(1, (Date.now() - began) / 1500);
      setProgress(ratio);
      if (ratio >= 1) { stop(); onArm(); }
    }, 50);
  };
  React.useEffect(() => () => clearInterval(timerRef.current), []);
  return (
    <div className="armSwitch">
      <button className={armed ? 'danger' : ''} disabled={!online || armed}
        onPointerDown={start} onPointerUp={stop} onPointerLeave={stop} onPointerCancel={stop}>
        {armed ? '总开关：已打开' : (progress ? `按住不放 ${Math.round(progress * 100)}%` : '长按 1.5 秒打开总开关')}
      </button>
      {armed && <button onClick={onDisarm}>立即关闭</button>}
    </div>
  );
}

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
  const [machine, setMachine] = React.useState(null);
  const [document, setDocument] = React.useState(null);
  const [networkError, setNetworkError] = React.useState('');
  const [message, setMessage] = React.useState('');
  const [busy, setBusy] = React.useState(false);
  const [task, setTask] = React.useState('writing');
  const [cameraView, setCameraView] = React.useState(requestedHardwareView ? 'overview' : 'writing');
  const [resetViewSeq, setResetViewSeq] = React.useState(0);
  const [ghost, setGhost] = React.useState(!requestedHardwareView);
  const [showPlan, setShowPlan] = React.useState(true);
  const [showTrail, setShowTrail] = React.useState(true);
  const [now, setNow] = React.useState(Date.now());
  const commandBusy = React.useRef(false);
  const gamepad = useGamepad();
  const gateway = useMotorGateway({ intervalMs: 2500 });

  React.useEffect(() => {
    let disposed = false;
    let simTimer; let machineTimer;
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
    poll(); pollMachine();
    const clock = setInterval(() => setNow(Date.now()), 400);
    api('/writing-program').then((data) => { if (!disposed) setDocument(data); })
      .catch((error) => { if (!disposed) setMessage(`程序读取失败：${error.message}`); });
    return () => { disposed = true; clearTimeout(simTimer); clearTimeout(machineTimer); clearInterval(clock); };
  }, []);

  const problem = networkError || feedbackProblem(state, now);
  const job = state?.job;
  const active = job?.status === 'running' || job?.status === 'paused';
  const moving = Boolean(state?.manual_motion || state?.playing);
  const locked = Boolean(problem || busy);
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
          <h2>控制台 <small>使能 · 限速 · 任务执行 · 手柄遥操作</small></h2>
          <p>电机模块负责使能与参数；任务程序自动计算各关节目标。目前自动任务执行于仿真，实机同步执行尚未接入。</p></div>
        <div className="consoleBadges">
          <span className={`badge ${problem ? 'err' : 'ok'}`}>{problem ? '仿真反馈异常' : '仿真在线'}</span>
          <span className={`badge ${gatewayOnline ? 'ok' : (gateway.status === 'offline' ? 'err' : '')}`}>
            {gatewayOnline ? `网关 ${readCount}/7 轴` : `网关：${GATEWAY_STATUS[gateway.status] || gateway.status}`}
          </span>
          <span className={`badge ${gateway.armed ? 'err' : ''}`}>{gateway.armed ? (gateway.drill ? '总开关开 · 演练' : '总开关开 · 实发') : '实机只读'}</span>
        </div>
        {gatewayOnline && (
          <div className="consoleEmergency">
            <button className="danger" onClick={() => gateway.stopAll()}>急停（对七轴发 stop）</button>
          </div>
        )}
      </div>
      {problem && <div className="consoleAlert" role="alert">{problem}。画面保留最后一次仿真反馈。</div>}
      {!gatewayOnline && <div className="consoleStatus" role="status">
        <strong>{gateway.error || `网关${GATEWAY_STATUS[gateway.status] || gateway.status}`}</strong>
        <span>七个电机模块将没有实机读数，也无法下发。可在下方「网关与总控」里重连。</span>
      </div>}

      <div className="card glass consoleGateway">
        <div className="gwRow">
          <label className="gwUrl">网关地址
            <input value={gateway.endpoint} onChange={(e) => gateway.setEndpoint(e.target.value)}
              disabled={gatewayOnline || gateway.status === 'connecting'} placeholder="ws://127.0.0.1:9002" />
          </label>
          <button disabled={gateway.status === 'connecting'} onClick={() => gateway.connect()}>
            {gatewayOnline ? '重连' : '连接'}</button>
          <button disabled={gateway.status === 'disconnected'} onClick={() => gateway.disconnect()}>断开</button>
          <span className="gwState">
            {GATEWAY_STATUS[gateway.status] || gateway.status}
            {gatewayOnline && ` · 每轮 ${gateway.cycleMs} ms · ${(gateway.updatedAt ? formatAge((now - gateway.updatedAt) / 1000) : '等待读取')}`}
          </span>
        </div>
        <div className="gwRow">
          <ArmSwitch armed={gateway.armed} onArm={gateway.arm} onDisarm={gateway.disarm} online={gatewayOnline} />
          <label className="gwCheck">
            <input type="checkbox" checked={gateway.drill} onChange={(e) => gateway.setDrill(e.target.checked)} />
            演练模式（走完全部校验但不发帧）
          </label>
          <label className="gwSpd">限速阈值
            <input type="number" step="0.05" min="0.05" max="2" value={gateway.maxSpd}
              onChange={(e) => gateway.setMaxSpd(Number(e.target.value) || 0.05)} />
            <small>rad/s，超过就不让使能也不让动</small>
          </label>
        </div>
        <p className="gwWarn">
          打开总开关只会让按钮可点；真正下发还要过限速兵与限位检查。
          急停不受总开关与演练模式限制，连着就能用。它不是硬件急停链，断电仍需现场处理。
        </p>
        {gateway.log.length > 0 && <ul className="gwLog">
          {gateway.log.slice(-6).reverse().map((entry) => (
            <li key={entry.t} className={entry.level}>{new Date(entry.t).toLocaleTimeString()} {entry.text}</li>
          ))}
        </ul>}
      </div>

      <HardwareMappingPanel profile={mapping} onChange={setMapping} mapped={mapped} source={displaySource}
        onSource={(source) => { setDisplaySource(source); setCameraView('overview'); setGhost(false); }} />

      <CartesianPointPanel state={state} locked={locked} active={active} moving={moving} now={now}
        onPreview={(plan) => { setPointPlan(plan); if (plan) { setDisplaySource('simulation'); setCameraView('overview'); setShowPlan(true); setGhost(false); } }} onBusy={(value) => { commandBusy.current = value; setBusy(value); }}
        onExecuted={() => { setDisplaySource('simulation'); setCameraView('overview'); setGhost(false); setShowPlan(true); }} />

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

      <div className="consoleModulesHead">
        <h3>电机使能与参数 <small>逐轴配置；位置由任务程序自动计算</small></h3>
        <span>实机读数来源：{realSource}
          {gateway.cycleMs ? ` · 每轮 ${gateway.cycleMs} ms` : ''}</span>
      </div>
      <div className="consoleModules">
        {AXES.map((axis) => <MotorModuleCard key={axis.id} id={axis.id} label={axis.label}
          motor={gateway.motors?.[axis.id]} status={gateway.status}
          plannedDeg={axis.id <= 6 ? state?.target_deg?.[axis.id - 1] : null} gateway={gateway} />)}
      </div>

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

        <section className="card glass consoleCard"><h3>虚实逐轴对比 <small>度</small></h3>
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

      <div className="consoleGrid">
        <section className="card glass consoleCard"><h3>速度上限概览 <small>逐轴设置见上方</small></h3>
          <table className="paramTable"><thead><tr><th>轴</th><th>当前模式速度上限</th><th>评估</th></tr></thead><tbody>
            {MACHINE_IDS.map((id) => {
              const motor = gateway.motors?.[id];
              const speedState = positionSpeedState(motor);
              const value = speedState.value;
              const reason = positionMotionRefusal(motor, gateway.maxSpd);
              return <tr key={id}><td>{id === 7 ? '夹爪' : `J${id}`} <span className="code">{MACHINE_MODELS[id]}</span></td>
                <td>{speedState.label} · {value == null ? '—' : `${value.toFixed(3)} rad/s`}</td>
                <td className={reason ? 'warn' : 'ok'}>{reason || '对应模式的限速检查通过'}</td></tr>;
            })}
          </tbody></table>
          <p className="consoleNote">首次实机动作建议压到 0.3 rad/s（约 17°/s）以下再谈放开。
            逐轴的「写限速」在电机模块里，需要先打开总开关。</p>
          <p className="consoleScope">请在对应电机卡片输入速度上限并点击「应用」。修改参数不会发送目标位置。</p>
        </section>

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

        <section className="card glass consoleCard"><h3>接入边界 <small>必读</small></h3>
          <p className="consoleNote">
            <strong>写入要过三道闸，都在 <code className="consoleCmd">useMotorGateway.js</code> 里强制：</strong>
            总开关（长按 1.5 秒）→ 演练开关 → 模式限速检查（PP 用 vel_max，CSP 用 limit_spd）
            与限位/单次位移检查（单次上限 15°）。<br />
            允许的写 op 仅：enable、disable、stop、ensure_mode、pos_vel、mit、vel、
            robstride_write_param。<strong>set_id、写零位、clear_error 不在白名单里。</strong><br />
            使能状态：<strong>读不到</strong>。RS 固件的 get_state 与流式帧在本机失效，
            模块只记录最近一次下发指令，不将网关应答或演练结果当作真实使能状态。<br />
            给未使能的电机发运动帧是<strong>静默无操作</strong>（总线返回 ok、固件忽略），
            所以顺序必须是先使能再下发。<br />
            红线：下发任何实机运动指令前必须确认正方向、零位、限位，且现场有人能断电。
          </p>
        </section>
      </div>
    </section>
  );
}
