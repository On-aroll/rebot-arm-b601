import React, { useCallback, useEffect, useRef, useState } from 'react';

const TERMINAL = new Set(['IDLE', 'ARRIVED', 'CANCELLED', 'FAULT']);
const LABELS = { IDLE: '尚未使能', CHECKING: '正在检查', READY: '准备完成，等待你确认', PREPARING: '正在启动试验', RUNNING: '正在请求保持当前位置', STOPPING: '正在停止并撤力', ARRIVED: '试验流程结束', CANCELLED: '试验已取消', FAULT: '试验故障' };
const CONDITIONS = [
  ['operator_present', '我在机械臂旁，并会全程观察'],
  ['power_cut_ready', '我能立即切断电机电源'],
  ['supported', '机械臂已固定，J2/J3 等关节已有可靠支撑'],
  ['space_clear', 'J1 周围无碰撞障碍，人员已避开运动范围'],
];

async function request(path, options = {}, timeout = 20000) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeout);
  try {
    const response = await fetch(path, { ...options, signal: controller.signal, cache: 'no-store' });
    const data = await response.json();
    if (!response.ok || data.ok === false) throw Object.assign(new Error(data.error || data.reason || '请求未完成'), { payload: data });
    return data;
  } catch (error) {
    if (error.name === 'AbortError') throw new Error('通信超时，不能确认电机状态；请现场观察，必要时立即断电。');
    throw error;
  } finally { clearTimeout(timer); }
}

export function MotorPowerPanel({ beforePrepare, onBusyChange }) {
  const [status, setStatus] = useState(null);
  const [plan, setPlan] = useState(null);
  const [confirmed, setConfirmed] = useState({});
  const [pending, setPending] = useState('');
  const [error, setError] = useState('');
  const token = useRef(null);
  const active = useRef(false);
  const sequence = useRef(0);
  const generation = useRef(0);
  const mounted = useRef(true);
  const pendingRef = useRef('');
  const stopPending = useRef(false);
  const heartbeatPending = useRef(false);
  const busyCallback = useRef(onBusyChange);
  busyCallback.current = onBusyChange;

  const acceptStatus = useCallback(data => {
    const next = data?.status && typeof data.status === 'object' ? data.status : data;
    if (!next?.state || !mounted.current) return;
    setStatus(next);
    if (TERMINAL.has(next.state) && !pendingRef.current) {
      active.current = false;
      setPlan(null);
    }
  }, []);

  const action = useCallback((body, options = {}, timeout = 20000) => request('/motor-power/action', {
    method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Motor-Session': token.current },
    body: JSON.stringify(body), ...options,
  }, timeout), []);

  const ensureSession = useCallback(async () => {
    if (!token.current) {
      const session = await request('/motor-power/session', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' });
      if (!session.token) throw new Error('服务未返回控制会话。');
      token.current = session.token;
    }
  }, []);

  const stop = useCallback(async () => {
    if (stopPending.current) return;
    generation.current += 1;
    active.current = false;
    stopPending.current = true;
    setError('');
    setPlan(null);
    try {
      await ensureSession();
      let result;
      try { result = await action({ action: 'stop' }); }
      catch (failure) {
        // Only retry a stop after the server explicitly rejects an expired session.
        // Never refresh a session to retry start or heartbeat automatically.
        if (!failure.payload?.error?.startsWith('单轴操作会话失效')) throw failure;
        token.current = null;
        await ensureSession();
        result = await action({ action: 'stop' });
      }
      acceptStatus(result);
    }
    catch (failure) { if (mounted.current) setError(`停止请求未确认：${failure.message} 请现场观察，必要时立即断电。`); }
    finally { stopPending.current = false; }
  }, [action, acceptStatus, ensureSession]);

  useEffect(() => {
    mounted.current = true;
    let pollPending = false;
    const poll = async () => {
      if (pollPending) return;
      pollPending = true;
      try { acceptStatus(await request('/motor-power', {}, 3000)); }
      catch (failure) { if (mounted.current) setError(`状态读取失败：${failure.message}`); }
      finally { pollPending = false; }
    };
    const heartbeat = async () => {
      if (!active.current || heartbeatPending.current) return;
      const focused = document.visibilityState === 'visible' && document.hasFocus();
      const ticket = generation.current;
      heartbeatPending.current = true;
      try { await action({ action: 'heartbeat', focused, sequence: ++sequence.current }, {}, 1500); }
      catch (failure) {
        if (!mounted.current || ticket !== generation.current) return;
        // A rejected heartbeat must not leave a client continuing the old lease.
        active.current = false;
        await stop();
        if (mounted.current) setError(`控制通信中断：${failure.message} 已请求停止，停止情况请以现场为准。`);
      } finally { heartbeatPending.current = false; }
    };
    const loseFocus = () => {
      if (!active.current) return;
      // Send the loss of focus immediately, even if an earlier heartbeat is pending.
      void action({ action: 'heartbeat', focused: false, sequence: ++sequence.current }, { keepalive: true }).catch(() => {});
    };
    const visibility = () => { if (document.visibilityState !== 'visible') loseFocus(); };
    void poll();
    const pollTimer = setInterval(poll, 500);
    const heartbeatTimer = setInterval(heartbeat, 250);
    window.addEventListener('blur', loseFocus);
    document.addEventListener('visibilitychange', visibility);
    return () => {
      mounted.current = false;
      generation.current += 1;
      clearInterval(pollTimer);
      clearInterval(heartbeatTimer);
      window.removeEventListener('blur', loseFocus);
      document.removeEventListener('visibilitychange', visibility);
      if (token.current && (active.current || pendingRef.current)) {
        void fetch('/motor-power/action', {
          method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Motor-Session': token.current },
          body: JSON.stringify({ action: 'stop' }), keepalive: true,
        }).catch(() => {});
      }
      active.current = false;
    };
  }, [action, acceptStatus, stop]);

  const busy = Boolean(status?.busy || pending || plan);
  useEffect(() => { busyCallback.current?.(busy); }, [busy]);
  useEffect(() => () => { busyCallback.current?.(false); }, []);

  const prepare = async () => {
    if (pendingRef.current || status?.busy) return;
    const ticket = ++generation.current;
    pendingRef.current = 'prepare'; setPending('prepare'); setError(''); setPlan(null); setConfirmed({});
    try {
      await beforePrepare?.();
      if (ticket !== generation.current || !mounted.current) return;
      await ensureSession();
      if (ticket !== generation.current || !mounted.current) return;
      const data = await action({ action: 'prepare' });
      if (ticket !== generation.current || !mounted.current) return;
      acceptStatus(data);
      if (!data.plan_id || !data.proposal) throw new Error(data.reason || data.status?.reason || '检查未产生完整试验方案，请查看原因后重试。');
      setPlan({ id: data.plan_id, proposal: data.proposal });
    } catch (failure) { acceptStatus(failure.payload); if (mounted.current) setError(failure.message); }
    finally { pendingRef.current = ''; if (mounted.current) setPending(''); }
  };

  const allConfirmed = CONDITIONS.every(([key]) => confirmed[key]) && confirmed.accept_proposal;
  const start = async () => {
    if (!plan || !allConfirmed || pendingRef.current || status?.busy) return;
    const ticket = generation.current;
    pendingRef.current = 'start'; setPending('start'); setError('');
    try {
      const data = await action({ action: 'start', plan_id: plan.id, confirmed });
      if (ticket !== generation.current || !mounted.current) {
        void action({ action: 'stop' }, { keepalive: true }).catch(() => {});
        return;
      }
      acceptStatus(data);
      setPlan(null);
      active.current = !TERMINAL.has((data.status || data).state);
      sequence.current = 0;
    } catch (failure) {
      // A timed-out response can still have reached the service.
      await stop();
      if (mounted.current) setError(`启动未确认：${failure.message} 已另发停止请求，停止情况请以现场为准。`);
    } finally { pendingRef.current = ''; if (mounted.current) setPending(''); }
  };

  return <section id="motor-power-panel" className="card glass motorPowerPanel" aria-label="电机使能与失能">
    <div className="deviceHeading"><div><h3>电机使能 / 失能</h3><p>先试 J1 底座：保持读取到的当前位置约 3 秒，然后撤力；启动和撤力通信另计。其他轴和 XYZ 实机运动尚未开放。</p></div>
      <strong role="status">{LABELS[status?.state] || '正在读取状态'}</strong></div>
    <div className="consoleActions motorPowerActions">
      <button className="primary" disabled={Boolean(pending || status?.busy || plan)} onClick={prepare}>{pending === 'prepare' ? '正在检查并准备…' : '检查并准备 J1'}</button>
      <button className="danger" disabled={(!token.current && !status?.busy && !pending) || stopPending.current} onClick={stop}>停止当前试验</button>
      <button disabled={(!token.current && !status?.busy && !pending) || stopPending.current} onClick={stop}>失能 J1</button>
    </div>
    <p className="motorPowerHint">停止与失能会撤去本会话 J1 的力矩，不提供重力保持；不会批量失能未知状态的电机。</p>
    {plan && <div className="motorPowerProposal">
      <strong>{plan.proposal.title || 'J1 使能保持试验'}</strong><p>{plan.proposal.summary}</p><p className="motorPowerHint">{plan.proposal.limitations}</p>
      <details><summary>本次运动限制与停止条件</summary><pre>{JSON.stringify({ limits: plan.proposal.limits, policy: plan.proposal.policy, config_revision: plan.proposal.config_revision }, null, 2)}</pre></details>
      <fieldset><legend>现场条件确认</legend>{CONDITIONS.map(([key, label]) => <label key={key}><input type="checkbox" checked={Boolean(confirmed[key])} onChange={event => setConfirmed(previous => ({ ...previous, [key]: event.target.checked }))} />{label}</label>)}</fieldset>
      <label className="motorPowerAccept"><input type="checkbox" checked={Boolean(confirmed.accept_proposal)} onChange={event => setConfirmed(previous => ({ ...previous, accept_proposal: event.target.checked }))} />我同意上述本次方案，使能后观察 J1 出力，约 3 秒撤力</label>
      <button className="primary" disabled={!allConfirmed || Boolean(pending || status?.busy)} onClick={start}>{pending === 'start' ? '正在请求使能…' : '确认使能 J1 · 约 3 秒'}</button>
      <p className="motorPowerHint">方案只可使用一次，60 秒后失效；改变现场状态后请重新检查。</p>
    </div>}
    {(status?.state === 'RUNNING' || status?.state === 'PREPARING') && <p>本会话已请求使能，真实出力需现场观察。请保持本页处于前台并有焦点。</p>}
    {status?.reason && <p>{status.reason}</p>}
    {status?.feedback && <div className="motorPowerFeedback"><table><caption>本次检查读到的原始反馈</caption><thead><tr><th>关节</th><th>角度（°）</th><th>速度（rad/s）</th><th>模式</th></tr></thead><tbody>{[1,2,3,4,5,6].map(id => {
      const sample = status.feedback[id];
      return <tr key={id}><th>J{id}</th><td>{Number.isFinite(sample?.q) ? (sample.q * 180 / Math.PI).toFixed(3) : '未读到'}</td><td>{Number.isFinite(sample?.qd) ? sample.qd.toFixed(4) : '未读到'}</td><td>{sample?.mode === 0 ? 'MIT' : Number.isFinite(sample?.mode) ? sample.mode : '未读到'}</td></tr>;
    })}</tbody></table></div>}
    {status?.result && <details><summary>查看本次结果</summary><pre>{JSON.stringify(status.result, null, 2)}</pre></details>}
    {error && <p className="motorPowerError" role="alert">{error}</p>}
    <p className="motorPowerHint">当前设备没有可用的“已使能”反馈位；流程结束不等于已完成实机验收。使能试验不会设置零位或修改已调参数。</p>
  </section>;
}
