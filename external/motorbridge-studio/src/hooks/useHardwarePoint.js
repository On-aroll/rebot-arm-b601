import React from 'react';

async function request(path, body, token, keepalive = false, timeout = 5000) {
  const response = await fetch(`/hardware-point/${path}`, body ? {
    method: 'POST', headers: { 'Content-Type': 'application/json', 'X-Motor-Session': token || '' },
    body: JSON.stringify(body), keepalive, signal: keepalive ? undefined : AbortSignal.timeout(timeout),
  } : { signal: AbortSignal.timeout(3000), cache: 'no-store' });
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || '实机请求未确认');
  return data;
}

// Reuses the existing server-side SDK worker and lease; never sends CAN frames.
export function useHardwarePoint({ beforeExecute, onFinished }) {
  const [state, setState] = React.useState(null);
  const [pending, setPending] = React.useState(false);
  const [error, setError] = React.useState('');
  const token = React.useRef(null), owned = React.useRef(false), active = React.useRef(false);
  const starting = React.useRef(false), stopping = React.useRef(false), generation = React.useRef(0);
  const sequence = React.useRef(0), mounted = React.useRef(true), callbacks = React.useRef({});
  callbacks.current = { beforeExecute, onFinished };

  const accept = React.useCallback(value => {
    const next = value.status || value;
    if (!mounted.current || !next.state) return;
    setState(next);
    active.current = Boolean(next.busy);
    if (owned.current && !starting.current && !next.busy) {
      owned.current = false;
      callbacks.current.onFinished?.(next);
    }
  }, []);
  const session = React.useCallback(async (fresh = false) => {
    if (fresh || !token.current) token.current = (await request('session', {})).token;
    if (!token.current) throw new Error('未取得本机实机控制会话');
    return token.current;
  }, []);
  const stop = React.useCallback(async () => {
    if (stopping.current) return;
    generation.current += 1; stopping.current = true; active.current = false;
    try { accept(await request('action', { action: 'stop' }, await session(), true)); }
    catch (failure) { if (mounted.current) setError(`停止未确认：${failure.message}；请使用现场独立断电`); }
    finally { stopping.current = false; }
  }, [accept, session]);
  const execute = React.useCallback(async (planId, confirmed, observeStartup = false, nativeAxis = 0) => {
    if (!confirmed) throw new Error('请勾选上方一次现场确认');
    if (![0, 3, 6, 7].includes(nativeAxis)) throw new Error('请选择既定 J3、J6 或 J7 小幅试验');
    if (starting.current || active.current) throw new Error('实机任务尚未结束');
    const ticket = ++generation.current;
    starting.current = true; setPending(true); setError('');
    try {
      // Renew before a new explicit task, never retry a possibly delivered start.
      await session(true);
      await callbacks.current.beforeExecute();
      owned.current = true;
      if (ticket !== generation.current || !mounted.current) throw new Error('执行已取消，未发送新任务');
      if (!document.hasFocus() || document.visibilityState !== 'visible') throw new Error('操作页面已失焦，未发送实机任务');
      if (nativeAxis) {
        const prepared = await request('action', { action: `prepare-csp-j${nativeAxis}` }, token.current, false, 15000);
        if (ticket !== generation.current || !mounted.current) throw new Error('CSP 准备已取消，未发送试验');
        if (!document.hasFocus() || document.visibilityState !== 'visible') throw new Error('操作页面已失焦，未发送 CSP 试验');
        if (!prepared.plan_id) throw new Error(prepared.status?.reason || 'CSP 只读准备未通过');
        planId = prepared.plan_id;
      }
      const result = await request('action', { action: nativeAxis ? `csp-j${nativeAxis}-trial` : observeStartup ? 'probe-startup' : 'execute', plan_id: planId, confirmed: {
        operator_present: true, power_cut_ready: true, supported: true, space_clear: true, accept_proposal: true,
      } }, token.current);
      if (ticket !== generation.current || !mounted.current) { await stop(); throw new Error('执行请求已取消'); }
      accept(result);
      return result;
    } catch (failure) {
      // A timeout can arrive after the worker starts. Stop before any reconnect.
      if (owned.current) await stop();
      if (mounted.current) setError(failure.message);
      throw failure;
    } finally { starting.current = false; if (mounted.current) setPending(false); }
  }, [accept, session, stop]);

  React.useEffect(() => {
    mounted.current = true;
    let polling = false, beating = false;
    const poll = async () => {
      if (polling) return;
      const ticket = generation.current; polling = true;
      try { const result = await request('state'); if (ticket === generation.current && !starting.current) accept(result); }
      catch (failure) { if (owned.current) { await stop(); if (mounted.current) setError(failure.message); } }
      finally { polling = false; }
    };
    const heartbeat = async () => {
      if (!owned.current || !active.current || beating || starting.current || stopping.current) return;
      const ticket = generation.current;
      beating = true;
      try { await request('action', { action: 'heartbeat', sequence: ++sequence.current,
        focused: document.hasFocus() && document.visibilityState === 'visible' }, token.current); }
      catch (failure) {
        if (ticket !== generation.current || !mounted.current) return;
        // A worker can finish between the last poll and this heartbeat. Keep
        // its actual terminal reason instead of replacing it with a lease error.
        try {
          const latest = await request('state');
          if (ticket !== generation.current || !mounted.current) return;
          const status = latest.status || latest;
          if (status.state && status.busy === false) { accept(latest); return; }
        } catch { /* Unknown/live task: retain the existing stop-on-failure. */ }
        if (ticket !== generation.current || !mounted.current || !owned.current || !active.current) return;
        await stop(); if (mounted.current) setError(failure.message);
      }
      finally { beating = false; }
    };
    const blur = () => { if (owned.current || starting.current) void stop(); };
    const visibility = () => { if (document.visibilityState !== 'visible') blur(); };
    void poll();
    const timer = setInterval(poll, 250), lease = setInterval(heartbeat, 250);
    window.addEventListener('blur', blur); document.addEventListener('visibilitychange', visibility);
    return () => {
      mounted.current = false; generation.current += 1;
      clearInterval(timer); clearInterval(lease);
      window.removeEventListener('blur', blur); document.removeEventListener('visibilitychange', visibility);
      if (owned.current && token.current) void request('action', { action: 'stop' }, token.current, true).catch(() => {});
    };
  }, [accept, stop]);
  return { state, pending, busy: Boolean(pending || state?.busy), execute,
    executeCspJ3: confirmed => execute(null, confirmed, false, 3),
    executeCspJ6: confirmed => execute(null, confirmed, false, 6),
    executeCspJ7: confirmed => execute(null, confirmed, false, 7), stop, error };
}
