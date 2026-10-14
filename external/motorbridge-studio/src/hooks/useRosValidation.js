import React from 'react';

// This bridge is fixed to ROS Fake Driver. It never connects a CAN gateway.
export function useRosValidation() {
  const [state, setState] = React.useState(null);
  const [error, setError] = React.useState('');
  const [pending, setPending] = React.useState(false);
  const alive = React.useRef(true);
  const actionPending = React.useRef(false);
  const revision = React.useRef(0);

  const request = React.useCallback(async (action) => {
    const response = await fetch(action ? '/ros-control/action' : '/ros-control/state', {
      method: action ? 'POST' : 'GET',
      headers: action ? { 'Content-Type': 'application/json' } : undefined,
      body: action ? JSON.stringify({ action }) : undefined,
      signal: AbortSignal.timeout(5000),
    });
    const next = await response.json();
    if (!response.ok) throw new Error(next.error || `HTTP ${response.status}`);
    if (next.backend !== 'ros2-fake' || next.hardware !== false || next.namespace !== 'rebotarm_validation') {
      throw new Error('反馈来源不是隔离的虚拟电机');
    }
    return next;
  }, []);

  const refresh = React.useCallback(async () => {
    const version = revision.current;
    try {
      const next = await request();
      if (alive.current && version === revision.current) { setState(next); setError(''); }
      return next;
    } catch (e) {
      if (alive.current && version === revision.current) { setState(null); setError(e.message); }
      return null;
    }
  }, [request]);

  React.useEffect(() => {
    alive.current = true;
    let timer;
    async function poll() {
      await refresh();
      if (alive.current) timer = setTimeout(poll, 200);
    }
    poll();
    return () => { alive.current = false; clearTimeout(timer); };
  }, [refresh]);

  async function action(op) {
    const immediate = op === 'stop' || op === 'disable';
    if (actionPending.current && !immediate) return;
    const version = ++revision.current;
    actionPending.current = true; setPending(true);
    try {
      const next = await request(op);
      if (alive.current && version === revision.current) { setState(next); setError(''); }
      return next;
    } catch (e) {
      if (alive.current && version === revision.current) setError(e.message);
      return null;
    } finally {
      if (version === revision.current) { actionPending.current = false; if (alive.current) setPending(false); }
    }
  }

  function acceptFeedback(next) {
    // Execution replies update controls immediately; discard older in-flight polls.
    if (next?.backend !== 'ros2-fake' || next.hardware !== false || next.namespace !== 'rebotarm_validation') {
      setError('反馈来源不是隔离的虚拟电机'); return;
    }
    ++revision.current;
    setState(next); setError('');
  }

  return { state, error, pending, refresh, action, acceptFeedback,
    running: ['starting', 'running', 'cancelling'].includes(state?.job?.status) };
}
