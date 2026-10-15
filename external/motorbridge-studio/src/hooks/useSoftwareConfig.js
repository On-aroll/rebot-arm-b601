import React from 'react';

export function useSoftwareConfig() {
  const [data, setData] = React.useState(null);
  const [error, setError] = React.useState('');
  const [busy, setBusy] = React.useState(false);
  const session = React.useRef(null);
  const mutex = React.useRef(false);
  const send = React.useCallback(async (path, body, token) => {
    const response = await fetch(path, { method: 'POST', headers: {
      'Content-Type': 'application/json', ...(token ? { 'X-Config-Session': token } : {}),
    }, body: JSON.stringify(body), signal: AbortSignal.timeout(10000) });
    const result = await response.json();
    if (!response.ok || result.error) throw new Error(result.error || `HTTP ${response.status}`);
    return result;
  }, []);
  const reload = React.useCallback(async () => {
    if (mutex.current) return;
    mutex.current = true; setBusy(true);
    try {
      const result = await send('/config/session', {});
      session.current = { token: result.token, seq: 0 };
      setData(result); setError(result.error || '');
    } catch (e) { session.current = null; setError(e.message); }
    finally { mutex.current = false; setBusy(false); }
  }, [send]);
  React.useEffect(() => { reload(); }, [reload]);
  const action = React.useCallback(async (body) => {
    if (mutex.current || !session.current) throw new Error('配置服务忙或未连接，请刷新配置');
    mutex.current = true; setBusy(true);
    try {
      const active = session.current;
      const result = await send('/config/action', { revision: data?.state?.revision, ...body, seq: ++active.seq }, active.token);
      if (result.state) setData(prev => ({ ...prev, ...result }));
      setError(''); return result;
    } catch (e) { setError(e.message); throw e; }
    finally { mutex.current = false; setBusy(false); }
  }, [data, send]);
  return { ...data, error, busy, action, reload };
}
