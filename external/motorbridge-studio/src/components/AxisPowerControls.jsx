import React from 'react';

const CONFIRMATION_KEYS = ['operator_present', 'power_cut_ready', 'supported', 'axis_verified'];

export function AxisPowerControls({ id, gateway, blocked = false, limits }) {
  const [operation, setOperation] = React.useState(null);
  const [confirmed, setConfirmed] = React.useState(false);
  const [error, setError] = React.useState('');
  const [enabling, setEnabling] = React.useState(false);
  const [disabling, setDisabling] = React.useState(false);
  const actionEpoch = React.useRef(0);
  const online = gateway?.status === 'online';
  const pending = gateway?.powerPending != null || enabling || disabling;
  const disabled = !online || pending || !gateway?.requestPower;
  const disableDisabled = !online || disabling || gateway?.powerDisabling?.includes(id) || !gateway?.requestPower;
  const result = gateway?.powerRequests?.[id];
  React.useEffect(() => {
    if (!online) { setOperation(null); setConfirmed(false); }
  }, [online]);
  function choose(op) { setOperation(op); setConfirmed(false); setError(''); }
  async function submitEnable() {
    if (disabled || (operation === 'enable' && blocked) || !operation || !confirmed) return;
    const ticket = ++actionEpoch.current;
    setEnabling(true); setError('');
    try {
      const evidence = Object.fromEntries(CONFIRMATION_KEYS.map(key => [key, true]));
      await gateway.requestPower(id, 'enable', evidence, limits);
      if (ticket === actionEpoch.current) { setOperation(null); setConfirmed(false); }
    } catch (failure) { if (ticket === actionEpoch.current) setError(`请求未确认：${failure.message}。请现场核对出力状态，必要时断电。`); }
    finally { setEnabling(false); }
  }
  async function emergencyDisable() {
    if (disableDisabled) return;
    actionEpoch.current += 1;
    setDisabling(true); setError(''); setOperation(null); setConfirmed(false);
    try {
      await gateway.requestPower(id, 'disable', {}, undefined, { emergency: true });
      setOperation(null); setConfirmed(false);
    } catch (failure) { setError(`失能请求未确认：${failure.message}。请立即断电。`); }
    finally { setDisabling(false); }
  }
  return <div className="axisPowerControls" aria-label={`J${id} 使能与失能`}>
    <div className="axisPowerRow">
      <span role="status">{result?.error ? '电源请求未确认 · 出力未知' : result?.op === 'enable' ? '真实使能请求已发送 · 出力待确认' : result?.op === 'disable' ? '真实失能请求已发送 · 撤力待确认' : '真实使能状态未知'}</span>
      <div><button className="axisEnable" disabled={disabled || blocked} aria-label={`使能 J${id}`} title="仅对所选实机轴发送使能，不改变模式或目标" onClick={() => choose('enable')}>实机使能</button>
        <button className="axisDisable" disabled={disableDisabled} aria-label={`立即失能 J${id}`} title="单击发送所选轴失能请求；承重关节须支撑，撤力需现场确认" onClick={emergencyDisable}>{disabling || gateway?.powerDisabling?.includes(id) ? '正在请求失能…' : '立即失能'}</button></div>
    </div>
    {operation && <div className="axisPowerConfirmation" role="group" aria-label={`确认${operation === 'enable' ? '使能' : '失能'} J${id}`}>
      <strong>J{id} · {operation === 'enable' ? '使能（允许电机出力）' : '失能（撤去电机力矩）'}</strong>
      <p>{operation === 'enable' ? '使能可能产生力矩或动作；不发送新目标，也不保证当前位置保持。' : '失能会撤力，承重关节可能下落。'}{id === 7 ? '夹爪位移与 TCP 尚未标定。' : ''}</p>
      {id <= 6 && <p>起点读数允许 1° 边界偏差，原始角度和运动目标限位不变。使能不等于承重保持，请使用机械支撑，不用手托着试验。</p>}
      <label><input type="checkbox" checked={confirmed} disabled={pending} onChange={event => setConfirmed(event.target.checked)} />我已确认：在场且周围空间清空、可立即断电、机械支撑可靠且撤力不会跌落，以及 J{id} 的方向、零位和限位。</label>
      <div className="consoleActions"><button className="primary" disabled={disabled || (operation === 'enable' && blocked) || !confirmed} onClick={submitEnable}>{enabling ? '正在发送…' : '确认使能 J' + id}</button>
        <button disabled={pending} onClick={() => setOperation(null)}>取消</button></div>
    </div>}
    {error && <p className="motorPowerError" role="alert">{error}</p>}
  </div>;
}
