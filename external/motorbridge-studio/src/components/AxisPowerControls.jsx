import React from 'react';

const CONDITIONS = [
  ['operator_present', '我在机械臂旁，周围空间已清空'],
  ['power_cut_ready', '我能立即切断电机电源'],
  ['supported', '该轴及承重关节已可靠支撑，撤力不会跌落'],
  ['axis_verified', '我已确认所选轴、方向、零位与限位'],
];

export function AxisPowerControls({ id, gateway, blocked = false, limits }) {
  const [operation, setOperation] = React.useState(null);
  const [confirmed, setConfirmed] = React.useState({});
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
    if (!online) { setOperation(null); setConfirmed({}); }
  }, [online]);
  function choose(op) { setOperation(op); setConfirmed({}); setError(''); }
  async function submitEnable() {
    if (disabled || (operation === 'enable' && blocked) || !operation || !CONDITIONS.every(([key]) => confirmed[key])) return;
    const ticket = ++actionEpoch.current;
    setEnabling(true); setError('');
    try {
      await gateway.requestPower(id, 'enable', confirmed, limits);
      if (ticket === actionEpoch.current) { setOperation(null); setConfirmed({}); }
    } catch (failure) { if (ticket === actionEpoch.current) setError(`请求未确认：${failure.message}。请现场核对出力状态，必要时断电。`); }
    finally { setEnabling(false); }
  }
  async function emergencyDisable() {
    if (disableDisabled) return;
    actionEpoch.current += 1;
    setDisabling(true); setError(''); setOperation(null); setConfirmed({});
    try {
      await gateway.requestPower(id, 'disable', {}, undefined, { emergency: true });
      setOperation(null); setConfirmed({});
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
      {CONDITIONS.map(([key, label]) => <label key={key}><input type="checkbox" checked={Boolean(confirmed[key])} disabled={pending} onChange={event => setConfirmed(previous => ({ ...previous, [key]: event.target.checked }))} />{label}</label>)}
      <div className="consoleActions"><button className="primary" disabled={disabled || (operation === 'enable' && blocked) || !CONDITIONS.every(([key]) => confirmed[key])} onClick={submitEnable}>{enabling ? '正在发送…' : '确认使能 J' + id}</button>
        <button disabled={pending} onClick={() => setOperation(null)}>取消</button></div>
    </div>}
    {error && <p className="motorPowerError" role="alert">{error}</p>}
  </div>;
}
