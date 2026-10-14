import React from 'react';

const EVIDENCE = { operator_present: true, power_cut_ready: true, supported: true, axis_verified: true };

export function MachinePowerControls({ gateway, limits, blocked = false, onConfirmationChange, taskActive = false, onStopTask }) {
  const [confirmed, setConfirmed] = React.useState(false);
  const [enabling, setEnabling] = React.useState(false);
  const [disabling, setDisabling] = React.useState(false);
  const [error, setError] = React.useState('');
  const epoch = React.useRef(0);
  const online = gateway.status === 'online';
  const group = gateway.powerGroup;
  const pending = enabling || gateway.powerPending != null || ['checking', 'sending'].includes(group?.status);
  React.useEffect(() => {
    if (!online) { setConfirmed(false); onConfirmationChange?.(false); setError(''); epoch.current += 1; }
  }, [online]);
  async function enable() {
    if (!online || !confirmed || pending || blocked) return;
    const ticket = ++epoch.current;
    setEnabling(true); setError('');
    try { await gateway.requestAllPower('enable', EVIDENCE, limits); }
    catch (failure) { if (ticket === epoch.current) setError(failure.message); }
    finally { setEnabling(false); }
  }
  async function disable() {
    if ((!online && !taskActive) || disabling) return;
    epoch.current += 1; setDisabling(true); setError('');
    try { if (taskActive) await onStopTask(); else await gateway.requestAllPower('disable'); }
    catch (failure) { setError(failure.message); }
    finally { setDisabling(false); }
  }
  return <section id="motor-control-panel" className="card glass machinePower" aria-label="七轴统一电源控制">
    <h3>2 · 现场确认与七轴电源</h3>
    <label className="machineConfirmation"><input type="checkbox" checked={confirmed} disabled={!online || pending}
      onChange={event => { setConfirmed(event.target.checked); onConfirmationChange?.(event.target.checked); }} />
      <span>我已确认七轴现场条件：我在机器旁、运动空间清空、能立即独立断电、机械支撑可靠且撤力不会跌落，J1–J7 的方向、零位和可动范围已核对。</span></label>
    <div className="consoleActions machinePowerActions">
      <button className="primary" disabled={!online || !confirmed || pending || blocked || !gateway.requestAllPower || gateway.pollingPaused}
        onClick={enable}>{enabling ? '正在检查并使能…' : '全部使能 · J1–J7'}</button>
      <button className="danger" disabled={(!online && !taskActive) || disabling || (!taskActive && !gateway.requestAllPower)} onClick={disable}>
        {disabling ? '正在请求失能…' : taskActive ? '停止实机任务并撤力' : '全部立即失能'}</button>
    </div>
    <p className="consoleNote">一次确认覆盖本次连接的七轴，不保存到下次启动。使能前检查全部轴，再依次发送；使能本身不保持姿态。失能单击发送，可中断使能，不能代替独立断电。</p>
    <div className="machineAxisStatus" aria-label="七轴电源请求记录">{Array.from({ length: 7 }, (_, i) => i + 1).map(id => {
      const result = gateway.powerRequests?.[id];
      return <span key={id} className={result?.error ? 'error' : ''}>J{id} <strong>{result?.error ? '未确认' : result?.op === 'enable' ? '使能已发' : result?.op === 'disable' ? '失能已发' : '未操作'}</strong></span>;
    })}</div>
    <small>以上是发送记录；设备无法读回使能状态，不代表真实出力或撤力已确认。</small>
    {group?.status === 'checking' && <p role="status">正在检查七轴位置、MIT 模式和六轴起点范围…</p>}
    {group?.status === 'sending' && <p role="status">正在请求 J{group.axis} 使能，已确认发送 {group.sent.length}/7。</p>}
    {gateway.pollingPaused && <p role="status">失能后查询已暂停。重新使能前请点击“刷新状态”。</p>}
    {error && <p role="alert" className="motorPowerError">{error}</p>}
  </section>;
}
