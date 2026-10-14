import React from 'react';

// Diagnostic details only. The shared controller also supplies the main view.
export function RosControlPanel({ controller, onSynced }) {
  const { state, error, pending, running, action } = controller;
  const [message, setMessage] = React.useState('');
  async function sync() {
    const response = await fetch('/ros-control/action', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ action: 'sync' }) });
    const result = await response.json();
    if (!response.ok) { setMessage(result.error); return; }
    onSynced?.(); setMessage('已复制一次虚拟姿态到 MuJoCo；主流程无需此操作。');
  }
  return <section id="ros-control-panel" className="card glass deviceConnection"><h3>ROS 控制链路 · 高级诊断</h3>
    <p>复用 Yang-Ci 的 Fake Driver 和轨迹 Action，固定在隔离的 domain 43，不连接 CAN。日常 XYZ 使用上方统一入口。</p>
    <div className="consoleActions"><button disabled={pending || running || !state?.fresh} onClick={() => sync().catch(e => setMessage(e.message))}>复制虚拟姿态到 MuJoCo（一次）</button>
      <button onClick={() => action('stop')}>停止虚拟任务</button></div>
    <p>任务：{state?.job?.status || '无'} · 后端：{state?.backend || '未连接'}</p>
    <div className="consoleJoints">{state?.q_rad?.map((q, i) => <span key={i}>J{i + 1} · {(q * 180 / Math.PI).toFixed(2)}°</span>)}</div>
    <p>{message || error || state?.job?.error}</p><small>Fake Driver 验证控制链路，不验证真实力矩、重力或物理停止。</small>
  </section>;
}
