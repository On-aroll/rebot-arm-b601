import React from 'react';

export function OfflineControls({ controller, fresh, blocked }) {
  const { state, pending, error, action, refresh, running } = controller;
  return <section className="card glass deviceConnection offlineControls" aria-label="离线虚拟电机">
    <div className="deviceHeading"><div><h3>1 · 虚拟电机</h3><p>无需机械臂。输入 XYZ、预演后执行，模型持续跟随虚拟反馈。</p></div>
      <strong className={fresh ? 'onlineBadge' : 'offlineBadge'}>{fresh ? '虚拟反馈在线' : '等待虚拟服务'}</strong></div>
    <div className="consoleActions">
      <button disabled={pending} onClick={refresh}>刷新虚拟反馈</button>
      <button className="primary" disabled={!fresh || state?.enabled || pending || blocked || running} onClick={() => action('enable')}>使能虚拟电机</button>
      <button className="danger" disabled={!state?.online} onClick={() => action('disable')}>失能虚拟电机</button>
    </div>
    <p role="status">{state?.enabled ? '虚拟电机已使能' : '虚拟电机未使能'} · 使能后仍需点击“执行虚拟动作”，不会自动运动。</p>
    {error && <p role="alert">{error}</p>}
    <small>当前使用 ROS Fake Driver；MuJoCo 提供模型路径检查。实机连接和电源控制在“实机操作”模式。</small>
  </section>;
}
