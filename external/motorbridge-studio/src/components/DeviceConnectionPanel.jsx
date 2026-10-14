import React from 'react';
const modeName = value => ({0:'MIT',1:'PP',2:'速度',3:'电流',5:'CSP'}[value] || '未读取');

export function DeviceConnectionPanel({ gateway, now, onSync, syncing, synchronization, blocked, readiness, powerBusy = false }) {
  const online=gateway.status==='online';
  const fresh=id=>online && Number.isFinite(gateway.motors[id]?.mechPosRad) &&
    Number.isFinite(gateway.motors[id]?.positionUpdatedAt) && now-gateway.motors[id].positionUpdatedAt>=0 && now-gateway.motors[id].positionUpdatedAt<3000 && !gateway.motors[id].error;
  const freshMode=id=>online && Number.isFinite(gateway.motors[id]?.fieldTimes?.runMode) && now-gateway.motors[id].fieldTimes.runMode>=0 && now-gateway.motors[id].fieldTimes.runMode<3000;
  const count=Array.from({length:7},(_,i)=>i+1).filter(fresh).length;
  const armFresh=Array.from({length:6},(_,i)=>i+1).every(fresh);
  const status=({online:'已连接',connecting:'连接中',offline:'连接中断',disconnected:'未连接'})[gateway.status];
  return <section className="card glass deviceConnection" aria-label="实机连接与姿态同步">
    <div className="deviceHeading"><div><h3>1 · 实机通信</h3><p>连接只读取反馈，不自动使能；XYZ 预演自动读取当前姿态。</p></div><strong className={online?'onlineBadge':'offlineBadge'}>{status}</strong></div>
    <div className="deviceMetrics"><span>CAN 通信<strong>{count===7?'七轴反馈正常':count?`${count} 轴有新鲜反馈`:online?'等待电机响应':'未查询'}</strong></span>
      <span>七轴在线<strong>{count} / 7</strong></span><span>六轴位置<strong>{armFresh?'反馈新鲜':'待完整新鲜反馈'}</strong></span></div>
    <div className="consoleActions"><button className="primary" disabled={powerBusy||gateway.powerPending!=null||['checking','sending'].includes(gateway.powerGroup?.status)||online||gateway.status==='connecting'} onClick={()=>gateway.connect()}>连接实机</button>
      <button disabled={gateway.status==='disconnected'||gateway.powerPending!=null||['checking','sending'].includes(gateway.powerGroup?.status)} onClick={gateway.disconnect}>断开连接</button>
      <button disabled={powerBusy||gateway.powerPending!=null||['checking','sending'].includes(gateway.powerGroup?.status)||!online||syncing} onClick={()=>gateway.refresh()}>刷新状态</button></div>
    {gateway.pollingPaused && <p role="status">已请求失能，自动查询已暂停。查看撤力后的反馈请点击“刷新状态”，不会自动使能。</p>}
    {synchronization && <p role="status">{synchronization.ok?`已同步 MuJoCo，六轴最大初始化误差 ${(synchronization.max_error_rad*180/Math.PI).toFixed(6)}°；实机未移动。`:synchronization.error}</p>}
    {synchronization?.ok && <details><summary>查看六轴同步误差</summary><div className="deviceModes">{synchronization.errors_rad.map((error,i)=><span key={i}>J{i+1} · {(error*180/Math.PI).toFixed(6)}°</span>)}</div></details>}
    <details className="deviceAdvanced"><summary>网关地址与通信说明</summary><label>网关地址<input value={gateway.endpoint} disabled={online} onChange={e=>gateway.setEndpoint(e.target.value)} /></label>
      <button disabled={!online||syncing||blocked} onClick={onSync}>{syncing?'正在读取并同步…':'手动更新 MuJoCo 起点（高级）'}</button>
      <div className="deviceModes">{Array.from({length:6},(_,i)=>i+1).map(id=><span key={id}>J{id} · {freshMode(id)?modeName(gateway.motors[id]?.runMode):'模式待刷新'}</span>)}</div>
      <p>执行器：{readiness?.executor_label || '状态未读取'}。{powerBusy ? 'ROS 虚拟任务执行中。' : ''}实机 XYZ 轨迹仍待验收。</p>
      <p>连接与手动刷新读取完整参数；自动查询仅读取位置，减少反馈延迟。七轴共用下方一个现场确认；失能不弹确认框。断开查询或关闭页面不会自动撤力，刷新页面不恢复使能。</p></details>
  </section>;
}
