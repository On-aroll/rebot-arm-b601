import React from 'react';

export function LiveCoordinates({ source, xyz, fresh, ageMs, cycleMs }) {
  return <section className="card glass liveCoordinates" aria-label="当前末端坐标">
    <div><strong>当前末端坐标 <small>mm · 基座坐标系</small></strong><span>{source}</span></div>
    <div className="consoleTcp">{['X', 'Y', 'Z'].map((axis, i) => <span key={axis}>{axis}<strong>{fresh && Number.isFinite(xyz?.[i]) ? xyz[i].toFixed(2) : '—'}</strong></span>)}</div>
    <small>{fresh ? 'gripper_end · 未计入额外工具长度' : '等待当前来源的新鲜反馈，不用目标值代替实测'}
      {fresh && Number.isFinite(ageMs) ? ` · 反馈距今 ${Math.max(0, ageMs).toFixed(0)} ms` : ''}
      {cycleMs > 0 ? ` · 最近一轮查询 ${cycleMs} ms` : ''}</small>
  </section>;
}
