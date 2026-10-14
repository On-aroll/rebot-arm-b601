import React from 'react';
import { defaultMapping, MAPPING_KEY, validateMapping } from '../lib/hardwareMapping';

export function HardwareMappingPanel({ profile, onChange, mapped, source, onSource, showSourceControls = true }) {
  const [message, setMessage] = React.useState('官方基准为同向、零修正；首次使用仍需要逐轴与实物核对。');
  const confirmed = profile.axes.filter(axis => axis.confirmed).length;
  function update(index, patch) {
    onChange({ ...profile, axes: profile.axes.map((axis, i) => i === index ? { ...axis, confirmed: false, ...patch } : axis) });
    setMessage('映射已修改，尚未保存；只影响模型显示，不修改电机零位或运动参数。');
  }
  function save() {
    try {
      validateMapping(profile);
      localStorage.setItem(MAPPING_KEY, JSON.stringify(profile));
      setMessage(`映射已保存到本浏览器，已人工核对 ${confirmed}/6 轴；不会因此开放实机运动。`);
    } catch (error) { setMessage(`保存失败：${error.message}`); }
  }
  return <section className="card glass hardwareMapping" aria-label="关节映射（首次验收/高级调试）">
    <h3>关节映射（首次验收/高级调试） <small>设备 {profile.deviceId}</small></h3>
    <p>实机模式的三维模型跟随电机反馈显示。方向与零位修正只改变软件换算，不向电机写入零位、位置或使能。</p>
    <div className="consoleActions">
      {showSourceControls && <><button aria-pressed={source === 'simulation'} onClick={() => onSource('simulation')}>显示仿真姿态</button>
      <button aria-pressed={source === 'hardware'} disabled={!mapped.complete && source !== 'hardware'} onClick={() => onSource('hardware')}>显示实机姿态（只读）</button></>}
      <button onClick={save}>保存映射到本机</button>
      <button onClick={() => { onChange(defaultMapping()); setMessage('已恢复官方基准草稿，全部核对标记清除；保存后才替换本机记录。'); }}>恢复官方基准草稿</button>
    </div>
    <p className="consoleNote">模型角度＝方向 × 电机原始角度＋零位修正。不要为了消除越限提示而随意修改零位。夹爪角度到开合量尚未标定，实机视图中的夹爪开合不代表实物。</p>
    <div className="mappingTableScroll"><table className="compareTable"><thead><tr>
      <th>对应</th><th>电机原始角度</th><th>方向</th><th>零位修正（°）</th><th>模型角度</th><th>核对记录</th>
    </tr></thead><tbody>{profile.axes.map((axis, index) => {
      const row = mapped.rows[index];
      return <tr key={axis.motorId}>
        <td>J{axis.motorId} ↔ ID {axis.motorId}</td>
        <td>{row.rawDeg == null ? '—' : `${row.rawDeg.toFixed(2)}°`}{!row.fresh && <small>（过期或未读到）</small>}</td>
        <td><select aria-label={`J${axis.motorId} 方向`} value={axis.direction} onChange={event => update(index, { direction: Number(event.target.value) })}><option value="1">同向 +1</option><option value="-1">反向 −1</option></select></td>
        <td><input aria-label={`J${axis.motorId} 零位修正`} type="number" step="0.1" value={axis.offsetDeg} onChange={event => update(index, { offsetDeg: event.target.value === '' ? '' : Number(event.target.value) })} /></td>
        <td className={row.outside ? 'mappingBad' : ''}>{row.modelDeg == null ? '—' : `${row.modelDeg.toFixed(2)}°`}{row.outside && <small>（超出模型限位）</small>}</td>
        <td><label><input type="checkbox" checked={axis.confirmed} disabled={!row.fresh || row.modelDeg == null} onChange={event => update(index, { confirmed: event.target.checked })} />已核对方向和零位</label></td>
      </tr>;
    })}</tbody></table></div>
    <p className="consoleNote">核对时先观察实物与模型姿态是否一致，再核对单轴角度增减方向。需要人为改变姿态时，应先确定机械臂有支撑、不会因释放保持力而下坠；此面板不会自动去使能或移动关节。</p>
    {mapped.issues.length > 0 && <ul className="mappingIssues">{mapped.issues.map(issue => <li key={issue}>{issue}</li>)}</ul>}
    <p role="status">{message}</p>
    <p className="pointHardwareReason">这里只完成反馈到模型的对应。坐标驱动实机还需要独立的六轴执行、停止和失联验收；勾选核对记录不会解除这些限制。</p>
  </section>;
}
