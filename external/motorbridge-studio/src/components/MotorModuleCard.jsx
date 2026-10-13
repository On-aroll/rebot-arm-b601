import React from 'react';
import { MOTOR_SETTINGS } from '../lib/motorSettings';
import { AxisPowerControls } from './AxisPowerControls';

const EMPTY_FIELDS = Object.freeze({});
const modeLabel = mode => ({ 0: 'MIT', 1: 'PP', 2: '速度', 3: '电流', 5: 'CSP' }[mode] || '未知');
const valueText = value => value == null ? '未确认' : String(value);
// Read-only reference from the pinned Seeed RS YAML, never draft defaults.
const OFFICIAL_RS_GAINS = { 1:[50,3], 2:[150,10], 3:[150,10], 4:[50,5], 5:[50,4], 6:[50,4], 7:[50,4] };
const OFFICIAL_RS_CONFIG = 'https://github.com/Seeed-Projects/reBotArm_control_py/blob/6415d43130d1e143c70dc106096a857ac5556f81/config/rebotarm_rs.yaml';

export function MotorModuleCard({ id, label, motor, status, plannedDeg, expanded = false, onToggle, config, gateway, powerBlocked, limits }) {
  const reading = motor || {};
  const [draft, setDraft] = React.useState({});
  const [dirty, setDirty] = React.useState(false);
  const [review, setReview] = React.useState(null);
  const [confirmed, setConfirmed] = React.useState(false);
  const [message, setMessage] = React.useState('');
  const [history, setHistory] = React.useState(false);
  const saved = config?.state?.drafts?.[id];
  const approved = config?.state?.approved?.[id];
  const fields = config?.fields || EMPTY_FIELDS;
  const reference = OFFICIAL_RS_GAINS[id];
  React.useEffect(() => {
    if (!dirty) setDraft(Object.fromEntries(Object.keys(fields).map(key => [key, saved?.fields?.[key]?.value ?? ''])));
  }, [saved, fields, dirty]);
  React.useEffect(() => { setReview(null); setConfirmed(false); }, [config?.state?.revision]);
  const fresh = status === 'online' && Number.isFinite(reading.positionUpdatedAt) && Date.now() - reading.positionUpdatedAt >= 0 && Date.now() - reading.positionUpdatedAt < 3000;
  const latest = config?.state?.snapshots?.filter(s => s.axis_id === id).at(-1);
  async function run(body) {
    try { return await config.action(body); }
    catch (e) { setMessage(e.message); return null; }
  }
  async function save() {
    const parsed = Object.fromEntries(Object.entries(fields).map(([key, spec]) => [key, { unit: spec.unit, value: draft[key] === '' || draft[key] == null ? null : Number(draft[key]) }]));
    if (Object.values(parsed).some(f => f.value != null && !Number.isFinite(f.value))) { setMessage('请输入有限数值'); return; }
    const result = await run({ action: 'draft', axis_id: id, fields: parsed });
    if (result) { setDirty(false); setMessage('草稿已保存到本地服务；没有写入电机'); }
  }
  async function inspect() {
    const result = await run({ action: 'review', axis_id: id });
    if (result) { setReview(result); setConfirmed(false); }
  }
  async function approve() {
    const result = await run({ action: 'approve', review_token: review.review_token, confirm: 'software-only' });
    if (result) { setReview(null); setMessage('已批准软件配置；物理身份、动态映射与真实执行仍未验收'); }
  }
  return <section className="motorModule compactMotor" aria-label={`J${id} 电机模块`}>
    <button className="motorSummary" aria-expanded={expanded} aria-controls={`motor-details-${id}`} onClick={onToggle}>
      <strong>{label}</strong><span>{reading.model || (id <= 3 ? 'rs-06' : 'rs-00')}</span>
      <span>{Number.isFinite(reading.mechPosDeg) ? `${reading.mechPosDeg.toFixed(2)}°` : '角度未知'}</span>
      <span>{modeLabel(reading.runMode)}{id === 7 ? ' · 夹爪待核实' : ''}</span>
      <span>{fresh ? '反馈新鲜' : reading.positionUpdatedAt ? '反馈过期' : '未读取'}</span>
      <span>{approved ? `软件批准 v${approved.version}` : '配置待确认'}</span><span>{expanded ? '收起 −' : '展开 +'}</span>
    </button>
    <AxisPowerControls id={id} gateway={gateway} blocked={powerBlocked} limits={limits} />
    {expanded && <div className="motorDetails" id={`motor-details-${id}`}>
      <details><summary>配置核对与阻断原因</summary><ul>{config?.preparation?.find(row => row.axis_id === id)?.reasons.map(reason => <li key={reason}>{reason}</li>) || <li>配置或设备快照尚不可用；实机锁定</li>}</ul><p>设备模式与 MIT 路线可比较；逐帧 MIT 增益和轨迹约束没有对应设备寄存器读回，不能拿 PP/CSP 数值证明一致。</p></details>
      <h4>只读测量</h4><p>真实出力状态未知。原始多圈角不归一化；模式：{modeLabel(reading.runMode)}。仿真目标：{Number.isFinite(plannedDeg) ? `${plannedDeg.toFixed(2)}°` : '未知'}。</p>
      <p>位置采集：{reading.positionUpdatedAt ? new Date(reading.positionUpdatedAt).toLocaleString() : '未读取'}。归档快照：{latest ? `${latest.source} · ${new Date(latest.measured_at * 1000).toLocaleString()}` : '尚未采集'}。设备身份未核验。</p>
      <h4>{id <= 6 ? 'MIT 基本参数' : '夹爪独立配置'}</h4>
      <p>配置来源：{saved?.source === 'user-provided-mit-initial-trial' ? '用户提供的 MIT 初始试验配置 · 尚未实机验证' : saved ? '人工编辑的软件草稿' : '尚无草稿'}。这里显示每帧控制参数，不是电机寄存器读回。</p>
      <div className="mitBasicInputs">{Object.entries(fields).filter(([key])=>['kp','kd','tau_ff'].includes(key)).map(([key,spec])=><label key={key} htmlFor={`draft-${id}-${key}`}>{spec.label}<small>{key} · {spec.unit} · 已批准：{valueText(approved?.fields?.[key]?.value)}</small><input aria-label={fields[key].label} id={`draft-${id}-${key}`} type="number" step="any" value={draft[key]??''} placeholder="待确认" onChange={e=>{setDraft(v=>({...v,[key]:e.target.value}));setDirty(true);setReview(null);setConfirmed(false);}} /></label>)}</div>
      <p className="consoleNote">Kp 是拉回目标位置的力度；Kd 是减缓运动的阻尼；前馈力矩是额外推力。MIT 需要连续控制帧，使能按钮本身不发送保持或运动目标。降低 Kp 不能代替限速。</p>
      {reference && <p className="consoleNote">本轴官方 RS 示例：Kp={reference[0]}，Kd={reference[1]}（<a href={OFFICIAL_RS_CONFIG} target="_blank" rel="noreferrer">官方 RS 配置（锁定版本）</a>）。仅供对照，这是软件示例增益，设备调定值与现场适用性仍需核实。</p>}
      <details className="engineeringParameters"><summary>运动规划、模型与高级工程参数（通常无需填写）</summary>
      <p>这些字段供轨迹程序和标定维护，分别约束运动快慢、坐标换算和异常保护。日常使能/失能无需填写；单轴运动试验使用独立试验方案。模型限位和方向映射不是实机安全验收，空白仍表示待确认。</p>
      {[
        ['运动规划',['v_max','a_max','jerk_max','timeout']],
        ['机械模型与映射',['q_min','q_max','direction','offset']],
        ['高级保护',['error_max','torque_max']],
      ].map(([title,keys])=><div key={title}><h5>{title}</h5><div className="mitBasicInputs">{keys.filter(key=>fields[key]).map(key=><label key={key} htmlFor={`draft-${id}-${key}`}>{fields[key].label}<small>{fields[key].unit} · 已批准：{valueText(approved?.fields?.[key]?.value)}</small><input aria-label={fields[key].label} id={`draft-${id}-${key}`} type="number" step="any" value={draft[key]??''} placeholder="待确认" onChange={e=>{setDraft(v=>({...v,[key]:e.target.value}));setDirty(true);setReview(null);setConfirmed(false);}} /></label>)}</div></div>)}
      </details>
      <p>批准时间：{approved ? new Date(approved.approved_at * 1000).toLocaleString() : '未批准'}；仅软件人工确认，未现场验证。映射草稿的批准不会自动替换显示映射。</p>
      <div className="motorActions"><button disabled={!config?.state || config.busy} onClick={save}>保存草稿</button>
        <button disabled={!saved || dirty || config?.busy} onClick={inspect}>审核变更</button>
        <button onClick={() => setHistory(v => !v)}>查看变更记录</button>
        <button disabled title="尚未取得当次现场执行许可">应用到真实电机（待验收）</button></div>
      {review && <section className="reviewBox" aria-label="软件配置审核"><p>{review.warning}</p>
        <ul>{review.changes.map(c => <li key={c.field}>{fields[c.field]?.label}：{valueText(c.before.value)} → {valueText(c.after.value)} {c.after.unit}</li>)}</ul>
        <label><input type="checkbox" checked={confirmed} onChange={e => setConfirmed(e.target.checked)} />我逐项核对了差异，仅批准本地软件配置，不授权电机动作</label>
        <button disabled={!confirmed || config.busy} onClick={approve}>确认本次软件批准</button></section>}
      {history && <ul aria-label="本轴变更记录">{(config?.state?.events || []).filter(e => e.axis_id === id).map(e => <li key={e.revision}>v{e.revision} · {new Date(e.at * 1000).toLocaleString()} · {e.action}{e.outcome ? ` · Mock ${e.outcome}` : ''}</li>)}</ul>}
      <details><summary>其他模式设备参数（只读 / 高级）</summary>{MOTOR_SETTINGS.map(field => <p key={field.key}>
        {field.label}：{reading[field.key] == null ? '未知' : reading[field.key]} {field.unit} · {reading.runMode === 0 ? '当前 MIT 不作为有效运动限制' : '适用模式见说明'}<br />
        <small>{field.hint} · 采集：{reading.fieldTimes?.[field.key] ? new Date(reading.fieldTimes[field.key]).toLocaleString() : '未知'}{reading.paramErrors?.[field.key] && ` · ${reading.paramErrors[field.key]}`}</small>
      </p>)}</details>
      {reading.error && <p role="alert">{reading.error}</p>}{message && <p role="status">{message}</p>}
    </div>}
  </section>;
}
