import React from 'react';
import { MOTOR_SETTINGS, settingPayload, positionSpeedState, positionMotionRefusal } from '../lib/motorSettings';

const MODES = { pos: '位置 PP 模式（程序规划位置）', mit: 'MIT 模式（高级调试）', vel: '速度模式（高级调试）' };

export function MotorModuleCard({ id, label, motor, status, gateway, plannedDeg }) {
  const [draft, setDraft] = React.useState({});
  const [mode, setMode] = React.useState('pos');
  const [result, setResult] = React.useState(null);
  const [pending, setPending] = React.useState(false);
  const [lastCommand, setLastCommand] = React.useState('未发送');
  const busyRef = React.useRef(false);
  const reading = motor || {};
  const fresh = Number.isFinite(reading.updatedAt) && Date.now() - reading.updatedAt < 10000;
  const lockReason = status !== 'online' ? '网关未连接'
    : !gateway.armed ? '总开关关闭：可编辑草稿，尚不能下发'
      : !fresh ? '本轴反馈过期，等待重新读取' : '';
  const writable = !lockReason && !pending;
  const speedState = positionSpeedState(reading);
  const enabledReason = writable ? positionMotionRefusal(reading, gateway.maxSpd) : '';

  async function run(op, payload) {
    if (busyRef.current) return;
    busyRef.current = true; setPending(true);
    setResult({ text: '处理中…', level: 'info' });
    try {
      const ret = await gateway.write(id, op, payload);
      if (ret.ok) {
        // A dry run and a gateway acknowledgement never prove physical enable state.
        if (!ret.dryRun && ['enable', 'disable', 'ensure_mode'].includes(op)) {
          setLastCommand(`${op === 'enable' ? '使能' : op === 'disable' ? '去使能' : '切模式'}指令已发送`);
        }
        setResult({ text: ret.dryRun ? '演练校验通过，未向电机写入'
          : ret.verified ? '已写入，回读一致' : '指令已发送；真实使能状态仍需确认', level: ret.dryRun ? 'drill' : 'ok' });
        if (!ret.dryRun) gateway.refresh();
      } else setResult({ text: ret.error || '操作失败', level: 'err' });
    } catch (error) { setResult({ text: error.message, level: 'err' }); }
    finally { busyRef.current = false; setPending(false); }
  }

  function applySetting(field) {
    try { run('robstride_write_param', settingPayload(field.key, draft[field.key])); }
    catch (error) { setResult({ text: error.message, level: 'err' }); }
  }

  function switchMode() {
    if (!window.confirm(`切换 ID ${id} 的运行模式会自动去使能，机械臂可能失去保持力。确认切换？`)) return;
    run('ensure_mode', { mode, timeout_ms: 2000 });
  }

  return (
    <section aria-label={`ID ${id} 电机设置`} className={`motorModule ${gateway.armed ? 'armed' : ''}`}>
      <header><span className="motorTag">ID {id}</span><strong>{label}</strong>
        <small>{reading.model || '—'}</small><span className={`motorDot ${status}`} /></header>
      <div className="motorReadout">
        <span>实际角度<strong>{reading.mechPosWrapped == null ? '—' : `${reading.mechPosWrapped.toFixed(2)}°`}</strong></span>
        <span>程序目标 · 仿真<strong>{Number.isFinite(plannedDeg) ? `${plannedDeg.toFixed(2)}°` : '—'}</strong></span>
      </div>
      <p className="motorHint">位置由任务轨迹和逆运动学计算，本模块不填写目标位置。</p>
      <div className="motorPower">
        <strong>使能状态：未知</strong>
        <small>当前模式：{speedState.label} · {speedState.register || reading.paramErrors?.runMode || '位置限速检查不适用'}</small><small>{status === 'online' ? lastCommand : '网关未连接'}</small>
        <div className="motorActions">
          <button disabled={!writable || Boolean(enabledReason)} title={lockReason || enabledReason || '允许本轴输出力矩'} onClick={() => run('enable', {})}>使能（出力）</button>
          <button disabled={!writable} title={lockReason || '释放保持力；不会切断供电'} onClick={() => run('disable', {})}>去使能（释放力）</button>
        </div>
        <small>使能控制是否出力，不是电源开关；去使能可能使机械臂下垂。</small>
      </div>
      <div className="motorSettingRows">
        {MOTOR_SETTINGS.map((field) => <div className="motorSettingRow" key={field.key}>
          <label htmlFor={`motor-${id}-${field.key}`}>{field.label} {field.unit && <small>({field.unit})</small>}</label>
          <span className="motorSettingRead">当前：{reading[field.key] == null ? '未读到' : reading[field.key].toFixed(3)}</span>
          <div className="motorSettingEdit">
            <input id={`motor-${id}-${field.key}`} type="number" step={field.step} min="0" placeholder="输入新值" value={draft[field.key] ?? ''}
              onChange={(event) => setDraft((prev) => ({ ...prev, [field.key]: event.target.value }))} />
            <button disabled={!writable || !draft[field.key]?.trim()} title={lockReason || '仅写此参数，不发送目标位置'} onClick={() => applySetting(field)}>应用</button>
          </div>
          <small>{reading.paramErrors?.[field.key] || field.hint}</small>
        </div>)}
      </div>
      <details className="motorAdvanced"><summary>高级：运行模式</summary>
        <label htmlFor={`motor-${id}-mode`}>下次切换模式</label>
        <select id={`motor-${id}-mode`} value={mode} onChange={(event) => setMode(event.target.value)}>
          {Object.entries(MODES).map(([value, text]) => <option key={value} value={value}>{text}</option>)}
        </select>
        <button disabled={!writable} onClick={switchMode}>应用模式</button>
        <p className="motorHint">上方显示当前读取到的模式；此处选择的是待切换模式。MIT 的 kp/kd/前馈力矩由未来执行器配置；位置模式使用上方 loc_kp。单轴位置调试保留在「机械臂模式」。</p>
      </details>
      <p className="motorHint">{lockReason || (gateway.drill ? '演练模式：只校验，不写入电机' : '实发模式：应用后会写入本轴参数')}{enabledReason && `。${enabledReason}`}</p>
      {reading.error && <p className="motorError">{reading.error}</p>}
      {result && <p role="status" className={`motorResult ${result.level}`}>{result.text}</p>}
    </section>
  );
}
