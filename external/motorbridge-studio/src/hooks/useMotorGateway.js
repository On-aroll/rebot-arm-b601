import React from 'react';
import { WsGatewayClient } from '../wsGatewayClient';
import { MOTOR_SETTINGS, settingsRefusal, positionMotionRefusal } from '../lib/motorSettings';

// 控制台直连 motorbridge ws_gateway；默认地址与官方前端一致。
export const MACHINE_GATEWAY_URL = 'ws://127.0.0.1:9002';
const FEEDBACK_ID = 0xfd;
const MECH_POS = 0x7019;   // 未归一化的多圈机械位置(rad)
// 官方 config/rebotarm_rs.yaml：motor_id N ↔ joint N，1–3 rs-06，4–7 rs-00
export const MACHINE_MODELS = { 1: 'rs-06', 2: 'rs-06', 3: 'rs-06', 4: 'rs-00', 5: 'rs-00', 6: 'rs-00', 7: 'rs-00' };
export const MACHINE_IDS = [1, 2, 3, 4, 5, 6, 7];

export const DEFAULT_MAX_SPD = 0.3;        // rad/s ≈ 17°/s，首次实机联调上限
export const MAX_SINGLE_STEP_DEG = 15;     // 单次目标位移上限，防一步跳太大

const DEG = 180 / Math.PI;

// 只读三件套：不开总开关也能用。
const READ_OPS = new Set(['set_target', 'robstride_read_param', 'close_bus']);
// 写操作白名单。set_id / 写零位 / clear_error 故意不在里面。
const WRITE_OPS = new Set(['enable', 'disable', 'stop', 'ensure_mode', 'pos_vel', 'mit', 'vel', 'robstride_write_param']);
// 会让电机出力或改变运动意图的 op，必须过限速兵。
const MOTION_OPS = new Set(['enable', 'pos_vel', 'mit', 'vel']);
// 底层唯一允许的 op 集合：不在这里的名字（比如 set_id、zero）连发都发不出去。
const ALLOWED_OPS = new Set([...READ_OPS, ...WRITE_OPS]);

function wrapDeg(angle) {
  const value = ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  return Math.abs(value + 180.0) < 1e-9 ? 180.0 : value;
}

/**
 * 实机网关总控：连接、只读轮询、受控写入、急停。
 *
 * 三道硬门禁都在这里，不在界面里：
 *   1. 总开关 armed —— 关着的时候任何写 op 直接抛错；
 *   2. 演练模式 drill —— 开着时写 op 只记账不发送（stop 永远真发，急停不能被演练挡住）；
 *   3. 限速兵 —— 使能与运动前，该轴当前模式的速度上限必须读到且不超过 maxSpd（PP: vel_max，CSP: limit_spd），
 *      目标还要落在模型限位与单次位移上限内。
 *
 * 刻意不用官方前端的 setTargetFor()：它会在选中电机时写 0x7026（主动上报周期，
 * 存进电机）并打开 set_active_report，见 src/hooks/useGatewayBridge.js:239-285。
 * 这里每次写之前只发 set_target，写完立刻归还总线。
 */
export function useMotorGateway({ url = MACHINE_GATEWAY_URL, intervalMs = 2500 } = {}) {
  const [status, setStatus] = React.useState('disconnected');   // disconnected|connecting|online|offline
  const [motors, setMotors] = React.useState({});
  const [meta, setMeta] = React.useState({ cycleMs: 0, updatedAt: 0, cycles: 0, error: '' });
  const [endpoint, setEndpoint] = React.useState(url);
  const [armed, setArmed] = React.useState(false);
  const [drill, setDrill] = React.useState(true);
  const [maxSpd, setMaxSpd] = React.useState(DEFAULT_MAX_SPD);
  const [log, setLog] = React.useState([]);

  const clientRef = React.useRef(null);
  const busyRef = React.useRef(false);
  const stoppedRef = React.useRef(true);
  const cyclesRef = React.useRef(0);
  const motorsRef = React.useRef({});
  const gateRef = React.useRef({ armed: false, drill: true, maxSpd: DEFAULT_MAX_SPD });

  gateRef.current = { armed, drill, maxSpd };
  motorsRef.current = motors;

  const note = React.useCallback((text, level = 'info') => {
    setLog((prev) => [...prev.slice(-199), { t: Date.now(), text, level }]);
  }, []);

  const send = React.useCallback((op, payload, timeoutMs = 4000) => {
    const client = clientRef.current;
    if (!ALLOWED_OPS.has(op)) return Promise.reject(new Error(`op 不在白名单：${op}`));
    if (!client || !client.isConnected()) return Promise.reject(new Error('网关未连接'));
    return client.send(op, payload, timeoutMs);
  }, []);

  /** 把当前连接的目标设到某个电机。set_target 是连接级状态，所以调用方必须持有 busyRef 锁。 */
  const selectMotor = React.useCallback(async (id) => {
    const ret = await send('set_target', {
      vendor: 'robstride', transport: 'socketcan', channel: 'can0',
      model: MACHINE_MODELS[id], motor_id: id, feedback_id: FEEDBACK_ID,
    }, 4000);
    if (!ret.ok) throw new Error(ret.error || `set_target ID ${id} 失败`);
  }, [send]);

  const readOne = React.useCallback(async (id) => {
    await selectMotor(id);
    const position = await send('robstride_read_param', { param_id: MECH_POS, type: 'f32', timeout_ms: 800 }, 3000);
    const positionUpdatedAt = Date.now();
    const params = {}; const paramErrors = {};
    let runMode = null;
    try {
      const response = await send('robstride_read_param', { param_id: 0x7005, type: 'i8', timeout_ms: 800 }, 3000);
      const value = response.ok && response.data?.value != null ? Number(response.data.value) : NaN;
      if (Number.isInteger(value)) runMode = value;
      else paramErrors.runMode = response.error || '运行模式读取失败';
    } catch (error) { paramErrors.runMode = error.message; }
    for (const field of MOTOR_SETTINGS) {
      try {
        const response = await send('robstride_read_param', { param_id: field.id, type: 'f32', timeout_ms: 800 }, 3000);
        const value = response.ok && response.data?.value != null ? Number(response.data.value) : NaN;
        params[field.key] = Number.isFinite(value) ? value : null;
        if (params[field.key] == null) paramErrors[field.key] = response.error || '读取失败';
      } catch (error) { params[field.key] = null; paramErrors[field.key] = error.message; }
    }
    const rad = position.ok && position.data?.value != null ? Number(position.data.value) : Number.NaN;
    const ok = Number.isFinite(rad);
    return {
      id,
      model: MACHINE_MODELS[id],
      mechPosRad: ok ? rad : null,
      mechPosDeg: ok ? Number((rad * DEG).toFixed(3)) : null,
      mechPosWrapped: ok ? Number(wrapDeg(rad * DEG).toFixed(3)) : null,
      ...params, runMode, paramErrors, positionUpdatedAt, updatedAt: Date.now(),
      error: ok ? '' : (position.error || 'mechPos 读取失败'),
    };
  }, [selectMotor, send]);

  const runCycle = React.useCallback(async () => {
    const client = clientRef.current;
    if (!client || busyRef.current || !client.isConnected()) return;
    busyRef.current = true;
    const started = Date.now();
    const next = {};
    let failure = '';
    try {
      for (const id of MACHINE_IDS) {
        if (stoppedRef.current) return;
        try {
          next[id] = await readOne(id);
        } catch (error) {
          next[id] = { id, model: MACHINE_MODELS[id], error: error.message || String(error) };
          failure = failure || error.message || String(error);
        }
      }
      try {
        await send('close_bus', {}, 2000);
      } catch {
        // 归还总线失败不影响读数；下一轮会重新 set_target。
      }
    } finally {
      busyRef.current = false;
    }
    if (stoppedRef.current) return;
    cyclesRef.current += 1;
    setMotors(next);
    setMeta({ cycleMs: Date.now() - started, updatedAt: Date.now(), cycles: cyclesRef.current, error: failure });
  }, [readOne, send]);

  const connect = React.useCallback(async (targetUrl) => {
    const nextUrl = (targetUrl || endpoint).trim();
    if (clientRef.current) {
      stoppedRef.current = true;
      try { clientRef.current.disconnect(); } catch { /* 忽略 */ }
      clientRef.current = null;
    }
    stoppedRef.current = false;
    setArmed(false);
    setMotors({});
    setStatus('connecting');
    setEndpoint(nextUrl);
    await new Promise((resolve) => {
      const client = new WsGatewayClient({
        onOpen: () => { setStatus('online'); note(`已连接 ${nextUrl}`, 'ok'); resolve(); },
        onClose: () => {
          setArmed(false);
          setStatus(stoppedRef.current ? 'disconnected' : 'offline');
          if (!stoppedRef.current) note('网关连接断开', 'err');
          resolve();
        },
        onError: () => { setStatus('offline'); note(`无法连接 ${nextUrl}`, 'err'); resolve(); },
      });
      clientRef.current = client;
      try {
        client.connect(nextUrl);
      } catch (error) {
        setStatus('offline');
        note(error.message || String(error), 'err');
        resolve();
      }
    });
    runCycle();
  }, [endpoint, note, runCycle]);

  const disconnect = React.useCallback(() => {
    stoppedRef.current = true;
    setArmed(false);
    try { clientRef.current?.disconnect(); } catch { /* 忽略 */ }
    clientRef.current = null;
    setStatus('disconnected');
    setMotors({});
    note('已断开网关，总开关同时复位', 'info');
  }, [note]);

  // 自动连一次，省掉每次开页都要点连接。
  React.useEffect(() => {
    connect(url);
    return () => { stoppedRef.current = true; try { clientRef.current?.disconnect(); } catch { /* 忽略 */ } };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  React.useEffect(() => {
    if (status !== 'online') return undefined;
    const poll = setInterval(() => { runCycle(); }, intervalMs);
    return () => clearInterval(poll);
  }, [intervalMs, runCycle, status]);

  /**
   * 限速兵与目标合法性检查。返回 null 表示放行，否则返回拒绝原因。
   */
  const refusal = React.useCallback((id, op, payload) => {
    const { maxSpd: cap } = gateRef.current;
    const motor = motorsRef.current[id];
    if (!['stop', 'disable'].includes(op) && (!motor?.updatedAt || Date.now() - motor.updatedAt > 10000)) return '本轴反馈已过期，拒绝下发';
    if (op === 'robstride_write_param') return settingsRefusal(payload, cap);
    if (MOTION_OPS.has(op)) {
      const reason = positionMotionRefusal(motor, cap);
      if (reason) return `ID ${id}：${reason}`;
      if (['mit', 'vel'].includes(op)) return '位置到点面板不提供 MIT/速度模式运动指令';
      if (op === 'pos_vel' && motor.runMode !== 1) return 'pos_vel 使用原生 PP 参数；CSP 轨迹执行器尚未接入';
    }
    if (payload && payload.pos != null) {
      if (!motor || motor.mechPosRad == null) return `ID ${id} 没有当前位置读数，无法校验目标`;
      const limits = payload.limitsDeg;
      const deg = payload.pos * DEG;
      const step = Math.abs(deg - motor.mechPosRad * DEG);
      if (step > MAX_SINGLE_STEP_DEG) return `ID ${id} 单次位移 ${step.toFixed(1)}° 超过上限 ${MAX_SINGLE_STEP_DEG}°`;
      if (Array.isArray(limits) && (deg < limits[0] - 1e-6 || deg > limits[1] + 1e-6)) {
        return `ID ${id} 目标 ${deg.toFixed(1)}° 超出模型限位 ${limits[0].toFixed(1)}°…${limits[1].toFixed(1)}°`;
      }
    }
    return null;
  }, []);

  /**
   * 受控写入。总开关关着一律拒绝；演练模式只记账不发帧（stop 例外，永远真发）。
   */
  const write = React.useCallback(async (id, op, payload = {}, options = {}) => {
    const gate = gateRef.current;
    if (!WRITE_OPS.has(op)) return { ok: false, error: `不允许的 op：${op}` };
    if (!gate.armed) return { ok: false, error: '全局下发总开关未打开' };
    if (status !== 'online') return { ok: false, error: '网关未连接' };
    const blocked = refusal(id, op, { ...payload, limitsDeg: options.limitsDeg });
    if (blocked) return { ok: false, error: blocked, refused: true };
    if (busyRef.current) return { ok: false, error: '轮询占用中，稍后再试' };

    const body = { vendor: 'robstride', motor_id: id, feedback_id: FEEDBACK_ID, ...payload };
    delete body.limitsDeg;

    busyRef.current = true;
    try {
      await selectMotor(id);
      if (!gateRef.current.armed) return { ok: false, error: '总开关已关闭，已取消写入' };
      if ((gate.drill || gateRef.current.drill) && op !== 'stop') {
        note(`演练 · ID ${id} ${op} ${JSON.stringify(body)}`, 'drill');
        return { ok: true, dryRun: true, op, id };
      }
      const ret = await send(op, body, 5000);
      if (ret.ok && op === 'robstride_write_param') {
        const readback = await send('robstride_read_param', { param_id: payload.param_id, type: 'f32', timeout_ms: 800 }, 3000);
        const actual = readback.ok && readback.data?.value != null ? Number(readback.data.value) : NaN;
        if (!Number.isFinite(actual) || Math.abs(actual - payload.value) > Math.max(1e-5, Math.abs(payload.value) * 1e-5)) {
          note(`ID ${id} 已发送参数，但回读未确认`, 'err');
          return { ok: false, error: '参数已发送，回读失败或不一致，请重新读取后确认' };
        }
        ret.verified = true;
      }
      note(`${ret.ok ? '已下发' : '下发失败'} · ID ${id} ${op}${ret.ok ? '' : `：${ret.error || ''}`}`, ret.ok ? 'ok' : 'err');
      return ret;
    } catch (error) {
      note(`ID ${id} ${op} 异常：${error.message || error}`, 'err');
      return { ok: false, error: error.message || String(error) };
    } finally {
      try { await send('close_bus', {}, 1500); } catch { /* 忽略 */ }
      busyRef.current = false;
    }
  }, [note, refusal, selectMotor, send, status]);

  /** 急停：不受总开关与演练模式限制，只要连着就真发。 */
  const stopAll = React.useCallback(async () => {
    stoppedRef.current = true;
    const results = [];
    for (const id of MACHINE_IDS) {
      try {
        await selectMotor(id);
        results.push(await send('stop', { vendor: 'robstride', motor_id: id, feedback_id: FEEDBACK_ID }, 3000));
      } catch (error) {
        results.push({ ok: false, error: error.message || String(error) });
      }
    }
    try { await send('close_bus', {}, 1500); } catch { /* 忽略 */ }
    note(`急停：${results.filter((r) => r.ok).length}/${MACHINE_IDS.length} 轴已发送 stop`, 'err');
    setTimeout(() => { stoppedRef.current = false; runCycle(); }, 400);
    return results;
  }, [note, runCycle, selectMotor, send]);

  const arm = React.useCallback(() => { setArmed(true); note('总开关已打开：写操作将按演练/实发设置生效', 'warn'); }, [note]);
  const disarm = React.useCallback(() => { setArmed(false); note('总开关已关闭', 'info'); }, [note]);

  return {
    status, motors, ...meta,
    endpoint, setEndpoint, connect, disconnect, refresh: runCycle,
    armed, arm, disarm, drill, setDrill, maxSpd, setMaxSpd,
    write, stopAll, log, note,
  };
}
