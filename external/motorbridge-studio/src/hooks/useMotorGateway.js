import React from 'react';
import { WsGatewayClient } from '../wsGatewayClient';
import { MOTOR_SETTINGS } from '../lib/motorSettings';

// 控制台直连 motorbridge ws_gateway；默认地址与官方前端一致。
export const MACHINE_GATEWAY_URL = 'ws://127.0.0.1:9002';
const FEEDBACK_ID = 0xfd;
const MECH_POS = 0x7019;   // 未归一化的多圈机械位置(rad)
// 官方 config/rebotarm_rs.yaml：motor_id N ↔ joint N，1–3 rs-06，4–7 rs-00
export const MACHINE_MODELS = { 1: 'rs-06', 2: 'rs-06', 3: 'rs-06', 4: 'rs-00', 5: 'rs-00', 6: 'rs-00', 7: 'rs-00' };
export const MACHINE_IDS = [1, 2, 3, 4, 5, 6, 7];

export const DEFAULT_MAX_SPD = 0.3;        // legacy display threshold only, never MIT safety evidence
export const MAX_SINGLE_STEP_DEG = 15;     // 单次目标位移上限，防一步跳太大
export const START_BOUNDARY_TOLERANCE_DEG = 1; // 用户允许的起点读数偏差；不用于运动目标限位

const DEG = 180 / Math.PI;

// 状态查询三件套；电源按钮使用单独的逐轴白名单和确认流程。
const READ_OPS = new Set(['set_target', 'robstride_read_param', 'close_bus']);
const ALLOWED_OPS = READ_OPS;
const POWER_CONFIRMATIONS = ['operator_present', 'power_cut_ready', 'supported', 'axis_verified'];

function wrapDeg(angle) {
  const value = ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  return Math.abs(value + 180.0) < 1e-9 ? 180.0 : value;
}

/** Explicit feedback connection with separately confirmed, serialized power commands. */
export function useMotorGateway({ url = MACHINE_GATEWAY_URL, intervalMs = 2500 } = {}) {
  const [status, setStatus] = React.useState('disconnected');   // disconnected|connecting|online|offline
  const [motors, setMotors] = React.useState({});
  const [meta, setMeta] = React.useState({ cycleMs: 0, updatedAt: 0, cycles: 0, error: '' });
  const [endpoint, setEndpoint] = React.useState(url);
  const [armed, setArmed] = React.useState(false);
  const [drill, setDrill] = React.useState(true);
  const [maxSpd, setMaxSpd] = React.useState(DEFAULT_MAX_SPD);
  const [log, setLog] = React.useState([]);
  const [powerRequests, setPowerRequests] = React.useState({});
  const [powerPending, setPowerPending] = React.useState(null);
  const [powerDisabling, setPowerDisabling] = React.useState([]);
  const [pollingPaused, setPollingPaused] = React.useState(false);

  const clientRef = React.useRef(null);
  const busyRef = React.useRef(false);
  const stoppedRef = React.useRef(true);
  const cyclesRef = React.useRef(0);
  const motorsRef = React.useRef({});
  const powerRef = React.useRef(null);
  const withdrawalsRef = React.useRef(new Map());
  const powerEpoch = React.useRef(0);
  const pollEpoch = React.useRef(0);
  const focusEpoch = React.useRef(0);



  motorsRef.current = motors;

  const note = React.useCallback((text, level = 'info') => {
    setLog((prev) => [...prev.slice(-199), { t: Date.now(), text, level }]);
  }, []);

  const publishPowerPending = React.useCallback(() => {
    const axes = [...withdrawalsRef.current.keys()];
    setPowerDisabling(axes);
    setPowerPending(axes[0] ?? powerRef.current);
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

  const readOne = React.useCallback(async (id, check) => {
    check();
    await selectMotor(id);
    check();
    const position = await send('robstride_read_param', { param_id: MECH_POS, type: 'f32', timeout_ms: 800 }, 3000);
    check();
    const positionUpdatedAt = Date.now();
    const params = {}; const paramErrors = {}; const fieldTimes = { mechPosRad: position.ok ? positionUpdatedAt : null };
    let runMode = null;
    try {
      const response = await send('robstride_read_param', { param_id: 0x7005, type: 'i8', timeout_ms: 800 }, 3000);
      check();
      const value = response.ok && response.data?.value != null ? Number(response.data.value) : NaN;
      if (Number.isInteger(value)) { runMode = value; fieldTimes.runMode = Date.now(); }
      else paramErrors.runMode = response.error || '运行模式读取失败';
    } catch (error) { check(); paramErrors.runMode = error.message; }
    for (const field of MOTOR_SETTINGS) {
      check();
      try {
        const response = await send('robstride_read_param', { param_id: field.id, type: 'f32', timeout_ms: 800 }, 3000);
        check();
        const value = response.ok && response.data?.value != null ? Number(response.data.value) : NaN;
        params[field.key] = Number.isFinite(value) ? value : null;
        fieldTimes[field.key] = params[field.key] == null ? null : Date.now();
        if (params[field.key] == null) paramErrors[field.key] = response.error || '读取失败';
      } catch (error) { check(); params[field.key] = null; paramErrors[field.key] = error.message; }
    }
    const rad = position.ok && position.data?.value != null ? Number(position.data.value) : Number.NaN;
    const ok = Number.isFinite(rad);
    return {
      id,
      model: MACHINE_MODELS[id],
      mechPosRad: ok ? rad : null,
      mechPosDeg: ok ? Number((rad * DEG).toFixed(3)) : null,
      mechPosWrapped: ok ? Number(wrapDeg(rad * DEG).toFixed(3)) : null,
      ...params, runMode, paramErrors, fieldTimes, positionUpdatedAt, updatedAt: Date.now(),
      error: ok ? '' : (position.error || 'mechPos 读取失败'),
    };
  }, [selectMotor, send]);

  const runCycle = React.useCallback(async () => {
    const client = clientRef.current;
    if (!client || stoppedRef.current || busyRef.current || powerRef.current || withdrawalsRef.current.size || !client.isConnected()) return;
    busyRef.current = true;
    const epoch = pollEpoch.current;
    const cancelled = () => stoppedRef.current || epoch !== pollEpoch.current || clientRef.current !== client;
    const check = () => { if (cancelled()) throw new Error('查询已中断'); };
    const started = Date.now();
    const next = {};
    let failure = '';
    try {
      for (const id of MACHINE_IDS) {
        if (cancelled()) return;
        try {
          next[id] = await readOne(id, check);
        } catch (error) {
          if (cancelled()) return;
          next[id] = { id, model: MACHINE_MODELS[id], error: error.message || String(error) };
          failure = failure || error.message || String(error);
        }
      }
    } finally {
      // Any disable has already been queued before this cleanup. No next read
      // may follow an interrupted cycle, even when a late reply arrives.
      try { if (clientRef.current === client && client.isConnected()) await client.send('close_bus', {}, 2000); }
      catch { /* 下一轮会重新 set_target。 */ }
      busyRef.current = false;
    }
    if (cancelled()) return;
    cyclesRef.current += 1;
    setMotors(next);
    setMeta({ cycleMs: Date.now() - started, updatedAt: Date.now(), cycles: cyclesRef.current, error: failure });
    return next;
  }, [readOne, send]);

  const connect = React.useCallback(async (targetUrl) => {
    if (powerRef.current || withdrawalsRef.current.size) throw new Error('逐轴电源请求尚未结束，请等待结果');
    pollEpoch.current += 1;
    powerEpoch.current += 1;
    const nextUrl = (targetUrl || endpoint).trim();
    if (clientRef.current) {
      stoppedRef.current = true;
      try { clientRef.current.disconnect(); } catch { /* 忽略 */ }
      clientRef.current = null;
    }
    stoppedRef.current = false;
    setPollingPaused(false);
    setArmed(false); setDrill(true);
    setMotors({});
    setPowerRequests({});
    setStatus('connecting');
    setEndpoint(nextUrl);
    await new Promise((resolve) => {
      const client = new WsGatewayClient({
        allowPowerControl: true,
        onOpen: () => { setStatus('online'); note(`已连接 ${nextUrl}`, 'ok'); resolve(); },
        onClose: () => {
          setArmed(false); setDrill(true);
          setPowerRequests({});
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
    pollEpoch.current += 1;
    powerEpoch.current += 1;
    setPollingPaused(false);
    setArmed(false); setDrill(true);
    try { clientRef.current?.disconnect(); } catch { /* 忽略 */ }
    clientRef.current = null;
    setStatus('disconnected');
    setMotors({});
    setPowerRequests({});
    note('已断开网关，总开关同时复位', 'info');
  }, [note]);

  const requestPower = React.useCallback(async (id, op, confirmed, limits, options = {}) => {
    if (!MACHINE_IDS.includes(id) || !['enable', 'disable'].includes(op)) throw new Error('只允许对指定单轴使能或失能');
    const emergencyDisable = op === 'disable' && options?.emergency === true;
    if (!emergencyDisable && !POWER_CONFIRMATIONS.every(key => confirmed?.[key] === true)) throw new Error('请确认现场人员、独立断电、机械支撑及所选轴');
    const client = clientRef.current;
    if (!client?.isConnected()) throw new Error('网关未连接，无法发送失能；请使用现场独立断电');
    if (emergencyDisable) {
      const previous = withdrawalsRef.current.get(id);
      if (previous) return previous.promise;
      // Queue immediately on the existing socket. Gateway 0.5.6 retargets
      // disable from vendor/model/motor_id; model must not inherit another axis.
      stoppedRef.current = true;
      setPollingPaused(true);
      pollEpoch.current += 1;
      powerEpoch.current += 1;
      const withdrawal = {};
      withdrawalsRef.current.set(id, withdrawal);
      publishPowerPending();
      withdrawal.promise = (async () => {
        try {
          const response = await client.send('disable', {
            vendor: 'robstride', model: MACHINE_MODELS[id], motor_id: id, feedback_id: FEEDBACK_ID,
          }, 4000);
          if (!response.ok) throw new Error(response.error || '网关未确认失能请求');
          if (clientRef.current === client && client.isConnected()) setPowerRequests(current => ({ ...current, [id]: { op: 'disable', sentAt: Date.now() } }));
          note(`J${id} 失能请求已发送；真实撤力需现场确认`, 'ok');
          return response;
        } catch (error) {
          if (clientRef.current === client) setPowerRequests(current => ({ ...current, [id]: { error: error.message, sentAt: Date.now() } }));
          note(`J${id} 失能请求未确认：${error.message}；请立即独立断电`, 'err');
          throw error;
        } finally {
          // The interrupted reader/enable owns its own bus cleanup. Avoid
          // releasing its context while it is still completing a request.
          if (!busyRef.current && !powerRef.current && withdrawalsRef.current.size === 1 && clientRef.current === client && client.isConnected()) {
            try { const reply = await client.send('close_bus', {}, 2000); if (!reply.ok) throw new Error(reply.error || '网关未归还总线'); }
            catch (error) { note(`失能后归还总线未确认：${error.message}`, 'err'); }
          }
          withdrawalsRef.current.delete(id);
          publishPowerPending();
        }
      })();
      return withdrawal.promise;
    }
    if (stoppedRef.current) throw new Error('状态查询已暂停；请先刷新状态或重新连接');
    if (powerRef.current) throw new Error('另一轴电源请求尚未结束');
    if (withdrawalsRef.current.size) throw new Error('失能请求尚未结束，不能使能');
    powerRef.current = id;
    publishPowerPending();
    const epoch = focusEpoch.current;
    const requestEpoch = powerEpoch.current;
    const connected = () => {
      if (requestEpoch !== powerEpoch.current) throw new Error('使能流程已被立即失能中断；请以现场出力状态为准');
      if (clientRef.current !== client || stoppedRef.current || !client.isConnected()) throw new Error('连接已变化；未继续电源请求');
      if (op === 'enable' && (epoch !== focusEpoch.current || document.visibilityState !== 'visible' || !document.hasFocus())) throw new Error('页面已失去焦点；未继续使能请求');
    };
    let ownsBus = false;
    try {
      const deadline = Date.now() + 15000;
      while (busyRef.current && Date.now() < deadline) { connected(); await new Promise(resolve => setTimeout(resolve, 50)); }
      connected();
      if (busyRef.current) throw new Error('只读查询尚未结束；未发送电源指令');
      busyRef.current = true;
      ownsBus = true;
      await selectMotor(id);
      connected();
      if (op === 'enable') {
        const position = await send('robstride_read_param', { param_id: MECH_POS, type: 'f32', timeout_ms: 800 }, 3000);
        const measuredAt = Date.now();
        connected();
        const mode = await send('robstride_read_param', { param_id: 0x7005, type: 'i8', timeout_ms: 800 }, 3000);
        connected();
        const q = position.data?.value;
        if (!position.ok || !Number.isFinite(q)) throw new Error('未取得新鲜位置；未发送使能');
        if (Date.now() - measuredAt >= 3000) throw new Error('位置读取已过期；未发送使能');
        if (!mode.ok || mode.data?.value !== 0) throw new Error('实际模式不是 MIT 或未读到；未发送使能，未自动切换模式');
        if (id <= 6 && (!limits || !Number.isFinite(limits[0]) || !Number.isFinite(limits[1]) || limits[0] >= limits[1])) throw new Error('模型限位不可用；未发送使能');
        if (id <= 6 && (q * DEG < limits[0] - START_BOUNDARY_TOLERANCE_DEG || q * DEG > limits[1] + START_BOUNDARY_TOLERANCE_DEG)) throw new Error(`J${id} 原始角度超出模型范围及 1° 起点容差；未发送使能`);
        if (id <= 6 && (q * DEG < limits[0] || q * DEG > limits[1])) note(`J${id} 起点在 1° 边界容差内；保留原始角度，不修改零位或目标限位`, 'warn');
      }
      connected();
      const response = await client.send(op, { vendor: 'robstride', motor_id: id, feedback_id: FEEDBACK_ID }, 4000);
      connected();
      if (!response.ok) throw new Error(response.error || '网关未确认电源请求');
      if (clientRef.current === client && client.isConnected()) setPowerRequests(previous => ({ ...previous, [id]: { op, sentAt: Date.now() } }));
      note(`J${id} ${op === 'enable' ? '使能' : '失能'}请求已发送；真实出力需现场确认`, 'ok');
      return response;
    } catch (error) {
      if (requestEpoch === powerEpoch.current && clientRef.current === client) setPowerRequests(previous => ({ ...previous, [id]: { error: error.message, sentAt: Date.now() } }));
      note(`J${id} 电源请求未确认：${error.message}`, 'err');
      throw error;
    } finally {
      if (ownsBus) {
        try {
          if (clientRef.current === client && client.isConnected()) {
            const reply = await client.send('close_bus', {}, 2000);
            if (!reply.ok) throw new Error(reply.error || '网关未归还总线');
          }
        } catch (error) { note(`电源请求后归还总线失败：${error.message}`, 'err'); }
        busyRef.current = false;
      }
      powerRef.current = null;
      publishPowerPending();
    }
  }, [note, selectMotor, send, publishPowerPending]);

  const refresh = React.useCallback(async () => {
    if (powerRef.current || withdrawalsRef.current.size || busyRef.current || !clientRef.current?.isConnected()) return;
    stoppedRef.current = false;
    setPollingPaused(false);
    return runCycle();
  }, [runCycle]);

  const releaseForTrial = React.useCallback(async () => {
    if (powerRef.current || withdrawalsRef.current.size) throw new Error('逐轴电源请求尚未结束，请等待结果');
    stoppedRef.current = true;
    pollEpoch.current += 1;
    const deadline = Date.now() + 15000;
    while (busyRef.current && Date.now() < deadline) await new Promise(resolve => setTimeout(resolve, 50));
    if (busyRef.current) throw new Error('只读查询尚未结束，请稍后重新准备；未启动实机');
    if (clientRef.current?.isConnected()) {
      const reply = await send('close_bus', {}, 2000);
      if (!reply.ok) throw new Error(reply.error || '只读网关未归还 CAN 总线');
    }
    disconnect();
  }, [send, disconnect]);

  // Mount/refresh never connects to CAN; explicit read authorization is required.
  React.useEffect(() => {
    const loseFocus = () => { focusEpoch.current += 1; };
    const visibility = () => { if (document.visibilityState !== 'visible') loseFocus(); };
    window.addEventListener('blur', loseFocus);
    document.addEventListener('visibilitychange', visibility);
    return () => {
      window.removeEventListener('blur', loseFocus);
      document.removeEventListener('visibilitychange', visibility);
      stoppedRef.current = true; try { clientRef.current?.disconnect(); } catch { /* 忽略 */ }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  React.useEffect(() => {
    if (status !== 'online') return undefined;
    const poll = setInterval(() => { runCycle(); }, intervalMs);
    return () => clearInterval(poll);
  }, [intervalMs, runCycle, status]);

  const blocked = React.useCallback(async () => ({ ok: false, refused: true,
    error: 'LOCKED：本批次无真实写入通道，软件停止也不发送 CAN' }), []);
  const write = blocked;
  const stopAll = blocked;
  const arm = React.useCallback(() => { setArmed(false); setDrill(true); note('LOCKED：需要后续现场授权与验收', 'warn'); }, [note]);
  const disarm = React.useCallback(() => { setArmed(false); setDrill(true); }, []);

  return {
    status, motors, ...meta,
    endpoint, setEndpoint, connect, disconnect, releaseForTrial, refresh,
    requestPower, powerRequests, powerPending, powerDisabling, pollingPaused,
    armed: false, arm, disarm, drill: true, setDrill: () => setDrill(true), maxSpd, setMaxSpd,
    write, stopAll, log, note,
  };
}
