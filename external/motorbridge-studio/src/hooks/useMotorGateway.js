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

const DEG = 180 / Math.PI;

// 只读三件套：不开总开关也能用。
const READ_OPS = new Set(['set_target', 'robstride_read_param', 'close_bus']);
const ALLOWED_OPS = READ_OPS;

function wrapDeg(angle) {
  const value = ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  return Math.abs(value + 180.0) < 1e-9 ? 180.0 : value;
}

/** Explicitly connected read-only feedback; no actuator write path in batch A. */
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
    const params = {}; const paramErrors = {}; const fieldTimes = { mechPosRad: position.ok ? positionUpdatedAt : null };
    let runMode = null;
    try {
      const response = await send('robstride_read_param', { param_id: 0x7005, type: 'i8', timeout_ms: 800 }, 3000);
      const value = response.ok && response.data?.value != null ? Number(response.data.value) : NaN;
      if (Number.isInteger(value)) { runMode = value; fieldTimes.runMode = Date.now(); }
      else paramErrors.runMode = response.error || '运行模式读取失败';
    } catch (error) { paramErrors.runMode = error.message; }
    for (const field of MOTOR_SETTINGS) {
      try {
        const response = await send('robstride_read_param', { param_id: field.id, type: 'f32', timeout_ms: 800 }, 3000);
        const value = response.ok && response.data?.value != null ? Number(response.data.value) : NaN;
        params[field.key] = Number.isFinite(value) ? value : null;
        fieldTimes[field.key] = params[field.key] == null ? null : Date.now();
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
      ...params, runMode, paramErrors, fieldTimes, positionUpdatedAt, updatedAt: Date.now(),
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
    setArmed(false); setDrill(true);
    setMotors({});
    setStatus('connecting');
    setEndpoint(nextUrl);
    await new Promise((resolve) => {
      const client = new WsGatewayClient({
        readOnly: true,
        onOpen: () => { setStatus('online'); note(`已连接 ${nextUrl}`, 'ok'); resolve(); },
        onClose: () => {
          setArmed(false); setDrill(true);
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
    setArmed(false); setDrill(true);
    try { clientRef.current?.disconnect(); } catch { /* 忽略 */ }
    clientRef.current = null;
    setStatus('disconnected');
    setMotors({});
    note('已断开网关，总开关同时复位', 'info');
  }, [note]);

  // Mount/refresh never connects to CAN; explicit read authorization is required.
  React.useEffect(() => {
    return () => { stoppedRef.current = true; try { clientRef.current?.disconnect(); } catch { /* 忽略 */ } };
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
    endpoint, setEndpoint, connect, disconnect, refresh: runCycle,
    armed: false, arm, disarm, drill: true, setDrill: () => setDrill(true), maxSpd, setMaxSpd,
    write, stopAll, log, note,
  };
}
