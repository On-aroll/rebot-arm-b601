// Default clients remain read-only. The console can opt into power commands only.
// No scan/verify/streams: their vendor-specific side effects have not been audited.
export const READ_ONLY_OPS = Object.freeze([
  'capabilities', 'set_target', 'close_bus', 'robstride_read_param',
  'robstride_ping', 'get_register_u32', 'get_register_f32',
]);

export class WsGatewayClient {
  #allowPowerControl;

  constructor({ onEvent, onState, onParams, onOpen, onClose, onError, onMessage, allowPowerControl = false }) {
    this.#allowPowerControl = allowPowerControl === true;
    this.readOnly = !this.#allowPowerControl;
    this.ws = null;
    this.pendingByReqId = new Map();
    this.nextReqId = 1;
    this.onEvent = onEvent;
    this.onState = onState;
    this.onParams = onParams;
    this.onOpen = onOpen;
    this.onClose = onClose;
    this.onError = onError;
    this.onMessage = onMessage;
  }

  rejectPending(reason = 'ws closed') {
    const err = reason instanceof Error ? reason : new Error(reason);
    this.pendingByReqId.forEach((p) => {
      clearTimeout(p.timer);
      p.reject(err);
    });

    this.pendingByReqId.clear();
  }

  isConnected() {
    return this.ws && this.ws.readyState === WebSocket.OPEN;
  }

  connect(url) {
    this.disconnect();
    const socket = new WebSocket(url);
    this.ws = socket;

    socket.onopen = () => {
      if (this.ws !== socket) return;
      this.onOpen?.();
    };

    socket.onclose = () => {
      if (this.ws !== socket) return;
      this.ws = null;
      this.rejectPending('ws closed');
      this.onClose?.();
    };

    socket.onerror = () => {
      if (this.ws !== socket) return;
      this.onError?.(new Error('ws error'));
    };

    socket.onmessage = (ev) => {
      if (this.ws !== socket) return;
      let msg;
      try {
        msg = JSON.parse(ev.data);
      } catch {
        this.onEvent?.(`non-json: ${String(ev.data).slice(0, 120)}`, 'err');
        return;
      }

      this.onMessage?.(msg);

      if (msg?.type === 'state') {
        this.onState?.(msg.data);
        return;
      }

      if (msg?.type === 'robstride_params' || msg?.type === 'damiao_params') {
        this.onParams?.(msg.data);
        return;
      }

      // Streaming progress events (e.g. robstride scan_progress) carry a
      // req_id but no `ok` field. Route them to the in-flight request's
      // onProgress hook instead of treating them as stray async messages.
      if (typeof msg?.ok !== 'boolean' && msg?.req_id != null) {
        const p = this.pendingByReqId.get(msg.req_id);
        if (p?.onProgress) {
          p.onProgress(msg);
          return;
        }
      }

      if (typeof msg?.ok === 'boolean') {
        const reqId = msg?.req_id;
        if (reqId == null) {
          this.onEvent?.(`async ws message: ${JSON.stringify(msg)}`, 'info');
          return;
        }
        const p = this.pendingByReqId.get(reqId);
        if (!p) {
          this.onEvent?.(`async ws message: ${JSON.stringify(msg)}`, 'info');
          return;
        }
        this.pendingByReqId.delete(reqId);
        clearTimeout(p.timer);
        p.resolve(msg);
        return;
      }

      this.onEvent?.(`ws msg: ${JSON.stringify(msg)}`, 'info');
    };
  }

  disconnect() {
    this.rejectPending('ws disconnected');
    const socket = this.ws;
    this.ws = null;
    if (socket) {
      try {
        socket.close();
      } catch {
        // ignore
      }
    }
    this.ws = null;
  }

  send(op, payload = {}, timeoutMs = 8000, { onProgress } = {}) {
    // Validate the final op; payload must not override protocol routing fields.
    const powerOp = this.#allowPowerControl && (op === 'enable' || op === 'disable');
    if ((!READ_ONLY_OPS.includes(op) && !powerOp) || !payload || typeof payload !== 'object' || Array.isArray(payload) || Object.hasOwn(payload, 'op') || Object.hasOwn(payload, 'req_id')) {
      return Promise.reject(new Error(`LOCKED：${op} 不在已审计只读白名单；写入、扫描和主动上报配置仍锁定`));
    }
    let safePayload;
    try {
      safePayload = JSON.parse(JSON.stringify(payload));
      if (!safePayload || typeof safePayload !== 'object' || Array.isArray(safePayload) || Object.hasOwn(safePayload, 'op') || Object.hasOwn(safePayload, 'req_id')) throw new Error('routing override');
      if (powerOp && (Object.keys(safePayload).sort().join(',') !== 'feedback_id,motor_id,vendor' || safePayload.vendor !== 'robstride' || !Number.isInteger(safePayload.motor_id) || safePayload.motor_id < 1 || safePayload.motor_id > 7 || safePayload.feedback_id !== 0xfd)) throw new Error('power routing must match a known axis');
    } catch {
      return Promise.reject(new Error('LOCKED：payload 必须是无协议覆盖的 JSON 对象'));
    }
    return new Promise((resolve, reject) => {
      if (!this.isConnected()) {
        reject(new Error('ws not connected'));
        return;
      }

      const reqId = this.nextReqId++;
      const req = { ...safePayload, op, req_id: reqId };
      const pending = {
        resolve,
        reject,
        reqId,
        onProgress,
        timer: setTimeout(() => {
          this.pendingByReqId.delete(reqId);
          reject(new Error(`timeout waiting response for op=${op}`));
        }, timeoutMs),
      };
      this.pendingByReqId.set(reqId, pending);

      try {
        this.ws.send(JSON.stringify(req));
      } catch (e) {
        clearTimeout(pending.timer);
        this.pendingByReqId.delete(reqId);
        reject(e);
      }
    });
  }
}
