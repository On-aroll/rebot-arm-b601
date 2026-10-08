export class WsGatewayClient {
  constructor({ readOnly = false, onEvent, onState, onParams, onOpen, onClose, onError, onMessage }) {
    this.readOnly = readOnly;
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
    // Legacy studio drivers are quarantined in this offline batch.
    if (!this.readOnly) { this.onError?.(new Error("LOCKED：旧实机控制入口已隔离，请使用控制台只读连接")); return; }
    this.disconnect();
    this.ws = new WebSocket(url);

    this.ws.onopen = () => {
      this.onOpen?.();
    };

    this.ws.onclose = () => {
      this.rejectPending('ws closed');
      this.onClose?.();
    };

    this.ws.onerror = () => {
      this.onError?.(new Error('ws error'));
    };

    this.ws.onmessage = (ev) => {
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
    if (this.ws) {
      try {
        this.ws.close();
      } catch {
        // ignore
      }
    }
    this.ws = null;
  }

  send(op, payload = {}, timeoutMs = 8000, { onProgress } = {}) {
    // Validate the final op; payload must not override protocol routing fields.
    if (!this.readOnly || !['set_target', 'robstride_read_param', 'close_bus'].includes(op) || Object.hasOwn(payload, 'op') || Object.hasOwn(payload, 'req_id')) {
      return Promise.reject(new Error('LOCKED：当前仅允许显式授权后的只读查询'));
    }
    return new Promise((resolve, reject) => {
      if (!this.isConnected()) {
        reject(new Error('ws not connected'));
        return;
      }

      const reqId = this.nextReqId++;
      const req = { op, req_id: reqId, ...payload };
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
