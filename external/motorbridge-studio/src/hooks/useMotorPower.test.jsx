// @vitest-environment jsdom
import { act, cleanup, renderHook, waitFor } from '@testing-library/react';
import { afterEach, beforeEach, expect, it, vi } from 'vitest';
import { useMotorGateway } from './useMotorGateway';

class Socket {
  static OPEN = 1;
  static instances = [];
  constructor() {
    this.readyState = 1;
    this.sent = [];
    this.axis = null;
    this.mode = 0;
    this.position = 0;
    Socket.instances.push(this);
    queueMicrotask(() => this.onopen?.());
  }
  send(raw) {
    const request = JSON.parse(raw);
    this.sent.push(request);
    if (request.op === 'set_target') this.axis = request.motor_id;
    const reply = () => this.onmessage?.({ data: JSON.stringify({
      req_id: request.req_id, ok: this.fail !== request.op,
      error: this.fail === request.op ? 'CAN 未确认' : undefined,
      data: { value: request.param_id === 0x7005 ? this.mode : request.param_id === 0x7019 ? this.position : 0 },
    }) });
    if (this.defer?.(request)) this.release = reply;
    else queueMicrotask(reply);
  }
  close() { this.readyState = 3; this.onclose?.(); }
}

const confirmations = { operator_present: true, power_cut_ready: true, supported: true, axis_verified: true };
beforeEach(() => {
  Socket.instances = [];
  vi.stubGlobal('WebSocket', Socket);
  vi.spyOn(document, 'hasFocus').mockReturnValue(true);
});
afterEach(() => { cleanup(); vi.useRealTimers(); vi.restoreAllMocks(); vi.unstubAllGlobals(); });

async function connected() {
  const hook = renderHook(() => useMotorGateway({ intervalMs: 60000 }));
  await act(async () => { await hook.result.current.connect('ws://mock.test'); });
  await waitFor(() => expect(hook.result.current.cycles).toBe(1));
  const socket = Socket.instances[0];
  expect(socket.sent.every(r => !['enable', 'disable'].includes(r.op))).toBe(true);
  socket.sent = [];
  return { ...hook, socket };
}

it('sends exactly one real protocol command to each selected axis and returns the bus without mode/parameter/target writes', async () => {
  const { result, socket } = await connected();
  for (let id = 1; id <= 7; id++) {
    await act(async () => { await result.current.requestPower(id, 'enable', confirmations, [-180, 180]); });
    expect(socket.sent.slice(-5).map(r => r.op)).toEqual(['set_target', 'robstride_read_param', 'robstride_read_param', 'enable', 'close_bus']);
    expect(socket.sent.at(-2)).toMatchObject({ op: 'enable', vendor: 'robstride', motor_id: id, feedback_id: 0xfd });
    expect(socket.axis).toBe(id);
    await act(async () => { await result.current.requestPower(id, 'disable', confirmations); });
    expect(socket.sent.slice(-3).map(r => r.op)).toEqual(['set_target', 'disable', 'close_bus']);
    expect(socket.sent.at(-2).motor_id).toBe(id);
    expect(result.current.powerRequests[id].op).toBe('disable');
  }
  expect(socket.sent.filter(r => r.op === 'enable')).toHaveLength(7);
  expect(socket.sent.filter(r => r.op === 'disable')).toHaveLength(7);
  expect(result.current.powerPending).toBeNull();
});

it('refuses missing confirmations, invalid axes, forbidden actions, bad modes and out-of-range raw positions', async () => {
  const { result, socket } = await connected();
  for (const args of [[1, 'enable', {}], [8, 'enable', confirmations], [1, 'mit', confirmations]]) {
    await act(async () => { await expect(result.current.requestPower(...args)).rejects.toThrow(); });
  }
  expect(socket.sent).toEqual([]);
  socket.mode = 1;
  await act(async () => { await expect(result.current.requestPower(1, 'enable', confirmations, [-180, 180])).rejects.toThrow('实际模式不是 MIT'); });
  socket.mode = 0; socket.position = -0.02;
  await act(async () => { await expect(result.current.requestPower(2, 'enable', confirmations, [0, 180])).rejects.toThrow('原始角度超出'); });
  await act(async () => { await expect(result.current.requestPower(1, 'enable', confirmations)).rejects.toThrow('模型限位不可用'); });
  expect(socket.sent.some(r => r.op === 'enable')).toBe(false);
  await act(async () => { await result.current.requestPower(2, 'disable', confirmations); });
  expect(socket.sent.at(-2).op).toBe('disable');
});

it('allows a confirmed sub-degree boundary reading without zero, gain, mode or motion writes', async () => {
  const { result, socket } = await connected();
  socket.position = -.39 * Math.PI / 180;
  await act(async () => { await result.current.requestPower(2, 'enable', confirmations, [0, 180]); });
  expect(socket.sent.map(r => r.op)).toEqual(['set_target', 'robstride_read_param', 'robstride_read_param', 'enable', 'close_bus']);
  expect(result.current.log.some(entry => entry.text.includes('1° 边界容差'))).toBe(true);
});

it('sends an emergency disable without confirmation gates and leaves polling stopped', async () => {
  const { result, socket } = await connected();
  await act(async () => {
    await result.current.requestPower(2, 'disable', {}, undefined, { emergency: true });
  });
  expect(socket.sent.map(r => r.op)).toEqual(['disable', 'close_bus']);
  expect(socket.sent[0]).toMatchObject({ motor_id: 2, model: 'rs-06' });
  expect(result.current.powerRequests[2].op).toBe('disable');
  expect(result.current.pollingPaused).toBe(true);
  const count = socket.sent.length;
  await act(async () => { await result.current.requestPower(2, 'disable', {}, undefined, { emergency: true }); });
  expect(socket.sent.slice(count).map(r => r.op)).toEqual(['disable', 'close_bus']);
  await act(async () => { await expect(result.current.requestPower(1, 'enable', confirmations, [-180, 180])).rejects.toThrow('查询已暂停'); });
});

it('sends emergency disable immediately while a read-only query is still pending', async () => {
  const { result, socket } = await connected();
  socket.defer = request => request.op === 'set_target';
  let refreshPromise;
  act(() => { refreshPromise = result.current.refresh(); });
  await waitFor(() => expect(socket.sent.at(-1)?.op).toBe('set_target'));
  const beforeDisable = socket.sent.length;

  let disablePromise;
  act(() => { disablePromise = result.current.requestPower(4, 'disable', {}, undefined, { emergency: true }); });
  await act(async () => { await disablePromise; });

  expect(socket.sent.slice(beforeDisable).map(request => request.op)).toEqual(['disable']);
  expect(socket.sent.find(request => request.op === 'disable')).toMatchObject({ motor_id: 4, model: 'rs-00', feedback_id: 0xfd });
  expect(result.current.powerRequests[4].op).toBe('disable');

  socket.defer = null;
  socket.release?.();
  await act(async () => { await refreshPromise; });
  expect(socket.sent.map(request => request.op)).toEqual(['set_target', 'disable', 'close_bus']);
});

it('preempts pending enable checks and never sends enable after withdrawal', async () => {
  const { result, socket } = await connected();
  socket.defer = request => request.param_id === 0x7019;
  let rejected;
  act(() => { rejected = expect(result.current.requestPower(1, 'enable', confirmations, [-180, 180])).rejects.toThrow('失能中断'); });
  await waitFor(() => expect(socket.release).toBeTruthy());
  await act(async () => { await result.current.requestPower(1, 'disable', {}, undefined, { emergency: true }); });
  expect(socket.sent.map(r => r.op)).toEqual(['set_target', 'robstride_read_param', 'disable']);
  socket.defer = null;
  await act(async () => { socket.release(); await rejected; });
  expect(socket.sent.some(r => r.op === 'enable')).toBe(false);
  expect(result.current.powerRequests[1].op).toBe('disable');
  expect(result.current.powerPending).toBeNull();
});

it('keeps a late enable reply from overriding a newer disable result', async () => {
  const { result, socket } = await connected();
  socket.defer = request => request.op === 'enable';
  let rejected;
  act(() => { rejected = expect(result.current.requestPower(1, 'enable', confirmations, [-180, 180])).rejects.toThrow('失能中断'); });
  await waitFor(() => expect(socket.release).toBeTruthy());
  await act(async () => { await result.current.requestPower(1, 'disable', {}, undefined, { emergency: true }); });
  socket.defer = null;
  await act(async () => { socket.release(); await rejected; });
  expect(socket.sent.filter(r => ['enable', 'disable'].includes(r.op)).map(r => r.op)).toEqual(['enable', 'disable']);
  expect(result.current.powerRequests[1].op).toBe('disable');
});

it('coalesces repeated withdrawal clicks but allows another axis immediately', async () => {
  const { result, socket } = await connected();
  socket.defer = request => request.op === 'disable' && request.motor_id === 1;
  let first, duplicate;
  act(() => {
    first = result.current.requestPower(1, 'disable', {}, undefined, { emergency: true });
    duplicate = result.current.requestPower(1, 'disable', {}, undefined, { emergency: true });
  });
  await act(async () => { await result.current.requestPower(7, 'disable', {}, undefined, { emergency: true }); });
  expect(socket.sent.filter(r => r.op === 'disable').map(r => [r.motor_id, r.model])).toEqual([[1, 'rs-06'], [7, 'rs-00']]);
  expect(result.current.powerDisabling).toEqual([1]);
  socket.defer = null;
  await act(async () => { socket.release(); await Promise.all([first, duplicate]); });
  expect(result.current.powerPending).toBeNull();
});

it('does not restart polling or emit enable automatically after withdrawal, and permits explicit read refresh', async () => {
  const { result, socket } = await connected();
  vi.useFakeTimers();
  await act(async () => { await result.current.requestPower(1, 'disable', {}, undefined, { emergency: true }); });
  const count = socket.sent.length;
  await act(async () => { await vi.advanceTimersByTimeAsync(120000); });
  expect(socket.sent.length).toBe(count);
  await act(async () => { await result.current.refresh(); });
  expect(result.current.pollingPaused).toBe(false);
  expect(socket.sent.slice(count).some(r => r.op === 'robstride_read_param')).toBe(true);
  expect(socket.sent.some(r => r.op === 'enable')).toBe(false);
});

it('serializes power with polling and refuses another axis while the first request is in flight', async () => {
  const { result, socket } = await connected();
  socket.defer = request => request.op === 'set_target';
  let first;
  act(() => { first = result.current.requestPower(1, 'enable', confirmations, [-180, 180]); });
  await waitFor(() => expect(socket.release).toBeTruthy());
  await act(async () => {
    await expect(result.current.requestPower(2, 'enable', confirmations, [-180, 180])).rejects.toThrow('另一轴');
    await result.current.refresh();
  });
  expect(socket.sent).toHaveLength(1);
  socket.defer = null;
  await act(async () => { socket.release(); await first; });
  expect(socket.sent.filter(r => r.op === 'enable')).toHaveLength(1);
});

it('cancels a queued enable after focus loss and does not automatically disable gravity joints', async () => {
  const { result, socket } = await connected();
  socket.defer = request => request.param_id === 0x7019;
  let first, rejected;
  act(() => {
    first = result.current.requestPower(2, 'enable', confirmations, [0, 180]);
    rejected = expect(first).rejects.toThrow('失去焦点');
  });
  await waitFor(() => expect(socket.release).toBeTruthy());
  window.dispatchEvent(new Event('blur'));
  socket.defer = null;
  await act(async () => { socket.release(); await rejected; });
  expect(socket.sent.some(r => ['enable', 'disable'].includes(r.op))).toBe(false);
});

it('does not retry an unconfirmed power command or claim a real enabled state', async () => {
  const { result, socket } = await connected();
  socket.fail = 'enable';
  await act(async () => { await expect(result.current.requestPower(1, 'enable', confirmations, [-180, 180])).rejects.toThrow('CAN 未确认'); });
  expect(socket.sent.filter(r => r.op === 'enable')).toHaveLength(1);
  expect(result.current.powerRequests[1].error).toBe('CAN 未确认');
  expect(result.current.powerRequests[1].op).toBeUndefined();
});
