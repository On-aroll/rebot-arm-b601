// @vitest-environment jsdom
import React from 'react';
import { afterEach, beforeEach, it, expect, vi } from 'vitest';
import { render, screen, fireEvent, cleanup, waitFor } from '@testing-library/react';
import { MotorPowerPanel } from './MotorPowerPanel';

let state;
let writes;
const proposal = { title: 'J1 三秒保持试验', summary: '当前位置保持 3 秒，Kp=3、Kd=1、前馈力矩=0；到时撤力。', limits: { v_max: 0.02 }, policy: { duration: 3 }, config_revision: 25 };
const response = (data, ok = true) => Promise.resolve({ ok, json: () => Promise.resolve(data) });
beforeEach(() => {
  state = { state: 'IDLE', busy: false, axis: 1 };
  writes = [];
  vi.spyOn(document, 'hasFocus').mockReturnValue(true);
  vi.stubGlobal('fetch', vi.fn((path, options = {}) => {
    if (path === '/motor-power') return response(state);
    if (path === '/motor-power/session') return response({ token: 'test-session', status: state });
    const body = JSON.parse(options.body);
    writes.push(body);
    if (body.action === 'prepare') {
      state = { state: 'READY', busy: false, axis: 1, proposal };
      return response({ plan_id: 'one-use-plan', proposal, status: state });
    }
    if (body.action === 'start') state = { state: 'RUNNING', busy: true, axis: 1 };
    if (body.action === 'stop') state = { state: 'CANCELLED', busy: false, axis: 1 };
    return response({ status: state });
  }));
});
afterEach(() => { cleanup(); vi.restoreAllMocks(); vi.unstubAllGlobals(); });

async function prepared() {
  fireEvent.click(screen.getByRole('button', { name: '检查并准备 J1' }));
  await screen.findByText('J1 三秒保持试验');
}
async function start() {
  await prepared();
  screen.getAllByRole('checkbox').forEach(checkbox => fireEvent.click(checkbox));
  fireEvent.click(screen.getByRole('button', { name: '确认使能 J1 · 约 3 秒' }));
  await screen.findByText(/本会话已请求使能/);
}

it('shows explicit controls but initially only queries state, without creating a write session or enabling', async () => {
  render(<MotorPowerPanel />);
  await screen.findByText('尚未使能');
  expect(fetch).toHaveBeenCalledTimes(1);
  expect(fetch.mock.calls[0][0]).toBe('/motor-power');
  expect(writes).toEqual([]);
  expect(screen.getByRole('button', { name: '失能 J1' }).disabled).toBe(true);
  expect(screen.getByRole('button', { name: '停止当前试验' })).toBeTruthy();
});

it('pauses read-only gateway before preparing and requires all confirmations before any start', async () => {
  const beforePrepare = vi.fn();
  const busy = vi.fn();
  render(<MotorPowerPanel beforePrepare={beforePrepare} onBusyChange={busy} />);
  await prepared();
  expect(beforePrepare).toHaveBeenCalledOnce();
  expect(busy).toHaveBeenLastCalledWith(true);
  expect(writes.map(body => body.action)).toEqual(['prepare']);
  const button = screen.getByRole('button', { name: '确认使能 J1 · 约 3 秒' });
  expect(button.disabled).toBe(true);
  fireEvent.click(button);
  const boxes = screen.getAllByRole('checkbox');
  boxes.slice(0, -1).forEach(box => fireEvent.click(box));
  expect(button.disabled).toBe(true);
  expect(writes.filter(body => body.action === 'start')).toHaveLength(0);
  fireEvent.click(boxes.at(-1));
  fireEvent.click(button);
  await screen.findByText(/本会话已请求使能/);
  expect(writes.find(body => body.action === 'start')).toEqual({ action: 'start', plan_id: 'one-use-plan', confirmed: { operator_present: true, power_cut_ready: true, supported: true, space_clear: true, accept_proposal: true } });
});

it('never prepares if pausing the existing gateway fails', async () => {
  render(<MotorPowerPanel beforePrepare={async () => { throw new Error('请等待查询完成'); }} />);
  fireEvent.click(screen.getByRole('button', { name: '检查并准备 J1' }));
  await screen.findByText('请等待查询完成');
  expect(writes).toEqual([]);
});

it('stop and disable share the owned-axis stop action and do not send arbitrary motor writes', async () => {
  render(<MotorPowerPanel />);
  await start();
  fireEvent.click(screen.getByRole('button', { name: '失能 J1' }));
  await screen.findByText('试验已取消');
  expect(writes.at(-1)).toEqual({ action: 'stop' });
  fireEvent.click(screen.getByRole('button', { name: '停止当前试验' }));
  await waitFor(() => expect(writes.filter(body => body.action === 'stop')).toHaveLength(2));
});

it('allows a refreshed browser to stop an ongoing service-owned trial without an old token', async () => {
  state = { state: 'RUNNING', busy: true, axis: 1 };
  render(<MotorPowerPanel />);
  await screen.findByText(/本会话已请求使能/);
  const button = screen.getByRole('button', { name: '停止当前试验' });
  expect(button.disabled).toBe(false);
  fireEvent.click(button);
  await screen.findByText('试验已取消');
  expect(writes).toEqual([{ action: 'stop' }]);
  expect(fetch.mock.calls.some(([path]) => path === '/motor-power/session')).toBe(true);
});

it('sends ordered heartbeats only after start and immediately reports loss of focus', async () => {
  render(<MotorPowerPanel />);
  await start();
  await waitFor(() => expect(writes.some(body => body.action === 'heartbeat' && body.focused)).toBe(true));
  fireEvent(window, new Event('blur'));
  await waitFor(() => expect(writes.some(body => body.action === 'heartbeat' && body.focused === false)).toBe(true));
  const sequences = writes.filter(body => body.action === 'heartbeat').map(body => body.sequence);
  expect(sequences).toEqual([...sequences].sort((a,b) => a-b));
  expect(new Set(sequences).size).toBe(sequences.length);
});

it('sends an owned-trial stop when unmounted', async () => {
  const view = render(<MotorPowerPanel />);
  await start();
  view.unmount();
  expect(writes.at(-1)).toEqual({ action: 'stop' });
  expect(fetch.mock.calls.at(-1)[1].keepalive).toBe(true);
});

it('retains failed checks and raw negative J2 feedback without offering a start plan', async () => {
  fetch.mockImplementation((path, options = {}) => {
    if (path === '/motor-power') return response(state);
    if (path === '/motor-power/session') return response({ token: 'test-session' });
    const body = JSON.parse(options.body);
    writes.push(body);
    return response({ ok: false, error: 'J2 原始角度不在模型范围', status: { state: 'FAULT', busy: false, reason: '检查未通过', feedback: { 2: { q: -0.31 * Math.PI / 180, qd: 0.05, mode: 0 } } } }, false);
  });
  render(<MotorPowerPanel />);
  fireEvent.click(screen.getByRole('button', { name: '检查并准备 J1' }));
  await screen.findByText('J2 原始角度不在模型范围');
  expect(screen.getByText('-0.310')).toBeTruthy();
  expect(screen.queryByRole('button', { name: '确认使能 J1 · 约 3 秒' })).toBeNull();
});

it('reports failed stop delivery without claiming that physical force was removed', async () => {
  render(<MotorPowerPanel />);
  await start();
  const original = fetch.getMockImplementation();
  fetch.mockImplementation((path, options = {}) => {
    if (path === '/motor-power/action' && JSON.parse(options.body).action === 'stop') return response({ error: 'CAN 状态未知' }, false);
    return original(path, options);
  });
  fireEvent.click(screen.getByRole('button', { name: '停止当前试验' }));
  await screen.findByText(/停止请求未确认：CAN 状态未知/);
  expect(screen.queryByText('试验已取消')).toBeNull();
  expect(screen.queryByText('试验流程结束')).toBeNull();
});

it('renews an explicitly expired session once for stop without retrying start', async () => {
  render(<MotorPowerPanel />);
  await start();
  const original = fetch.getMockImplementation();
  let expired = true;
  fetch.mockImplementation((path, options = {}) => {
    if (path === '/motor-power/action' && JSON.parse(options.body).action === 'stop' && expired) {
      expired = false;
      return response({ error: '单轴操作会话失效，请重新打开操作入口' }, false);
    }
    return original(path, options);
  });
  fireEvent.click(screen.getByRole('button', { name: '停止当前试验' }));
  await screen.findByText('试验已取消');
  expect(fetch.mock.calls.filter(([path]) => path === '/motor-power/session')).toHaveLength(2);
  expect(writes.filter(body => body.action === 'start')).toHaveLength(1);
});
