// @vitest-environment jsdom
import React from 'react';
import { afterEach, expect, it, vi } from 'vitest';
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { AxisPowerControls } from './AxisPowerControls';

afterEach(cleanup);
const gateway = () => ({ status: 'online', powerPending: null, powerRequests: {}, requestPower: vi.fn().mockResolvedValue({ ok: true }) });

it('shows independent controls for seven axes without sending power commands on render', () => {
  const g = gateway();
  render(<>{[1, 2, 3, 4, 5, 6, 7].map(id => <AxisPowerControls key={id} id={id} gateway={g} />)}</>);
  expect(screen.getAllByRole('button')).toHaveLength(14);
  expect(g.requestPower).not.toHaveBeenCalled();
  expect(screen.queryByRole('checkbox')).toBeNull();
});

it('uses one checkbox for all onsite conditions and forwards only the selected axis operation', async () => {
  const g = gateway();
  render(<AxisPowerControls id={3} gateway={g} limits={[0, 180]} />);
  fireEvent.click(screen.getByRole('button', { name: '使能 J3' }));
  const button = screen.getByRole('button', { name: '确认使能 J3' });
  expect(button.disabled).toBe(true);
  expect(screen.getAllByRole('checkbox')).toHaveLength(1);
  fireEvent.click(screen.getByRole('checkbox'));
  fireEvent.click(button);
  await waitFor(() => expect(g.requestPower).toHaveBeenCalledWith(3, 'enable', { operator_present: true, power_cut_ready: true, supported: true, axis_verified: true }, [0, 180]));
  await waitFor(() => expect(screen.queryByRole('checkbox')).toBeNull());
  fireEvent.click(screen.getByRole('button', { name: '立即失能 J3' }));
  await waitFor(() => expect(g.requestPower).toHaveBeenCalledWith(3, 'disable', {}, undefined, { emergency: true }));
});

it('clears confirmations on disconnect and blocks disconnected or concurrent requests', () => {
  const g = gateway();
  const view = render(<AxisPowerControls id={1} gateway={g} />);
  fireEvent.click(screen.getByRole('button', { name: '使能 J1' }));
  fireEvent.click(screen.getByRole('checkbox'));
  view.rerender(<AxisPowerControls id={1} gateway={{ ...g, status: 'offline' }} />);
  expect(screen.queryByRole('checkbox')).toBeNull();
  expect(screen.getByRole('button', { name: '使能 J1' }).disabled).toBe(true);
  view.rerender(<AxisPowerControls id={1} gateway={{ ...g, powerPending: 2 }} />);
  expect(screen.getByRole('button', { name: '立即失能 J1' }).disabled).toBe(false);
  view.rerender(<AxisPowerControls id={1} gateway={g} />);
  fireEvent.click(screen.getByRole('button', { name: '使能 J1' }));
  expect(screen.getByRole('checkbox').checked).toBe(false);
  expect(screen.getByRole('button', { name: '确认使能 J1' }).disabled).toBe(true);
  expect(g.requestPower).not.toHaveBeenCalled();
});

it('reports transport success as a request and failed delivery as unknown output', async () => {
  const g = gateway(); g.powerRequests[2] = { op: 'enable' };
  g.requestPower.mockRejectedValue(new Error('通信超时'));
  render(<AxisPowerControls id={2} gateway={g} />);
  expect(screen.getByRole('status').textContent).toBe('真实使能请求已发送 · 出力待确认');
  fireEvent.click(screen.getByRole('button', { name: '立即失能 J2' }));
  await screen.findByRole('alert');
  expect(screen.getByRole('alert').textContent).toContain('通信超时');
});

it('keeps immediate withdrawal available while simulation or ROS blocks new enables', async () => {
  const g = gateway();
  render(<AxisPowerControls id={2} gateway={g} blocked />);
  expect(screen.getByRole('button', { name: '使能 J2' }).disabled).toBe(true);
  expect(screen.getByRole('button', { name: '立即失能 J2' }).disabled).toBe(false);
  fireEvent.click(screen.getByRole('button', { name: '立即失能 J2' }));
  await waitFor(() => expect(g.requestPower).toHaveBeenCalledWith(2, 'disable', {}, undefined, { emergency: true }));
});

it('allows one-click disable during an in-flight enable and removes its confirmation form', async () => {
  const g = gateway();
  let rejectEnable;
  g.requestPower.mockImplementation((id, op) => op === 'enable' ? new Promise((resolve, reject) => { rejectEnable = reject; }) : Promise.resolve({ ok: true }));
  render(<AxisPowerControls id={1} gateway={g} limits={[-180, 180]} />);
  fireEvent.click(screen.getByRole('button', { name: '使能 J1' }));
  fireEvent.click(screen.getByRole('checkbox'));
  fireEvent.click(screen.getByRole('button', { name: '确认使能 J1' }));
  expect(screen.getByRole('button', { name: '立即失能 J1' }).disabled).toBe(false);
  fireEvent.click(screen.getByRole('button', { name: '立即失能 J1' }));
  await waitFor(() => expect(g.requestPower).toHaveBeenCalledWith(1, 'disable', {}, undefined, { emergency: true }));
  expect(screen.queryByRole('checkbox')).toBeNull();
  rejectEnable(new Error('使能已中断'));
  await waitFor(() => expect(screen.getByRole('button', { name: '使能 J1' }).disabled).toBe(false));
  expect(screen.queryByRole('alert')).toBeNull();
});

it('blocks only the axis with a pending disable, leaving other axes withdrawal available', () => {
  const g = { ...gateway(), powerPending: 2, powerDisabling: [2] };
  render(<><AxisPowerControls id={1} gateway={g} /><AxisPowerControls id={2} gateway={g} /></>);
  expect(screen.getByRole('button', { name: '立即失能 J1' }).disabled).toBe(false);
  expect(screen.getByRole('button', { name: '立即失能 J2' }).disabled).toBe(true);
});
