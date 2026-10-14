// @vitest-environment jsdom
import React from 'react';
import { afterEach, expect, it, vi } from 'vitest';
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { MachinePowerControls } from './MachinePowerControls';
afterEach(cleanup);
const gateway = () => ({ status: 'online', requestAllPower: vi.fn(async () => ({ ok: true })), powerRequests: {} });
const limits = Array(6).fill([-180, 180]);

it('uses one session checkbox for all seven, without enabling when checked or mounted', async () => {
  const g = gateway();
  const { rerender } = render(<MachinePowerControls gateway={g} limits={limits} />);
  expect(screen.getAllByRole('checkbox')).toHaveLength(1);
  expect(screen.getByRole('button', { name: '全部使能 · J1–J7' }).disabled).toBe(true);
  fireEvent.click(screen.getByRole('checkbox'));
  expect(g.requestAllPower).not.toHaveBeenCalled();
  fireEvent.click(screen.getByRole('button', { name: '全部使能 · J1–J7' }));
  await waitFor(() => expect(g.requestAllPower).toHaveBeenCalledWith('enable', { operator_present: true, power_cut_ready: true, supported: true, axis_verified: true }, limits));
  expect(screen.getByRole('checkbox').checked).toBe(true);
  rerender(<MachinePowerControls gateway={{ ...g, status: 'offline' }} limits={limits} />);
  rerender(<MachinePowerControls gateway={g} limits={limits} />);
  expect(screen.getByRole('checkbox').checked).toBe(false);
  expect(g.requestAllPower).toHaveBeenCalledOnce();
});

it('allows a single-click all-axis disable without confirmation even while enable is pending', async () => {
  const g = gateway();
  let resolve;
  g.requestAllPower.mockImplementation(op => op === 'enable' ? new Promise(r => { resolve = r; }) : Promise.resolve({ ok: true }));
  render(<MachinePowerControls gateway={g} limits={limits} />);
  fireEvent.click(screen.getByRole('checkbox'));
  fireEvent.click(screen.getByRole('button', { name: '全部使能 · J1–J7' }));
  fireEvent.click(screen.getByRole('button', { name: '全部立即失能' }));
  await waitFor(() => expect(g.requestAllPower).toHaveBeenCalledWith('disable'));
  resolve({ ok: true });
  await waitFor(() => expect(screen.getByRole('button', { name: '全部使能 · J1–J7' }).disabled).toBe(false));
});

it('never enables offline, after an unchecked condition, or during a simulation task', () => {
  const g = gateway();
  const { rerender } = render(<MachinePowerControls gateway={g} limits={limits} blocked />);
  fireEvent.click(screen.getByRole('checkbox'));
  fireEvent.click(screen.getByRole('button', { name: '全部使能 · J1–J7' }));
  expect(g.requestAllPower).not.toHaveBeenCalled();
  expect(screen.getByRole('button', { name: '全部立即失能' }).disabled).toBe(false);
  rerender(<MachinePowerControls gateway={{ ...g, status: 'disconnected' }} limits={limits} />);
  expect(screen.getByRole('button', { name: '全部立即失能' }).disabled).toBe(true);
});
