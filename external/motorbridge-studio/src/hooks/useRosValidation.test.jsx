// @vitest-environment jsdom
import { afterEach, expect, it, vi } from 'vitest';
import { act, cleanup, renderHook, waitFor } from '@testing-library/react';
import { useRosValidation } from './useRosValidation';

const feedback = () => ({backend:'ros2-fake',namespace:'rebotarm_validation',hardware:false,fresh:true,feedback_time:Date.now()/1000,enabled:false,q_rad:[0,0,0,0,0,0]});
afterEach(() => { cleanup(); vi.unstubAllGlobals(); });

it('polls only read-only feedback and never enables on mount', async () => {
  const fetch = vi.fn(async () => ({ok:true,json:async()=>feedback()}));
  vi.stubGlobal('fetch', fetch);
  const { result } = renderHook(() => useRosValidation());
  await waitFor(() => expect(result.current.state?.backend).toBe('ros2-fake'));
  expect(fetch.mock.calls.every(([url, options]) => url === '/ros-control/state' && options.method === 'GET')).toBe(true);
  expect(result.current.state.enabled).toBe(false);
});

it('rejects a physical or unexpected backend instead of showing it as virtual feedback', async () => {
  vi.stubGlobal('fetch', vi.fn(async () => ({ok:true,json:async()=>({...feedback(),hardware:true})})));
  const { result } = renderHook(() => useRosValidation());
  await waitFor(() => expect(result.current.error).toContain('不是隔离的虚拟电机'));
  expect(result.current.state).toBeNull();
});

it('drops previous feedback when a fresh query fails', async () => {
  let fail = false;
  vi.stubGlobal('fetch', vi.fn(async () => { if (fail) throw Error('offline'); return {ok:true,json:async()=>feedback()}; }));
  const { result } = renderHook(() => useRosValidation());
  await waitFor(() => expect(result.current.state).toBeTruthy());
  fail = true;
  await act(async () => result.current.refresh());
  expect(result.current.state).toBeNull();
  expect(result.current.error).toBe('offline');
});

it('sends stop while enable is pending and ignores its late reply', async () => {
  let resolveEnable;
  const fetch = vi.fn(async (url, options) => {
    const op = options.body && JSON.parse(options.body).action;
    if (op === 'enable') return await new Promise(resolve => { resolveEnable = resolve; });
    return {ok:true,json:async()=>feedback()};
  });
  vi.stubGlobal('fetch', fetch);
  const { result } = renderHook(() => useRosValidation());
  await waitFor(() => expect(result.current.state).toBeTruthy());
  let enable;
  act(() => { enable = result.current.action('enable'); });
  await waitFor(() => expect(result.current.pending).toBe(true));
  await act(async () => result.current.action('stop'));
  expect(fetch.mock.calls.some(([,o]) => o.body && JSON.parse(o.body).action === 'stop')).toBe(true);
  await act(async () => { resolveEnable({ok:true,json:async()=>({...feedback(),enabled:true})}); await enable; });
  expect(result.current.state.enabled).toBe(false);
  expect(result.current.pending).toBe(false);
});

it('uses the execution reply immediately and rejects an older completed poll', async () => {
  let oldPoll;
  let defer = false;
  vi.stubGlobal('fetch', vi.fn(async () => {
    if (defer) return await new Promise(resolve => { oldPoll = resolve; });
    return {ok:true,json:async()=>({...feedback(),job:{status:'succeeded'}})};
  }));
  const { result } = renderHook(() => useRosValidation());
  await waitFor(() => expect(result.current.state?.job?.status).toBe('succeeded'));
  defer = true;
  let refresh;
  act(() => { refresh = result.current.refresh(); });
  act(() => result.current.acceptFeedback({...feedback(),job:{status:'starting'}}));
  expect(result.current.running).toBe(true);
  await act(async () => { oldPoll({ok:true,json:async()=>({...feedback(),job:{status:'succeeded'}})}); await refresh; });
  expect(result.current.state.job.status).toBe('starting');
});
