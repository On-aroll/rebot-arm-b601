// @vitest-environment jsdom
import { afterEach, expect, it, vi } from 'vitest';
import { act, cleanup, renderHook, waitFor } from '@testing-library/react';
import { useFeedbackCoordinates } from './useFeedbackCoordinates';
afterEach(() => { cleanup(); vi.unstubAllGlobals(); });

it('uses only finite fresh feedback, hides stale values and ignores late replies for old poses', async () => {
  const replies = [];
  const fetch = vi.fn(() => new Promise(resolve => replies.push(value => resolve({ ok: true, json: async () => ({ ok: true, xyz_mm: value }) }))));
  vi.stubGlobal('fetch', fetch);
  const q = [0, 1, 2, 3, 4, 5];
  const { result, rerender } = renderHook(({ q, fresh }) => useFeedbackCoordinates(q, fresh), { initialProps: { q, fresh: false } });
  expect(fetch).not.toHaveBeenCalled();
  rerender({ q, fresh: true });
  const newer = [1, 1, 2, 3, 4, 5];
  rerender({ q: newer, fresh: true });
  await act(async () => { replies[1]([10, 20, 30]); });
  await waitFor(() => expect(result.current).toEqual([10, 20, 30]));
  await act(async () => { replies[0]([99, 99, 99]); });
  expect(result.current).toEqual([10, 20, 30]);
  expect(JSON.parse(fetch.mock.calls[1][1].body)).toEqual({ q_deg: newer });
  rerender({ q: newer, fresh: false });
  expect(result.current).toBeNull();
  rerender({ q: [NaN, ...q.slice(1)], fresh: true });
  expect(fetch).toHaveBeenCalledTimes(2);
});
