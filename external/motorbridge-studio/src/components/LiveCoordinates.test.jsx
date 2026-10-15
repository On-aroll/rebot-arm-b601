// @vitest-environment jsdom
import React from 'react';
import { afterEach, expect, it } from 'vitest';
import { cleanup, render, screen } from '@testing-library/react';
import { LiveCoordinates } from './LiveCoordinates';
afterEach(cleanup);
it('identifies the source and suppresses coordinates once feedback is stale', () => {
  const props = { source: '实机关节反馈 → 模型计算', xyz: [120, -33.1, 200], fresh: true, ageMs: 123, cycleMs: 80 };
  const { rerender } = render(<LiveCoordinates {...props} />);
  expect(screen.getByText(props.source)).toBeTruthy();
  expect(screen.getByText('120.00')).toBeTruthy();
  expect(screen.getByText(/反馈距今 123 ms/)).toBeTruthy();
  rerender(<LiveCoordinates {...props} fresh={false} />);
  expect(screen.queryByText('120.00')).toBeNull();
  expect(screen.getAllByText('—')).toHaveLength(3);
});
