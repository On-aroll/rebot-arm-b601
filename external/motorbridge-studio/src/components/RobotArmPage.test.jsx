// @vitest-environment jsdom
import React from 'react';
import { cleanup, render } from '@testing-library/react';
import { afterEach, expect, it, vi } from 'vitest';
import { I18nProvider } from '../i18n';
import { MotorStudioProvider } from '../hooks/useMotorStudioContext';
import { RobotArmPage } from './RobotArmPage';

vi.mock('./robot-arm/managers', () => ({
  ToastManager: ({ children }) => children,
  ZeroDialogManager: () => null,
  TrailManager: () => null,
  SequenceManager: () => null,
  ParamManager: () => null,
  LiveMoveScheduler: () => null,
  clampByLimit: vi.fn(),
}));
afterEach(cleanup);

it('entering/re-entering the arm page does not overwrite edited gains or target', () => {
  const patchControl = vi.fn();
  const value = {
    connection: { connected: false, readOnly: true },
    control: { patchControl },
    robotArm: { robotArmModel: 'rebot-arm-robstride', robotArmJointRows: [
      { key: 'robstride:1:253', joint: 1, hit: { vendor: 'robstride', online: true, pos: .7 }, control: { kp: 3, kd: 1, tau: 0, target: .8 } },
    ] },
    preferences: { uiPrefs: {} }, dev: { devMode: false },
    scan: {}, logs: {}, workspace: {},
  };
  const ui = <I18nProvider><MotorStudioProvider value={value}><RobotArmPage /></MotorStudioProvider></I18nProvider>;
  const first = render(ui);
  first.unmount();
  render(ui);
  expect(patchControl).not.toHaveBeenCalled();
});
