// @vitest-environment jsdom
import React from 'react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { act, cleanup, fireEvent, render, screen } from '@testing-library/react';
import { I18nProvider } from '../i18n';
import { useGatewayBridge } from './useGatewayBridge';

class FakeWebSocket {
  static OPEN = 1;

  constructor(url) {
    this.url = url;
    this.readyState = 0;
    this.onopen = null;
    this.onclose = null;
    this.onerror = null;
    this.onmessage = null;
    FakeWebSocket.instances.push(this);
  }

  close() {
    this.readyState = 2;
  }

  send() {}

  emitOpen() {
    this.readyState = FakeWebSocket.OPEN;
    this.onopen?.();
  }
}

FakeWebSocket.instances = [];

function Harness({ wsUrl = 'ws://example.com:9002', wsTokenEnabled = false, wsToken = '' }) {
  const bridge = useGatewayBridge({
    wsUrl,
    channel: 'can0',
    wsTokenEnabled,
    wsToken,
    pushLog: vi.fn(),
    setStateSnapshot: vi.fn(),
    onGatewayState: vi.fn(),
    onGatewayParams: vi.fn(),
  });

  return (
    <>
      <div data-testid="status">{bridge.connText}</div>
      <button onClick={bridge.connectWs}>Connect</button>
      <button onClick={bridge.disconnectWs}>Disconnect</button>
    </>
  );
}

afterEach(() => {
  cleanup();
  vi.useRealTimers();
  vi.restoreAllMocks();
  vi.unstubAllGlobals();
  FakeWebSocket.instances = [];
});

describe('legacy gateway quarantine', () => {
  it.each([false, true])('cannot connect a legacy driver even with token auth %s', enabled => {
    vi.stubGlobal('WebSocket', FakeWebSocket);
    const { rerender } = render(<I18nProvider><Harness wsTokenEnabled={enabled} wsToken="secret-token" /></I18nProvider>);
    fireEvent.click(screen.getByRole('button', { name: 'Connect' }));
    expect(FakeWebSocket.instances).toHaveLength(0);
    rerender(<I18nProvider><Harness wsTokenEnabled={enabled} wsToken="updated-token" /></I18nProvider>);
    fireEvent.click(screen.getByRole('button', { name: 'Connect' }));
    fireEvent.click(screen.getByRole('button', { name: 'Disconnect' }));
    expect(FakeWebSocket.instances).toHaveLength(0);
  });
  it('cannot recover a connection through mount or timer retry', async () => {
    vi.useFakeTimers(); vi.stubGlobal('WebSocket', FakeWebSocket);
    const { unmount } = render(<I18nProvider><Harness /></I18nProvider>);
    fireEvent.click(screen.getByRole('button', { name: 'Connect' }));
    await act(async () => { vi.advanceTimersByTime(60000); });
    unmount(); render(<I18nProvider><Harness /></I18nProvider>);
    expect(FakeWebSocket.instances).toHaveLength(0);
  });
});
