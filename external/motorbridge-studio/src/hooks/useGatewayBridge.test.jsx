// @vitest-environment jsdom
import { afterEach, describe, expect, it, vi } from 'vitest';
import { act, cleanup, renderHook } from '@testing-library/react';
import React from 'react';
import { I18nProvider } from '../i18n';
import { useGatewayBridge } from './useGatewayBridge';

class FakeWebSocket {
  static OPEN = 1;
  static instances = [];
  constructor(url) { this.url=url; this.readyState=0; this.sent=[]; FakeWebSocket.instances.push(this); }
  close() { this.readyState=3; this.onclose?.(); }
  send(raw) {
    const request=JSON.parse(raw); this.sent.push(request);
    this.onmessage?.({data:JSON.stringify({ok:true,req_id:request.req_id,data:{}})});
  }
  emitOpen() { this.readyState=1; this.onopen?.(); }
}
function setup(extra={}) {
  vi.stubGlobal('WebSocket',FakeWebSocket);
  return renderHook(props=>useGatewayBridge({wsUrl:'ws://example.test:9002',channel:'can0',
    pushLog:vi.fn(),setStateSnapshot:vi.fn(),onGatewayState:vi.fn(),onGatewayParams:vi.fn(),...props}),
    {initialProps:extra,wrapper:({children})=><I18nProvider>{children}</I18nProvider>});
}
afterEach(()=>{cleanup();vi.useRealTimers();vi.unstubAllGlobals();FakeWebSocket.instances=[];});

describe('legacy explicit read-only connections',()=>{
  it.each([false,true])('allows manual connection and preserves token setting %s',async enabled=>{
    const {result}=setup({wsTokenEnabled:enabled,wsToken:'test-token'});
    expect(FakeWebSocket.instances).toHaveLength(0);
    act(()=>result.current.connectWs()); const socket=FakeWebSocket.instances[0];
    expect(new URL(socket.url).searchParams.get('motorbridge_ws_token')).toBe(enabled?'test-token':null);
    await act(async()=>socket.emitOpen());
    expect(result.current.connected).toBe(true);
    expect(socket.sent.map(x=>x.op)).toEqual(['capabilities']);
    await act(async()=>result.current.setTargetFor('robstride','rs-06',1,253));
    expect(socket.sent.map(x=>x.op)).toEqual(['capabilities','set_target']);
    await act(async()=>result.current.sendCmd('robstride_read_param',{param_id:0x7019,type:'f32'}));
    expect(socket.sent.at(-1).op).toBe('robstride_read_param');
    await expect(result.current.sendCmd('mit',{kp:1})).rejects.toThrow('LOCKED');
    expect(socket.sent).toHaveLength(3);
  });
  it('does not auto-connect/reconnect on mount, timer, close or remount',async()=>{
    vi.useFakeTimers();const {result,unmount}=setup();
    await act(async()=>vi.advanceTimersByTime(60000));expect(FakeWebSocket.instances).toHaveLength(0);
    act(()=>result.current.connectWs()); const socket=FakeWebSocket.instances[0];
    await act(async()=>socket.emitOpen());act(()=>socket.close());
    await act(async()=>vi.advanceTimersByTime(60000));expect(FakeWebSocket.instances).toHaveLength(1);
    expect(result.current.connected).toBe(false);
    unmount();setup();await act(async()=>vi.advanceTimersByTime(60000));
    expect(FakeWebSocket.instances).toHaveLength(1);
  });
  it('cleans up the socket and ignores late close events from an old connection',async()=>{
    const {result,unmount}=setup();act(()=>result.current.connectWs());const first=FakeWebSocket.instances[0];
    await act(async()=>first.emitOpen());act(()=>result.current.disconnectWs());act(()=>result.current.connectWs());
    const second=FakeWebSocket.instances[1];await act(async()=>second.emitOpen());
    act(()=>first.onclose?.());expect(result.current.connected).toBe(true);
    unmount();expect(second.readyState).toBe(3);
  });
});
