// @vitest-environment jsdom
import { afterEach, expect, it, vi } from 'vitest';
import { act, cleanup, renderHook } from '@testing-library/react';
import { useMotorGateway } from './useMotorGateway';
import { WsGatewayClient } from '../wsGatewayClient';
afterEach(()=>{ cleanup(); vi.unstubAllGlobals(); });
it('refresh, mount, arm, drill toggle and stop cannot create sockets or writes', async () => {
  const Socket=vi.fn(); vi.stubGlobal('WebSocket',Socket);
  const {result,unmount}=renderHook(()=>useMotorGateway());
  await act(async()=>{ result.current.arm(); result.current.setDrill(false); await result.current.refresh(); await result.current.stopAll(); });
  for (const op of ['enable','disable','stop','ensure_mode','mit','pos_vel','vel','robstride_write_param','set_zero']) {
    expect((await result.current.write(1,op,{})).ok).toBe(false);
  }
  expect(result.current.armed).toBe(false); expect(result.current.drill).toBe(true);
  unmount(); renderHook(()=>useMotorGateway()); expect(Socket).not.toHaveBeenCalled();
});
it('quarantines all legacy clients and denies final-op overrides', async () => {
  const Socket=vi.fn(); vi.stubGlobal('WebSocket',Socket);
  const legacy=new WsGatewayClient({onError:vi.fn()}); legacy.connect('ws://127.0.0.1:9002');
  expect(Socket).not.toHaveBeenCalled();
  const readonly=new WsGatewayClient({readOnly:true}); readonly.ws={send:vi.fn(),readyState:1};
  for (const op of ['enable','disable','stop','ensure_mode','mit','pos_vel','vel','robstride_write_param','set_zero','set_active_report']) {
    await expect(readonly.send(op,{})).rejects.toThrow('LOCKED');
  }
  await expect(readonly.send('close_bus',{op:'mit'})).rejects.toThrow('LOCKED');
  await expect(readonly.send('close_bus',{req_id:1})).rejects.toThrow('LOCKED');
  expect(readonly.ws.send).not.toHaveBeenCalled();
});
