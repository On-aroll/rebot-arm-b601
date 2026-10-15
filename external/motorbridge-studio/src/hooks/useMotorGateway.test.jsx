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
it('all clients reject writes and protocol overrides, regardless of constructor flags', async () => {
  vi.stubGlobal('WebSocket', {OPEN:1});
  for (const options of [{}, {readOnly:true}, {readOnly:false}]) {
    const client=new WsGatewayClient(options); client.readOnly=false;
    client.ws={send:vi.fn(),readyState:1};
    for (const op of ['enable','disable','stop','ensure_mode','mit','pos_vel','vel','robstride_write_param','set_zero','set_zero_position','set_active_report','state_stream','param_stream','scan','verify','state_once','store','set_id','unknown']) {
      await expect(client.send(op,{})).rejects.toThrow('LOCKED');
    }
    for (const payload of [{op:'mit'},{req_id:1},{toJSON:()=>({op:'mit'})},null,[]]) {
      await expect(client.send('close_bus',payload)).rejects.toThrow('LOCKED');
    }
    expect(client.ws.send).not.toHaveBeenCalled();
  }
});
