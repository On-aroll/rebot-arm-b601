import { afterEach, expect, it, vi } from 'vitest';
import { READ_ONLY_OPS, WsGatewayClient } from './wsGatewayClient';
class Socket {
  static OPEN=1;
  constructor(){this.readyState=1;this.sent=[];}
  send(raw){this.sent.push(JSON.parse(raw));}
  close(){this.readyState=3;this.onclose?.();}
  reply(request,data={}){this.onmessage?.({data:JSON.stringify({ok:true,req_id:request.req_id,data})});}
}
afterEach(()=>{vi.useRealTimers();vi.unstubAllGlobals();});
it('supports every audited read operation with correlated replies, including legacy default clients',async()=>{
  vi.stubGlobal('WebSocket',Socket);const client=new WsGatewayClient({});client.connect('ws://mock.test');
  for(const op of READ_ONLY_OPS){
    const promise=client.send(op,{});const request=client.ws.sent.at(-1);
    expect(request.op).toBe(op);client.ws.reply(request,{value:0});
    expect((await promise).data.value).toBe(0);
  }
  expect(client.pendingByReqId.size).toBe(0);client.disconnect();
});
it('disconnect rejects outstanding reads; old-socket messages cannot settle a new read',async()=>{
  vi.stubGlobal('WebSocket',Socket);const client=new WsGatewayClient({});client.connect('ws://mock.test');
  const first=client.ws;const old=client.send('robstride_read_param',{});
  const rejected=expect(old).rejects.toThrow('disconnected');client.connect('ws://mock.test');await rejected;
  const next=client.send('robstride_read_param',{});const request=client.ws.sent.at(-1);
  first.reply(request,{value:999});expect(client.pendingByReqId.size).toBe(1);
  client.ws.reply(request,{value:1});expect((await next).data.value).toBe(1);client.disconnect();
});
it('times out unacknowledged reads without retry or extra socket traffic',async()=>{
  vi.useFakeTimers();vi.stubGlobal('WebSocket',Socket);const client=new WsGatewayClient({});client.connect('ws://mock.test');
  const response=client.send('robstride_read_param',{},20);const rejected=expect(response).rejects.toThrow('timeout');
  await vi.advanceTimersByTimeAsync(21);await rejected;
  expect(client.pendingByReqId.size).toBe(0);expect(client.ws.sent).toHaveLength(1);client.disconnect();
});
it('explicit console clients permit only correctly routed enable/disable while all motion and parameter writes stay locked', async () => {
  vi.stubGlobal('WebSocket', Socket);
  const client = new WsGatewayClient({ allowPowerControl: true });
  client.connect('ws://mock.test');
  for (const op of ['enable', 'disable']) {
    const promise = client.send(op, { vendor: 'robstride', motor_id: 2, feedback_id: 0xfd });
    const request = client.ws.sent.at(-1);
    client.ws.reply(request);
    expect((await promise).ok).toBe(true);
  }
  for (const op of ['mit', 'pos_vel', 'vel', 'ensure_mode', 'robstride_write_param', 'set_zero', 'set_id', 'stop', 'clear_error']) {
    await expect(client.send(op, {})).rejects.toThrow('LOCKED');
  }
  for (const payload of [{}, { vendor: 'robstride', motor_id: 8, feedback_id: 0xfd }, { vendor: 'robstride', motor_id: 2, feedback_id: 0xfd, op: 'mit' }]) {
    await expect(client.send('enable', payload)).rejects.toThrow('LOCKED');
  }
  expect(client.ws.sent).toHaveLength(2);
  client.disconnect();
});
