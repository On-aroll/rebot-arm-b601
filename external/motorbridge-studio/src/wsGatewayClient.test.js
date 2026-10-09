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
