// @vitest-environment jsdom
import {act,cleanup,renderHook,waitFor} from '@testing-library/react';
import {afterEach,beforeEach,expect,it,vi} from 'vitest';
import {useHardwarePoint} from './useHardwarePoint';

let server,fetch;
beforeEach(()=>{
  server={state:'IDLE',busy:false,feedback:{}};
  vi.spyOn(document,'hasFocus').mockReturnValue(true);
  fetch=vi.fn(async(path,options)=>{
    const body=options?.body ? JSON.parse(options.body) : {};
    if(path.endsWith('/session')) return {ok:true,json:async()=>({token:'virtual-session'})};
    if(body.action==='execute') server={state:'RUNNING',busy:true,feedback:{}};
    if(body.action==='stop') server={state:'STOPPING',busy:true,feedback:{}};
    return {ok:true,json:async()=>body.action ? {status:{...server}} : {...server}};
  });
  vi.stubGlobal('fetch',fetch);
});
afterEach(()=>{cleanup();vi.useRealTimers();vi.restoreAllMocks();vi.unstubAllGlobals();});
const actions=()=>fetch.mock.calls.filter(([path,o])=>path.endsWith('/action')&&o?.body).map(([,o])=>JSON.parse(o.body));

it('mount is read-only, and only a confirmed click submits a reviewed token after bus release',async()=>{
  const release=vi.fn(async()=>{});
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:release}));
  await act(async()=>{});
  expect(actions()).toEqual([]);
  await act(async()=>{await expect(result.current.execute('plan-token',false)).rejects.toThrow('现场确认');});
  expect(release).not.toHaveBeenCalled();
  await act(async()=>{await result.current.execute('plan-token',true);});
  const call=fetch.mock.calls.find(([,o])=>o?.body&&JSON.parse(o.body).action==='execute');
  const index=fetch.mock.calls.indexOf(call);
  expect(release.mock.invocationCallOrder[0]).toBeLessThan(fetch.mock.invocationCallOrder[index]);
  expect(actions()[0]).toMatchObject({action:'execute',plan_id:'plan-token'});
  expect(actions()[0].target).toBeUndefined();
  expect(result.current.busy).toBe(true);
});

it('stop cancels pending bus handoff before any motion request is sent',async()=>{
  let finish;
  const release=vi.fn(()=>new Promise(resolve=>{finish=resolve;}));
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:release}));
  let attempt;
  act(()=>{attempt=result.current.execute('plan-token',true).catch(e=>e);});
  await waitFor(()=>expect(release).toHaveBeenCalledOnce());
  await act(async()=>{await result.current.stop();finish();await attempt;});
  expect(actions().some(body=>body.action==='execute')).toBe(false);
  expect(actions().some(body=>body.action==='stop')).toBe(true);
});

it('loss of focus stops, and reconnect waits until the worker has finished cleanup',async()=>{
  const finished=vi.fn();
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{},onFinished:finished}));
  await act(async()=>{await result.current.execute('plan-token',true);});
  act(()=>window.dispatchEvent(new Event('blur')));
  await waitFor(()=>expect(actions().some(body=>body.action==='stop')).toBe(true));
  expect(finished).not.toHaveBeenCalled();
  server={state:'CANCELLED',busy:false,feedback:{}};
  await waitFor(()=>expect(finished).toHaveBeenCalledOnce());
  expect(actions().filter(body=>body.action==='execute')).toHaveLength(1);
});

it('losing focus during CAN handoff cannot start a motor task afterward',async()=>{
  let finish;
  const release=vi.fn(()=>new Promise(resolve=>{finish=resolve;}));
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:release}));
  let attempt;
  act(()=>{attempt=result.current.execute('plan-token',true).catch(e=>e);});
  await waitFor(()=>expect(release).toHaveBeenCalledOnce());
  act(()=>window.dispatchEvent(new Event('blur')));
  await waitFor(()=>expect(actions().some(body=>body.action==='stop')).toBe(true));
  await act(async()=>{finish();await attempt;});
  expect(actions().some(body=>body.action==='execute')).toBe(false);
});

it('an uncertain start response sends stop without retrying the motion',async()=>{
  const base=fetch.getMockImplementation();
  fetch.mockImplementation(async(path,options)=>{
    if(options?.body&&JSON.parse(options.body).action==='execute') throw new Error('reply lost');
    return base(path,options);
  });
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{}}));
  await act(async()=>{await expect(result.current.execute('plan-token',true)).rejects.toThrow('reply lost');});
  expect(actions().filter(body=>body.action==='execute')).toHaveLength(1);
  expect(actions().some(body=>body.action==='stop')).toBe(true);
});

it('a heartbeat rejected after worker completion preserves the actual motor fault',async()=>{
  vi.useFakeTimers();
  const base=fetch.getMockImplementation();
  fetch.mockImplementation(async(path,options)=>{
    if(options?.body&&JSON.parse(options.body).action==='heartbeat') {
      server={state:'FAULT',busy:false,reason:'J2 实测速度超过试验边界',feedback:{}};
      return {ok:false,json:async()=>({error:'试验心跳重复、过期或页面不匹配'})};
    }
    return base(path,options);
  });
  const finished=vi.fn();
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{},onFinished:finished}));
  await act(async()=>{await result.current.execute('plan-token',true);});
  await act(async()=>{await vi.advanceTimersByTimeAsync(250);});
  expect(result.current.state).toMatchObject({state:'FAULT',busy:false,reason:'J2 实测速度超过试验边界'});
  expect(result.current.error).toBe('');
  expect(finished).toHaveBeenCalledOnce();
  expect(actions().some(body=>body.action==='stop')).toBe(false);
});

it('a live task still stops when its heartbeat fails',async()=>{
  vi.useFakeTimers();
  const base=fetch.getMockImplementation();
  fetch.mockImplementation(async(path,options)=>{
    if(options?.body&&JSON.parse(options.body).action==='heartbeat') throw new Error('heartbeat connection lost');
    return base(path,options);
  });
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{}}));
  await act(async()=>{await result.current.execute('plan-token',true);});
  await act(async()=>{await vi.advanceTimersByTimeAsync(250);});
  expect(actions().some(body=>body.action==='stop')).toBe(true);
  expect(result.current.error).toBe('heartbeat connection lost');
});

it('a late rejected heartbeat from a completed task cannot stop a new task',async()=>{
  vi.useFakeTimers();
  const base=fetch.getMockImplementation();let rejectHeartbeat;
  fetch.mockImplementation(async(path,options)=>{
    if(options?.body&&JSON.parse(options.body).action==='heartbeat') return new Promise((_,reject)=>{rejectHeartbeat=reject;});
    return base(path,options);
  });
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{}}));
  await act(async()=>{await result.current.execute('first-plan',true);});
  await act(async()=>{await vi.advanceTimersByTimeAsync(250);});
  expect(rejectHeartbeat).toBeTypeOf('function');
  server={state:'FAULT',busy:false,reason:'old task fault',feedback:{}};
  await act(async()=>{await vi.advanceTimersByTimeAsync(250);});
  await act(async()=>{await result.current.execute('new-plan',true);rejectHeartbeat(new Error('old heartbeat rejected'));});
  expect(result.current.state).toMatchObject({state:'RUNNING',busy:true});
  expect(result.current.error).toBe('');
  expect(actions().some(body=>body.action==='stop')).toBe(false);
});
