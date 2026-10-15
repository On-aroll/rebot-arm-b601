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

it('renews an idle page session before each explicit task without retrying starts',async()=>{
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{}}));
  await act(async()=>{await result.current.execute('first-plan',true);});
  server={state:'CANCELLED',busy:false,feedback:{}};
  await waitFor(()=>expect(result.current.busy).toBe(false));
  await act(async()=>{await result.current.execute('next-plan',true,true);});
  expect(fetch.mock.calls.filter(([path])=>path.endsWith('/session'))).toHaveLength(2);
  expect(actions().filter(body=>['execute','probe-startup'].includes(body.action))).toEqual([
    expect.objectContaining({action:'execute',plan_id:'first-plan'}),
    expect.objectContaining({action:'probe-startup',plan_id:'next-plan'}),
  ]);
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

it('CSP uses the freshly prepared server token and one confirmed start after bus release',async()=>{
  const base=fetch.getMockImplementation();
  fetch.mockImplementation(async(path,options)=>{
    const body=options?.body ? JSON.parse(options.body) : {};
    if(body.action==='prepare-csp-j3') return {ok:true,json:async()=>({plan_id:'fixed-j3-plan',status:{state:'READY',busy:false}})};
    if(body.action==='csp-j3-trial') server={state:'RUNNING',busy:true,feedback:{}};
    return base(path,options);
  });
  const release=vi.fn(async()=>{});
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:release}));
  await act(async()=>{await expect(result.current.executeCspJ3(false)).rejects.toThrow('现场确认');});
  expect(release).not.toHaveBeenCalled();
  await act(async()=>{await result.current.executeCspJ3(true);});
  expect(actions().filter(b=>b.action!=='heartbeat')).toEqual([
    {action:'prepare-csp-j3'},expect.objectContaining({action:'csp-j3-trial',plan_id:'fixed-j3-plan'}),
  ]);
  expect(actions().at(-1).target).toBeUndefined();
  expect(result.current.busy).toBe(true);
});

it('stop or focus loss during CSP preparation cannot send a late start',async()=>{
  let finish;
  const base=fetch.getMockImplementation();
  fetch.mockImplementation(async(path,options)=>{
    if(options?.body&&JSON.parse(options.body).action==='prepare-csp-j3')
      return new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({plan_id:'late-plan'})});});
    return base(path,options);
  });
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{}}));
  let attempt;
  act(()=>{attempt=result.current.executeCspJ3(true).catch(e=>e);});
  await waitFor(()=>expect(finish).toBeTypeOf('function'));
  await act(async()=>{await result.current.stop();finish();await attempt;});
  expect(actions().some(b=>b.action==='csp-j3-trial')).toBe(false);
});

it('failed CSP preparation preserves its actual reason without starting',async()=>{
  const base=fetch.getMockImplementation();
  fetch.mockImplementation(async(path,options)=>{
    if(options?.body&&JSON.parse(options.body).action==='prepare-csp-j3')
      return {ok:true,json:async()=>({status:{state:'FAULT',busy:false,reason:'J3 起点变化'}})};
    return base(path,options);
  });
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{}}));
  await act(async()=>{await expect(result.current.executeCspJ3(true)).rejects.toThrow('J3 起点变化');});
  expect(actions().some(b=>b.action==='csp-j3-trial')).toBe(false);
});


it('J6 uses its own fixed preparation and start, never the J3 action',async()=>{
  const base=fetch.getMockImplementation();
  fetch.mockImplementation(async(path,options)=>{
    const body=options?.body?JSON.parse(options.body):{};
    if(body.action==='prepare-csp-j6') return {ok:true,json:async()=>({plan_id:'fixed-j6-plan'})};
    if(body.action==='csp-j6-trial') server={state:'RUNNING',busy:true,feedback:{}};
    return base(path,options);
  });
  const {result}=renderHook(()=>useHardwarePoint({beforeExecute:async()=>{}}));
  await act(async()=>{await expect(result.current.executeCspJ6(false)).rejects.toThrow('现场确认');});
  await act(async()=>{await result.current.executeCspJ6(true);});
  expect(actions().filter(b=>b.action!=='heartbeat')).toEqual([
    {action:'prepare-csp-j6'},expect.objectContaining({action:'csp-j6-trial',plan_id:'fixed-j6-plan'}),
  ]);
});
