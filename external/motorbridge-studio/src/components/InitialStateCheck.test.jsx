// @vitest-environment jsdom
import React from 'react';
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { afterEach, beforeEach, expect, it, vi } from 'vitest';
import { InitialStateCheck } from './InitialStateCheck';
import { defaultMapping } from '../lib/hardwareMapping';

let time;
const RAD = Math.PI / 180;
const limits = Array(6).fill([-180,180]);
function readings(deg=0) {
  return Object.fromEntries(Array.from({length:7},(_,i)=>[i+1,{
    mechPosRad:(i===0?deg:0)*RAD,positionUpdatedAt:time,fieldTimes:{runMode:time},runMode:0,error:''
  }]));
}
function props() {
  const motors = readings();
  return {gateway:{status:'online',endpoint:'ws://mock.test',motors,refresh:vi.fn(async()=>motors),
    requestPower:vi.fn(),requestAllPower:vi.fn(),connect:vi.fn(),powerRequests:{}},mapping:defaultMapping(),limits,now:time,onBusyChange:vi.fn()};
}
async function detect() {
  fireEvent.click(screen.getByRole('button',{name:'一键检测当前状态'}));
  await screen.findByText('位置反馈：7/7');
}
function begin() { fireEvent.click(screen.getByRole('button',{name:'已确认撤力与支撑，开始记录'})); }
beforeEach(()=>{time=10000;vi.spyOn(Date,'now').mockImplementation(()=>time);});
afterEach(()=>{cleanup();vi.restoreAllMocks();vi.unstubAllGlobals();});

it('reads seven axes only on click without enabling, moving, connecting or editing calibration', async () => {
  const p = props(), before = JSON.stringify(p.mapping);
  render(<InitialStateCheck {...p} />);
  expect(p.gateway.refresh).not.toHaveBeenCalled();
  await detect();
  expect(p.gateway.refresh).toHaveBeenCalledExactlyOnceWith({waitForIdle:true});
  expect(p.gateway.requestPower).not.toHaveBeenCalled();
  expect(p.gateway.requestAllPower).not.toHaveBeenCalled();
  expect(p.gateway.connect).not.toHaveBeenCalled();
  expect(JSON.stringify(p.mapping)).toBe(before);
  expect(screen.getByText('实机轨迹：另见任务记录')).toBeTruthy();
  expect(screen.getByRole('button',{name:'已确认撤力与支撑，开始记录'}).disabled).toBe(false);
});

it('starts manual observation at current readings and confirms only axes which actually changed', async () => {
  const p = props(), {rerender}=render(<InitialStateCheck {...p} />);
  await detect();
  time+=100; p.now=time; p.gateway={...p.gateway,motors:readings(10)};
  rerender(<InitialStateCheck {...p} />); begin();
  expect(screen.getAllByText(/未观察到明显变化/)).toHaveLength(7);
  expect(p.onBusyChange).toHaveBeenLastCalledWith(true);
  time+=100; p.now=time; p.gateway={...p.gateway,motors:readings(11)};
  rerender(<InitialStateCheck {...p} />);
  expect(screen.getByText('Δ 1.00° · 跨度 1.00°')).toBeTruthy();
  expect(screen.getAllByText(/未观察到明显变化/)).toHaveLength(6);
  fireEvent.click(screen.getByRole('button',{name:'结束手动记录'}));
  expect(screen.getAllByRole('checkbox')).toHaveLength(1);
  fireEvent.click(screen.getByRole('checkbox'));
  expect(screen.getByRole('checkbox').checked).toBe(true);
  expect(p.onBusyChange).toHaveBeenLastCalledWith(false);
  expect(p.gateway.requestPower).not.toHaveBeenCalled();
});

it('does not turn a static pose or small noise into a reviewed axis', async () => {
  const p=props(), {rerender}=render(<InitialStateCheck {...p} />);
  await detect(); begin();
  time+=100; p.now=time; p.gateway={...p.gateway,motors:readings(.1)};
  rerender(<InitialStateCheck {...p} />);
  fireEvent.click(screen.getByRole('button',{name:'结束手动记录'}));
  expect(screen.getByRole('checkbox').disabled).toBe(true);
});

it('permits read-only detection with an enable record but requires release before manual observation', async () => {
  const p=props(); p.gateway.powerRequests={1:{op:'enable'}};
  const {rerender}=render(<InitialStateCheck {...p} />); await detect();
  expect(screen.getByRole('button',{name:'已确认撤力与支撑，开始记录'}).disabled).toBe(true);
  p.gateway={...p.gateway,powerRequests:{1:{op:'disable'}}}; rerender(<InitialStateCheck {...p} />);
  expect(screen.getByRole('button',{name:'已确认撤力与支撑，开始记录'}).disabled).toBe(false);
  expect(screen.queryByRole('button',{name:'结束手动记录'})).toBeNull();
});

it('interrupts observation on stale feedback and revokes completed confirmation on subsequent power changes', async () => {
  const p=props(), {rerender}=render(<InitialStateCheck {...p} />);
  await detect(); begin();
  time+=100; p.now=time; p.gateway={...p.gateway,motors:readings(1)}; rerender(<InitialStateCheck {...p} />);
  fireEvent.click(screen.getByRole('button',{name:'结束手动记录'}));
  fireEvent.click(screen.getByRole('checkbox'));
  p.gateway={...p.gateway,powerRequests:{2:{op:'disable'}}}; rerender(<InitialStateCheck {...p} />);
  expect(screen.getByRole('checkbox').checked).toBe(false);
  begin();
  time+=3000; p.now=time; rerender(<InitialStateCheck {...p} />);
  expect(screen.queryByRole('button',{name:'结束手动记录'})).toBeNull();
  expect(screen.getByRole('button',{name:'已确认撤力与支撑，开始记录'}).disabled).toBe(true);
  expect(p.onBusyChange).toHaveBeenLastCalledWith(false);
});

it.each(['disconnect','mapping','power'])('ignores a late detection reply after %s changes', async cause => {
  const p=props(); let resolve;
  p.gateway.refresh.mockImplementation(()=>new Promise(r=>{resolve=r;}));
  const {rerender}=render(<InitialStateCheck {...p} />);
  fireEvent.click(screen.getByRole('button',{name:'一键检测当前状态'}));
  if(cause==='disconnect') p.gateway={...p.gateway,status:'offline'};
  if(cause==='mapping') p.mapping={...p.mapping,axes:p.mapping.axes.map((a,i)=>i===0?{...a,offsetDeg:1}:a)};
  if(cause==='power') p.gateway={...p.gateway,powerPending:1};
  rerender(<InitialStateCheck {...p} />);
  await act(async()=>{resolve(readings());});
  expect(screen.queryByText('位置反馈：7/7')).toBeNull();
  expect(p.onBusyChange).toHaveBeenLastCalledWith(false);
});

it('cannot start while offline or during another task and reports an incomplete query without retry', async () => {
  const p=props(), {rerender}=render(<InitialStateCheck {...p} blocked />);
  expect(screen.getByRole('button',{name:'一键检测当前状态'}).disabled).toBe(true);
  rerender(<InitialStateCheck {...p} gateway={{...p.gateway,status:'offline'}} />);
  expect(screen.getByRole('button',{name:'一键检测当前状态'}).disabled).toBe(true);
  p.gateway.refresh.mockResolvedValue(undefined); rerender(<InitialStateCheck {...p} />);
  fireEvent.click(screen.getByRole('button',{name:'一键检测当前状态'}));
  await waitFor(()=>expect(screen.getByText(/未取得完整查询/)).toBeTruthy());
  expect(p.gateway.refresh).toHaveBeenCalledOnce();
});

it('exports sources and only observed confirmations without granting hardware execution or modifying zero', async () => {
  const p=props(); let exported;
  vi.stubGlobal('URL', class extends URL {
    static createObjectURL = vi.fn(blob => { exported=blob; return 'blob:mock'; });
    static revokeObjectURL = vi.fn();
  });
  vi.spyOn(HTMLAnchorElement.prototype,'click').mockImplementation(()=>{});
  const {rerender}=render(<InitialStateCheck {...p} />);
  await detect(); begin();
  time+=100; p.now=time; p.gateway={...p.gateway,motors:readings(1)};
  rerender(<InitialStateCheck {...p} />);
  fireEvent.click(screen.getByRole('button',{name:'结束手动记录'}));
  fireEvent.click(screen.getByRole('checkbox'));
  fireEvent.click(screen.getByRole('button',{name:'导出检测记录'}));
  const record=JSON.parse(await new Promise((resolve,reject)=>{
    const reader=new FileReader(); reader.onload=()=>resolve(reader.result);reader.onerror=reject;reader.readAsText(exported);
  }));
  expect(record).toMatchObject({reviewed:true,reviewedAt:time,changedAxes:[1],hardwareExecutionAllowed:false,hardwareParametersWritten:false});
  expect(record.report.source).toBe('explicit-gateway-read');
  expect(record.mappingAtCapture.axes.every(a=>a.offsetDeg===0 && a.direction===1 && !a.confirmed)).toBe(true);
  expect(record.observations[1].samples).toBe(2);
  expect(record.observations[1].spanDeg).toBe(0);
});
