// @vitest-environment jsdom
import React from 'react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { CartesianPointPanel } from './CartesianPointPanel';

afterEach(() => { cleanup(); vi.unstubAllGlobals(); });
const planned = { ok: true, plan_id: 'point-token', expires_in_s: 60, xyz_mm: [200,150,200], q_deg: [1,2,3,4,5,6], start_deg: [0,0,0,0,0,0], path_xyz: [], seconds: 10, position_error_mm: .001 };
function setup(overrides = {}) {
  const fetch = vi.fn(async (url, options) => ({ ok: true, json: async () => !options ? { hardware: false, hardware_reason: '真实轨迹待验收' }
    : ['preview', 'point_plan'].includes(JSON.parse(options.body).action) ? planned : { ok: true, job: {status:'starting'} } }));
  vi.stubGlobal('fetch', fetch);
  const props = { mode: 'offline', feedback: {q_deg: [0,0,0,0,0,0], job: {status:'idle'}}, currentXyz: [300,0,400], previewReady:true, virtualEnabled:true,
    locked:false, active:false, moving:false, now:Date.now(), onPreview:vi.fn(), onBusy:vi.fn(), onExecuted:vi.fn(), onStop:vi.fn(), onPrepareHardwarePreview:vi.fn(), ...overrides };
  const rendered = render(<CartesianPointPanel {...props} />);
  return { fetch, props, ...rendered };
}
const posts = fetch => fetch.mock.calls.filter(([,o]) => o?.body).map(([url,o]) => ({url,body:JSON.parse(o.body)}));

describe('Two-mode XYZ workflow', () => {
  it('keeps the last CSP fault and pre-release evidence visible after reload without claiming XYZ arrival', () => {
    const rad = value => value * Math.PI / 180;
    setup({mode:'hardware',hardwareState:{control:{mode:'native-csp-j3-v1'},busy:false,motion_arrived_before_stop:false,
      reason:'CSP 首次使能起 5 秒时限已到，撤力',expected_start:[0,0,rad(.313)],target:[0,0,rad(.813)],
      failure_feedback:{3:{q:rad(.454)}},feedback:{3:{q:rad(.317)}},
      stop:{disabled_status:{mode_state:0,fault_bits:0}},restoration:{ok:true,restored:['0x7005','0x7017','0x7018','0x700b']}}});
    const record=screen.getByRole('group',{name:'上次 J3 CSP 试验记录'});
    expect(record.textContent).toContain('未到位');
    expect(record.textContent).toContain('撤力前读数 0.454°');
    expect(record.textContent).toContain('撤力后读数 0.317°');
    expect(record.textContent).toContain('剩余误差 0.359°');
    expect(record.textContent).toContain('四项临时运行参数已恢复并读回核对');
    expect(screen.queryByText(/实机反馈已到位，任务已结束/)).toBeNull();
  });
  it('does not infer stopped or restored status from missing CSP cleanup evidence', () => {
    setup({mode:'hardware',hardwareState:{control:{mode:'native-csp-j3-v1'},busy:false}});
    const record=screen.getByRole('group',{name:'上次 J3 CSP 试验记录'});
    expect(record.textContent).toContain('失能状态未完全确认');
    expect(record.textContent).toContain('运行参数恢复未完全确认');
    expect(record.textContent).not.toContain('NaN');
  });
  it('offers the fixed CSP trial with shared confirmation without requiring a MIT XYZ plan', async () => {
    const native = vi.fn(async () => ({status:{state:'PREPARING',busy:true}}));
    const {props,rerender,fetch}=setup({mode:'hardware',onExecuteCspJ3:native});
    fireEvent.click(screen.getByText('原生位置闭环 · J3 小幅试验'));
    expect(screen.getByRole('button',{name:'执行 J3 CSP +0.5°'}).disabled).toBe(true);
    rerender(<CartesianPointPanel {...props} hardwareConfirmed />);
    fireEvent.click(screen.getByRole('button',{name:'执行 J3 CSP +0.5°'}));
    await waitFor(()=>expect(native).toHaveBeenCalledOnce());
    expect(posts(fetch)).toEqual([]);
    expect(props.onExecuted).toHaveBeenCalledOnce();
    rerender(<CartesianPointPanel {...props} hardwareConfirmed feedback={{job:{status:'succeeded'}}} />);
    await screen.findByText(/J3 CSP 反馈已到位并稳定/);
    expect(screen.queryByText(/当前末端误差/)).toBeNull();
  });
  it('previews from virtual feedback and submits only the reviewed token to ROS', async () => {
    const { fetch } = setup();
    expect(screen.getByRole('button',{name:'执行虚拟动作'}).disabled).toBe(true);
    expect(screen.queryByRole('button',{name:/确认实机执行/})).toBeNull();
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    expect(posts(fetch)[0]).toMatchObject({url:'/ros-control/action',body:{action:'preview',xyz_mm:[200,150,200],orientation_mode:'position'}});
    fireEvent.click(screen.getByRole('button',{name:'执行虚拟动作'}));
    await waitFor(() => expect(posts(fetch)).toHaveLength(2));
    expect(posts(fetch)[1]).toEqual({url:'/ros-control/action',body:{action:'execute',plan_id:'point-token'}});
    expect(posts(fetch).some(p => p.body.action === 'sync' || p.body.action === 'enable' || p.url === '/command')).toBe(false);
  });
  it('allows preview before virtual enable, but never enables implicitly', async () => {
    const { fetch } = setup({virtualEnabled:false});
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    expect(screen.getByRole('button',{name:'执行虚拟动作'}).disabled).toBe(true);
    expect(posts(fetch)).toHaveLength(1);
  });
  it('invalidates edited coordinates and rejects incomplete input', async () => {
    const { fetch } = setup();
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    fireEvent.change(screen.getByLabelText('X（mm）'),{target:{value:''}});
    expect(screen.getByRole('button',{name:'执行虚拟动作'}).disabled).toBe(true);
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/请完整填写/);
    expect(posts(fetch)).toHaveLength(1);
  });
  it('checks current execution feedback, expiry and freshness before execute', async () => {
    const { props, rerender } = setup();
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    rerender(<CartesianPointPanel {...props} feedback={{q_deg:[2,0,0,0,0,0]}} />);
    expect(screen.getByRole('button',{name:'执行虚拟动作'}).disabled).toBe(true);
    rerender(<CartesianPointPanel {...props} now={Date.now()+61000} />);
    await screen.findByText(/预演已超过/);
    rerender(<CartesianPointPanel {...props} previewReady={false} />);
    expect(screen.getByRole('button',{name:'执行虚拟动作'}).disabled).toBe(true);
  });
  it('hardware preview first reads the actual pose, and never falls back to virtual execution', async () => {
    const { fetch, props, rerender } = setup({mode:'hardware'});
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    expect(props.onPrepareHardwarePreview).toHaveBeenCalledOnce();
    expect(props.onPrepareHardwarePreview.mock.invocationCallOrder[0]).toBeLessThan(fetch.mock.invocationCallOrder.at(-1));
    expect(posts(fetch)[0]).toMatchObject({url:'/command',body:{action:'point_plan',execution_source:'hardware-preview',target:'simulation'}});
    expect(screen.getByRole('button',{name:/确认实机执行/}).disabled).toBe(true);
    expect(screen.queryByRole('button',{name:'执行虚拟动作'})).toBeNull();
    rerender(<CartesianPointPanel {...props} mode="offline" />);
    await waitFor(() => expect(screen.getByRole('button',{name:'执行虚拟动作'}).disabled).toBe(true));
  });
  it('keeps stop available while trajectory or other controls are busy', () => {
    const { props } = setup({locked:true, feedback:{q_deg:[0,0,0,0,0,0],job:{status:'running'}}});
    const stop = screen.getByRole('button',{name:'停止当前任务'});
    expect(stop.disabled).toBe(false);
    fireEvent.click(stop);
    expect(props.onStop).toHaveBeenCalledOnce();
  });
  it('unlocks a reviewed hardware plan with the shared confirmation and submits only its token', async () => {
    const execute = vi.fn(async () => ({status:{state:'PREPARING',busy:true}}));
    const {fetch,props,rerender}=setup({mode:'hardware',onExecuteHardware:execute,hardwareConfirmed:false});
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    expect(screen.getByRole('button',{name:'确认实机执行'}).disabled).toBe(true);
    rerender(<CartesianPointPanel {...props} hardwareConfirmed />);
    expect(screen.getByRole('button',{name:'确认实机执行'}).disabled).toBe(false);
    fireEvent.click(screen.getByRole('button',{name:'确认实机执行'}));
    await waitFor(()=>expect(execute).toHaveBeenCalledExactlyOnceWith('point-token'));
    expect(posts(fetch).some(p=>p.url==='/ros-control/action')).toBe(false);
    rerender(<CartesianPointPanel {...props} hardwareConfirmed locked feedback={{q_deg:[0,0,0,0,0,0],job:{status:'running'}}} />);
    expect(screen.getByRole('button',{name:'立即停止实机任务'}).disabled).toBe(false);
  });
  it('shows an execution rejection once when the hook and panel report the same reason', async () => {
    const message='轨迹至少需 77 秒；本次试验时限只有 5 秒。';
    const execute=vi.fn(async()=>{throw new Error(message);});
    const {props,rerender}=setup({mode:'hardware',hardwareConfirmed:true,onExecuteHardware:execute});
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    fireEvent.click(screen.getByRole('button',{name:'确认实机执行'}));
    await screen.findByText(message);
    rerender(<CartesianPointPanel {...props} hardwareError={message} />);
    expect(screen.getAllByText(message)).toHaveLength(1);
  });
  it('uses visible hardware speed and automatic duration in preview, and edits invalidate the task',async()=>{
    const {fetch}=setup({mode:'hardware'});
    expect(screen.getByLabelText('实机关节速度上限（rad/s）').value).toBe('0.02');
    expect(screen.getByLabelText('实机任务时限').value).toBe('auto');
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    expect(posts(fetch)[0].body).toMatchObject({speed_rad_s:.02,hardware_motion:{speed_rad_s:.02,timeout_mode:'auto'}});
    fireEvent.change(screen.getByLabelText('实机关节速度上限（rad/s）'),{target:{value:'0.05'}});
    expect(screen.getByRole('button',{name:'确认实机执行'}).disabled).toBe(true);
    fireEvent.change(screen.getByLabelText('实机任务时限'),{target:{value:'manual'}});
    fireEvent.change(screen.getByLabelText('任务时限（秒）'),{target:{value:'90'}});
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));
    await screen.findByText(/预演通过/);
    expect(posts(fetch)[1].body).toMatchObject({speed_rad_s:.05,hardware_motion:{speed_rad_s:.05,timeout_mode:'manual',timeout_s:90}});
    expect(posts(fetch)).toHaveLength(2);
  });
  it('selects official gravity per task, invalidates preview, and sends no enable or parameter writes',async()=>{
    const {fetch}=setup({mode:'hardware'});
    expect(screen.getByLabelText('轨迹控制方案').value).toBe('none');
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));await screen.findByText(/预演通过/);
    fireEvent.change(screen.getByLabelText('轨迹控制方案'),{target:{value:'seeed-rs-gravity-v1'}});
    expect(screen.getByRole('button',{name:'确认实机执行'}).disabled).toBe(true);
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));await screen.findByText(/预演通过/);
    expect(posts(fetch)[1].body.hardware_motion.feedforward_mode).toBe('seeed-rs-gravity-v1');
    expect(posts(fetch).map(p=>p.body.action)).toEqual(['point_plan','point_plan']);
  });
  it('selects reviewed official point gains without applying device settings or executing',async()=>{
    const {fetch}=setup({mode:'hardware'});
    fireEvent.change(screen.getByLabelText('轨迹控制方案'),{target:{value:'seeed-rs-slow-position-v1'}});
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));await screen.findByText(/预演通过/);
    expect(posts(fetch)[0].body.hardware_motion.feedforward_mode).toBe('seeed-rs-slow-position-v1');
    expect(posts(fetch).map(p=>p.body.action)).toEqual(['point_plan']);
    fireEvent.change(screen.getByLabelText('轨迹控制方案'),{target:{value:'none'}});
    expect(screen.getByRole('button',{name:'确认实机执行'}).disabled).toBe(true);
  });
  it('submits startup observation separately from XYZ and requires the same fresh plan and confirmation',async()=>{
    const observe=vi.fn(async()=>({status:{state:'PREPARING',busy:true}}));
    const execute=vi.fn();
    const {props,rerender}=setup({mode:'hardware',hardwareConfirmed:true,onExecuteHardware:execute,onObserveStartup:observe});
    expect(screen.queryByRole('button',{name:'启动采集 · 3 秒自动撤力'})).toBeNull();
    fireEvent.change(screen.getByLabelText('轨迹控制方案'),{target:{value:'seeed-rs-slow-position-v1'}});
    expect(screen.getByRole('button',{name:'启动采集 · 3 秒自动撤力',hidden:true}).disabled).toBe(true);
    fireEvent.click(screen.getByRole('button',{name:'计算并预演'}));await screen.findByText(/预演通过/);
    fireEvent.click(screen.getByText('启动诊断（按需）'));
    fireEvent.click(screen.getByRole('button',{name:'启动采集 · 3 秒自动撤力'}));
    await waitFor(()=>expect(observe).toHaveBeenCalledExactlyOnceWith('point-token'));
    expect(execute).not.toHaveBeenCalled();
    rerender(<CartesianPointPanel {...props} feedback={{q_deg:[0,0,0,0,0,0],job:{status:'cancelled',error:'3 秒启动采集结束，自动撤力；未执行 XYZ 轨迹'}}} />);
    expect(screen.getByRole('status').textContent).toContain('3 秒启动采集结束，自动撤力；未执行 XYZ 轨迹');
  });
});
