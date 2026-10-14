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
});
