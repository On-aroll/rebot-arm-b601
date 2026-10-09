// @vitest-environment jsdom
import React from 'react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { CartesianPointPanel } from './CartesianPointPanel';

afterEach(() => { cleanup(); vi.unstubAllGlobals(); });
const planned = { ok: true, plan_id: 'point-token', expires_in_s: 60, xyz_mm: [200,150,200], q_deg: [1,2,3,4,5,6], start_deg: [0,0,0,0,0,0], path_xyz: [], seconds: 10, position_error_mm: .001 };
function setup() {
  const fetch = vi.fn(async (url, options) => ({ ok: true, json: async () => !options ? { hardware: false, hardware_reason: '零位未验收' }
    : JSON.parse(options.body).action === 'point_plan' ? planned : { ok: true, job: {id:'job-1'} } }));
  vi.stubGlobal('fetch', fetch);
  const props = { state: {q_deg: [0,0,0,0,0,0], xyz: [.3,0,.4]}, locked:false, active:false, moving:false, now:Date.now(), onPreview:vi.fn(), onBusy:vi.fn(), onExecuted:vi.fn() };
  const rendered = render(<CartesianPointPanel {...props} />);
  return { fetch, props, ...rendered };
}

describe('Cartesian point controls', () => {
  it('previews millimetres with free orientation, then executes only the reviewed token', async () => {
    const { fetch } = setup();
    expect(screen.getByRole('button',{name:'执行仿真到点'}).disabled).toBe(true);
    expect(screen.getByRole('button',{name:/确认实机执行/}).disabled).toBe(true);
    fireEvent.click(screen.getByRole('button',{name:'计算并预览'}));
    await screen.findByText(/预览通过/);
    const preview = JSON.parse(fetch.mock.calls.find(([,o])=>o?.body)[1].body);
    expect(preview).toMatchObject({action:'point_plan',xyz_mm:[200,150,200],orientation_mode:'position',target:'simulation'});
    fireEvent.click(screen.getByRole('button',{name:'执行仿真到点'}));
    await waitFor(()=>expect(fetch.mock.calls.some(([,o])=>o?.body && JSON.parse(o.body).action==='point_execute')).toBe(true));
    const execution = JSON.parse(fetch.mock.calls.at(-1)[1].body);
    expect(execution).toEqual({action:'point_execute',target:'simulation',plan_id:'point-token'});
  });
  it('invalidates preview after coordinate editing and blocks incomplete inputs', async () => {
    const { fetch } = setup();
    fireEvent.click(screen.getByRole('button',{name:'计算并预览'}));
    await screen.findByText(/预览通过/);
    fireEvent.change(screen.getByLabelText('X（mm）'),{target:{value:''}});
    expect(screen.getByRole('button',{name:'执行仿真到点'}).disabled).toBe(true);
    fireEvent.click(screen.getByRole('button',{name:'计算并预览'}));
    await screen.findByText(/请完整填写/);
    expect(fetch.mock.calls.filter(([,o])=>o?.body)).toHaveLength(1);
  });
  it('locks reviewed results when the simulation start changes or preview expires', async () => {
    const { props, rerender } = setup();
    fireEvent.click(screen.getByRole('button',{name:'计算并预览'}));
    await screen.findByText(/预览通过/);
    rerender(<CartesianPointPanel {...props} state={{...props.state,q_deg:[2,0,0,0,0,0]}} />);
    expect(screen.getByRole('button',{name:'执行仿真到点'}).disabled).toBe(true);
    rerender(<CartesianPointPanel {...props} now={Date.now()+61000} />);
    expect(screen.getByRole('button',{name:'执行仿真到点'}).disabled).toBe(true);
    await screen.findByText(/预览已超过/);
  });
});
