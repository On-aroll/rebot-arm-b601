// @vitest-environment jsdom
import React from 'react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { cleanup, fireEvent, render, screen } from '@testing-library/react';
import { HardwareMappingPanel } from './HardwareMappingPanel';
import { defaultMapping, mapHardwareFeedback, MAPPING_KEY } from '../lib/hardwareMapping';

afterEach(() => { cleanup(); localStorage.clear(); });
function setup() {
  const profile=defaultMapping(); profile.axes[0].confirmed=true;
  const motors=Object.fromEntries(Array.from({length:6},(_, i)=>[i+1,{mechPosRad:0,positionUpdatedAt:1000}]));
  const mapped=mapHardwareFeedback(motors,profile,Array.from({length:6},()=>[-90,90]),1500,true);
  const onChange=vi.fn(); const onSource=vi.fn();
  render(<HardwareMappingPanel profile={profile} onChange={onChange} mapped={mapped} source="simulation" onSource={onSource} />);
  return {profile,onChange,onSource};
}
describe('hardware mapping panel',()=>{
  it('switches only the display source and invalidates confirmation on calibration edits',()=>{
    const {onChange,onSource}=setup();
    fireEvent.click(screen.getByRole('button',{name:'显示实机姿态（只读）'}));
    expect(onSource).toHaveBeenCalledWith('hardware');
    fireEvent.change(screen.getByLabelText('J1 方向'),{target:{value:'-1'}});
    expect(onChange.mock.calls[0][0].axes[0]).toMatchObject({direction:-1,confirmed:false});
    expect(screen.queryByRole('button',{name:/执行实机/})).toBeNull();
  });
  it('saves only mapping data in this browser without motor or network calls',()=>{
    const {profile}=setup();
    fireEvent.click(screen.getByRole('button',{name:'保存映射到本机'}));
    expect(JSON.parse(localStorage.getItem(MAPPING_KEY))).toEqual(profile);
    expect(screen.getByRole('status').textContent).toContain('不会因此开放实机运动');
  });
});
