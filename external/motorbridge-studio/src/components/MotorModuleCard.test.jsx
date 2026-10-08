// @vitest-environment jsdom
import React from 'react';
import { afterEach, expect, it, vi } from 'vitest';
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { MotorModuleCard } from './MotorModuleCard';
afterEach(cleanup);
const config = () => ({ state:{revision:0,drafts:{},approved:{},events:[]},fields:{kp:{unit:'N*m/rad',label:'MIT 刚度',group:'mit'}},action:vi.fn().mockResolvedValue({state:{}}) });
it('has seven compact modules without editable defaults or a mode switch', () => {
  const write=vi.fn(), onToggle=vi.fn();
  render(<>{[1,2,3,4,5,6,7].map(id => <MotorModuleCard key={id} id={id} label={`J${id}`} config={config()} gateway={{write,armed:true}} onToggle={onToggle} />)}</>);
  expect(screen.getAllByRole('region')).toHaveLength(7);
  expect(screen.queryByRole('spinbutton')).toBeNull();
  expect(screen.queryByRole('combobox')).toBeNull();
  fireEvent.click(screen.getAllByRole('button')[0]); expect(onToggle).toHaveBeenCalledOnce(); expect(write).not.toHaveBeenCalled();
});
it('saves a separate software draft and never calls gateway write', async () => {
  const c=config(); const write=vi.fn();
  render(<MotorModuleCard id={1} label="J1" expanded config={c} gateway={{write,armed:true}} motor={{runMode:0,mechPosDeg:721,positionUpdatedAt:1}} />);
  expect(screen.getByRole('button',{name:'应用到真实电机（待验收）'}).disabled).toBe(true);
  expect(screen.getByText('721.00°')).toBeTruthy(); expect(screen.getByText('反馈过期')).toBeTruthy();
  fireEvent.change(screen.getByLabelText('MIT 刚度'),{target:{value:'12'}});
  fireEvent.click(screen.getByRole('button',{name:'保存草稿'}));
  await waitFor(()=>expect(c.action).toHaveBeenCalledWith({action:'draft',axis_id:1,fields:{kp:{unit:'N*m/rad',value:12}}}));
  expect(write).not.toHaveBeenCalled();
});
it('requires an explicit per-change confirmation', async () => {
  const c=config(); c.state.drafts[1]={fields:{kp:{value:12,unit:'N*m/rad'}}};
  c.action.mockResolvedValueOnce({review_token:'one-use',warning:'只批准软件配置',changes:[{field:'kp',before:{value:null},after:{value:12,unit:'N*m/rad'}}]}).mockResolvedValue({state:{}});
  render(<MotorModuleCard id={1} label="J1" expanded config={c} />);
  fireEvent.click(screen.getByRole('button',{name:'审核变更'}));
  const approve=await screen.findByRole('button',{name:'确认本次软件批准'});
  expect(approve.disabled).toBe(true);
  fireEvent.click(screen.getByRole('checkbox')); fireEvent.click(approve);
  await waitFor(()=>expect(c.action).toHaveBeenLastCalledWith({action:'approve',review_token:'one-use',confirm:'software-only'}));
});
it('renders an unavailable configuration service without an update loop', () => {
  const spy=vi.spyOn(console,'error').mockImplementation(()=>{});
  render(<MotorModuleCard id={1} label="J1" expanded />);
  expect(screen.getByRole('button',{name:'保存草稿'}).disabled).toBe(true);
  expect(spy).not.toHaveBeenCalled(); spy.mockRestore();
});
