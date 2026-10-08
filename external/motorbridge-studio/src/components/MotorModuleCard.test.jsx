// @vitest-environment jsdom
import React from 'react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { MotorModuleCard } from './MotorModuleCard';

afterEach(cleanup);
function setup(armed = false) {
  const gateway = { armed, drill: true, maxSpd: 0.3, refresh: vi.fn(), write: vi.fn().mockResolvedValue({ok:true,dryRun:true}) };
  render(<MotorModuleCard id={1} label="J1" status="online" plannedDeg={25}
    motor={{updatedAt:Date.now(),mechPosWrapped:12,runMode:5,limitSpd:0.2,locKp:13,limitCur:57,model:'rs-06'}} gateway={gateway} />);
  return gateway;
}

describe('motor configuration module', () => {
  it('lets users prepare settings while writes are locked and removes manual positions', () => {
    const gateway = setup();
    const kp = screen.getByLabelText('位置环刚度 Kp');
    expect(kp.disabled).toBe(false);
    fireEvent.change(kp,{target:{value:'12'}});
    expect(screen.queryByLabelText('目标位置')).toBeNull();
    expect(screen.queryByRole('slider')).toBeNull();
    expect(screen.getByRole('button',{name:'使能（出力）'}).disabled).toBe(true);
    screen.getAllByRole('button',{name:'应用',exact:true}).forEach(button => expect(button.disabled).toBe(true));
    expect(gateway.write).not.toHaveBeenCalled();
  });
  it('writes a stiffness register without any position target', async () => {
    const gateway = setup(true);
    fireEvent.change(screen.getByLabelText('位置环刚度 Kp'),{target:{value:'12'}});
    fireEvent.click(screen.getAllByRole('button',{name:'应用',exact:true})[1]);
    await waitFor(()=>expect(gateway.write).toHaveBeenCalledWith(1,'robstride_write_param',{param_id:0x701e,type:'f32',value:12,timeout_ms:800}));
    await screen.findByText('演练校验通过，未向电机写入');
    expect(gateway.refresh).not.toHaveBeenCalled();
  });
  it('does not show a dry-run enable as physical enabled state', async () => {
    const gateway = setup(true);
    fireEvent.click(screen.getByRole('button',{name:'使能（出力）'}));
    await screen.findByText('演练校验通过，未向电机写入');
    expect(screen.getByText('使能状态：未知')).toBeTruthy();
    expect(screen.queryByText('使能指令已发送')).toBeNull();
    expect(gateway.write).toHaveBeenCalledWith(1,'enable',{});
  });
});
