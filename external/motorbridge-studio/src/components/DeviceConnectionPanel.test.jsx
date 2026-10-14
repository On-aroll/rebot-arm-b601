// @vitest-environment jsdom
import React from 'react';
import {afterEach,it,expect,vi} from 'vitest';
import {render,screen,fireEvent,cleanup} from '@testing-library/react';
import {DeviceConnectionPanel} from './DeviceConnectionPanel';
afterEach(cleanup);
const gateway=()=>({status:'disconnected',motors:{},endpoint:'ws://127.0.0.1:9002',connect:vi.fn(),disconnect:vi.fn(),refresh:vi.fn(),setEndpoint:vi.fn(),write:vi.fn()});
it('connects only on explicit click and never enables hardware',()=>{
 const g=gateway(); const sync=vi.fn(); render(<DeviceConnectionPanel gateway={g} now={1000} onSync={sync}/>);
 expect(g.connect).not.toHaveBeenCalled();fireEvent.click(screen.getByRole('button',{name:'连接实机'}));expect(g.connect).toHaveBeenCalledOnce();
 expect(screen.getByRole('button',{name:'手动更新 MuJoCo 起点（高级）'}).disabled).toBe(true);expect(g.write).not.toHaveBeenCalled();
});
it('does not present fresh positions as fresh modes',()=>{
 const g=gateway();g.status='online';g.motors=Object.fromEntries([1,2,3,4,5,6,7].map(i=>[i,{mechPosRad:0,positionUpdatedAt:1000,runMode:0,fieldTimes:{runMode:1}}]));
 render(<DeviceConnectionPanel gateway={g} now={4000} />);
 expect(screen.getByText('0 / 7')).toBeTruthy();expect(screen.getByText('J1 · 模式待刷新')).toBeTruthy();
});
it('shows only measured fresh count and invokes simulation-only sync',()=>{
 const g=gateway();g.status='online';g.motors=Object.fromEntries([1,2,3,4,5,6,7].map(i=>[i,{mechPosRad:0,positionUpdatedAt:1000,runMode:0,fieldTimes:{runMode:1000}}]));const sync=vi.fn();
 render(<DeviceConnectionPanel gateway={g} now={1200} onSync={sync}/>);expect(screen.getByText('7 / 7')).toBeTruthy();
 fireEvent.click(screen.getByRole('button',{name:'手动更新 MuJoCo 起点（高级）'}));expect(sync).toHaveBeenCalledOnce();expect(g.write).not.toHaveBeenCalled();
});
