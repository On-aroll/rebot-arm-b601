// @vitest-environment jsdom
import React from 'react';
import { afterEach, beforeEach, expect, it, vi } from 'vitest';
import { act, cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
import { ConsoleWorkspace } from './ConsoleWorkspace';

const g = vi.hoisted(() => ({ status: 'disconnected', motors: {}, endpoint: 'ws://mock.test',
  connect: vi.fn(), disconnect: vi.fn(), refresh: vi.fn(), requestAllPower: vi.fn(), log: [] }));
const ros = vi.hoisted(() => ({ state: {}, pending: false, error: '', running:false, action:vi.fn(), refresh:vi.fn(), acceptFeedback:vi.fn() }));
vi.mock('../hooks/useMotorGateway', () => ({ useMotorGateway: () => g, START_BOUNDARY_TOLERANCE_DEG:1, MACHINE_IDS: [1,2,3,4,5,6,7], MACHINE_MODELS: {1:'rs-06',2:'rs-06',3:'rs-06',4:'rs-00',5:'rs-00',6:'rs-00',7:'rs-00'} }));
vi.mock('../hooks/useRosValidation', () => ({ useRosValidation: () => ros }));
vi.mock('../hooks/useSoftwareConfig', () => ({ useSoftwareConfig: () => ({ state: { drafts: {}, approved: {} }, fields: {}, action: vi.fn(), provenance: {} }) }));
vi.mock('./ArmUrdfViewer', () => ({ ArmUrdfViewer: props => <div data-testid="model" data-feedback-driven={props.feedbackDriven} data-joints={JSON.stringify(props.jointTargets)} /> }));
vi.mock('./RosControlPanel', () => ({ RosControlPanel: () => <div>ROS 虚拟诊断</div> }));
beforeEach(() => {
  window.history.replaceState({}, '', '/zh/studio/console');
  g.status = 'disconnected';
  g.motors = {}; g.updatedAt = 0; g.powerRequests = {}; g.powerPending = null; g.powerGroup = null;
  ros.running = false;
  ros.state = {backend:'ros2-fake',namespace:'rebotarm_validation',hardware:false,fresh:true,enabled:false,feedback_time:Date.now()/1000,q_rad:[.1,0,0,0,0,0],job:{status:'idle'}};
  vi.stubGlobal('fetch', vi.fn(async path => ({ ok: true, json: async () => path === '/studio-state'
    ? { backend: 'mujoco', feedback_source: 'simulation', feedback_time_unix_s: Date.now()/1000, q_rad: [0,0,0,0,0,0], q_deg:[0,0,0,0,0,0], xyz:[.5,.1,.3], limits_deg:Array(6).fill([-180,180]) }
    : path === '/kinematics/fk' ? {ok:true,xyz_mm:[200,100,300]} : path === '/writing-program' ? { steps: [] } : {} })));
});
afterEach(() => { cleanup(); vi.unstubAllGlobals(); vi.clearAllMocks(); });

it('defaults to virtual execution feedback, only offers two modes and does not connect or enable automatically', async () => {
  render(<ConsoleWorkspace />);
  await waitFor(() => expect(screen.getByText('200.00')).toBeTruthy());
  expect(screen.getByRole('button',{name:'离线验证'}).getAttribute('aria-pressed')).toBe('true');
  expect(screen.getByRole('button',{name:'实机操作'})).toBeTruthy();
  expect(screen.queryByRole('button',{name:'MuJoCo 仿真（预测）'})).toBeNull();
  expect(screen.queryByRole('button',{name:'ROS 虚拟反馈'})).toBeNull();
  expect(JSON.parse(screen.getByTestId('model').getAttribute('data-joints')).joint1).toBe(.1);
  expect(screen.getByTestId('model').getAttribute('data-feedback-driven')).toBe('true');
  expect(screen.queryByRole('button',{name:'连接实机'})).toBeNull();
  expect(screen.queryByRole('button',{name:'一键检测当前状态'})).toBeNull();
  expect(g.connect).not.toHaveBeenCalled();
  expect(g.requestAllPower).not.toHaveBeenCalled();
  expect(ros.action).not.toHaveBeenCalled();
  fireEvent.click(screen.getByRole('button', { name: '填入当前坐标' }));
  expect(screen.getByLabelText('X（mm）').value).toBe('200.00');
});

it('switches to shared hardware controls without issuing commands or showing virtual execute', async () => {
  render(<ConsoleWorkspace />);
  await waitFor(() => expect(screen.getByRole('button',{name:'实机操作'}).disabled).toBe(false));
  fireEvent.click(screen.getByRole('button',{name:'实机操作'}));
  const panel = screen.getByRole('region', { name: '七轴统一电源控制' });
  expect(panel.querySelectorAll('input[type=checkbox]')).toHaveLength(1);
  expect(screen.getByRole('button', { name: '全部使能 · J1–J7' }).disabled).toBe(true);
  expect(screen.queryByRole('button', { name: '使能 J1' })).toBeNull();
  expect(screen.getByRole('button', { name: '一键检测当前状态' }).disabled).toBe(true);
  expect(screen.getByRole('button',{name:/确认实机执行/}).disabled).toBe(true);
  expect(screen.queryByRole('button',{name:'执行虚拟动作'})).toBeNull();
  expect(g.connect).not.toHaveBeenCalled();
  expect(g.requestAllPower).not.toHaveBeenCalled();
  expect(ros.action).not.toHaveBeenCalled();
});

it('keeps newly received feedback fresh between clock ticks and still expires silent axes', () => {
  vi.useFakeTimers();
  try {
    vi.setSystemTime(1000);
    vi.stubGlobal('fetch', vi.fn(() => new Promise(() => {})));
    window.history.replaceState({}, '', '/zh/studio/console?pose=hardware');
    g.status = 'online';
    const readings = time => Object.fromEntries([1,2,3,4,5,6,7].map(id => [id,
      { mechPosRad: 0, positionUpdatedAt: time, fieldTimes: {runMode: time}, runMode: 0, error: '' }]));
    g.motors = readings(1000); g.updatedAt = 1000;
    const view = render(<ConsoleWorkspace />);
    expect(screen.getByText('7 / 7')).toBeTruthy();

    // A query completes before the independent 400 ms display clock ticks.
    vi.setSystemTime(1250);
    g.motors = readings(1250); g.updatedAt = 1250;
    view.rerender(<ConsoleWorkspace />);
    expect(screen.getByText('七轴反馈正常')).toBeTruthy();
    expect(screen.getByText('7 / 7')).toBeTruthy();
    expect(screen.getByText('反馈新鲜', {selector: '.deviceMetrics strong'})).toBeTruthy();

    // A timestamp beyond the completed query remains invalid.
    g.motors = {...g.motors, 7: {...g.motors[7], positionUpdatedAt: 1251}};
    view.rerender(<ConsoleWorkspace />);
    expect(screen.getByText('6 / 7')).toBeTruthy();
    g.motors = readings(1250);
    view.rerender(<ConsoleWorkspace />);

    act(() => vi.advanceTimersByTime(3200));
    expect(screen.getByText('0 / 7')).toBeTruthy();
    expect(screen.getByText('待完整新鲜反馈')).toBeTruthy();
    expect(g.requestAllPower).not.toHaveBeenCalled();
  } finally { cleanup(); vi.useRealTimers(); }
});

it('locks mode switching during virtual motion and leaves task stop available', async () => {
  ros.running = true; ros.state.job = {status:'running'};
  render(<ConsoleWorkspace />);
  expect(screen.getByRole('button',{name:'实机操作'}).disabled).toBe(true);
  fireEvent.click(screen.getByRole('button',{name:'停止当前任务'}));
  expect(ros.action).toHaveBeenCalledWith('stop');
});

it('blocks enable, mode switching and preview during initial-state detection while keeping immediate disable available', async () => {
  g.status='online'; let resolve;
  g.refresh.mockImplementation(()=>new Promise(r=>{resolve=r;}));
  render(<ConsoleWorkspace />);
  await waitFor(()=>expect(screen.getByRole('button',{name:'实机操作'}).disabled).toBe(false));
  fireEvent.click(screen.getByRole('button',{name:'实机操作'}));
  const panel=screen.getByRole('region',{name:'七轴统一电源控制'});
  fireEvent.click(panel.querySelector('input[type=checkbox]'));
  fireEvent.click(screen.getByRole('button',{name:'一键检测当前状态'}));
  expect(screen.getByRole('button',{name:'全部使能 · J1–J7'}).disabled).toBe(true);
  expect(screen.getByRole('button',{name:'离线验证'}).disabled).toBe(true);
  expect(screen.getByRole('button',{name:'计算并预演'}).disabled).toBe(true);
  expect(screen.getByRole('button',{name:'全部立即失能'}).disabled).toBe(false);
  fireEvent.click(screen.getByRole('button',{name:'全部立即失能'}));
  await waitFor(()=>expect(g.requestAllPower).toHaveBeenCalledExactlyOnceWith('disable'));
  resolve(undefined);
  await screen.findByText(/未取得完整查询/);
  expect(screen.getByRole('button',{name:/确认实机执行/}).disabled).toBe(true);
});
