import { describe, expect, it, vi } from 'vitest';
import {
  refreshMotorStateOp,
  checkOnlineOnceOp,
  invalidControlFields,
  mapParamStreamToHit,
  mapResponseToHit,
  probeHitMatchesTarget,
} from './motorStudioOps';

describe('motor studio ops', () => {
  it('unwraps gateway state_once payloads before merging RobStride telemetry', () => {
    const hit = {
      vendor: 'robstride',
      model: 'rs-00',
      esc_id: 4,
      mst_id: 0xfd,
    };
    const next = mapResponseToHit(hit, {
      state: {
        vendor: 'robstride',
        has_value: true,
        status_code: 0,
        pos: -0.138,
        vel: 0.25,
        torq: 0.03,
        t_mos: 31.5,
      },
    });

    expect(next.pos).toBeCloseTo(-0.138);
    expect(next.vel).toBeCloseTo(0.25);
    expect(next.torq).toBeCloseTo(0.03);
    expect(next.t_mos).toBeCloseTo(31.5);
    expect(next.pmax).toBeCloseTo(4 * Math.PI);
    expect(next.vmax).toBe(50);
    expect(next.tmax).toBe(17);
  });

  it('maps RobStride observation params into live feedback fields', () => {
    const next = mapParamStreamToHit(
      { vendor: 'robstride', pos: 0, vel: 0, torq: 0, status_name: 'Motor' },
      {
        vendor: 'robstride',
        values: {
          run_mode: 1,
          mechPos: -0.82,
          mechVel: 0.25,
          iqf: 0.31,
          VBUS: 24.1,
          torque_fdb: 0.08,
          drv_temp: 0,
        },
      }
    );

    expect(next.pos).toBeCloseTo(-0.82);
    expect(next.vel).toBeCloseTo(0.25);
    expect(next.iqf).toBeCloseTo(0.31);
    expect(next.torq).toBeCloseTo(0.08);
    expect(next.vbus).toBeCloseTo(24.1);
    expect(next.t_mos).toBeUndefined();
    expect(next.status_name).toBe('Motor');
    expect(next.run_mode).toBe(1);
    expect(next.run_mode_name).toBe('Position');
    expect(next.feedback_source).toBe('robstride_observation_params');
  });

  it('maps Damiao param stream values into diagnostic fields without overriding feedback', () => {
    const next = mapParamStreamToHit(
      { vendor: 'damiao', pos: 0 },
      {
        vendor: 'damiao',
        values: {
          CTRL_MODE: 2,
          VBus: 24.2,
          Tpcb: 35,
          Tmtr: 32,
          p_m: 0.5,
          xout: 0.45,
          PMAX: 12.5,
          VMAX: 50,
          TMAX: 17,
        },
      }
    );

    expect(next.pos).toBe(0);
    expect(next.motor_pos).toBeCloseTo(0.5);
    expect(next.output_pos).toBeCloseTo(0.45);
    expect(next.vbus).toBeCloseTo(24.2);
    expect(next.t_mos).toBeCloseTo(35);
    expect(next.t_rotor).toBeCloseTo(32);
    expect(next.status).toBe(2);
    expect(next.pmax).toBeCloseTo(12.5);
  });

  it('flags cleared control fields before commands can be sent', () => {
    const invalid = invalidControlFields(
      {
        target: '',
        vlim: '1',
        kp: '',
        kd: 'bad',
        tau: '0',
      },
      ['target', 'vlim', 'kp', 'kd', 'tau']
    );

    expect(invalid).toEqual(['target', 'kp', 'kd']);
  });

  it('keeps RobStride probe hits scoped to the requested feedback id by default', () => {
    const target = { vendor: 'robstride', esc_id: 6, mst_id: 0xfd };

    expect(probeHitMatchesTarget({ esc_id: 6, mst_id: 0xfd }, target)).toBe(true);
    expect(probeHitMatchesTarget({ esc_id: 6, mst_id: 0xff }, target)).toBe(false);
    expect(probeHitMatchesTarget({ esc_id: 7, mst_id: 0xfd }, target)).toBe(false);
  });

  it('allows robot-arm RobStride probes to accept alternate host ids for the same device', () => {
    const target = { vendor: 'robstride', esc_id: 6, mst_id: 0xfd };

    expect(
      probeHitMatchesTarget({ esc_id: 6, mst_id: 0xff }, target, {
        acceptAnyFeedbackId: true,
      })
    ).toBe(true);
    expect(
      probeHitMatchesTarget({ esc_id: 7, mst_id: 0xff }, target, {
        acceptAnyFeedbackId: true,
      })
    ).toBe(false);
  });
});


describe('read-only RobStride refresh',()=>{
  const hit={vendor:'robstride',model:'rs-06',esc_id:1,mst_id:253,pos:99,vel:8,online:false};
  const args=sendCmd=>({h:hit,vendors:{},setTargetFor:vi.fn(),sendCmd,setHits:vi.fn(),pushLog:vi.fn()});
  it('uses only parameter queries and preserves raw multi-turn positions',async()=>{
    const values={0x7019:-7.2,0x701a:.1,0x701b:0,0x701c:24,0x7005:0};
    const send=vi.fn(async(op,p)=>({ok:true,data:{value:values[p.param_id]}}));
    const next=await refreshMotorStateOp(args(send));
    expect(next.pos).toBe(-7.2);expect(next.vel).toBe(0);expect(next.run_mode).toBe(0);expect(next.online).toBe(true);
    expect(send.mock.calls.map(([op])=>op)).toEqual(Array(5).fill('robstride_read_param'));
  });
  it.each([null,undefined,NaN,Infinity,'',false])('missing/invalid position %s is never a fresh zero or old value',async value=>{
    const send=vi.fn(async()=>({ok:true,data:{value}}));
    const next=await refreshMotorStateOp(args(send));
    expect(Number.isNaN(next.pos)).toBe(true);expect(next.online).toBe(false);
    expect(await checkOnlineOnceOp(args(send))).toBe(false);
  });
  it('missing optional reads do not revive historical velocity/mode',async()=>{
    const send=vi.fn(async(op,p)=>({ok:true,data:{value:p.param_id===0x7019?.2:null}}));
    const next=await refreshMotorStateOp(args(send));expect(next.pos).toBe(.2);
    expect(Number.isNaN(next.vel)).toBe(true);expect(Number.isNaN(next.run_mode)).toBe(true);
    expect(await checkOnlineOnceOp(args(send))).toBe(true);
  });
});


it('legacy DM diagnostic refresh uses only explicit register reads; does not invent position',async()=>{
  const sendCmd=vi.fn(async()=>({ok:true,data:{value:2}}));
  const args={h:{vendor:'damiao',model:'dm4310',esc_id:1,mst_id:17,pos:99},vendors:{},setTargetFor:vi.fn(),sendCmd,setHits:vi.fn(),pushLog:vi.fn()};
  const next=await refreshMotorStateOp(args);
  expect(next.online).toBe(true);expect(Number.isNaN(next.pos)).toBe(true);
  expect(sendCmd.mock.calls.every(([op])=>['get_register_f32','get_register_u32'].includes(op))).toBe(true);
  expect(await checkOnlineOnceOp(args)).toBe(true);
});
