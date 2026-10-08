import { describe, expect, it } from 'vitest';
import { settingPayload, settingsRefusal } from './motorSettings';

describe('configuration-only parameter validation', () => {
  it('rejects empty, negative, zero and nonfinite values', () => {
    for (const value of ['',-1,0,NaN,Infinity,'abc',1e40]) expect(()=>settingPayload('limitSpd',value)).toThrow();
  });
  it('never accepts target registers and enforces the speed threshold', () => {
    expect(settingsRefusal({param_id:0x7016,type:'f32',value:0.1},0.3)).toContain('只允许');
    expect(settingsRefusal(settingPayload('limitSpd',0.4),0.3)).toContain('阈值');
    expect(settingsRefusal(settingPayload('limitSpd',0.2),0.3)).toBeNull();
    expect(settingsRefusal(settingPayload('locKp',13),0.3)).toBeNull();
  });
});

describe('mode-aware speed guard', () => {
  it('checks PP vel_max instead of accepting the unrelated CSP limit_spd', async () => {
    const { positionMotionRefusal, positionSpeedState } = await import('./motorSettings');
    const pp = {runMode:1,limitSpd:0.1,limitPpSpd:1};
    expect(positionSpeedState(pp).register).toContain('7024');
    expect(positionMotionRefusal(pp,0.3)).toContain('高于');
    expect(positionMotionRefusal({...pp,limitPpSpd:0.1},0.3)).toBe('');
    expect(positionMotionRefusal({...pp,limitPpSpd:null},0.3)).toContain('未读到');
  });
  it('uses CSP limit_spd and rejects unknown or non-position modes', async () => {
    const { positionMotionRefusal } = await import('./motorSettings');
    expect(positionMotionRefusal({runMode:5,limitSpd:0.1,limitPpSpd:1},0.3)).toBe('');
    for (const runMode of [null,0,2,3]) expect(positionMotionRefusal({runMode,limitSpd:0.1,limitPpSpd:0.1},0.3)).toContain('运行模式');
    expect(settingsRefusal(settingPayload('limitPpSpd',1),0.3)).toContain('阈值');
  });
});
