import { describe, expect, it } from 'vitest';
import { defaultMapping, mapHardwareFeedback, modelToMotor, motorToModel, validateMapping } from './hardwareMapping';

const RAD = Math.PI / 180;
function readings() {
  return Object.fromEntries(Array.from({length:6}, (_, i) => [i + 1, {mechPosRad: 0, positionUpdatedAt: 1000, updatedAt: 1000, error: ''}]));
}
const limits = Array.from({length:6}, () => [-90,90]);
describe('motor to model calibration', () => {
  it('uses identity as an unconfirmed baseline, preserves turns and has an exact inverse', () => {
    const profile = defaultMapping();
    expect(profile.axes.every(axis => axis.direction === 1 && axis.offsetDeg === 0 && !axis.confirmed)).toBe(true);
    const axis = { direction: -1, offsetDeg: 450 };
    expect(motorToModel(440*RAD, axis)).toBeCloseTo(10);
    expect(modelToMotor(10, axis)).toBeCloseTo(440*RAD);
    expect(motorToModel(440*RAD, profile.axes[0])).toBeCloseTo(440);
  });
  it('reports wrist limit mismatch without clamping its displayed angle', () => {
    const motors = readings(); motors[5].mechPosRad = -102.1 * RAD;
    const result = mapHardwareFeedback(motors, defaultMapping(), limits, 1500, true);
    expect(result.complete).toBe(true);
    expect(result.withinLimits).toBe(false);
    expect(result.joints.joint5).toBeCloseTo(-102.1 * RAD);
    expect(result.issues.some(issue => issue.includes('J5') && issue.includes('超出'))).toBe(true);
  });
  it('rejects partial, invalid and stale position snapshots even when other params are fresh', () => {
    const motors = readings(); motors[2].positionUpdatedAt = 0; motors[2].updatedAt = 4000;
    const stale = mapHardwareFeedback(motors, defaultMapping(), limits, 4000, true);
    expect(stale.joints).toBeNull();
    motors[2].mechPosRad = null;
    expect(mapHardwareFeedback(motors, defaultMapping(), limits, 1500, true).complete).toBe(false);
    const profile = defaultMapping(); profile.axes[0].offsetDeg = '';
    expect(() => validateMapping(profile)).toThrow('J1');
    expect(mapHardwareFeedback(readings(), profile, limits, 1500, true).joints).toBeNull();
    expect(mapHardwareFeedback(readings(), defaultMapping(), limits, 1500, false).joints).toBeNull();
  });
});
