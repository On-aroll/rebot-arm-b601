import { expect, it } from 'vitest';
import { defaultMapping } from './hardwareMapping';
import { emptyObservations, initialFeedbackReport, observeFeedback } from './initialStateCheck';

const now = 10000, RAD = Math.PI / 180;
const limits = [[-180,180],[0,180],[-180,180],[-180,180],[-180,180],[-180,180]];
const motors = () => Object.fromEntries(Array.from({length:7}, (_, i) => [i+1, {
  mechPosRad:(i === 1 ? -.39 : i === 6 ? 800 : 0)*RAD, positionUpdatedAt:now-100,
  fieldTimes:{runMode:now-100}, runMode:0, error:'' } ]));

it('preserves the sub-degree starting error and treats the gripper independently', () => {
  const report = initialFeedbackReport(motors(), defaultMapping(), limits, now, true, 1);
  expect(report).toMatchObject({positionsOk:true,modesOk:true,startOk:true,hardwareExecutionAllowed:false,identityStatus:'not-independently-verified'});
  expect(report.rows[1]).toMatchObject({nominal:false,tolerated:true});
  expect(report.rows[1].rawDeg).toBeCloseTo(-.39); expect(report.rows[1].modelDeg).toBeCloseTo(-.39);
  expect(report.rows[6].rawDeg).toBeCloseTo(800);
  expect(report.rows[6].modelDeg).toBeNull();
});

it('rejects larger starting deviations without clipping or changing existing mapping', () => {
  const m = motors(), mapping = defaultMapping();
  m[2].mechPosRad = -1.2*RAD;
  mapping.axes[0] = {...mapping.axes[0],direction:-1,offsetDeg:12,confirmed:true};
  const before = JSON.stringify([m,mapping,limits]);
  const report = initialFeedbackReport(m,mapping,limits,now,true,1);
  expect(report.startOk).toBe(false);
  expect(report.rows[1].modelDeg).toBeCloseTo(-1.2);
  expect(report.rows[0]).toMatchObject({modelDeg:12,previouslyConfirmed:true});
  expect(JSON.stringify([m,mapping,limits])).toBe(before);
});

it('uses separate position and mode timestamps and refuses stale or future samples', () => {
  const m = motors();
  m[1].positionUpdatedAt = now-3000; m[1].updatedAt = now;
  m[2].fieldTimes.runMode = now-3000;
  m[3].positionUpdatedAt = now+1;
  m[4].runMode = 1;
  const report = initialFeedbackReport(m,defaultMapping(),limits,now,true,1);
  expect(report.positionsOk).toBe(false); expect(report.modesOk).toBe(false);
  expect(report.rows[0].rawDeg).toBeNull();
  expect(report.rows[1].mode).toBeNull();
  expect(report.rows[2].fresh).toBe(false);
  expect(report.rows[3]).toMatchObject({mode:1,modeOk:false});
});

it('never validates disconnected data, unknown limits or malformed mappings', () => {
  expect(initialFeedbackReport(motors(),defaultMapping(),limits,now,false,1).positionsOk).toBe(false);
  expect(initialFeedbackReport(motors(),defaultMapping(),[],now,true,1).startOk).toBe(false);
  const mapping = defaultMapping(); mapping.axes[0].direction = 0;
  expect(() => initialFeedbackReport(motors(),mapping,limits,now,true,1)).toThrow();
});

it('records only new fresh position readings and retains multi-turn deltas', () => {
  const m = motors(); m[1].mechPosRad = 179*RAD;
  const report = initialFeedbackReport(m,defaultMapping(),limits,now,true,1);
  const baseline = emptyObservations(report);
  const repeat = observeFeedback(baseline,m,now);
  expect(repeat[0].samples).toBe(1);
  const changed = {...m,1:{...m[1],mechPosRad:181*RAD,positionUpdatedAt:now}};
  const next = observeFeedback(baseline,changed,now);
  expect(next[0]).toMatchObject({samples:2});
  expect(next[0].deltaDeg).toBeCloseTo(2); expect(next[0].spanDeg).toBeCloseTo(2);
  expect(baseline[0].spanDeg).toBe(0);
  expect(observeFeedback(next,changed,now+3000)[0].samples).toBe(2);
  changed[1] = {...changed[1],positionUpdatedAt:now+1,error:'timeout'};
  expect(observeFeedback(next,changed,now+1)[0].samples).toBe(2);
});
