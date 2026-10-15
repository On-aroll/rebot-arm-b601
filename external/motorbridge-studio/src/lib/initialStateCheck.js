import { motorToModel, validateMapping } from './hardwareMapping';

const DEG = 180 / Math.PI;
export const OBSERVATION_SPAN_DEG = 0.5; // Display threshold only, not a motion or limit setting.

export function initialFeedbackReport(motors, mapping, limits, now, online, toleranceDeg) {
  validateMapping(mapping);
  const rows = Array.from({ length: 7 }, (_, i) => {
    const id = i + 1, motor = motors?.[id];
    const age = now - motor?.positionUpdatedAt;
    const fresh = Boolean(online && !motor?.error && Number.isFinite(motor?.mechPosRad) && age >= 0 && age < 3000);
    const rawDeg = fresh ? motor.mechPosRad * DEG : null;
    const modelDeg = id <= 6 && fresh ? motorToModel(motor.mechPosRad, mapping.axes[i]) : null;
    const range = limits?.[i];
    const knownRange = id <= 6 && Array.isArray(range) && range.length === 2 && range.every(Number.isFinite);
    const nominal = knownRange && modelDeg != null && modelDeg >= range[0] && modelDeg <= range[1];
    const tolerated = knownRange && modelDeg != null && modelDeg >= range[0] - toleranceDeg && modelDeg <= range[1] + toleranceDeg;
    const modeAge = now - motor?.fieldTimes?.runMode;
    const modeFresh = Boolean(online && Number.isInteger(motor?.runMode) && modeAge >= 0 && modeAge < 3000);
    return { id, fresh, rawDeg, modelDeg, nominal, tolerated, range: knownRange ? [...range] : null,
      mode: modeFresh ? motor.runMode : null, modeOk: modeFresh && (id === 7 || motor.runMode === 0),
      previouslyConfirmed: id <= 6 && mapping.axes[i].confirmed,
      measuredAt: fresh ? motor.positionUpdatedAt : null,
      modeMeasuredAt: modeFresh ? motor.fieldTimes.runMode : null };
  });
  return { source: 'explicit-gateway-read', capturedAt: now, rows,
    positionsOk: rows.every(row => row.fresh), modesOk: rows.every(row => row.modeOk),
    startOk: rows.slice(0, 6).every(row => row.tolerated),
    identityStatus: 'not-independently-verified', hardwareExecutionAllowed: false };
}

export function emptyObservations(report) {
  return report.rows.map(row => ({ id: row.id, baselineDeg: row.rawDeg, minDeg: row.rawDeg, maxDeg: row.rawDeg,
    deltaDeg: 0, spanDeg: 0, samples: 1, lastMeasuredAt: row.measuredAt }));
}

export function observeFeedback(previous, motors, now) {
  return previous.map(row => {
    const reading = motors?.[row.id], time = reading?.positionUpdatedAt;
    if (reading?.error || !Number.isFinite(reading?.mechPosRad) || !Number.isFinite(time) ||
        time <= row.lastMeasuredAt || now - time < 0 || now - time >= 3000) return row;
    const value = reading.mechPosRad * DEG;
    const minDeg = Math.min(row.minDeg, value), maxDeg = Math.max(row.maxDeg, value);
    return { ...row, minDeg, maxDeg, deltaDeg: value - row.baselineDeg, spanDeg: maxDeg - minDeg,
      samples: row.samples + 1, lastMeasuredAt: time };
  });
}
