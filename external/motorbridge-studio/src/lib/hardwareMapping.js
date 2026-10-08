// Calibration in model degrees: q_model = direction * q_motor + offset.
// Preserve multi-turn readings. Display calibration is not hardware zeroing.
export const DEVICE_ID = '20250907852512190405';
export const MAPPING_KEY = `rebot.mapping.${DEVICE_ID}.v1`;
const RAD = Math.PI / 180;

export function defaultMapping() {
  return { version: 1, deviceId: DEVICE_ID, axes: Array.from({ length: 6 }, (_, i) => ({
    motorId: i + 1, direction: 1, offsetDeg: 0, confirmed: false,
  })) };
}

export function validateMapping(profile) {
  if (profile?.version !== 1 || profile.deviceId !== DEVICE_ID || !Array.isArray(profile.axes) || profile.axes.length !== 6) {
    throw new Error('映射文件版本、设备号或轴数量不匹配');
  }
  profile.axes.forEach((axis, i) => {
    if (axis.motorId !== i + 1 || ![1, -1].includes(axis.direction) ||
      typeof axis.offsetDeg !== 'number' || !Number.isFinite(axis.offsetDeg) || typeof axis.confirmed !== 'boolean') {
      throw new Error(`J${i + 1} 的方向或零位修正无效`);
    }
  });
  return profile;
}

export function loadMapping() {
  try {
    const saved = localStorage.getItem(MAPPING_KEY);
    if (saved) return validateMapping(JSON.parse(saved));
  } catch { /* Unreadable storage never implies calibrated axes. */ }
  return defaultMapping();
}

export function motorToModel(rawRad, axis) {
  if (!Number.isFinite(rawRad) || ![1, -1].includes(axis?.direction) || typeof axis.offsetDeg !== 'number' || !Number.isFinite(axis.offsetDeg)) return null;
  return axis.direction * rawRad / RAD + axis.offsetDeg;
}

export function modelToMotor(modelDeg, axis) {
  if (!Number.isFinite(modelDeg) || ![1, -1].includes(axis?.direction) || typeof axis.offsetDeg !== 'number' || !Number.isFinite(axis.offsetDeg)) throw new Error('无效的关节映射');
  return (modelDeg - axis.offsetDeg) / axis.direction * RAD;
}

export function mapHardwareFeedback(motors, profile, limits, now, online) {
  let profileError = '';
  try { validateMapping(profile); } catch (error) { profileError = error.message; }
  const rows = Array.from({ length: 6 }, (_, i) => {
    const reading = motors?.[i + 1];
    const axis = profile?.axes?.[i];
    const raw = reading?.mechPosRad;
    const valid = Number.isFinite(raw) && !reading?.error;
    const age = Number.isFinite(reading?.positionUpdatedAt) ? now - reading.positionUpdatedAt : Infinity;
    const fresh = Boolean(online && valid && age >= 0 && age < 3000);
    const deg = valid ? motorToModel(raw, axis) : null;
    const range = limits?.[i];
    const knownLimit = Array.isArray(range) && range.length === 2 && range.every(Number.isFinite);
    const outside = deg != null && knownLimit && (deg < range[0] - .01 || deg > range[1] + .01);
    return { id: i + 1, rawDeg: valid ? raw / RAD : null, modelDeg: deg, ageMs: age, fresh, outside, range: knownLimit ? range : null };
  });
  const complete = !profileError && rows.every(row => row.fresh && row.modelDeg != null);
  const issues = [];
  if (profileError) issues.push(profileError);
  if (!online) issues.push('网关未连接');
  rows.forEach(row => {
    if (!row.fresh) issues.push(`J${row.id} 缺少 3 秒内的位置反馈`);
    if (row.outside) issues.push(`J${row.id} 映射后 ${row.modelDeg.toFixed(2)}°，超出模型范围 ${row.range[0].toFixed(0)}°…${row.range[1].toFixed(0)}°`);
  });
  if (profile?.axes?.some(axis => !axis.confirmed)) issues.push('方向和零位尚未全部人工核对');
  const joints = complete ? Object.fromEntries(rows.map(row => [`joint${row.id}`, row.modelDeg * RAD])) : null;
  return { rows, joints, complete, issues, withinLimits: rows.every(row => row.range && !row.outside),
    confirmed: !profileError && profile.axes.every(axis => axis.confirmed) };
}
