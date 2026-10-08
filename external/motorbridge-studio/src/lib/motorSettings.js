// Configuration only. Position/current/velocity target registers are excluded.
export const MOTOR_SETTINGS = [
  { key: 'limitSpd', id: 0x7017, label: 'CSP 速度上限', unit: 'rad/s', hint: 'limit_spd · 仅 CSP（模式 5）使用', step: '0.05' },
  { key: 'locKp', id: 0x701e, label: '位置环刚度 Kp', unit: '', hint: 'loc_kp · 位置模式有效，不是 MIT 的 kp', step: '0.1' },
  { key: 'limitCur', id: 0x7018, label: '电流上限', unit: 'A', hint: 'limit_cur · 速度 / 位置模式有效', step: '0.1' },
  { key: 'limitPpSpd', id: 0x7024, label: 'PP 速度上限', unit: 'rad/s', hint: 'vel_max · 位置 PP（模式 1）使用', step: '0.05' },
];

export function settingPayload(key, input) {
  const field = MOTOR_SETTINGS.find((item) => item.key === key);
  const value = input === '' || input == null ? NaN : Number(input);
  if (!field || !Number.isFinite(value) || value <= 0 || value > 3.402823466e38) {
    throw new Error('参数必须是大于 0 的有限数值');
  }
  return { param_id: field.id, type: 'f32', value, timeout_ms: 800 };
}

export function settingsRefusal(payload, cap) {
  const field = MOTOR_SETTINGS.find((item) => item.id === payload.param_id);
  if (!field || payload.type !== 'f32') return '此面板只允许写速度上限、位置环刚度和电流上限';
  try { settingPayload(field.key, payload.value); } catch (error) { return error.message; }
  if (['limitSpd', 'limitPpSpd'].includes(field.key) && (!Number.isFinite(cap) || payload.value > cap)) {
    return `速度上限不能超过当前下发阈值 ${cap} rad/s`;
  }
  return null;
}


export function positionSpeedState(motor) {
  if (motor?.runMode === 1) return { label: '位置 PP', register: 'vel_max / 0x7024', value: motor.limitPpSpd };
  if (motor?.runMode === 5) return { label: '位置 CSP', register: 'limit_spd / 0x7017', value: motor.limitSpd };
  const names = { 0: 'MIT', 2: '速度', 3: '电流' };
  return { label: names[motor?.runMode] || '模式未读到', register: null, value: null };
}

export function positionMotionRefusal(motor, cap) {
  const speed = positionSpeedState(motor);
  if (!speed.register) return `当前为${speed.label}；本面板仅开放位置 PP/CSP 使能，请先核对运行模式`;
  if (!Number.isFinite(speed.value) || speed.value <= 0) return `${speed.label} 的有效速度上限未读到或无效`;
  if (!Number.isFinite(cap) || cap <= 0 || speed.value > cap) return `${speed.label} 速度上限 ${speed.value.toFixed(3)} rad/s 高于下发阈值 ${cap}，请调整对应的速度参数`;
  return '';
}
