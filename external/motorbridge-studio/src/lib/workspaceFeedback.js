// The Studio viewer rotates ROS Z-up by -pi/2 about X and raises its base 2 cm.
export function toStudioPoint([x, y, z]) {
  return { x, y: z + 0.02, z: -y };
}

export function feedbackJoints(state) {
  const values = state?.q_rad;
  if (!Array.isArray(values) || values.length !== 6 || !values.every(Number.isFinite)) return {};
  const joints = Object.fromEntries(values.map((value, i) => [`joint${i + 1}`, value]));
  return { ...joints, gripper_joint1: Number(state.finger_m) || 0 };
}

export function feedbackProblem(state, now = Date.now()) {
  if (!state) return '正在连接仿真服务…';
  if (state.backend !== 'mujoco' || state.feedback_source !== 'simulation') return '反馈来源不符合仿真任务要求';
  if (state.error) return state.error;
  if (!Number.isFinite(state.feedback_time_unix_s) || now - state.feedback_time_unix_s * 1000 > 2000) {
    return '仿真反馈已过期，操作已锁定';
  }
  if (!Array.isArray(state.q_rad) || state.q_rad.length !== 6 || !state.q_rad.every(Number.isFinite)) return '关节反馈无效';
  return '';
}
