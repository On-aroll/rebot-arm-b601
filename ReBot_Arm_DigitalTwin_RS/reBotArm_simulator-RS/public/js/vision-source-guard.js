(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) module.exports = api;
  if (root) root.ReBotVisionSourceGuard = api;
})(typeof window !== 'undefined' ? window : globalThis, function () {
  'use strict';

  const MAX_CAMERA_AGE_MS = 250;
  const MAX_CAMERA_ROBOT_SKEW_MS = 100;
  const MIN_CONFIDENCE = 0.8;

  function check({ target, payload, targetMode, nowMs = Date.now() } = {}) {
    if (targetMode !== 'hardware') return { allowed: true, reason: '' };
    const source = String(payload?.source || '');
    if (source === 'rs_mujoco_object_states' || target?.source === 'mujoco_ground_truth') {
      return { allowed: false, reason: '仿真真值检测不能用于真机控制' };
    }
    if (source !== 'camera_rgbd_calibrated' || target?.source !== source) {
      return { allowed: false, reason: '真机动作需要经标定的真实相机 RGB-D 目标' };
    }
    if (target.frame_id !== 'base_link' || payload.base_frame !== 'base_link') {
      return { allowed: false, reason: '视觉目标尚未变换到 base_link' };
    }
    if (payload.calibration_verified !== true || !payload.calibration_id ||
        target.calibration_id !== payload.calibration_id) {
      return { allowed: false, reason: '相机外参尚未验证或与目标不匹配' };
    }
    if (![target.x, target.y, target.z].every((value) => Number.isFinite(Number(value)))) {
      return { allowed: false, reason: '视觉目标没有有效三维坐标' };
    }
    if (!Number.isFinite(target.confidence) || target.confidence < MIN_CONFIDENCE) {
      return { allowed: false, reason: '视觉置信度低于真机门槛' };
    }
    const cameraMs = Number(payload.camera_stamp_ms);
    const robotMs = Number(payload.robot_stamp_ms);
    if (!Number.isFinite(cameraMs) || !Number.isFinite(robotMs) ||
        nowMs - cameraMs > MAX_CAMERA_AGE_MS || cameraMs - nowMs > 100) {
      return { allowed: false, reason: '相机帧时间戳缺失、过期或来自未来' };
    }
    if (Math.abs(cameraMs - robotMs) > MAX_CAMERA_ROBOT_SKEW_MS) {
      return { allowed: false, reason: '相机帧与机器人反馈没有时间同步' };
    }
    return { allowed: true, reason: '' };
  }

  return { check, MAX_CAMERA_AGE_MS, MAX_CAMERA_ROBOT_SKEW_MS, MIN_CONFIDENCE };
});
