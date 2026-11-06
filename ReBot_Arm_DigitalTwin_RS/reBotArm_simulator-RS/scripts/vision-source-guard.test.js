'use strict';
const test = require('node:test');
const assert = require('node:assert/strict');
const guard = require('../public/js/vision-source-guard.js');

const now = 100000;
function valid() {
  const target = { source:'camera_rgbd_calibrated', frame_id:'base_link', calibration_id:'gemini-336l-v1', x:.3, y:.1, z:.2, confidence:.92 };
  const payload = { source:'camera_rgbd_calibrated', base_frame:'base_link', calibration_id:'gemini-336l-v1', calibration_verified:true, camera_stamp_ms:now-40, robot_stamp_ms:now-60 };
  return { target, payload, targetMode:'hardware', nowMs:now };
}

test('simulation accepts its own ground-truth detections', () => {
  assert.equal(guard.check({ target:{source:'mujoco_ground_truth'}, payload:{source:'rs_mujoco_object_states'}, targetMode:'simulation' }).allowed, true);
});

test('hardware rejects MuJoCo ground truth', () => {
  const input = valid();
  input.target.source = 'mujoco_ground_truth';
  input.payload.source = 'rs_mujoco_object_states';
  assert.match(guard.check(input).reason, /仿真真值/);
});

test('hardware needs verified base-frame RGB-D calibration and fresh synchronized feedback', () => {
  assert.equal(guard.check(valid()).allowed, true);
  for (const mutate of [
    (v) => { v.target.frame_id = 'camera_optical_frame'; },
    (v) => { v.payload.calibration_verified = false; },
    (v) => { v.target.calibration_id = 'other-calibration'; },
    (v) => { v.payload.camera_stamp_ms = now - 300; },
    (v) => { v.payload.robot_stamp_ms = now - 200; },
    (v) => { v.target.confidence = .5; },
    (v) => { delete v.target.z; }
  ]) {
    const input = valid();
    mutate(input);
    assert.equal(guard.check(input).allowed, false);
  }
});

test('hardware rejects unlabelled RGB-only detections', () => {
  const input = valid();
  input.payload.source = 'camera_rgb_only';
  input.target.source = 'camera_rgb_only';
  assert.match(guard.check(input).reason, /RGB-D/);
});
