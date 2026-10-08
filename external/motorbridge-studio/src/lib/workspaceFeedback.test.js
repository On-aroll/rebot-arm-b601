import { describe, expect, it } from 'vitest';
import { feedbackJoints, feedbackProblem, toStudioPoint } from './workspaceFeedback';

describe('workspace feedback adapter', () => {
  it('maps ROS axes and base offset consistently with the Studio robot', () => {
    expect(toStudioPoint([0, 0, 0])).toEqual({ x: 0, y: 0.02, z: -0 });
    expect(toStudioPoint([1, 2, 3])).toEqual({ x: 1, y: 3.02, z: -2 });
    const a = toStudioPoint([0.38, -0.12, 0.45]);
    const b = toStudioPoint([0.38, 0.12, 0.45]);
    expect(Math.hypot(a.x-b.x, a.y-b.y, a.z-b.z)).toBeCloseTo(0.24);
  });
  it('drives joints from actual radians instead of desired angles', () => {
    const map = feedbackJoints({ q_rad: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], target_rad: [9,9,9,9,9,9], finger_m: 0.016 });
    expect(map.joint1).toBe(0.1);
    expect(map.joint6).toBe(0.6);
    expect(map.gripper_joint1).toBe(0.016);
    expect(feedbackJoints({ q_rad: [NaN] })).toEqual({});
  });
  it('locks stale, failed and incorrectly labelled feedback', () => {
    const state = { backend: 'mujoco', feedback_source: 'simulation', feedback_time_unix_s: 100, q_rad: [0,0,0,0,0,0] };
    expect(feedbackProblem(state, 101000)).toBe('');
    expect(feedbackProblem(state, 103000)).toContain('过期');
    expect(feedbackProblem({ ...state, error: 'engine stopped' },101000)).toBe('engine stopped');
    expect(feedbackProblem({ ...state, feedback_source: 'hardware' },101000)).toContain('来源');
  });
});
