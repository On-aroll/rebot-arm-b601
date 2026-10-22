(function (root) {
  'use strict';
  const keys = { KeyW:[0,1], KeyS:[0,-1], KeyA:[1,1], KeyD:[1,-1], KeyR:[2,1], KeyF:[2,-1], KeyU:[3,1], KeyO:[3,-1], KeyI:[4,1], KeyK:[4,-1], KeyJ:[5,1], KeyL:[5,-1] };
  function deadzone(value, threshold = 0.16) {
    if (!Number.isFinite(value) || Math.abs(value) <= threshold) return 0;
    return Math.sign(value) * Math.min(1, (Math.abs(value) - threshold) / (1 - threshold));
  }
  function keyboard(pressed) {
    const values = [0,0,0,0,0,0];
    if (!pressed.has('Space')) return values;
    for (const [key, [axis, sign]] of Object.entries(keys)) if (pressed.has(key)) values[axis] += sign;
    return values;
  }
  function keyboardGripper(pressed) {
    return pressed.has('Space') ? Number(pressed.has('KeyQ'))-Number(pressed.has('KeyE')) : 0;
  }
  function response(value) {
    const scaled = deadzone(value);
    return scaled * Math.abs(scaled);
  }
  function gamepad(pad) {
    if (!pad || pad.mapping !== 'standard') return { enabled:false, values:[0,0,0,0,0,0] };
    const button = (i) => Number(pad.buttons[i]?.value) || 0;
    const values = [-response(pad.axes[1]), -response(pad.axes[0]), button(12)-button(13), response(button(7)-button(6)), -response(pad.axes[3]), -response(pad.axes[2])];
    const grip = button(0)-button(1);
    const enabled = button(5) > 0.5;
    const scale = button(4) > 0.5 ? 0.25 : 1;
    return { enabled, values:values.map((v) => enabled ? v*scale : 0), gripper:enabled?grip*scale:0, neutral:values.every((v) => Math.abs(v) < 0.001) && Math.abs(grip)<.001 && !enabled };
  }
  function twist(values, linear, angular) {
    const result = values.slice();
    // Diagonal input must respect the same translational/angular speed caps.
    for (const offset of [0,3]) {
      const norm = Math.hypot(...result.slice(offset,offset+3));
      const scale = (offset ? angular : linear) / Math.max(1,norm);
      for (let i=offset;i<offset+3;i++) result[i] *= scale;
    }
    return result;
  }
  function manualSpeeds(base, limits, percent=100, precision=false) {
    const ratio = Math.max(.1,Math.min(2,Number.isFinite(percent)?percent/100:1))*(precision ? .25 : 1);
    return {
      // Keep the hard joint cap fixed while changing requested tool velocity;
      // lowering it mid-ramp would force an abrupt joint-velocity clamp.
      joint:Math.max(limits.minimum,Math.min(limits.joint,base.joint)),
      linear:Math.min(limits.linear,base.linear*ratio),
      angular:Math.min(limits.angular,base.angular*ratio),
      gripper:Math.min(limits.gripper,base.gripper*ratio),
    };
  }
  class SpeedShortcut {
    constructor() { this.reset(); }
    reset() { this.last=0; }
    sample(pad,ready) {
      const direction=pad?.mapping==='standard'?Number(pad.buttons[3]?.value>.5)-Number(pad.buttons[2]?.value>.5):0;
      const change=ready&&direction!==this.last?direction:0;
      this.last=direction;
      return change;
    }
  }
  function speedPreset(name, maximum, minimum=.005, linearMax=40, angularMax=20) {
    const ratios={slow:[.02,.1,.1],normal:[.1,.625,.5],fast:[.5,1,1],max:[1,1,1]};
    const r=ratios[name];
    if(!r || !Number.isFinite(maximum) || maximum<minimum)throw new Error('速度预设或设备上限无效');
    return {joint:Math.max(minimum,maximum*r[0]),linear:Math.max(.5,linearMax*r[1]),angular:Math.max(.5,angularMax*r[2])};
  }
  class ManualRecovery {
    constructor() { this.cancel(); }
    cancel() { this.armed=false;this.pending=false;this.busy=false;this.awaitingHold=false;this.waitingActive=false;this.stableAt=null;this.attempts=0;this.retryAt=0; }
    arm() { this.armed=true; }
    lost(state) {
      if (!this.armed) return;
      // ROS may revoke the lease one frame before the owner enters HOLDING.
      if (!state.communication_hold?.automatic_manual_recovery) {
        if(state.hardware_phase==='ACTIVE'){this.awaitingHold=true;return;}
        this.cancel();return;
      }
      this.awaitingHold=false;
      this.pending=true;this.since=state.communication_hold.since;this.stableAt=null;
    }
    ready(state,now,{fresh,neutral,allowed}) {
      if (!this.pending || this.busy) return false;
      if (!allowed || state.hardware_phase!=='HOLDING' || state.recovery_required ||
          state.communication_hold?.since!==this.since || !state.communication_hold?.automatic_manual_recovery) {
        this.cancel();return false;
      }
      if (!fresh || state.owned || !state.communication_hold.feedback_confirmed) { this.stableAt=null;return false; }
      if (this.stableAt===null) this.stableAt=now;
      return neutral && now-this.stableAt>=500 && now>=this.retryAt && this.attempts<3;
    }
    begin(now) { this.busy=true;this.attempts++;this.retryAt=now+1500; }
    finish(success) { this.busy=false;this.stableAt=null;if(success){this.pending=false;this.waitingActive=true;this.attempts=0;} }
  }
  const api = { keys, deadzone, response, keyboard, keyboardGripper, gamepad, twist, manualSpeeds, SpeedShortcut, speedPreset, ManualRecovery };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.ReBotTwinInput = api;
})(typeof window === 'undefined' ? globalThis : window);
