(function (root) {
  'use strict';
  // Presentation only: callers keep raw feedback for control and diagnostics.
  class FeedbackDisplay {
    constructor() { this.reset(); }
    reset() { this.axes = {}; this.at = null; }
    push(angles, now, periodMs = 50) {
      if (this.at !== null && now-this.at >= 500) this.reset();
      const shown = this.sample(now) || {};
      // ACTIVE per-axis polling measured ~220ms. A 150ms cap left a pause
      // before each new sample. Bound the display delay, not the raw feedback.
      const duration = Math.max(20,Math.min(300,Number.isFinite(periodMs) ? periodMs : 50));
      for (const [key,value] of Object.entries(angles)) {
        if (!Number.isFinite(value)) continue;
        // Status republishing the same encoder value must not restart a blend.
        if (this.axes[key]?.to === value) continue;
        this.axes[key] = {from:shown[key] ?? value,to:value,at:now,duration};
      }
      this.at = now;
    }
    sample(now, raw = false) {
      if (this.at === null || now-this.at >= 500) return null;
      return Object.fromEntries(Object.entries(this.axes).map(([key,a]) =>
        [key,raw ? a.to : a.from+(a.to-a.from)*Math.max(0,Math.min(1,(now-a.at)/a.duration))]));
    }
  }
  if (typeof module !== 'undefined' && module.exports) module.exports = FeedbackDisplay;
  else root.ReBotFeedbackDisplay = FeedbackDisplay;
})(typeof window === 'undefined' ? globalThis : window);
