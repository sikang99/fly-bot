import { sanitizeCommand } from './config.js';
export class ManualControl {
  constructor() { this.enabled = false; this.stop(); }
  enter(enabled) { this.enabled = enabled; this.latched = false; this.mode = 'stand'; this.armed = false; this.lastAt = -Infinity; this.value = sanitizeCommand(); }
  setMode(mode) {
    if (!['passive', 'stand', 'walk'].includes(mode) || this.latched) return;
    this.mode = mode; this.armed = false; this.lastAt = -Infinity; this.value = sanitizeCommand();
  }
  stop() { this.latched = true; this.armed = false; this.value = sanitizeCommand(); }
  receive(input, now) {
    if (!this.enabled || this.latched || this.mode !== 'walk') return;
    this.lastAt = now;
    if (!input.held) { this.armed = true; this.value = sanitizeCommand(); return; }
    this.value = this.armed ? sanitizeCommand(input.command) : sanitizeCommand();
  }
  sample(now) {
    if (now - this.lastAt > 300) { this.value = sanitizeCommand(); this.armed = false; }
    return this.value;
  }
}
export function gamepadCommand(pad) {
  const axis = i => { const v = pad.axes[i] || 0; return Math.abs(v) < .15 ? 0 : Math.sign(v) * (Math.abs(v) - .15) / .85; };
  return { held: !!pad.buttons[5]?.pressed, emergency: !!pad.buttons[1]?.pressed,
    command: { vx: -axis(1) * .6, vy: -axis(0) * .35, yawRate: -axis(2) * .35, frequency: 1.4 } };
}
