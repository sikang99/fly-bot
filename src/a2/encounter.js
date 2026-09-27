import { waypointRouteClear } from './avoidance.js';
import { isDynamic } from './dynamic.js';
import { sanitizeCommand } from './config.js';

export const ENCOUNTER = Object.freeze({ horizon: 4, dt: .1, lateralSpeed: .30, acceleration: .25, retreat: .45 });
const approach = (v, target, amount) => v + Math.max(-amount, Math.min(amount, target - v));

// Relative swept segments detect fast actors between prediction samples.
// Constant-velocity actors are a prediction, not a guarantee of human intent.
export function encounterTrajectory(pose, command, obstacles, velocity = {}, reverseBudget = ENCOUNTER.retreat) {
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  let vx = velocity.vx || 0, vy = velocity.vy || 0, p = { x: pose.x, y: pose.y }, reversed = 0;
  const path = [p];
  for (let step = 1; step <= ENCOUNTER.horizon / ENCOUNTER.dt; step++) {
    const t = step * ENCOUNTER.dt, before = t - ENCOUNTER.dt;
    const forward = command.vx < 0 && reversed >= reverseBudget ? 0 : command.vx * .65;
    const lateral = Math.max(-ENCOUNTER.lateralSpeed, Math.min(ENCOUNTER.lateralSpeed, command.vy * .65));
    const tx = c * forward - s * lateral, ty = s * forward + c * lateral;
    vx = approach(vx, tx, ENCOUNTER.acceleration * ENCOUNTER.dt);
    vy = approach(vy, ty, ENCOUNTER.acceleration * ENCOUNTER.dt);
    const q = { x: p.x + vx * ENCOUNTER.dt, y: p.y + vy * ENCOUNTER.dt };
    reversed += Math.max(0, -(vx * c + vy * s)) * ENCOUNTER.dt;
    for (const o of obstacles) {
      const ox = isDynamic(o) ? o.vx || 0 : 0, oy = isDynamic(o) ? o.vy || 0 : 0;
      if (!waypointRouteClear({ x: p.x - ox * before, y: p.y - oy * before },
        { x: q.x - ox * t, y: q.y - oy * t }, [o], pose.yaw)) return { safe: false, path, collisionAt: t };
    }
    p = q; path.push(p);
  }
  return { safe: true, path, end: p };
}

export class EncounterPlanner {
  constructor() { this.reset(); }
  reset() { this.active = false; this.side = 0; this.clearAt = null; this.origin = null; this.heading = null; this.sampleAt = -Infinity; this.last = null; }
  plan(pose, requested, obstacles, now, velocity = {}, target = null) {
    if (now >= this.sampleAt && now - this.sampleAt < .05) return this.last;
    const result = this.decide(pose, requested, obstacles, now, velocity, target);
    this.sampleAt = now; this.last = result; return result;
  }
  decide(pose, requested, obstacles, now, velocity, target) {
    const moving = obstacles.filter(isDynamic);
    if (!moving.length) { this.reset(); return null; }
    const cruising = encounterTrajectory(pose, requested, moving, velocity).safe;
    const standing = encounterTrajectory(pose, { vx: 0, vy: 0 }, moving, velocity).safe;
    if (cruising && standing) {
      this.clearAt ??= now;
      if (!this.active || now - this.clearAt >= .6) { this.reset(); return null; }
    } else this.clearAt = null;
    if (!this.active) { this.active = true; this.origin = { ...pose }; this.heading = pose.yaw; }
    const headingPose = { ...pose, yaw: this.heading };
    const retreatUsed = Math.max(0, (this.origin.x - pose.x) * Math.cos(this.heading)
      + (this.origin.y - pose.y) * Math.sin(this.heading));
    const budget = Math.max(0, ENCOUNTER.retreat - retreatUsed);
    const candidates = [
      { strategy: 'dynamic-dodge', vx: 0, vy: .5, side: 1 },
      { strategy: 'dynamic-dodge', vx: 0, vy: -.5, side: -1 },
      { strategy: 'dynamic-dodge', vx: .18, vy: .5, side: 1 },
      { strategy: 'dynamic-dodge', vx: .18, vy: -.5, side: -1 },
      ...(budget > .04 ? [{ strategy: 'dynamic-retreat', vx: -Math.min(.3, budget * .8), vy: 0, side: 0 }] : []),
      { strategy: 'dynamic-wait', vx: 0, vy: 0, side: 0 },
    ];
    const safe = candidates.map(candidate => ({ ...candidate,
      prediction: encounterTrajectory(headingPose, candidate, obstacles, velocity, budget) }))
      .filter(candidate => candidate.prediction.safe);
    const cost = candidate => {
      const p = candidate.prediction.end;
      return (candidate.strategy === 'dynamic-wait' ? 6 : candidate.strategy === 'dynamic-retreat' ? 3 : 0)
        + (this.side && candidate.side && candidate.side !== this.side ? 2 : 0)
        + (target ? .2 * Math.hypot(target.x - p.x, target.y - p.y) : 0);
    };
    safe.sort((a, b) => cost(a) - cost(b));
    const selected = safe[0] || candidates.at(-1);
    if (selected.side) this.side = selected.side;
    return { strategy: selected.strategy, heading: this.heading, direction: selected.side,
      blocked: !safe.length, retreatRemaining: budget, command: sanitizeCommand({ vx: selected.vx, vy: selected.vy, yawRate: 0 }) };
  }
}
