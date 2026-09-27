import { A2_FOOTPRINT, waypointRouteClear } from './avoidance.js';
import { isTraversable } from './terrain.js';
import { isDynamic, obstacleWorldBounds } from './dynamic.js';
import { sanitizeCommand } from './config.js';

function onLowStep(pose, obstacles) {
  return obstacles.some(o => isTraversable(o)
    && Math.abs(pose.x - o.x) < o.halfX + .56
    && Math.abs(pose.y - o.y) < o.halfY + .42);
}

// A starting point inside the planning buffer is not necessarily a collision.
// Permit only increasing separation along an already separated body axis;
// never enter another buffer or relax a moving obstacle's clearance.
export function detourEdgeClear(pose, next, obstacles, yaw) {
  return obstacles.filter(o => !isTraversable(o)).every(o => {
    if (waypointRouteClear(pose, next, [o], yaw)) return true;
    if (isDynamic(o) || waypointRouteClear(pose, pose, [o], yaw)
      || !waypointRouteClear(pose, next, [o], yaw, .02)) return false;
    const box = obstacleWorldBounds(o), c = Math.abs(Math.cos(yaw)), s = Math.abs(Math.sin(yaw));
    return [['x', box.halfX + c * A2_FOOTPRINT.halfLength + s * A2_FOOTPRINT.halfWidth],
      ['y', box.halfY + s * A2_FOOTPRINT.halfLength + c * A2_FOOTPRINT.halfWidth]].some(([axis, extent]) => {
      const offset = pose[axis] - box[axis];
      return Math.abs(offset) > extent + .02
        && (next[axis] - pose[axis]) * Math.sign(offset) > 1e-8;
    });
  });
}

// Bounded local search. Every edge checks the whole swept body, not only its
// centre. Keep heading fixed until there is room to resume normal navigation.
export class LocalDetour {
  reset() { this.route = null; this.key = ''; this.retryAt = -Infinity; }
  constructor() { this.reset(); }
  plan(pose, target, obstacles, now, enabled) {
    if (!enabled || !target) { this.reset(); return null; }
    const solid = obstacles.filter(o => !isTraversable(o) && !isDynamic(o));
    // Traversable terrain must remain traversable during detours as well.
    const key = JSON.stringify([target.x, target.y, solid]);
    if (key !== this.key) { this.reset(); this.key = key; }
    if (this.route && Math.abs(Math.atan2(Math.sin(pose.yaw - this.heading), Math.cos(pose.yaw - this.heading))) > .15) {
      this.route = null; this.retryAt = now;
    }
    if (waypointRouteClear(pose, target, solid) && (!this.route || !onLowStep(pose, obstacles))) { this.reset(); return null; }
    const checks = obstacles;
    if (this.route && !detourEdgeClear(pose, this.route[0], checks, this.heading)) {
      this.route = null; this.retryAt = now + .5;
    }
    if (!this.route && now >= this.retryAt) {
      this.heading = pose.yaw;
      this.route = searchDetour(pose, target, obstacles.filter(o => !isDynamic(o)));
      this.retryAt = now + 1;
    }
    if (!this.route) return { strategy: 'detour-blocked', heading: pose.yaw,
      command: sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 }) };
    // Follow a visible point ahead, not every 20cm search cell. Braking at
    // every grid cell can leave insufficient forward motion to climb a step.
    for (let i = this.route.length - 1; i > 0; i--) {
      if (Math.hypot(this.route[i].x - pose.x, this.route[i].y - pose.y) <= .8
        && detourEdgeClear(pose, this.route[i], checks, this.heading)) {
        this.route.splice(0, i); break;
      }
    }
    while (this.route.length > 1 && Math.hypot(this.route[0].x - pose.x, this.route[0].y - pose.y) < .08
      && detourEdgeClear(pose, this.route[1], checks, this.heading)) this.route.shift();
    const next = this.route[0];
    if (this.route.length === 1 && Math.hypot(next.x - pose.x, next.y - pose.y) < .07) {
      this.route = null; this.retryAt = now;
      return { strategy: 'detour', heading: this.heading, command: sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 }) };
    }
    if (!detourEdgeClear(pose, next, checks, this.heading)) {
      return { strategy: 'detour-blocked', heading: this.heading, command: sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 }) };
    }
    const dx = next.x - pose.x, dy = next.y - pose.y;
    const c = Math.cos(this.heading), s = Math.sin(this.heading);
    const forward = (dx * c + dy * s) * 1.5, lateral = (-dx * s + dy * c) * 1.5;
    const scale = Math.min(1, (forward < 0 ? .12 : .45) / Math.max(Math.abs(forward), 1e-9), .22 / Math.max(Math.abs(lateral), 1e-9));
    return { strategy: 'detour', heading: this.heading, command: sanitizeCommand({
      vx: forward * scale, vy: lateral * scale, yawRate: 0 }) };
  }
}

export function searchDetour(pose, target, obstacles) {
  if (!waypointRouteClear(pose, pose, obstacles, pose.yaw, .02)) return null;
  const distance = p => Math.hypot(target.x - p.x, target.y - p.y);
  const start = { ...pose, i: 0, j: 0, g: 0, f: distance(pose), parent: null };
  const open = [start], best = new Map([['0,0', 0]]);
  for (let count = 0; open.length && count < 1800; count++) {
    open.sort((a, b) => b.f - a.f);
    const p = open.pop();
    const freeToTurn = waypointRouteClear(p, target, obstacles) && !onLowStep(p, obstacles);
    const dock = distance(p) < 1.2 && waypointRouteClear(p, target, obstacles, pose.yaw);
    if (p.parent && (freeToTurn || dock)) {
      const path = []; for (let n = p; n.parent; n = n.parent) path.unshift({ x: n.x, y: n.y });
      // A nearby goal can fit the rectangular footprint even when a turn at
      // that goal cannot. Finish at fixed heading rather than declaring no path.
      if (!freeToTurn) path.push({ x: target.x, y: target.y });
      return path;
    }
    for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
      if (!di && !dj) continue;
      const i = p.i + di, j = p.j + dj;
      if (Math.abs(i) > 20 || Math.abs(j) > 20) continue;
      const q = { i, j, x: pose.x + i * .2, y: pose.y + j * .2 };
      const g = p.g + Math.hypot(di, dj) * .2, key = `${i},${j}`;
      if (g >= (best.get(key) ?? Infinity) || !detourEdgeClear(p, q, obstacles, pose.yaw)) continue;
      best.set(key, g); open.push({ ...q, g, f: g + distance(q), parent: p });
    }
  }
  return null;
}
