import { waypointRouteClear } from './avoidance.js';
import { isTraversable } from './terrain.js';
import { isDynamic } from './dynamic.js';
import { clamp, sanitizeCommand } from './config.js';

// Bounded local search. Every edge checks the whole swept body, not only its
// centre. Keep heading fixed until there is room to resume normal navigation.
export class LocalDetour {
  reset() { this.route = null; this.key = ''; this.retryAt = -Infinity; }
  constructor() { this.reset(); }
  plan(pose, target, obstacles, now, enabled) {
    if (!enabled || !target) { this.reset(); return null; }
    const solid = obstacles.filter(o => !isTraversable(o) && !isDynamic(o));
    const key = JSON.stringify([target.x, target.y, solid]);
    if (key !== this.key) { this.reset(); this.key = key; }
    if (this.route && Math.abs(Math.atan2(Math.sin(pose.yaw - this.heading), Math.cos(pose.yaw - this.heading))) > .15) {
      this.route = null; this.retryAt = now;
    }
    if (waypointRouteClear(pose, target, solid)) { this.reset(); return null; }
    if (this.route && !waypointRouteClear(pose, this.route[0], obstacles, this.heading)) {
      this.route = null; this.retryAt = now + .5;
    }
    if (!this.route && now >= this.retryAt) {
      this.heading = pose.yaw;
      this.route = searchDetour(pose, target, solid);
      this.retryAt = now + 1;
    }
    if (!this.route) return { strategy: 'detour-blocked', heading: pose.yaw,
      command: sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 }) };
    while (this.route.length > 1 && Math.hypot(this.route[0].x - pose.x, this.route[0].y - pose.y) < .08) this.route.shift();
    const next = this.route[0];
    if (Math.hypot(next.x - pose.x, next.y - pose.y) < .07) {
      this.route = null; this.retryAt = now; return this.plan(pose, target, obstacles, now, enabled);
    }
    if (!waypointRouteClear(pose, next, obstacles, this.heading)) {
      return { strategy: 'detour-blocked', heading: this.heading, command: sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 }) };
    }
    const dx = next.x - pose.x, dy = next.y - pose.y;
    const c = Math.cos(this.heading), s = Math.sin(this.heading);
    return { strategy: 'detour', heading: this.heading, command: sanitizeCommand({
      vx: clamp((dx * c + dy * s) * 1.5, -.12, .28),
      vy: clamp((-dx * s + dy * c) * 1.5, -.22, .22), yawRate: 0 }) };
  }
}

export function searchDetour(pose, target, obstacles) {
  if (!waypointRouteClear(pose, pose, obstacles, pose.yaw)) return null;
  const distance = p => Math.hypot(target.x - p.x, target.y - p.y);
  const start = { ...pose, i: 0, j: 0, g: 0, f: distance(pose), parent: null };
  const open = [start], best = new Map([['0,0', 0]]);
  for (let count = 0; open.length && count < 1800; count++) {
    open.sort((a, b) => b.f - a.f);
    const p = open.pop();
    if (p.parent && waypointRouteClear(p, target, obstacles)) {
      const path = []; for (let n = p; n.parent; n = n.parent) path.unshift({ x: n.x, y: n.y });
      return path;
    }
    for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) {
      if (!di && !dj) continue;
      const i = p.i + di, j = p.j + dj;
      if (Math.abs(i) > 20 || Math.abs(j) > 20) continue;
      const q = { i, j, x: pose.x + i * .2, y: pose.y + j * .2 };
      const g = p.g + Math.hypot(di, dj) * .2, key = `${i},${j}`;
      if (g >= (best.get(key) ?? Infinity) || !waypointRouteClear(p, q, obstacles, pose.yaw)) continue;
      best.set(key, g); open.push({ ...q, g, f: g + distance(q), parent: p });
    }
  }
  return null;
}
