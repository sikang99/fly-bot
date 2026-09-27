import { isDynamic } from './dynamic.js';
import { A2_FOOTPRINT, waypointRouteClear } from './avoidance.js';
import { actorHeading } from './dynamic.js';
import { isTraversable } from './terrain.js';

// Only release the extra rear planning buffer, never the body/foot envelope.
// Keep a fixed heading: this exception does not authorize turning or reversing.
export function rearBufferDeparture(pose, target, obstacles) {
  if (!target) return false;
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  if ((target.x - pose.x) * c + (target.y - pose.y) * s < .25) return false;
  const end = { x: pose.x + c * .4, y: pose.y + s * .4 };
  let released = false;
  for (const o of obstacles) {
    if (isTraversable(o)) continue;
    if (isDynamic(o)) {
      if (![0, .5, 1, 2].every(t => waypointRouteClear(pose, end,
        [{ ...o, x: o.x + (o.vx || 0) * t, y: o.y + (o.vy || 0) * t }], pose.yaw))) return false;
      continue;
    }
    if (waypointRouteClear(pose, end, [o], pose.yaw)) continue;
    const forward = (o.x - pose.x) * c + (o.y - pose.y) * s;
    const relative = pose.yaw - actorHeading(o);
    const radius = Math.abs(Math.cos(relative)) * o.halfX + Math.abs(Math.sin(relative)) * o.halfY;
    if (forward + radius >= -A2_FOOTPRINT.halfLength - .02
      || !waypointRouteClear(pose, end, [o], pose.yaw, .02)) return false;
    released = true;
  }
  return released;
}

export function rearEscapeSafe(pose, obstacles, forwardSpeed = 0) {
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  const moving = obstacles.filter(isDynamic);
  const threat = moving.some(o => {
    const dx = o.x - pose.x, dy = o.y - pose.y;
    const forward = dx * c + dy * s;
    const closing = (o.vx || 0) * c + (o.vy || 0) * s;
    return forward < 0.1 && forward > -2.5 && closing > 0.1
      && [0.5, 1, 1.5].some(t => !waypointRouteClear(pose, pose,
        [{ ...o, x: o.x + o.vx * t, y: o.y + o.vy * t }], pose.yaw));
  });
  if (!threat) return false;
  const end = { x: pose.x + c * 1.5, y: pose.y + s * 1.5 };
  if (!waypointRouteClear(pose, end, obstacles.filter(o => !isDynamic(o)), pose.yaw)) return false;
  // Accelerate from rest at the escape servo's bounded acceleration. Do not
  // escape into another actor or assume an instantaneous speed change.
  return [0.25, 0.5, 0.75, 1, 1.5].every(t => {
    const distance = Math.min(1.0 * t, Math.max(0, forwardSpeed) * t + 0.3 * t * t);
    const future = { x: pose.x + c * distance, y: pose.y + s * distance };
    return waypointRouteClear(future, future, moving.map(o => ({ ...o,
      x: o.x + o.vx * t, y: o.y + o.vy * t })), pose.yaw);
  });
}
