import { isDynamic } from './dynamic.js';
import { waypointRouteClear } from './avoidance.js';

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
