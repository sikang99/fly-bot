import { A2_FOOTPRINT, waypointRouteClear } from './avoidance.js';
import { isTraversable } from './terrain.js';
import { clamp, sanitizeCommand } from './config.js';

function rotationClear(pose, heading, obstacles) {
  const delta = Math.atan2(Math.sin(heading - pose.yaw), Math.cos(heading - pose.yaw));
  for (let i = 0; i <= 24; i++) {
    if (!waypointRouteClear(pose, pose, obstacles, pose.yaw + delta * i / 24)) return false;
  }
  return true;
}

// Axis-aligned editor boxes define paired-wall passages. Keep the long axis
// along the overlap, with the normal swept-footprint clearance intact.
export function planPassage(pose, target, obstacles) {
  if (!target || Math.hypot(target.x - pose.x, target.y - pose.y) < 0.45) return null;
  const solid = obstacles.filter(o => !isTraversable(o));
  const passages = [];
  for (let i = 0; i < solid.length; i++) for (let j = i + 1; j < solid.length; j++) {
    for (const axis of ['x', 'y']) {
      const lateral = axis === 'x' ? 'y' : 'x';
      const half = axis === 'x' ? 'halfX' : 'halfY', sideHalf = axis === 'x' ? 'halfY' : 'halfX';
      const [a, b] = [solid[i], solid[j]].sort((u, v) => u[lateral] - v[lateral]);
      const low = a[lateral] + a[sideHalf], high = b[lateral] - b[sideHalf];
      const start = Math.max(a[axis] - a[half], b[axis] - b[half]);
      const end = Math.min(a[axis] + a[half], b[axis] + b[half]);
      const width = high - low, centre = (low + high) / 2;
      if (width < 2 * (A2_FOOTPRINT.halfWidth + A2_FOOTPRINT.margin) + 0.02 || width > 1.35 || end - start < 0.3) continue;
      if (pose[axis] < start - 1 || pose[axis] > end + 0.65 || Math.abs(pose[lateral] - centre) > width / 2 + 0.2) continue;
      const sign = target[axis] >= pose[axis] ? 1 : -1;
      if ((sign > 0 && pose[axis] > end + 0.56) || (sign < 0 && pose[axis] < start - 0.56)) continue;
      const heading = axis === 'x' ? (sign > 0 ? 0 : Math.PI) : sign * Math.PI / 2;
      passages.push({ axis, lateral, centre, heading, sign, width, score: Math.abs(pose[lateral] - centre) });
    }
  }
  const passage = passages.sort((a, b) => a.score - b.score)[0];
  if (!passage) return null;
  const error = Math.atan2(Math.sin(passage.heading - pose.yaw), Math.cos(passage.heading - pose.yaw));
  if (Math.abs(error) > 0.12 && !rotationClear(pose, passage.heading, obstacles)) {
    // Seek a nearby rotation pocket without rotating into either wall.
    for (const distance of [0.25, 0.5, 0.8, 1.1]) for (const sign of [-1, 1]) {
      const pocket = { x: pose.x + sign * Math.cos(pose.yaw) * distance, y: pose.y + sign * Math.sin(pose.yaw) * distance, yaw: pose.yaw };
      if (waypointRouteClear(pose, pocket, obstacles, pose.yaw) && rotationClear(pocket, passage.heading, obstacles)) {
        return { strategy: 'passage-retreat', heading: pose.yaw, command: sanitizeCommand({ vx: sign * 0.12, vy: 0, yawRate: 0 }) };
      }
    }
    return { strategy: 'passage-blocked', heading: pose.yaw, command: sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 }) };
  }
  if (Math.abs(error) > 0.12) return { strategy: 'passage-align', heading: passage.heading, command: sanitizeCommand({ vx: 0, vy: 0, yawRate: clamp(error, -0.35, 0.35) }) };
  const offset = passage.centre - pose[passage.lateral];
  const vy = clamp(offset * (passage.axis === 'x' ? passage.sign : -passage.sign), -0.12, 0.12);
  const next = { x: pose.x + Math.cos(passage.heading) * 0.35 - Math.sin(passage.heading) * vy,
    y: pose.y + Math.sin(passage.heading) * 0.35 + Math.cos(passage.heading) * vy };
  const safe = waypointRouteClear(pose, next, obstacles, passage.heading);
  return { strategy: safe ? 'passage' : 'passage-blocked', heading: passage.heading,
    command: sanitizeCommand({ vx: safe ? 0.24 : 0, vy: safe ? vy : 0, yawRate: 0 }) };
}
