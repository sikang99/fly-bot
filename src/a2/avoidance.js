import { A2_OBSTACLES, sanitizeCommand } from './config.js';
import { isTraversable } from './terrain.js';
import { actorHeading, obstacleWorldBounds } from './dynamic.js';

// Includes the body, hips and the normal foot sweep, not just base_link.
export const A2_FOOTPRINT = Object.freeze({ halfLength: 0.46, halfWidth: 0.32, margin: 0.10, bypassMargin: 0.08, lookAhead: 1.65 });

export function obstacleRelativeGeometry(pose, obstacle) {
  const dx = obstacle.x - pose.x, dy = obstacle.y - pose.y;
  const forward = Math.cos(pose.yaw) * dx + Math.sin(pose.yaw) * dy;
  const lateral = -Math.sin(pose.yaw) * dx + Math.cos(pose.yaw) * dy;
  const c = Math.abs(Math.cos(pose.yaw - actorHeading(obstacle))), s = Math.abs(Math.sin(pose.yaw - actorHeading(obstacle)));
  const obstacleForwardRadius = c * obstacle.halfX + s * obstacle.halfY;
  const obstacleLateralRadius = s * obstacle.halfX + c * obstacle.halfY;
  const clearance = forward - obstacleForwardRadius - A2_FOOTPRINT.halfLength - A2_FOOTPRINT.margin;
  const lateralClearance = Math.abs(lateral) - obstacleLateralRadius - A2_FOOTPRINT.halfWidth - A2_FOOTPRINT.margin;
  const behind = forward + obstacleForwardRadius < -A2_FOOTPRINT.halfLength - A2_FOOTPRINT.margin;
  return { forward, lateral, clearance, lateralClearance, behind };
}

// Segment/AABB test with a rotation-safe body envelope. Used to release a
// detour only when the complete route to the current target is unobstructed.
export function waypointRouteClear(pose, target, obstacles, fixedYaw = null) {
  const radius = Math.hypot(A2_FOOTPRINT.halfLength, A2_FOOTPRINT.halfWidth) + A2_FOOTPRINT.margin;
  return obstacles.filter(o => !isTraversable(o)).map(obstacleWorldBounds).every(o => {
    let lo = 0, hi = 1;
    for (const [axis, half] of [['x', o.halfX], ['y', o.halfY]]) {
      const delta = target[axis] - pose[axis];
      const c = Math.abs(Math.cos(fixedYaw ?? 0)), s = Math.abs(Math.sin(fixedYaw ?? 0));
      const extent = fixedYaw === null ? radius : A2_FOOTPRINT.margin + (axis === 'x'
        ? c * A2_FOOTPRINT.halfLength + s * A2_FOOTPRINT.halfWidth
        : s * A2_FOOTPRINT.halfLength + c * A2_FOOTPRINT.halfWidth);
      const min = o[axis] - half - extent, max = o[axis] + half + extent;
      if (Math.abs(delta) < 1e-9) { if (pose[axis] < min || pose[axis] > max) return true; }
      else {
        const a = (min - pose[axis]) / delta, b = (max - pose[axis]) / delta;
        lo = Math.max(lo, Math.min(a, b)); hi = Math.min(hi, Math.max(a, b));
        if (lo > hi) return true;
      }
    }
    return false;
  });
}

export function planNearbyDocking(pose, target, obstacles) {
  if (!target || Math.hypot(target.x - pose.x, target.y - pose.y) > 1.2) return null;
  if (!obstacles.some(o => !isTraversable(o) && Math.hypot(o.x - target.x, o.y - target.y) < 1.5)) return null;
  if (!waypointRouteClear(pose, target, obstacles, pose.yaw)) return null;
  const dx = target.x - pose.x, dy = target.y - pose.y;
  const forward = dx * Math.cos(pose.yaw) + dy * Math.sin(pose.yaw);
  const lateral = -dx * Math.sin(pose.yaw) + dy * Math.cos(pose.yaw);
  const sidePoint = { x: pose.x - Math.sin(pose.yaw) * lateral, y: pose.y + Math.cos(pose.yaw) * lateral };
  const lateralFirst = Math.abs(lateral) > 0.12 && waypointRouteClear(pose, sidePoint, obstacles, pose.yaw);
  return sanitizeCommand({ vx: lateralFirst ? 0 : Math.max(-0.12, Math.min(0.12, forward * 1.2)),
    vy: Math.max(-0.22, Math.min(0.22, lateral * 1.2)), yawRate: 0, frequency: 1.4 });
}

export function planObstacleAvoidance(pose, requested, obstacles = A2_OBSTACLES, target = null) {
  const command = sanitizeCommand(requested);
  let nearest = null;
  for (const obstacle of obstacles) {
    if (isTraversable(obstacle)) continue;
    const { forward, lateral, clearance, lateralClearance, behind } = obstacleRelativeGeometry(pose, obstacle);
    const c = Math.abs(Math.cos(pose.yaw - actorHeading(obstacle))), s = Math.abs(Math.sin(pose.yaw - actorHeading(obstacle)));
    const obstacleForwardRadius = c * obstacle.halfX + s * obstacle.halfY;
    const ahead = forward + obstacleForwardRadius > 0.10;
    const blocked = ahead && !behind && forward - obstacleForwardRadius < A2_FOOTPRINT.lookAhead && lateralClearance < A2_FOOTPRINT.bypassMargin;
    if (blocked && (!nearest || clearance < nearest.clearance)) {
      nearest = { obstacle, forward, lateral, clearance, lateralClearance };
    }
  }

  if (command.vx <= 0.05 || !nearest) {
    return { active: false, direction: 0, clearance: nearest?.clearance ?? Infinity, obstacle: nearest?.obstacle.id ?? null, command };
  }
  let direction = nearest.lateral <= 0 ? 1 : -1;
  if (target) {
    // Prefer the side whose clearance point has the shortest remaining route,
    // rather than choosing a side solely from the obstacle's tiny offset.
    const sideRadius = Math.abs(Math.sin(pose.yaw - actorHeading(nearest.obstacle))) * nearest.obstacle.halfX
      + Math.abs(Math.cos(pose.yaw - actorHeading(nearest.obstacle))) * nearest.obstacle.halfY
      + A2_FOOTPRINT.halfWidth + A2_FOOTPRINT.margin + A2_FOOTPRINT.bypassMargin;
    const cost = sign => {
      const shift = nearest.lateral + sign * sideRadius;
      const point = { x: pose.x - Math.sin(pose.yaw) * shift, y: pose.y + Math.cos(pose.yaw) * shift };
      return Math.abs(shift) + Math.hypot(target.x - point.x, target.y - point.y);
    };
    if (Math.abs(cost(1) - cost(-1)) > 0.05) direction = cost(1) < cost(-1) ? 1 : -1;
  }
  const urgency = 1 - Math.max(0, Math.min(1, (nearest.clearance + 0.1) / 0.9));
  const collisionRisk = nearest.clearance < 0.35 && nearest.lateralClearance < 0.05;
  const strategy = collisionRisk ? 'sidestep' : 'arc';
  const speed = Math.min(command.vx, collisionRisk ? 0 : 0.30 + 0.14 * (1 - urgency));
  const lateralSpeed = direction * (collisionRisk ? 0.30 + 0.18 * urgency : 0.06);
  const yawRate = collisionRisk ? 0 : direction * (0.18 + 0.10 * urgency);
  return {
    active: true,
    strategy,
    direction,
    clearance: nearest.clearance,
    lateralClearance: nearest.lateralClearance,
    obstacle: nearest.obstacle.id,
    collisionRisk,
    command: sanitizeCommand({ ...command, vx: speed, vy: lateralSpeed, yawRate }),
  };
}
