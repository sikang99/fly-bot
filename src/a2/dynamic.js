export const isDynamic = o => o.kind === 'person' || o.kind === 'car';
export function actorHeading(o) {
  return o.kind === 'car' ? Math.atan2(o.vy || 0, o.vx || 0) : 0;
}
export function obstacleWorldBounds(o) {
  const yaw = actorHeading(o), c = Math.abs(Math.cos(yaw)), s = Math.abs(Math.sin(yaw));
  return { ...o, halfX: c * o.halfX + s * o.halfY, halfY: s * o.halfX + c * o.halfY };
}
export function actorShape(kind) {
  return kind === 'person' ? { halfX: 0.22, halfY: 0.22, halfZ: 0.85 }
    : { halfX: 0.9, halfY: 0.45, halfZ: 0.65 };
}
export function crossingActor(pose, target, random = Math.random) {
  if (!target?.segmentStart) return null;
  const start = target.segmentStart;
  if (Math.hypot(target.x - start.x, target.y - start.y) < 10) return null;
  const dx = target.x - pose.x, dy = target.y - pose.y, distance = Math.hypot(dx, dy);
  if (distance < 5) return null;
  const ux = dx / distance, uy = dy / distance;
  const kind = random() < 0.5 ? 'person' : 'car', side = random() < 0.5 ? -1 : 1;
  const ahead = Math.min(distance - 1, 3.5 + random() * 2), speed = kind === 'person' ? 0.6 : 1.2;
  return { kind, ...actorShape(kind), x: pose.x + ux * ahead - uy * side * 3,
    y: pose.y + uy * ahead + ux * side * 3, vx: uy * side * speed, vy: -ux * side * speed };
}
