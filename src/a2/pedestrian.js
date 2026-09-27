import { obstacleWorldBounds } from './dynamic.js';
import { isTraversable } from './terrain.js';

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const approach = (v, goal, amount) => v + clamp(goal - v, -amount, amount);
export function pedestrianState(actor) {
  return { originX: actor.x, originY: actor.y, cruiseVx: actor.vx, cruiseVy: actor.vy,
    vx: actor.vx, vy: actor.vy, side: 0, active: false };
}

// Swept point vs expanded obstacles. Do not sidestep through a wall or another actor.
function clearSegment(actor, end, blockers) {
  return blockers.filter(o => o.id !== actor.id && !isTraversable(o)).map(obstacleWorldBounds).every(o => {
    let lo = 0, hi = 1;
    for (const [axis, radius] of [['x', actor.halfX], ['y', actor.halfY]]) {
      const half = axis === 'x' ? o.halfX : o.halfY;
      const delta = end[axis] - actor[axis], min = o[axis] - half - radius - .06, max = o[axis] + half + radius + .06;
      if (Math.abs(delta) < 1e-8) { if (actor[axis] < min || actor[axis] > max) return true; }
      else { const a = (min - actor[axis]) / delta, b = (max - actor[axis]) / delta;
        lo = Math.max(lo, Math.min(a, b)); hi = Math.min(hi, Math.max(a, b)); if (lo > hi) return true; }
    }
    return false;
  });
}

export function stepPedestrian(actor, state, robot, blockers, dt) {
  const speed = Math.hypot(state.cruiseVx, state.cruiseVy);
  if (speed < 1e-6) return { x: actor.x, y: actor.y, vx: 0, vy: 0, yielding: 'idle' };
  const ux = state.cruiseVx / speed, uy = state.cruiseVy / speed, nx = -uy, ny = ux;
  const dx = robot.x - actor.x, dy = robot.y - actor.y;
  const rvx = (robot.vx || 0) - state.cruiseVx, rvy = (robot.vy || 0) - state.cruiseVy;
  const v2 = rvx * rvx + rvy * rvy;
  const t = v2 > 1e-8 ? clamp(-(dx * rvx + dy * rvy) / v2, 0, 2.5) : 0;
  const distance = Math.hypot(dx, dy), closest = Math.hypot(dx + rvx * t, dy + rvy * t);
  const encounter = distance < 3 && closest < .85 + Math.max(actor.halfX, actor.halfY)
    && (dx * rvx + dy * rvy < -.02 || distance < .9);
  const passed = dx * ux + dy * uy < -1.1;
  // Keep the selected passing side until clear, avoiding reciprocal left/right oscillation.
  if (state.active && (passed || distance > 3.2)) { state.active = false; state.side = 0; }
  const offset = (actor.x - state.originX) * nx + (actor.y - state.originY) * ny;
  const robotOffset = (robot.x - state.originX) * nx + (robot.y - state.originY) * ny;
  const c = Math.cos(robot.yaw || 0), s = Math.sin(robot.yaw || 0);
  const width = Math.abs(nx * c + ny * s) * .56 + Math.abs(-nx * s + ny * c) * .42
    + Math.abs(nx) * actor.halfX + Math.abs(ny) * actor.halfY + .12;
  const targetOffset = side => clamp(robotOffset + side * width, -1.2, 1.2);
  if (!state.active && encounter && !passed) {
    const preferred = robotOffset > offset ? -1 : robotOffset < offset ? 1 : -1;
    for (const side of [preferred, -preferred]) {
      const shift = targetOffset(side) - offset;
      if (clearSegment(actor, { x: actor.x + nx * shift, y: actor.y + ny * shift }, blockers)) {
        state.active = true; state.side = side; break;
      }
    }
  }
  const target = state.active ? targetOffset(state.side) : 0;
  const lateral = clamp((target - offset) * 1.8, -.4, .4);
  const forwardScale = encounter && (!state.active || Math.abs(target - offset) > .2) ? .2 : 1;
  let vx = ux * speed * forwardScale + nx * lateral;
  let vy = uy * speed * forwardScale + ny * lateral;
  const total = Math.hypot(vx, vy), limit = Math.max(speed, .4);
  if (total > limit) { vx *= limit / total; vy *= limit / total; }
  state.vx = approach(state.vx, vx, .8 * dt); state.vy = approach(state.vy, vy, .8 * dt);
  let next = { x: actor.x + state.vx * dt, y: actor.y + state.vy * dt };
  const robotBlocker = { id: '__robot__', kind: 'box', x: robot.x, y: robot.y,
    halfX: Math.abs(c) * .56 + Math.abs(s) * .42, halfY: Math.abs(s) * .56 + Math.abs(c) * .42 };
  if (!clearSegment(actor, next, [...blockers, robotBlocker])) {
    // Kinematic people must not be slid through occupied space by the avoidance rule.
    state.vx = state.vy = 0; next = { x: actor.x, y: actor.y };
    return { ...next, vx: 0, vy: 0, yielding: 'waiting' };
  }
  return { ...next, vx: state.vx, vy: state.vy,
    yielding: state.active ? 'sidestep' : Math.abs(offset) > .05 ? 'returning' : 'walking' };
}
