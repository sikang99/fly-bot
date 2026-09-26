import { A2_GAIT_PROFILE, A2_LIMITS, VELOCITY_ASSIST_PROFILE, clamp, sanitizeCommand } from './config.js';

// Arrival is measured from the base_link centre projected onto the ground,
// never from the nose, feet, swept footprint, or sensor cone.
export const WAYPOINT_RADIUS = 0.06;
export const WAYPOINT_REWARD = 1;
export const WAYPOINT_ALIGNMENT_TOLERANCE = 0.12;
export const WAYPOINT_BRAKING_ACCELERATION = VELOCITY_ASSIST_PROFILE.maxPlanarDeceleration;
export const WAYPOINT_PROGRESS_EPSILON = 0.02;
export const WAYPOINT_DIVERGENCE_LIMIT = 0.18;
export const WAYPOINT_STALL_TIMEOUT = 1.5;
export const WAYPOINT_RECOVERY_TOLERANCE = 0.18;

// Preview yaw assist only: leave walking joint commands and moving turns at
// their existing safe limits. Slow down as heading error approaches zero.
export function alignmentYawRate(error, stability = 1) {
  return clamp(error * 1.6, -0.70, 0.70) * clamp(stability, 0, 1);
}

export function assessWaypointProgress(distance) {
  const arrived = distance <= WAYPOINT_RADIUS;
  return { arrived, reward: arrived ? WAYPOINT_REWARD : 0 };
}

export function wrapAngle(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

export function waypointDistanceFromBodyCenter(pose, waypoint) {
  return Math.hypot(waypoint.x - pose.x, waypoint.y - pose.y);
}

// Follow the segment under the body centre, not a permanent diagonal shortcut
// from the current position to the destination. Projection never extends
// beyond either endpoint, and near arrival the true endpoint takes priority.
export function segmentAim(pose, start, end) {
  const dx = end.x - start.x, dy = end.y - start.y;
  const length = Math.hypot(dx, dy);
  if (length < 1e-6 || waypointDistanceFromBodyCenter(pose, end) < 0.45) return end;
  const projection = clamp(((pose.x - start.x) * dx + (pose.y - start.y) * dy) / length, 0, length);
  const along = Math.min(length, projection + 0.55);
  return { x: start.x + dx * along / length, y: start.y + dy * along / length };
}

export function planSegmentCommand(pose, start, end, options) {
  const endpoint = planWaypointCommand(pose, end, options);
  if (endpoint.arrived || endpoint.centering) return endpoint;
  const aimed = planWaypointCommand(pose, segmentAim(pose, start, end), options);
  return { ...aimed, arrived: false, distance: endpoint.distance };
}

export function waypointNeedsRecovery(distance, bestDistance, secondsWithoutProgress) {
  return distance > bestDistance + WAYPOINT_DIVERGENCE_LIMIT || secondsWithoutProgress > WAYPOINT_STALL_TIMEOUT;
}

export function planWaypointRecovery(headingError) {
  const aligned = Math.abs(headingError) <= WAYPOINT_RECOVERY_TOLERANCE;
  return {
    aligned,
    command: sanitizeCommand({ vx: 0, vy: 0,
      yawRate: aligned ? 0 : clamp(headingError * 0.8, -A2_LIMITS.maxYawRate, A2_LIMITS.maxYawRate), frequency: 1.4 }),
  };
}

export function planWaypointCommand(pose, waypoint, { tracking = false } = {}) {
  const dx = waypoint.x - pose.x, dy = waypoint.y - pose.y;
  const distance = waypointDistanceFromBodyCenter(pose, waypoint);
  const targetHeading = Math.atan2(dy, dx);
  const headingError = wrapAngle(targetHeading - pose.heading);
  if (distance <= WAYPOINT_RADIUS) {
    return { arrived: true, aligning: false, distance, headingError, command: sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 }) };
  }
  // A quadruped can centre over a nearby point without turning a full circle.
  // Keep its heading and use small body-frame corrections at the destination.
  if (tracking && distance < 0.45) {
    const forward = dx * Math.cos(pose.heading) + dy * Math.sin(pose.heading);
    const lateral = -dx * Math.sin(pose.heading) + dy * Math.cos(pose.heading);
    return { arrived: false, aligning: false, centering: true, distance, headingError: 0,
      command: sanitizeCommand({ vx: clamp(forward * 1.2, -0.22, 0.22),
        vy: clamp(lateral * 1.2, -0.20, 0.20), yawRate: 0, frequency: 1.4 }) };
  }
  // Do not let a waypoint behind the robot turn into a blind forward arc.  A
  // gentle in-place alignment also keeps the obstacle planner from reacting to
  // obstacles that are not on the requested route.
  // Every new segment acquires its heading before starting, including queued
  // waypoints. Only segment acquisition may stop based on angle alone. Re-entering an
  // in-place turn whenever gait yaw crosses a threshold is a deadlock risk;
  // the worker's progress watchdog handles genuine divergence instead.
  if (!tracking && Math.abs(headingError) > WAYPOINT_ALIGNMENT_TOLERANCE) {
    return {
      arrived: false,
      aligning: true,
      distance,
      headingError,
      command: sanitizeCommand({ vx: 0, vy: 0, yawRate: clamp(headingError * 0.8, -A2_LIMITS.maxYawRate, A2_LIMITS.maxYawRate), frequency: 1.4 }),
    };
  }
  // If tracking error becomes large, recover with a slow forward arc instead
  // of another stop-and-turn. The 35% floor stays useful to the pursuit
  // geometry without requesting the full cruise stride during a sharp turn.
  const alignment = Math.max(tracking ? 0.35 : 0.12, Math.cos(headingError));
  const remaining = Math.max(0, distance - WAYPOINT_RADIUS);
  // Cruise at the already verified stable gait limit while the route is clear,
  // then brake early enough for base_link (rather than the nose) to settle over
  // the waypoint without overshooting it.
  const brakingSpeed = Math.sqrt(2 * WAYPOINT_BRAKING_ACCELERATION * remaining)
    / VELOCITY_ASSIST_PROFILE.cruiseScale;
  // Account for the command filter and velocity-servo lag as well as ideal
  // braking distance. Otherwise a fast detour return overshoots a close goal.
  const approachSpeed = remaining * 1.5 / VELOCITY_ASSIST_PROFILE.cruiseScale;
  const vx = clamp(Math.min(brakingSpeed, approachSpeed), 0.08, A2_GAIT_PROFILE.cruiseSpeed) * alignment;
  const vy = clamp(distance * Math.sin(headingError) * 0.35, -0.28, 0.28);
  return {
    arrived: false,
    aligning: false,
    distance,
    headingError,
    command: sanitizeCommand({ vx, vy, yawRate: clamp(headingError * 0.65, -A2_LIMITS.maxYawRate, A2_LIMITS.maxYawRate), frequency: 1.4 }),
  };
}
