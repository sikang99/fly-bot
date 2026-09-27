// A2 Pro perception prototype. Inputs are already calibrated clusters/detections,
// not pixels or raw point clouds. Simulation adapter below uses world truth.
export const PERCEPTION_PROFILE = Object.freeze({ robot: 'A2 Pro', hz: 10, maxAge: .3, trackTTL: .5,
  horizon: 2, cameraHalfFov: 39 * Math.PI / 180, range: 8 });
const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));
const fresh = (stamp, now) => Number.isFinite(stamp) && stamp <= now + 1e-6 && now - stamp <= PERCEPTION_PROFILE.maxAge;
const finite = (...values) => values.every(Number.isFinite);
const point = (p, x, y) => ({ x: p.x + Math.cos(p.yaw) * x - Math.sin(p.yaw) * y,
  y: p.y + Math.sin(p.yaw) * x + Math.cos(p.yaw) * y });
const body = (p, x, y) => ({ x: Math.cos(p.yaw) * x + Math.sin(p.yaw) * y,
  y: -Math.sin(p.yaw) * x + Math.cos(p.yaw) * y });

// Continuous slab intersection, with body+foot-sweep envelope inflated by cluster radius.
export function collisionTime(x, y, vx, vy, radius, horizon = PERCEPTION_PROFILE.horizon) {
  let enter = 0, leave = horizon;
  for (const [position, velocity, extent] of [[x, vx, .56 + radius], [y, vy, .42 + radius]]) {
    if (Math.abs(velocity) < 1e-8) { if (Math.abs(position) > extent) return null; }
    else {
      const a = (-extent - position) / velocity, b = (extent - position) / velocity;
      enter = Math.max(enter, Math.min(a, b)); leave = Math.min(leave, Math.max(a, b));
      if (enter > leave) return null;
    }
  }
  return enter;
}

export class RiskTracker {
  constructor() { this.reset(); }
  reset() { this.tracks = []; this.previous = null; this.serial = 0; }
  update(frame, now = frame.stamp) {
    const p = frame.pose;
    if (!finite(now, frame.stamp, p?.x, p?.y, p?.yaw) || !fresh(frame.stamp, now)
      || !fresh(frame.lidar?.stamp, now) || !Array.isArray(frame.lidar?.clusters)
      || !frame.lidar.clusters.every(c => c && finite(c.x, c.y, c.radius) && c.radius > 0)
      || (this.previous && frame.stamp <= this.previous.stamp)) {
      return { source: frame.source, health: 'unknown', reason: 'LiDAR/pose stale, missing or out of order', tracks: [],
        front: { level: 'unknown', clearance: null, ttc: null }, rear: { level: 'unknown', clearance: null, ttc: null } };
    }
    const dt = this.previous ? frame.stamp - this.previous.stamp : 0;
    const ego = dt > 0 ? { vx: (p.x - this.previous.pose.x) / dt, vy: (p.y - this.previous.pose.y) / dt } : { vx: 0, vy: 0 };
    const cameraHealthy = ['front', 'rear'].every(side => fresh(frame.cameras?.[side]?.stamp, now)
      && Array.isArray(frame.cameras[side].detections));
    const labels = Object.values(frame.cameras || {}).filter(c => fresh(c?.stamp, now))
      .flatMap(c => Array.isArray(c.detections) ? c.detections : [])
      .filter(d => finite(d.bearing, d.confidence) && d.confidence >= .5 && ['person', 'car'].includes(d.kind));
    const remaining = new Set(this.tracks.filter(t => frame.stamp - t.stamp <= PERCEPTION_PROFILE.trackTTL));
    const next = [];
    for (const cluster of frame.lidar.clusters) {
      if (!finite(cluster.x, cluster.y, cluster.radius) || cluster.radius <= 0) continue;
      const world = point(p, cluster.x, cluster.y);
      let match = null, best = Infinity;
      for (const track of remaining) {
        const age = frame.stamp - track.stamp;
        const distance = Math.hypot(world.x - track.x - track.vx * age, world.y - track.y - track.vy * age);
        if (distance < best && distance < .4 + 4 * age) { match = track; best = distance; }
      }
      remaining.delete(match);
      const age = match ? frame.stamp - match.stamp : 0;
      const bearing = Math.atan2(cluster.y, cluster.x);
      // Class is optional. LiDAR geometry stays an obstacle even with no camera label.
      const candidates = labels.filter(d => Math.abs(wrap(d.bearing - bearing)) < .10
        && frame.lidar.clusters.filter(c => Math.abs(wrap(d.bearing - Math.atan2(c.y, c.x))) < .10).length === 1);
      const kind = candidates.length === 1 ? candidates[0].kind : 'unknown';
      next.push({ id: match?.id ?? ++this.serial, ...world, radius: cluster.radius, kind,
        vx: age > 0 ? (world.x - match.x) / age : 0, vy: age > 0 ? (world.y - match.y) / age : 0,
        velocityKnown: age > 0, stamp: frame.stamp, predicted: false });
    }
    // Briefly retain missing observations; never treat a dropped detection as instant clearance.
    for (const t of remaining) next.push({ ...t, kind: 'unknown', predicted: true });
    this.tracks = next;
    const tracks = next.map(t => {
      const age = frame.stamp - t.stamp;
      const relative = body(p, t.x + t.vx * age - p.x, t.y + t.vy * age - p.y);
      const velocity = body(p, t.vx - ego.vx, t.vy - ego.vy);
      const clearance = Math.max(0, Math.hypot(Math.max(0, Math.abs(relative.x) - .56), Math.max(0, Math.abs(relative.y) - .42)) - t.radius);
      const ttc = collisionTime(relative.x, relative.y, velocity.x, velocity.y, t.radius);
      const level = ttc !== null ? (ttc < .7 ? 'critical' : 'warning') : clearance < .5 ? 'near' : 'clear';
      return { id: t.id, kind: t.kind, ...relative, vx: velocity.x, vy: velocity.y, clearance, ttc, level,
        side: relative.x >= 0 ? 'front' : 'rear', predicted: t.predicted, velocityKnown: t.velocityKnown };
    });
    const rank = { clear: 0, near: 1, warning: 2, critical: 3 };
    const sector = side => {
      const list = tracks.filter(t => t.side === side);
      const threat = [...list].sort((a, b) => rank[b.level] - rank[a.level] || (a.ttc ?? Infinity) - (b.ttc ?? Infinity))[0];
      return { level: threat?.level ?? 'clear', ttc: threat?.ttc ?? null,
        clearance: list.length ? Math.min(...list.map(t => t.clearance)) : null,
        kind: threat?.kind ?? 'unknown', count: list.length };
    };
    this.previous = { stamp: frame.stamp, pose: { ...p } };
    return { source: frame.source, health: cameraHealthy ? 'ok' : 'degraded',
      reason: cameraHealthy ? '' : 'Camera missing/stale: geometry-only', tracks, front: sector('front'), rear: sector('rear') };
  }
}

export function simulatedSensorFrame(pose, obstacles, stamp) {
  const clusters = [], front = [], rear = [];
  for (const o of obstacles) {
    const r = body(pose, o.x - pose.x, o.y - pose.y);
    if (Math.hypot(r.x, r.y) > PERCEPTION_PROFILE.range) continue;
    clusters.push({ ...r, radius: Math.hypot(o.halfX, o.halfY) });
    const bearing = Math.atan2(r.y, r.x);
    if (!['person', 'car'].includes(o.kind)) continue;
    const detection = { bearing, confidence: 1, kind: o.kind };
    if (Math.abs(bearing) <= PERCEPTION_PROFILE.cameraHalfFov) front.push(detection);
    if (Math.abs(wrap(bearing - Math.PI)) <= PERCEPTION_PROFILE.cameraHalfFov) rear.push(detection);
  }
  return { source: 'simulation-ground-truth', stamp, pose, lidar: { stamp, clusters },
    cameras: { front: { stamp, detections: front }, rear: { stamp, detections: rear } } };
}
