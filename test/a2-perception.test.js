import test from 'node:test';
import assert from 'node:assert/strict';
import { RiskTracker, simulatedSensorFrame, collisionTime } from '../src/a2/perception.js';

const pose = { x: 0, y: 0, yaw: 0 };
const actor = (x, y = 0) => ({ x, y, halfX: .22, halfY: .22, kind: 'person' });
const frame = (t, x, p = pose) => simulatedSensorFrame(p, [actor(x)], t);

test('rear approaching person has estimated relative velocity and collision time', () => {
  const tracker = new RiskTracker();
  tracker.update(frame(0, -2));
  const risk = tracker.update(frame(.1, -1.9));
  assert.equal(risk.health, 'ok'); assert.equal(risk.rear.kind, 'person');
  assert.equal(risk.rear.level, 'warning'); assert.ok(risk.rear.ttc > .7 && risk.rear.ttc < 1.5);
  assert.ok(Math.abs(risk.tracks[0].vx - 1) < 1e-8);
});

test('ego motion is removed from world tracking and retained in relative collision prediction', () => {
  const tracker = new RiskTracker(); tracker.update(frame(0, -2));
  const risk = tracker.update(frame(.1, -2, { x: .1, y: 0, yaw: 0 }));
  assert.equal(risk.rear.ttc, null); assert.ok(risk.tracks[0].vx < -.9);
  assert.ok(Math.abs(tracker.tracks[0].vx) < 1e-8, 'stationary object stays stationary in world');
});

test('rotating robot does not make fixed obstacles into moving actors', () => {
  const tracker = new RiskTracker(); tracker.update(frame(0, 2));
  tracker.update(frame(.1, 2, { ...pose, yaw: .3 }));
  assert.ok(Math.abs(tracker.tracks[0].vx) < 1e-8);
  assert.ok(Math.abs(tracker.tracks[0].vy) < 1e-8);
});

test('camera loss keeps lidar risk but unknown identity, lidar loss never reports clear', () => {
  const tracker = new RiskTracker(); tracker.update(frame(0, -2));
  const input = frame(.1, -1.9); input.cameras.rear.stamp = -1;
  const risk = tracker.update(input);
  assert.equal(risk.health, 'degraded'); assert.equal(risk.rear.level, 'warning');
  assert.equal(risk.rear.kind, 'unknown');
  input.stamp = .2; input.lidar.stamp = -1;
  assert.equal(tracker.update(input).rear.level, 'unknown');
  assert.equal(tracker.update(frame(.1, -1.9)).health, 'unknown');
});

test('missing detections coast briefly then expire; camera cannot manufacture distance', () => {
  const tracker = new RiskTracker(); tracker.update(frame(0, -2)); tracker.update(frame(.1, -1.9));
  const empty = t => simulatedSensorFrame(pose, [], t);
  assert.equal(tracker.update(empty(.2)).tracks[0].predicted, true);
  assert.equal(tracker.update(empty(.7)).tracks.length, 0);
  const imageOnly = empty(.8); imageOnly.cameras.front.detections.push({ kind: 'car', confidence: .9, bearing: 0 });
  assert.equal(tracker.update(imageOnly).tracks.length, 0);
});

test('continuous intersection detects crossings and rejects diverging motion', () => {
  assert.equal(collisionTime(-2, 0, -1, 0, .2), null);
  assert.ok(collisionTime(0, 2, 0, -2, .2) < 1);
  assert.equal(collisionTime(0, 0, 0, 0, .2), 0);
});

test('malformed LiDAR never becomes clear and ambiguous image associations stay unknown', () => {
  const tracker = new RiskTracker();
  const input = frame(0, 2); input.lidar.clusters[0].x = NaN;
  assert.equal(tracker.update(input).health, 'unknown');
  const ambiguous = frame(.1, 2);
  ambiguous.lidar.clusters.push({ x: 4, y: 0, radius: .2 });
  assert.ok(tracker.update(ambiguous).tracks.every(t => t.kind === 'unknown'));
});
