import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { rearEscapeSafe } from '../src/a2/dynamic-safety.js';

test('rear approaching actor causes escape only when forward space is safe', () => {
  const pose = { x: 0, y: 0, yaw: 0 };
  const person = { kind: 'person', x: -1.5, y: 0, vx: .6, vy: 0, halfX: .22, halfY: .22 };
  assert.equal(rearEscapeSafe(pose, [person]), true);
  assert.equal(rearEscapeSafe(pose, [{ ...person, vx: -.6 }]), false);
  assert.equal(rearEscapeSafe(pose, [person, { kind: 'box', x: .9, y: 0, halfX: .2, halfY: .3 }]), false);
});

test('rear encounter escapes without falling; assisted side recovery preserves route and obeys cancellation', async () => {
  let pose;
  globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  // Retain the adversarial non-cooperative pedestrian regression: robot safety
  // must not depend on a person choosing to yield.
  await send({ type: 'personYield', enabled: false });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'editObstacle', action: 'create', obstacle: { kind: 'person', x: -1.5, y: 0, vx: .6, vy: 0, halfX: .22, halfY: .22, halfZ: .85 } });
  const personId = pose.obstacles.at(-1).id;
  await send({ type: 'addWaypoint', waypoint: { x: 5, y: 0 } });
  let escaped = false, rearRiskObserved = false;
  for (let i = 0; i < 20000; i++) {
    sim.step();
    if (i % 25 === 0) { sim.postPose(); assert.equal(pose.fault, null); assert.equal(pose.recovery, null);
      if (pose.avoidance.strategy === 'dynamic-escape') escaped = true;
      if (['warning', 'critical'].includes(pose.perception?.rear?.level)) rearRiskObserved = true;
      if (pose.reward.total) break; }
  }
  assert.ok(escaped); assert.ok(rearRiskObserved, 'rear risk must be published by the perception tracker'); assert.equal(pose.reward.total, 1);
  await send({ type: 'editObstacle', action: 'delete', id: personId });
  await send({ type: 'reset' });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'addWaypoint', waypoint: { x: 5, y: 0 } });
  sim.simulateSideFallForTest();
  let recovered = false;
  for (let i = 0; i < 2500; i++) { sim.step(); if (i % 25 === 0) { sim.postPose(); if (pose.recovery) recovered = true; } }
  assert.ok(recovered); assert.equal(pose.recovery, null); assert.equal(pose.fault, null);
  assert.ok(Math.abs(pose.orientation.roll) < .2); assert.equal(pose.navigation.remaining, 1);
  sim.simulateSideFallForTest(); sim.step();
  await send({ type: 'mode', mode: 'passive' });
  sim.step(); sim.postPose(); assert.equal(pose.recovery, null); assert.equal(pose.mode, 'passive');
  await send({ type: 'reset' });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 0.8, y: 0 });
  sim.simulateSideFallForTest(); sim.step();
  for (let i = 0; i < 250; i++) sim.step();
  sim.postPose(); assert.equal(pose.recovery, 'waiting');
  assert.ok(Math.abs(pose.orientation.roll) > 0.6, 'must not right into an obstacle');
  await send({ type: 'autoRecover', enabled: false });
  sim.step(); sim.postPose(); assert.equal(pose.recovery, null); assert.equal(pose.mode, 'passive');
});
