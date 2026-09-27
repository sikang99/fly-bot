import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { crossingActor, isDynamic, actorHeading, obstacleWorldBounds } from '../src/a2/dynamic.js';

test('car heading and avoidance bounds follow velocity, people remain unoriented', () => {
  for (const angle of [0, Math.PI / 2, Math.PI, -Math.PI / 4]) {
    const car = { kind: 'car', halfX: 0.9, halfY: 0.45, vx: Math.cos(angle), vy: Math.sin(angle) };
    assert.ok(Math.abs(actorHeading(car) - angle) < 1e-9);
    const bounds = obstacleWorldBounds(car);
    assert.ok(Math.abs(bounds.halfX - (Math.abs(car.vx) * .9 + Math.abs(car.vy) * .45)) < 1e-9);
    assert.equal(actorHeading({ ...car, kind: 'person' }), 0);
  }
});

test('random crossings require a segment at least 10m and spawn off the robot', () => {
  const pose = { x: 0, y: 0 }, start = { x: 0, y: 0 };
  assert.equal(crossingActor(pose, { x: 9.9, y: 0, segmentStart: start }), null);
  const actor = crossingActor(pose, { x: 10, y: 0, segmentStart: start }, () => 0.2);
  assert.equal(actor.kind, 'person'); assert.equal(Math.abs(actor.y), 3);
  assert.ok(Math.hypot(actor.x, actor.y) > 3);
});

test('moving actors can be created/deleted and long routes spawn and finish', async () => {
  let pose, bodyNames, creations = 0, waits = 0, yields = 0;
  globalThis.postMessage = m => { if (m.type === 'pose') pose = m; if (m.bodyNames) bodyNames = m.bodyNames; if (m.type === 'worldChanged' && m.automatic && m.selectedId) creations++; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (const kind of ['person', 'car']) {
    await send({ type: 'editObstacle', action: 'create', obstacle: { kind, x: 4, y: 4, halfX: .3, halfY: .3, halfZ: .5 } });
    const id = pose.obstacles.at(-1).id;
    for (let i = 0; i < 500; i++) sim.step();
    sim.postPose(); assert.ok(pose.obstacles.at(-1).y < 4);
    if (kind === 'car') {
      const actor = pose.obstacles.at(-1), index = bodyNames.indexOf(`obstacle_${id}`) * 4;
      const yaw = 2 * Math.atan2(pose.xquat[index + 3], pose.xquat[index]);
      assert.ok(Math.abs(yaw - actorHeading(actor)) < 1e-6, 'physics and travel headings differ');
      for (let i = 0; i < 3000; i++) sim.step();
      sim.postPose();
      assert.ok(pose.obstacles.at(-1).vy > 0, 'manual car reverses its route');
      const reversedYaw = 2 * Math.atan2(pose.xquat[index + 3], pose.xquat[index]);
      assert.ok(Math.abs(reversedYaw - actorHeading(pose.obstacles.at(-1))) < 1e-6);
    }
    await send({ type: 'editObstacle', action: 'delete', id });
    assert.ok(!pose.obstacles.some(o => o.id === id));
  }
  for (const cooperative of [false, true]) for (const seed of [12, 40, 9876]) {
    await send({ type: 'personYield', enabled: cooperative });
    await send({ type: 'randomActors', enabled: false });
    for (const o of pose.obstacles.filter(isDynamic)) await send({ type: 'editObstacle', action: 'delete', id: o.id });
    await send({ type: 'reset' });
    for (let i = 0; i < 1500; i++) sim.step();
    const previous = creations;
    await send({ type: 'randomActors', enabled: true, seed });
    await send({ type: 'addWaypoint', waypoint: { x: 15, y: 0 } });
    for (let i = 0; i < 60000; i++) {
      sim.step();
      if (i % 25 === 0) {
        sim.postPose(); assert.equal(pose.fault, null, `seed ${seed}`);
        if (pose.avoidance.strategy === 'dynamic-wait') waits++;
        if (pose.obstacles.some(o => o.yielding === 'sidestep')) yields++;
        if (pose.reward.total) break;
      }
    }
    assert.ok(creations > previous); assert.equal(pose.reward.total, 1, `seed ${seed}: ${JSON.stringify(pose.navigation)}`);
  }
  assert.ok(waits > 0);
  assert.ok(yields > 0, 'cooperative pedestrians should sidestep during long routes');
});
