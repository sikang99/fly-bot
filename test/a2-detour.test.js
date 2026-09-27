import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { LocalDetour, searchDetour, detourEdgeClear } from '../src/a2/detour.js';
import { worldObstacles } from '../src/a2/terrain.js';
import { waypointRouteClear } from '../src/a2/avoidance.js';

const boxes = [
  { id: 'front', x: 1.1, y: -.1, halfX: .3, halfY: .4, halfZ: .2 },
  { id: 'side', x: .1, y: 1.25, halfX: .6, halfY: .25, halfZ: .2 },
];
test('live-screen pose can leave overlapping buffer without reducing body clearance', () => {
  const pose = { x: .14, y: .37, yaw: 0 }, obstacles = worldObstacles('rough');
  assert.equal(waypointRouteClear(pose, pose, obstacles, 0), false);
  const route = searchDetour(pose, { x: 5, y: 0 }, obstacles);
  assert.ok(route?.length);
  let previous = pose;
  for (const next of route) {
    assert.ok(detourEdgeClear(previous, next, obstacles, 0));
    assert.ok(waypointRouteClear(previous, next, obstacles, 0, .02));
    previous = next;
  }
  assert.equal(detourEdgeClear(pose, { x: .24, y: .37 }, obstacles, 0), false);
  assert.equal(detourEdgeClear(pose, { x: -.06, y: .37 }, obstacles, 0), true);
  assert.equal(detourEdgeClear(pose, { x: -.06, y: .37 },
    [{ ...obstacles[0], kind: 'person', vx: 0, vy: 0 }], 0), false);
});
test('crowded detour keeps every edge clear and exits toward waypoint without ping-pong', () => {
  for (const sign of [-1, 1]) {
    const obstacles = boxes.map(o => ({ ...o, y: o.y * sign }));
    const target = { x: 4, y: 0 }, pose = { x: 0, y: 0, yaw: 0 };
    const planner = new LocalDetour();
    let done = false;
    for (let step = 0; step < 1800; step++) {
      const plan = planner.plan(pose, target, obstacles, step * .05, true);
      if (!plan) { done = true; break; }
      assert.notEqual(plan.strategy, 'detour-blocked');
      const next = { ...pose, x: pose.x + plan.command.vx * .05, y: pose.y + plan.command.vy * .05 };
      assert.equal(waypointRouteClear(pose, next, obstacles, 0), true);
      assert.equal(plan.command.yawRate, 0);
      Object.assign(pose, next);
    }
    assert.ok(done, JSON.stringify(pose));
    assert.equal(waypointRouteClear(pose, target, obstacles), true);
  }
});
test('blocked/overlapping start is not forced through an obstacle; low step is traversable', () => {
  const pose = { x: 0, y: 0, yaw: 0 }, target = { x: 4, y: 0 };
  assert.equal(searchDetour(pose, target, [{ ...boxes[0], x: 0 }]), null);
  const planner = new LocalDetour();
  assert.equal(planner.plan(pose, target, [{ ...boxes[0], kind: 'step', halfZ: .01 }], 0, true), null);
  assert.equal(planner.plan(pose, target, [{ ...boxes[0], x: 0 }], 1, true).strategy, 'detour-blocked');
  assert.equal(planner.plan(pose, target, [], 2, true), null);
});

test('MuJoCo crowded two-box detour reaches the body-centred waypoint', async () => {
  let pose;
  globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (const obstacle of boxes) await send({ type: 'editObstacle', action: 'create', obstacle });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'addWaypoint', waypoint: { x: 4, y: 0 } });
  let detourSeen = false;
  for (let i = 0; i < 90000; i++) {
    sim.step();
    if (i % 25 === 0) {
      sim.postPose(); assert.equal(pose.fault, null);
      detourSeen ||= pose.avoidance.strategy === 'detour';
      if (pose.reward.total) break;
    }
  }
  assert.ok(detourSeen);
  assert.equal(pose.reward.total, 1, JSON.stringify({ nav: pose.navigation, base: pose.base, avoidance: pose.avoidance }));
  assert.ok(Math.hypot(pose.base[0] - 4, pose.base[1]) < .2);
});

test('MuJoCo default rough terrain clears the initial obstacle and reaches 5m goal', async () => {
  let pose;
  globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', terrain: 'rough', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'addWaypoint', waypoint: { x: 5, y: 0 } });
  for (let i = 0; i < 90000; i++) {
    sim.step();
    if (i % 25 === 0) {
      sim.postPose(); assert.equal(pose.fault, null);
      if (pose.reward.total) break;
    }
  }
  assert.equal(pose.reward.total, 1, JSON.stringify({ base: pose.base, command: pose.command, nav: pose.navigation, avoidance: pose.avoidance }));
});
