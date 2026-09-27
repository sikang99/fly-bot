import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { isTraversable } from '../src/a2/terrain.js';
import { LocalDetour } from '../src/a2/detour.js';

test('low steps never alter a high-obstacle detour and height threshold remains conservative', () => {
  const low = { id: 'low', kind: 'step', x: 1.5, y: .8, halfX: .45, halfY: .7, halfZ: .01 };
  const high = { id: 'high', x: .85, y: 0, halfX: .16, halfY: .28, halfZ: .16 };
  const pose = { x: 0, y: 0, yaw: 0 }, goal = { x: 5, y: 0 };
  assert.ok(isTraversable(low));
  assert.equal(isTraversable({ ...low, halfZ: .0101 }), false);
  const a = new LocalDetour(), b = new LocalDetour();
  assert.deepEqual(a.plan(pose, goal, [high], 0, true), b.plan(pose, goal, [high, low], 0, true));
  assert.deepEqual(a.route, b.route);
  assert.equal(new LocalDetour().plan(pose, goal, [low], 0, true), null);
});

test('MuJoCo crosses 1cm and 2cm steps without obstacle avoidance', async t => {
  const sim = await import('../src/a2/a2.worker.js');
  const xml = fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8');
  for (const height of [.01, .02]) await t.test(`${height}m`, async () => {
    let pose;
    globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
    const send = data => globalThis.onmessage({ data });
    await send({ type: 'init', xml });
    await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
    await send({ type: 'editObstacle', action: 'create', obstacle: {
      kind: 'step', x: 1.5, y: 0, halfX: .45, halfY: .7, halfZ: height / 2 } });
    for (let i = 0; i < 1500; i++) sim.step();
    await send({ type: 'addWaypoint', waypoint: { x: 4, y: 0 } });
    let crossed = false;
    for (let i = 0; i < 60000; i++) {
      sim.step();
      if (i % 25) continue;
      sim.postPose();
      assert.equal(pose.fault, null);
      assert.equal(pose.avoidance.active, false, JSON.stringify(pose.avoidance));
      assert.ok(Math.abs(pose.base[1]) < .2);
      crossed ||= pose.base[0] > 1.4 && pose.base[0] < 1.6;
      if (pose.reward.total) break;
    }
    assert.ok(crossed);
    assert.equal(pose.reward.total, 1, JSON.stringify({ base: pose.base, nav: pose.navigation }));
  });
});
