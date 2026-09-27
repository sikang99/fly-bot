import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { searchDetour, detourEdgeClear } from '../src/a2/detour.js';
import { waypointRouteClear } from '../src/a2/avoidance.js';
import { worldObstacles } from '../src/a2/terrain.js';

test('observed diagonal corner pose escapes with whole swept body clearance', () => {
  const obstacles = worldObstacles('rough');
  for (const degrees of [-126, -121.6, -116]) for (const dx of [-.02, 0, .02]) {
    const pose = { x: .23 + dx, y: .71, yaw: degrees * Math.PI / 180 };
    const route = searchDetour(pose, { x: -4, y: 3 }, obstacles);
    assert.ok(route?.length, JSON.stringify(pose));
    let previous = pose;
    for (const next of route) {
      assert.ok(detourEdgeClear(previous, next, obstacles, pose.yaw));
      assert.ok(waypointRouteClear(previous, next, obstacles, pose.yaw, .02)); previous = next;
    }
    assert.equal(waypointRouteClear(pose, obstacles[0], obstacles, pose.yaw, .02), false);
  }
});
test('MuJoCo observed stuck pose and nearby headings resume and reach target', async () => {
  let pose; globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  const xml = fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8');
  for (const degrees of [-126, -121.6, -116]) {
    await send({ type: 'init', xml, terrain: 'rough' });
    for (let i = 0; i < 1500; i++) sim.step();
    sim.setPlanarPoseForTest(.23, .71, degrees * Math.PI / 180);
    await send({ type: 'addWaypoint', waypoint: { x: -4, y: 3 } });
    for (let i = 0; i < 60000; i++) {
      sim.step(); if (i % 25) continue; sim.postPose();
      assert.equal(pose.fault, null); assert.equal(pose.recovery, null);
      if (pose.reward.total) break;
    }
    assert.equal(pose.reward.total, 1, JSON.stringify({ degrees, base: pose.base, nav: pose.navigation, avoidance: pose.avoidance }));
  }
});
