import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

test('1m/s preview cruise remains stable for 60s and brakes at a waypoint', async () => {
  let pose;
  globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'command', command: { vx: 0.6, vy: 0, yawRate: 0, frequency: 1.4 } });
  await send({ type: 'mode', mode: 'walk' });
  let total = 0, count = 0, peak = 0;
  for (let i = 0; i < 30000; i++) {
    sim.step();
    if (i % 25 === 0) {
      sim.postPose(); assert.equal(pose.fault, null);
      assert.ok(pose.base[2] > 0.28); assert.ok(Math.abs(pose.orientation.pitch) < 0.2);
      peak = Math.max(peak, pose.forwardSpeed);
      if (i > 5000) { total += pose.forwardSpeed; count++; }
    }
  }
  assert.ok(total / count > 0.97 && total / count < 1.03, `mean ${total / count}`);
  assert.ok(peak < 1.05, `peak ${peak}`);
  await send({ type: 'reset' });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'addWaypoint', waypoint: { x: 10, y: 0 } });
  let fastSeen = false;
  for (let i = 0; i < 30000; i++) {
    sim.step();
    if (i % 25 === 0) {
      sim.postPose(); assert.equal(pose.fault, null);
      if (pose.forwardSpeed > 0.95) fastSeen = true;
      if (pose.reward.total) break;
    }
  }
  assert.ok(fastSeen); assert.equal(pose.reward.total, 1);
  assert.ok(Math.hypot(pose.base[0] - 10, pose.base[1]) < 0.062);
  assert.equal(pose.mode, 'stand');
});
