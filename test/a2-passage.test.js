import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { planPassage } from '../src/a2/passage.js';

const walls = [{ id: 'left', x: -0.8, y: 1.7, halfX: 0.3, halfY: 1, halfZ: 0.3 },
  { id: 'right', x: 0.8, y: 1.7, halfX: 0.3, halfY: 1, halfZ: 0.3 }];
test('narrow passage aligns the long axis before entering and rejects unsafe rotation', () => {
  const plan = planPassage({ x: 0, y: 0, yaw: 0 }, { x: 0, y: 4 }, walls);
  assert.equal(plan.strategy, 'passage-align');
  assert.equal(plan.command.vx, 0);
  assert.equal(plan.heading, Math.PI / 2);
  assert.equal(planPassage({ x: 0, y: 1.5, yaw: 0 }, { x: 0, y: 4 }, walls).strategy, 'passage-blocked');
  const forward = planPassage({ x: 0, y: 1.5, yaw: Math.PI / 2 }, { x: 0, y: 4 }, walls);
  assert.equal(forward.strategy, 'passage'); assert.ok(forward.command.vx > 0);
});

test('MuJoCo enters and exits a 1m-wide passage lengthwise', async () => {
  let pose;
  globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (const obstacle of walls) await send({ type: 'editObstacle', action: 'create', obstacle });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'addWaypoint', waypoint: { x: 0, y: 4 } });
  let passageSeen = false;
  for (let i = 0; i < 60000; i++) {
    sim.step();
    if (i % 25 === 0) {
      sim.postPose(); assert.equal(pose.fault, null);
      if (pose.base[1] > 1 && pose.base[1] < 2.5) {
        passageSeen = true;
        assert.ok(Math.abs(pose.base[0]) < 0.1, `lateral ${pose.base[0]}`);
        assert.ok(Math.abs(pose.orientation.yaw - Math.PI / 2) < 0.16, `yaw ${pose.orientation.yaw}`);
      }
      if (pose.reward.total) break;
    }
  }
  assert.ok(passageSeen); assert.equal(pose.reward.total, 1, JSON.stringify(pose.navigation));
});
