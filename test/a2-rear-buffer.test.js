import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { rearBufferDeparture } from '../src/a2/dynamic-safety.js';
import { waypointRouteClear } from '../src/a2/avoidance.js';

const pose = { x: 0, y: 0, yaw: 0 }, target = { x: 3, y: 0 };
const rear = { id: 'rear', x: -.72, y: 0, halfX: .2, halfY: .3, halfZ: .2 };
test('rear buffer alone permits separation but not physical overlap, reversing, side/front blockage or moving threats', () => {
  assert.equal(waypointRouteClear(pose, target, [rear], 0), false);
  assert.equal(rearBufferDeparture(pose, target, [rear]), true);
  assert.equal(rearBufferDeparture(pose, target, [{ ...rear, x: -.64 }]), false);
  assert.equal(rearBufferDeparture(pose, { x: -3, y: 0 }, [rear]), false);
  assert.equal(rearBufferDeparture(pose, { x: 0, y: 3 }, [rear]), false);
  assert.equal(rearBufferDeparture(pose, target, [rear, { ...rear, id: 'front', x: .9 }]), false);
  assert.equal(rearBufferDeparture(pose, target, [{ ...rear, x: 0, y: .7 }]), false);
  assert.equal(rearBufferDeparture(pose, target, [{ ...rear, kind: 'person', vx: .6, vy: 0 }]), false);
  assert.equal(rearBufferDeparture({ x: 0, y: 0, yaw: Math.PI / 2 }, { x: 0, y: 3 },
    [{ ...rear, x: 0, y: -.72, halfX: .3, halfY: .2 }]), true);
});

test('MuJoCo departs a rear buffer and reaches the forward waypoint', async () => {
  let state;
  globalThis.postMessage = m => { if (m.type === 'pose') state = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (let i = 0; i < 1500; i++) sim.step();
  sim.postPose();
  await send({ type: 'editObstacle', action: 'create', obstacle: { ...rear, x: state.base[0] - .72, y: state.base[1] } });
  await send({ type: 'addWaypoint', waypoint: target });
  let seen = false;
  for (let i = 0; i < 60000; i++) {
    sim.step();
    if (i % 10 === 0) {
      sim.postPose(); assert.equal(state.fault, null);
      seen ||= state.avoidance.strategy === 'rear-buffer-departure';
      if (state.reward.total) break;
    }
  }
  assert.ok(seen);
  assert.equal(state.reward.total, 1, JSON.stringify(state.navigation));
});
