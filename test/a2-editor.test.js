import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

test('route editor keeps robot stopped, edits queue and requires explicit Walk after leaving', async () => {
  let pose; const reached = [];
  globalThis.postMessage = m => { if (m.type === 'pose') pose = m; if (m.type === 'waypoint' && m.event === 'reached') reached.push(m.waypoint.id); };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'routeEditor', enabled: true });
  await send({ type: 'addWaypoint', waypoint: { id: 'discard', x: 3, y: 0 } });
  await send({ type: 'mode', mode: 'walk' });
  for (let i = 0; i < 1000; i++) sim.step();
  sim.postPose(); assert.equal(pose.mode, 'stand'); assert.equal(pose.command.vx, 0); assert.equal(pose.reward.total, 0);
  await send({ type: 'editRoute', waypoints: [{ id: 'first', x: .8, y: 0, action: 'front' }, { id: 'second', x: 1.6, y: 0 }] });
  await send({ type: 'editRoute', waypoints: [{ id: 'bad', x: NaN, y: 0 }] });
  await send({ type: 'routeEditor', enabled: false });
  sim.step(); sim.postPose(); assert.equal(pose.mode, 'stand'); assert.equal(pose.navigation.remaining, 2);
  await send({ type: 'mode', mode: 'walk' });
  for (let i = 0; i < 40000 && reached.length < 2; i++) sim.step();
  sim.postPose(); assert.deepEqual(reached, ['first', 'second']); assert.equal(pose.reward.total, 2); assert.equal(pose.fault, null);
});
