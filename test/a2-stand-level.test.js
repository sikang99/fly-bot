import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';

test('Stand levels the body at startup and after forward, sideways and turning walks', async () => {
  let pose; globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  const settle = () => {
    for (let i = 0; i < 5000; i++) sim.step();
    sim.postPose();
    assert.equal(pose.mode, 'stand'); assert.equal(pose.fault, null);
    assert.ok(Math.abs(pose.orientation.pitch) < Math.PI / 180, `pitch ${pose.orientation.pitch}`);
    assert.ok(Math.abs(pose.orientation.roll) < Math.PI / 180, `roll ${pose.orientation.roll}`);
    assert.ok(pose.base[2] > .30);
  };
  settle();
  for (const command of [{ vx: .6, vy: 0, yawRate: 0 }, { vx: 0, vy: .3, yawRate: 0 }, { vx: 0, vy: 0, yawRate: .3 }]) {
    await send({ type: 'command', command, avoidance: false });
    await send({ type: 'mode', mode: 'walk' });
    for (let i = 0; i < 1500; i++) sim.step();
    await send({ type: 'mode', mode: 'stand' }); settle();
  }
});
