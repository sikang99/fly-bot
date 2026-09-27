import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { ArmWork, armAction, armWorkPose } from '../src/a2/arm-work.js';
import { ARM_STOW_POSE } from '../src/a2/config.js';

test('front and both side poses extend straight and finish folded; invalid actions are rejected', () => {
  assert.equal(armAction('invalid'), 'none');
  for (const [action, yaw] of [['front', 0], ['left', Math.PI / 2], ['right', -Math.PI / 2]]) {
    const work = new ArmWork(); work.start({ id: action, action }, 10);
    assert.equal(work.sample(10).stage, 'settling');
    assert.deepEqual(work.sample(14).pose, { yaw, shoulder: Math.PI / 2, elbow: 0, wrist: 0 });
    assert.equal(work.sample(17).done, true);
    assert.equal(work.sample(17).pose.shoulder, ARM_STOW_POSE.shoulderPitch);
    work.cancel(); assert.equal(work.sample(20).active, false);
    assert.deepEqual(work.sample(20).pose, armWorkPose());
  }
});

test('MuJoCo executes three arm waypoints stationary, folds, resumes and rewards once', async () => {
  let pose;
  const events = [];
  globalThis.postMessage = m => { if (m.type === 'pose') pose = m; if (m.type === 'waypoint') events.push(m); };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (let i = 0; i < 1500; i++) sim.step();
  const route = ['front', 'left', 'right', 'none'].map((action, i) => ({ id: `w${i}`, x: (i + 1) * .8, y: 0, action }));
  for (const waypoint of route) await send({ type: 'addWaypoint', waypoint });
  const stages = new Set(), actions = new Set();
  for (let i = 0; i < 60000; i++) {
    sim.step(); if (i % 10) continue; sim.postPose();
    assert.equal(pose.fault, null);
    if (pose.armWork.active) {
      actions.add(pose.armWork.action); stages.add(pose.armWork.stage);
      assert.equal(pose.mode, 'stand');
      assert.equal(pose.command.vx, 0); assert.equal(pose.command.vy, 0);
      const target = route.find(w => w.id === pose.armWork.id);
      assert.ok(Math.hypot(pose.base[0] - target.x, pose.base[1] - target.y) < .2);
    }
    if (pose.reward.total === 4 && !pose.navigation.active) break;
  }
  assert.deepEqual([...actions].sort(), ['front', 'left', 'right']);
  assert.equal(stages.size, 4); assert.equal(pose.reward.total, 4);
  assert.equal(pose.armWork.active, false); assert.equal(pose.mode, 'stand');
  assert.equal(events.filter(m => m.event === 'reached').length, 4);

  // Editing a queued point is effective; cancellation never consumes it.
  await send({ type: 'addWaypoint', waypoint: { id: 'cancel', x: pose.base[0], y: pose.base[1], action: 'none' } });
  await send({ type: 'waypointAction', id: 'cancel', action: 'right' });
  sim.step(); sim.postPose(); assert.equal(pose.armWork.action, 'right');
  await send({ type: 'waypointAction', id: 'cancel', action: 'left' });
  sim.step(); sim.postPose(); assert.equal(pose.armWork.action, 'right');
  await send({ type: 'emergencyStop' });
  sim.step(); sim.postPose(); assert.equal(pose.armWork.active, false); assert.equal(pose.mode, 'passive');
  assert.equal(events.filter(m => m.event === 'reached' && m.waypoint.id === 'cancel').length, 0);
  await send({ type: 'clearWaypoints' }); sim.step(); sim.postPose();
  assert.equal(pose.navigation.remaining, 0); assert.equal(pose.armWork.active, false);
});
