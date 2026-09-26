import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { WAYPOINT_RADIUS } from '../src/a2/navigation.js';

// Run the actual browser worker, controller, MJCF and MuJoCo step. No ideal
// kinematic substitute: this includes avoidance, recovery and velocity assist.
test('MuJoCo waypoint routes: distances, bearings, detours and queue transitions', async () => {
  let pose, reached;
  globalThis.postMessage = message => {
    if (message.type === 'pose') pose = message;
    if (message.type === 'waypoint') reached.push(message);
  };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8'), terrain: 'flat' });

  async function run(name, targets, obstacle = { x: 15, y: 15 }) {
    reached = [];
    await send({ type: 'reset' });
    await send({ type: 'moveObstacle', id: 'demo_box', ...obstacle });
    for (let i = 0; i < 1500; i++) sim.step();
    for (const waypoint of targets) await send({ type: 'addWaypoint', waypoint });
    let checked = 0, maxYaw = 0, maxDeviation = 0, travel = 0, previous = null, alignmentSeconds = 0, dockingSamples = 0;
    for (let i = 0; i < 60000; i++) {
      sim.step();
      if (i % 25 === 0 || reached.length > checked) {
        sim.postPose();
        if (pose.avoidance.strategy === 'docking') {
          dockingSamples++;
          assert.equal(pose.command.yawRate, 0, `${name}: docking rotation`);
          assert.ok(Math.abs(pose.command.vx) <= 0.12 && Math.abs(pose.command.vy) <= 0.22);
        }
        if (pose.navigation.aligning) alignmentSeconds += 0.05;
        if (previous) travel += Math.hypot(pose.base[0] - previous[0], pose.base[1] - previous[1]);
        previous = [...pose.base];
        assert.equal(pose.fault, null, `${name}: ${pose.fault}`);
        assert.equal(pose.mode === 'passive', false, `${name}: unexpectedly passive`);
        assert.ok(!(pose.avoidance.active && pose.navigation.recovering), `${name}: competing control owners`);
        if (pose.avoidance.active && pose.avoidance.strategy === 'sidestep') {
          assert.equal(pose.command.vx, 0, `${name}: sidestep must stop forward command`);
          assert.equal(Math.abs(pose.command.yawRate), 0, `${name}: sidestep must hold heading`);
        }
        maxYaw = Math.max(maxYaw, Math.abs(pose.orientation.yaw));
        if (name.startsWith('corridor')) {
          const end = targets[Math.min(checked, targets.length - 1)];
          const start = checked ? targets[checked - 1] : { x: 0, y: 0 };
          const dx = end.x - start.x, dy = end.y - start.y;
          const t = Math.max(0, Math.min(1, ((pose.base[0] - start.x) * dx + (pose.base[1] - start.y) * dy) / (dx * dx + dy * dy)));
          const deviation = Math.hypot(pose.base[0] - start.x - t * dx, pose.base[1] - start.y - t * dy);
          maxDeviation = Math.max(maxDeviation, deviation);
          assert.ok(deviation < 0.25, `${name}: route deviation ${deviation}`);
        }
        if (reached.length > checked) {
          const target = targets[checked];
          const error = Math.hypot(pose.base[0] - target.x, pose.base[1] - target.y);
          assert.ok(error <= WAYPOINT_RADIUS + 0.002, `${name}: arrival error ${error}`);
          assert.equal(pose.reward.total, reached.length, `${name}: reward count`);
          checked = reached.length;
        }
        if (checked === targets.length) break;
      }
    }
    assert.equal(checked, targets.length, `${name}: incomplete ${checked}/${targets.length}; ${JSON.stringify({ base: pose.base, navigation: pose.navigation, avoidance: pose.avoidance })}`);
    assert.equal(pose.mode, 'stand', `${name}: final stop`);
    if (name.startsWith('near-goal')) assert.ok(dockingSamples > 0, `${name}: docking never activated`);
    if (name.startsWith('clear 90deg') || name.startsWith('clear -90deg')) {
      assert.ok(alignmentSeconds < 10, `${name}: alignment too slow ${alignmentSeconds}s`);
    }
    if (name.startsWith('target-aware')) {
      const direct = Math.hypot(targets[0].x, targets[0].y);
      assert.ok(travel < direct * 2 + 2, `${name}: excessive detour ${travel}m`);
      console.log(`${name}: travelled ${travel.toFixed(2)}m`);
    }
    if (name.startsWith('corridor')) console.log(`${name}: maximum segment deviation ${maxDeviation.toFixed(3)}m`);
    if (name.startsWith('clear 0deg')) assert.ok(maxYaw < 0.1, `${name}: unexpected straight-route turn ${maxYaw}`);
    // Ensure completion is not rewarded again in subsequent physics steps.
    for (let i = 0; i < 500; i++) sim.step();
    sim.postPose();
    assert.equal(pose.reward.total, targets.length, `${name}: duplicate reward`);
  }

  for (const degrees of [-180, -135, -90, -45, 0, 45, 90, 135, 180]) {
    for (const distance of [0.25, 0.8, 1.5, 3, 5]) {
      const angle = degrees * Math.PI / 180;
      await run(`clear ${degrees}deg ${distance}m`, [{ x: distance * Math.cos(angle), y: distance * Math.sin(angle) }]);
    }
  }
  // Reproduces the reported left turn and permanent stop around x=1.09m.
  for (const x of [1.8, 2, 2.5]) {
    for (const y of [-0.1, 0, 0.1]) await run(`detour ${x},${y}`, [{ x: 5, y: 0 }], { x, y });
  }
  await run('zigzag', [{ x: 0.25, y: 0 }, { x: 2, y: 1 }, { x: -1, y: 1 }, { x: 2, y: -1 }, { x: 0, y: 0 }]);
  await run('corridor square', [{ x: 2, y: 0 }, { x: 2, y: 2 }, { x: 0, y: 2 }, { x: 0, y: 0 }]);
  await run('corridor reversal', [{ x: 2, y: 0 }, { x: 0, y: 0 }, { x: 1, y: -1 }]);
  await run('corridor short zigzag', [{ x: 0.5, y: 0 }, { x: 0.5, y: 0.5 }, { x: 1, y: 0.5 }, { x: 1, y: 0 }]);
  await run('target-aware goal before obstacle', [{ x: 0.6, y: 0 }], { x: 1.8, y: 0 });
  for (const y of [-0.8, 0.8]) {
    await run(`near-goal docking ${y}`, [{ x: 2, y: 0 }], { x: 2, y });
  }
  for (const y of [-2, -1, 1, 2]) {
    await run(`target-aware detour ${y}`, [{ x: 4, y }], { x: 1.8, y: y * 0.3 });
  }
  // The user's page uses rough terrain: both collision platforms must be
  // represented in avoidance, not just the draggable demo box.
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8'), terrain: 'rough' });
  for (const target of [{ x: 3.5, y: 0 }, { x: 3.5, y: 0.15 }, { x: 4, y: 0 }, { x: 5, y: 0 }, { x: 5, y: 2 }, { x: 5, y: -2 }]) {
    await run(`rough ${target.x},${target.y}`, [target], { x: 0, y: -3 });
  }
  await send({ type: 'moveObstacle', id: 'step_right', x: 15, y: -15 });
  for (const x of [1, 1.5, 2.5]) {
    await send({ type: 'moveObstacle', id: 'step_left', x, y: 0 });
    await run(`clear 0deg low step at ${x}`, [{ x: 4, y: 0 }]);
    assert.ok(Math.abs(pose.base[1]) < 0.06, 'cross low step without lateral detour');
    assert.equal(pose.obstacles.find(o => o.id === 'step_left').x, x);
  }
  await send({ type: 'moveObstacle', id: 'step_left', x: 15, y: 15 });
  await send({ type: 'moveObstacle', id: 'step_right', x: 2, y: 0 });
  await run('moved high step detour', [{ x: 5, y: 0 }]);
  assert.equal(pose.obstacles.find(o => o.id === 'step_right').x, 2);
});
