import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { LocalDetour } from '../src/a2/detour.js';
import { waypointRouteClear } from '../src/a2/avoidance.js';

test('buffer escape matrix: four headings, both sides and three clearances', () => {
  for (const yaw of [0, Math.PI / 2, Math.PI, -Math.PI / 2]) for (const side of [-1, 1]) for (const gap of [.03, .06, .09]) {
    const c = Math.cos(yaw), s = Math.sin(yaw);
    const box = { id: 'near', x: c * (.66 + gap) * side, y: s * (.66 + gap) * side,
      halfX: Math.abs(c) * .2 + Math.abs(s) * .3, halfY: Math.abs(s) * .2 + Math.abs(c) * .3, halfZ: .2 };
    const pose = { x: 0, y: 0, yaw }, goal = { x: 4 * c, y: 4 * s }, planner = new LocalDetour();
    let released = false;
    for (let i = 0; i < 2400; i++) {
      const p = planner.plan(pose, goal, [box], i * .05, true);
      if (!p) { released = true; break; }
      assert.notEqual(p.strategy, 'detour-blocked', JSON.stringify({ yaw, side, gap, pose }));
      const next = { ...pose, x: pose.x + (c * p.command.vx - s * p.command.vy) * .05,
        y: pose.y + (s * p.command.vx + c * p.command.vy) * .05 };
      assert.ok(waypointRouteClear(pose, next, [box], yaw, .02));
      Object.assign(pose, next);
    }
    assert.ok(released, JSON.stringify({ yaw, side, gap, pose }));
  }
});

// Deterministic perturbations: every failure prints its complete replay input.
// No random seed changes between runs and no assertions accepting timeouts.
test('MuJoCo environment matrix', async t => {
  const sim = await import('../src/a2/a2.worker.js');
  const xml = fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8');
  const scenarios = [];
  for (const mirror of [-1, 1]) for (const offset of [-.12, 0, .12]) for (const goalY of [-1, 1])
    scenarios.push({ mirror, offset, goal: { x: 5, y: goalY } });
  for (const mirror of [-1, 1]) for (const length of [3, 8, 12]) for (const width of [.12, .24])
    scenarios.push({ mirror, offset: 0, width, goal: { x: length, y: mirror * .5 } });
  for (const scenario of scenarios) {
    const { mirror, offset } = scenario;
    await t.test(JSON.stringify(scenario), async () => {
      let pose;
      globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
      const send = data => globalThis.onmessage({ data });
      await send({ type: 'init', terrain: 'rough', xml });
      await send({ type: 'moveObstacle', id: 'demo_box', x: .85 + offset, y: offset * mirror });
      await send({ type: 'moveObstacle', id: 'step_left', x: 1.5 + offset, y: .85 * mirror });
      await send({ type: 'moveObstacle', id: 'step_right', x: 2.5 - offset, y: -.85 * mirror });
      if (scenario.width) await send({ type: 'editObstacle', action: 'resize', id: 'demo_box',
        obstacle: { x: .85, y: 0, halfX: scenario.width, halfY: scenario.width + .12, halfZ: .16 } });
      for (let i = 0; i < 1500; i++) sim.step();
      await send({ type: 'addWaypoint', waypoint: scenario.goal });
      let movedAt = 0, anchor = null, distance = 0, previous = null, maxTilt = 0;
      for (let i = 0; i < 90000; i++) {
        sim.step();
        if (i % 25) continue;
        sim.postPose();
        const failure = () => JSON.stringify({ scenario, base: pose.base, command: pose.command, avoidance: pose.avoidance, navigation: pose.navigation });
        assert.equal(pose.fault, null, failure());
        assert.equal(pose.recovery, null, failure());
        maxTilt = Math.max(maxTilt, Math.abs(pose.orientation.roll), Math.abs(pose.orientation.pitch));
        if (previous) distance += Math.hypot(pose.base[0] - previous[0], pose.base[1] - previous[1]);
        previous = pose.base;
        if (!anchor || Math.hypot(pose.base[0] - anchor[0], pose.base[1] - anchor[1]) > .08) {
          anchor = pose.base; movedAt = pose.time;
        }
        assert.ok(pose.time - movedAt < 20, `stalled: ${failure()}`);
        assert.ok(distance < 25, `excessive detour: ${failure()}`);
        if (pose.reward.total) break;
      }
      assert.equal(pose.reward.total, 1, JSON.stringify({ scenario, pose: pose.base, navigation: pose.navigation }));
      assert.ok(Math.hypot(pose.base[0] - scenario.goal.x, pose.base[1] - scenario.goal.y) < .2);
      t.diagnostic(`${JSON.stringify(scenario)}: ${pose.time.toFixed(1)}s, ${distance.toFixed(2)}m, tilt ${(maxTilt * 180 / Math.PI).toFixed(1)}deg`);
    });
  }
});
