import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { EncounterPlanner, encounterTrajectory } from '../src/a2/encounter.js';
import { waypointRouteClear } from '../src/a2/avoidance.js';

const origin = { x: 0, y: 0, yaw: 0 };
const person = { kind: 'person', x: 2.8, y: 0, vx: -.6, vy: 0, halfX: .22, halfY: .22, halfZ: .85 };
const wall = side => ({ kind: 'box', x: 0, y: side * .85, halfX: .8, halfY: .15, halfZ: .3 });
test('encounter chooses open side, short retreat, or blocked stop; never blind reverse', () => {
  for (const side of [-1, 1]) {
    const obstacles = [person, wall(side)];
    const result = new EncounterPlanner().plan(origin, { vx: .5, vy: 0 }, obstacles, 0);
    assert.equal(result.strategy, 'dynamic-dodge');
    assert.equal(result.direction, -side);
    assert.ok(encounterTrajectory(origin, result.command, obstacles).safe);
  }
  const crossing = { ...person, x: 1, y: -1.5, vx: 0, vy: .6 };
  const corridor = [crossing, wall(-1), wall(1)];
  const planner = new EncounterPlanner();
  assert.equal(planner.plan(origin, { vx: .5, vy: 0 }, corridor, 0).strategy, 'dynamic-retreat');
  const back = { kind: 'box', x: -.85, y: 0, halfX: .2, halfY: .3 };
  assert.equal(new EncounterPlanner().plan(origin, { vx: .5, vy: 0 }, [...corridor, back], 0).strategy, 'dynamic-wait');
  assert.notEqual(planner.plan({ ...origin, x: -.46 }, { vx: .5, vy: 0 }, corridor, .2).strategy, 'dynamic-retreat');
  assert.equal(new EncounterPlanner().plan(origin, { vx: .5, vy: 0 }, [{ ...person, y: 4 }], 0), null);
  const trapped = new EncounterPlanner().plan(origin, { vx: .5, vy: 0 }, [{ ...person, x: .3 }], 0);
  assert.equal(trapped.blocked, true);
  assert.equal(trapped.strategy, 'dynamic-wait');
  const turned = new EncounterPlanner().plan({ ...origin, yaw: Math.PI / 2 }, { vx: .5, vy: 0 },
    [{ ...person, x: 0, y: 2.8, vx: 0, vy: -.6 }], 0);
  assert.equal(turned.strategy, 'dynamic-dodge');
});

test('continuous prediction catches a fast cross-through between samples and retains side preference', () => {
  const fast = { ...person, x: 0, y: -2, vx: 0, vy: 40 };
  assert.equal(encounterTrajectory(origin, { vx: 0, vy: 0 }, [fast]).safe, false);
  const planner = new EncounterPlanner();
  const first = planner.plan(origin, { vx: .5, vy: 0 }, [person], 0);
  const second = planner.plan(origin, { vx: .5, vy: 0 }, [{ ...person, y: .02 }], .1);
  assert.equal(first.direction, second.direction);
  assert.equal(planner.plan(origin, { vx: .5, vy: 0 }, [], .2), null);
});

test('MuJoCo moving encounters dodge and resume body-centred waypoints', async t => {
  const sim = await import('../src/a2/a2.worker.js');
  const xml = fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8');
  const scenarios = [
    { name: 'person head-on', actor: person, walls: [], expected: 'dynamic-dodge' },
    { name: 'person appears while cruising', actor: person, spawnAfter: 1, walls: [], expected: 'dynamic-dodge' },
    { name: 'two people constrain the left side', actor: person,
      extraActors: [{ ...person, x: 0, y: 1.3, vx: 0, vy: 0 }], walls: [], expected: 'dynamic-dodge' },
    { name: 'faster person head-on', actor: { ...person, x: 3.8, vx: -.8 }, walls: [], expected: 'dynamic-dodge' },
    { name: 'car head-on with warning distance', actor: { kind: 'car', x: 6, y: 0, vx: -1.2, vy: 0,
      halfX: .9, halfY: .45, halfZ: .65 }, walls: [], expected: 'dynamic-dodge' },
    ...[-1, 1].map(side => ({ name: `person open side ${-side}`, actor: person, walls: [wall(side)], expected: 'dynamic-dodge' })),
    { name: 'person corridor crossing', actor: { ...person, x: 1, y: -1.5, vx: 0, vy: .6 }, walls: [wall(-1), wall(1)], expected: 'dynamic-retreat' },
    ...[-1, 1].map(side => ({ name: `car close crossing ${side}`, actor: { kind: 'car', x: 1.2, y: side * 3, vx: 0, vy: -side * 1.2,
      halfX: .9, halfY: .45, halfZ: .65 }, walls: [], expected: 'dynamic-dodge' })),
    ...[-1, 1].map(side => ({ name: `car distant crossing ${side}`, actor: { kind: 'car', x: 2, y: side * 3, vx: 0, vy: -side * 1.2,
      halfX: .9, halfY: .45, halfZ: .65 }, walls: [], expected: 'dynamic-dodge' })),
  ];
  for (const scenario of scenarios) await t.test(scenario.name, async () => {
    let pose;
    globalThis.postMessage = message => { if (message.type === 'pose') pose = message; };
    const send = data => globalThis.onmessage({ data });
    await send({ type: 'init', xml });
    await send({ type: 'personYield', enabled: false });
    await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
    for (const obstacle of scenario.walls) await send({ type: 'editObstacle', action: 'create', obstacle });
    for (let i = 0; i < 1500; i++) sim.step();
    if (!scenario.spawnAfter) await send({ type: 'editObstacle', action: 'create', automatic: true, obstacle: scenario.actor });
    for (const obstacle of scenario.extraActors || []) await send({ type: 'editObstacle', action: 'create', automatic: true, obstacle });
    await send({ type: 'addWaypoint', waypoint: { x: 6, y: 0 } });
    const strategies = new Set();
    let spawned = !scenario.spawnAfter;
    let resumedBeforeExpiry = false, peakTilt = 0;
    for (let i = 0; i < 60000; i++) {
      sim.step();
      if (i % 10) continue;
      sim.postPose(); strategies.add(pose.avoidance.strategy);
      peakTilt = Math.max(peakTilt, Math.abs(pose.orientation.roll), Math.abs(pose.orientation.pitch));
      if (strategies.has(scenario.expected) && !pose.avoidance.active && pose.forwardSpeed > .1
        && pose.obstacles.some(o => o.kind === 'person' || o.kind === 'car')) resumedBeforeExpiry = true;
      if (!spawned && i * .002 >= scenario.spawnAfter) {
        spawned = true;
        await send({ type: 'editObstacle', action: 'create', automatic: true,
          obstacle: { ...scenario.actor, x: pose.base[0] + 4.5, y: pose.base[1] } });
      }
      const detail = JSON.stringify({ scenario: scenario.name, base: pose.base, avoidance: pose.avoidance });
      assert.equal(pose.fault, null, detail);
      assert.equal(pose.recovery, null, detail);
      if (scenario.expected === 'dynamic-retreat') assert.ok(pose.base[0] > -.7, `unbounded retreat: ${detail}`);
      const p = { x: pose.base[0], y: pose.base[1], yaw: pose.orientation.yaw };
      assert.ok(waypointRouteClear(p, p, pose.obstacles, p.yaw, 0), `body envelope contact: ${detail}`);
      if (pose.reward.total) break;
    }
    assert.ok(strategies.has(scenario.expected), JSON.stringify([...strategies]));
    assert.ok(resumedBeforeExpiry, 'must resume before automatic actors disappear');
    assert.equal(pose.reward.total, 1, JSON.stringify(pose.navigation));
    assert.ok(Math.hypot(pose.base[0] - 6, pose.base[1]) < .2);
    t.diagnostic(`${scenario.name}: arrived at ${pose.time.toFixed(1)}s; peak tilt ${(peakTilt * 180 / Math.PI).toFixed(1)}deg`);
  });
});
