import test from 'node:test';
import assert from 'node:assert/strict';
import { pedestrianState, stepPedestrian } from '../src/a2/pedestrian.js';

const person = () => ({ id: 'person', kind: 'person', x: -2.5, y: 0, vx: .6, vy: 0, halfX: .22, halfY: .22, halfZ: .85 });
const robot = { x: 0, y: 0, yaw: 0, vx: 0, vy: 0 };

test('person yields consistently, passes the robot and smoothly rejoins original line', () => {
  const p = person(), state = pedestrianState(p);
  let minY = 0, previousVx = p.vx, previousVy = p.vy, side;
  for (let i = 0; i < 2500; i++) {
    Object.assign(p, stepPedestrian(p, state, robot, [], .01));
    minY = Math.min(minY, p.y);
    if (state.active) { side ??= state.side; assert.equal(state.side, side); }
    assert.ok(Math.abs(p.vx - previousVx) <= .008001);
    assert.ok(Math.abs(p.vy - previousVy) <= .008001);
    previousVx = p.vx; previousVy = p.vy;
    assert.ok(!(Math.abs(p.x) < .78 && Math.abs(p.y) < .64), 'must not pass through robot');
  }
  assert.ok(minY < -.65 && minY > -1.2);
  assert.ok(p.x > 2); assert.ok(Math.abs(p.y) < .05);
});

test('person chooses open side and waits if both sides and forward path are blocked', () => {
  const wall = (id, y) => ({ id, kind: 'box', x: -1.5, y, halfX: 2, halfY: .1 });
  const p = person(), state = pedestrianState(p);
  for (let i = 0; i < 600; i++) Object.assign(p, stepPedestrian(p, state, robot, [wall('right', -.5)], .01));
  assert.ok(p.y > .4, 'should choose left if right side is blocked');
  const q = person(), qs = pedestrianState(q);
  for (let i = 0; i < 2500; i++) Object.assign(q, stepPedestrian(q, qs, robot, [wall('right', -.5), wall('left', .5)], .01));
  assert.ok(q.x < -.8); assert.equal(q.yielding, 'waiting');
});

test('distant or non-intersecting robot does not alter normal walking', () => {
  const p = person(), state = pedestrianState(p);
  for (let i = 0; i < 100; i++) Object.assign(p, stepPedestrian(p, state, { ...robot, y: 5 }, [], .01));
  assert.ok(Math.abs(p.x + 1.9) < 1e-8); assert.equal(p.y, 0);
});
