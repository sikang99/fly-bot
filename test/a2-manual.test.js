import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { ManualControl, gamepadCommand, webManualCommand } from '../src/a2/manual.js';

test('web arrows and hold buttons map body-relative movement and release to zero', () => {
  for (const [key, direction, axis, value] of [
    ['ArrowUp', 'front', 'vx', .6], ['ArrowDown', 'back', 'vx', -.6],
    ['ArrowLeft', 'left', 'vy', .35], ['ArrowRight', 'right', 'vy', -.35],
    ['KeyA', 'turnLeft', 'yawRate', .35], ['KeyD', 'turnRight', 'yawRate', -.35],
  ]) {
    const keyboard = webManualCommand(new Set([key, 'ShiftLeft']));
    assert.equal(keyboard.held, true); assert.equal(keyboard.command[axis], value);
    const pointer = webManualCommand(new Set(), direction);
    assert.equal(pointer.held, true); assert.equal(pointer.command[axis], value);
  }
  assert.equal(webManualCommand(new Set(['ArrowUp'])).held, true);
  assert.equal(webManualCommand(new Set(['KeyW'])).held, false);
  assert.deepEqual(webManualCommand(new Set()), { held: false, command: { vx: 0, vy: 0, yawRate: 0 } });
  assert.equal(webManualCommand(new Set(['ArrowUp', 'ArrowDown'])).command.vx, 0);
});

test('manual deadman, watchdog and latched emergency stop require explicit rearming', () => {
  const manual = new ManualControl(); manual.enter(true); manual.setMode('walk');
  const input = { held: true, command: { vx: 0.6 } };
  manual.receive(input, 0); assert.equal(manual.sample(1).vx, 0);
  manual.receive({ held: false }, 2); manual.receive(input, 3); assert.equal(manual.sample(4).vx, 0.6);
  assert.equal(manual.sample(304).vx, 0);
  manual.receive(input, 305); assert.equal(manual.sample(306).vx, 0);
  manual.receive({ held: false }, 307); manual.receive(input, 308); assert.equal(manual.sample(309).vx, .6);
  manual.stop(); manual.receive({ held: false }, 310); manual.receive(input, 311); assert.equal(manual.sample(312).vx, 0);
  manual.enter(true); manual.setMode('walk'); manual.receive({ held: false }, 313); manual.receive(input, 314); assert.equal(manual.sample(315).vx, .6);
});
test('manual Passive / Stand / Walk transitions clear input and emergency remains latched', () => {
  const manual = new ManualControl(); manual.enter(true);
  for (const mode of ['passive', 'stand', 'walk', 'stand', 'passive', 'stand', 'walk']) {
    manual.setMode(mode); assert.equal(manual.mode, mode);
    manual.receive({ held: true, command: { vx: .6 } }, 0);
    assert.equal(manual.sample(1).vx, 0);
    manual.receive({ held: false }, 2);
    manual.receive({ held: true, command: { vx: .6 } }, 3);
    assert.equal(manual.sample(4).vx, mode === 'walk' ? .6 : 0);
  }
  manual.stop(); manual.setMode('stand'); manual.setMode('walk');
  assert.equal(manual.latched, true); assert.equal(manual.sample(5).vx, 0);
});
test('standard gamepad maps sticks and dedicated deadman/emergency buttons', () => {
  const pad = { axes: [1, -1, -1], buttons: Array.from({ length: 16 }, () => ({ pressed: false })) };
  pad.buttons[5].pressed = true;
  const input = gamepadCommand(pad);
  assert.equal(input.held, true); assert.equal(input.command.vx, .6); assert.equal(input.command.vy, -.35); assert.equal(input.command.yawRate, .35);
  pad.axes = [.1, -.1, .1]; assert.equal(Math.abs(gamepadCommand(pad).command.vx), 0);
  pad.buttons[1].pressed = true; assert.equal(gamepadCommand(pad).emergency, true);
});
test('manual overrides a queued waypoint without reward and emergency cannot be undone by heartbeats', async () => {
  let pose; globalThis.postMessage = m => { if (m.type === 'pose') pose = m; };
  const sim = await import('../src/a2/a2.worker.js');
  const send = data => globalThis.onmessage({ data });
  await send({ type: 'init', xml: fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8') });
  await send({ type: 'moveObstacle', id: 'demo_box', x: 15, y: 15 });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'addWaypoint', waypoint: { x: -3, y: 0 } });
  await send({ type: 'manualMode', enabled: true });
  await send({ type: 'mode', mode: 'passive' });
  for (let i = 0; i < 1500; i++) sim.step();
  await send({ type: 'mode', mode: 'stand' });
  for (let i = 0; i < 1500; i++) sim.step();
  sim.postPose(); assert.equal(pose.mode, 'stand'); assert.ok(pose.base[2] > .25);
  await send({ type: 'manualArm', action: 'left' });
  sim.step(); sim.postPose(); assert.equal(pose.armWork.active, true); assert.equal(pose.command.vx, 0);
  await send({ type: 'manualArm', action: 'home' });
  sim.step(); sim.postPose(); assert.equal(pose.armWork.active, false);
  await send({ type: 'manualArm', action: 'front' });
  for (let i = 0; i < 4000; i++) sim.step();
  sim.postPose(); assert.equal(pose.armWork.active, false); assert.equal(pose.mode, 'stand');
  assert.equal(pose.reward.total, 0); assert.equal(pose.navigation.remaining, 1);
  await send({ type: 'mode', mode: 'walk' });
  await send({ type: 'manualArm', action: 'front' });
  sim.step(); sim.postPose(); assert.equal(pose.armWork.active, false);
  for (const mode of ['passive', 'stand', 'walk', 'stand', 'passive', 'stand', 'walk']) {
    await send({ type: 'mode', mode }); sim.step(); sim.postPose(); assert.equal(pose.mode, mode);
  }
  await send({ type: 'manualCommand', held: false });
  for (let i = 0; i < 2000; i++) {
    if (i % 25 === 0) await send({ type: 'manualCommand', held: true, command: { vx: .6 } });
    sim.step();
  }
  sim.postPose(); assert.ok(pose.base[0] > .5); assert.equal(pose.reward.total, 0);
  assert.equal(pose.navigation.remaining, 1); assert.equal(pose.navigation.active, false);
  await send({ type: 'emergencyStop' });
  await send({ type: 'manualCommand', held: true, command: { vx: .6 } });
  sim.step(); sim.postPose(); assert.equal(pose.mode, 'passive'); assert.equal(pose.command.vx, 0);
  await send({ type: 'manualMode', enabled: false });
  sim.step(); sim.postPose(); assert.equal(pose.mode, 'stand');
});
