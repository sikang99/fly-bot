import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { JETSON_BOX, buildJetsonBox, buildBodyBranding, renderBeacon, EquipmentBeacon } from '../src/a2/equipment.js';
import { ARM_STOW_POSE } from '../src/a2/config.js';

const pose = (overrides = {}) => ({ time: 0, base: [0, 0, .32], orientation: { yaw: 0 }, mode: 'stand', ...overrides });

test('TeamGRIT labels face outward on both sides and stay upright without mirrored text', () => {
  const labels = buildBodyBranding(new THREE.Texture());
  assert.equal(labels.children.length, 2);
  for (const label of labels.children) {
    const side = Math.sign(label.position.y);
    const up = new THREE.Vector3(0, 1, 0).applyQuaternion(label.quaternion);
    const right = new THREE.Vector3(1, 0, 0).applyQuaternion(label.quaternion);
    const normal = new THREE.Vector3(0, 0, 1).applyQuaternion(label.quaternion);
    assert.ok(up.distanceTo(new THREE.Vector3(0, 0, 1)) < 1e-8);
    assert.ok(normal.distanceTo(new THREE.Vector3(0, side, 0)) < 1e-8);
    assert.ok(right.distanceTo(new THREE.Vector3(-side, 0, 0)) < 1e-8);
    assert.ok(Math.abs(label.position.y) > .12);
  }
});

test('Jetson enclosure sits within rear body width, behind the arm, above the deck', () => {
  const { group, lamps } = buildJetsonBox();
  assert.ok(JETSON_BOX.width < .24);
  assert.ok(JETSON_BOX.x + JETSON_BOX.length / 2 < ARM_STOW_POSE.mountX);
  assert.ok(JETSON_BOX.z - JETSON_BOX.height / 2 > .09);
  const bounds = new THREE.Box3().setFromObject(group);
  assert.ok(bounds.min.x > -.335 && bounds.max.x < 0);
  assert.ok(bounds.min.y > -.12 && bounds.max.y < .12);
  assert.ok(bounds.min.z >= .09);
  renderBeacon(lamps, { red: true, green: false });
  assert.equal(lamps.red.material.emissiveIntensity, 3);
  assert.equal(lamps.green.material.emissiveIntensity, 0);
});

test('Jetson mushroom emergency button is above the enclosure and separate from lamps', () => {
  const { group, lamps } = buildJetsonBox();
  const stop = group.getObjectByName('jetson_emergency_stop');
  group.updateMatrixWorld(true);
  assert.ok(stop.getObjectByName('estop_red_mushroom'));
  assert.ok(stop.getObjectByName('estop_yellow_collar'));
  const bounds = new THREE.Box3().setFromObject(stop);
  assert.ok(bounds.min.z > JETSON_BOX.z + JETSON_BOX.height / 2);
  for (const lamp of Object.values(lamps)) {
    assert.equal(bounds.intersectsBox(new THREE.Box3().setFromObject(lamp)), false);
  }
});

test('moving alternates red/green, including yaw-only turns, idle is off', () => {
  const b = new EquipmentBeacon(); b.receive(pose(), 0);
  assert.equal(b.sample(0).mode, 'idle');
  b.receive(pose({ time: .1, base: [.05, 0, .32], mode: 'walk' }), .4);
  assert.deepEqual([b.sample(.4).red, b.sample(.4).green], [true, false]);
  assert.deepEqual([b.sample(.55).red, b.sample(.55).green], [false, true]);
  b.receive(pose({ time: .2, base: [.05, 0, .32], orientation: { yaw: .1 }, mode: 'walk' }), .6);
  assert.equal(b.sample(.6).mode, 'moving');
  assert.equal(b.sample(.9).mode, 'idle');
});

test('waypoint event lasts three seconds, avoidance persists and errors override all blinking', () => {
  const b = new EquipmentBeacon();
  b.receive(pose({ reward: { total: 1 } }), 0);
  assert.equal(b.sample(.1).mode, 'event');
  assert.equal(b.sample(.1).red, true);
  assert.equal(b.sample(.3).red, false);
  assert.equal(b.sample(.3).green, false);
  assert.equal(b.sample(3).mode, 'idle');
  b.receive(pose({ time: 1, avoidance: { active: true } }), 5);
  b.receive(pose({ time: 2, avoidance: { active: true } }), 7);
  assert.equal(b.sample(9).mode, 'event');
  b.receive(pose({ time: 3, fault: 'tilt', avoidance: { active: true } }), 9);
  for (const now of [9.1, 9.3, 9.6]) {
    assert.equal(b.sample(now).mode, 'error');
    assert.equal(b.sample(now).red, true);
    assert.equal(b.sample(now).green, false);
  }
});

test('worker error and stale telemetry stay red; reset clears old event state', () => {
  const b = new EquipmentBeacon();
  b.receive(pose({ time: 10 }), 0);
  assert.equal(b.sample(1.1, { ready: true }).mode, 'error');
  b.error = 'worker failure';
  assert.equal(b.sample(.1).mode, 'error');
  b.error = ''; b.event('arrival', .2);
  b.receive(pose({ time: 0 }), .3);
  assert.equal(b.sample(.3, { ready: true }).mode, 'idle');
  assert.equal(b.sample(.3, { stopped: true }).mode, 'event');
});
