import test from 'node:test';
import assert from 'node:assert/strict';
import { telemetryDrives, selectLinkGroups, LINK_GROUPS } from '../src/a2/connectome-link.js';
test('robot state encodes bounded neural drive including lateral walking, turns, avoidance and arm', () => {
  const p = { mode: 'walk', forwardSpeed: .6, command: { yawRate: -.35 }, avoidance: { active: true }, armWork: { active: true } };
  assert.deepEqual(telemetryDrives(p), { motion: 120, turn: 100, obstacle: 100, arm: 100 });
  assert.equal(telemetryDrives({ mode: 'walk', lateralSpeed: .3 }).motion, 60);
  assert.equal(telemetryDrives({ ...p, forwardSpeed: 9 }).motion, 120);
  assert.deepEqual(telemetryDrives(null), { motion: 0, turn: 0, obstacle: 0, arm: 0 });
  for (const q of [{ ...p, mode: 'passive' }, { ...p, fault: 'tilt' }]) assert.deepEqual(telemetryDrives(q), telemetryDrives(null));
  assert.equal(telemetryDrives({ mode: 'walk', forwardSpeed: NaN }).motion, 0);
});
test('neural groups are deterministic bounded disjoint real neuron indices', () => {
  const data = { N: 400, superclass: Array.from({ length: 400 }, (_, i) => i % 4), meta: { superclasses: Object.values(LINK_GROUPS) } };
  const groups = selectLinkGroups(data);
  for (const [name, ids] of Object.entries(groups)) {
    assert.equal(ids.length, 64); assert.equal(new Set(ids).size, 64);
    for (const i of ids) assert.equal(data.meta.superclasses[data.superclass[i]], LINK_GROUPS[name]);
  }
  assert.equal(new Set(Object.values(groups).flat()).size, 256);
  assert.deepEqual(selectLinkGroups({ N: 0, superclass: [], meta: { superclasses: [] } }).motion, []);
});
