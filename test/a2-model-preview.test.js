import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { robotSnapshot } from '../src/a2/model-preview.js';

test('visual snapshot removes helpers/cameras and centers without changing simulation', () => {
  const robot = new THREE.Group(); robot.rotation.x = -Math.PI / 2;
  const body = new THREE.Mesh(new THREE.BoxGeometry(2, 1, 1), new THREE.MeshStandardMaterial());
  body.position.set(8, 4, 3); robot.add(body);
  const helper = new THREE.Mesh(new THREE.BoxGeometry(50, 50, 50)); helper.layers.set(1); robot.add(helper);
  robot.add(new THREE.PerspectiveCamera());
  const snapshot = robotSnapshot(robot);
  assert.equal(snapshot.children.length, 1);
  assert.equal(robot.children.length, 3);
  assert.deepEqual(body.position.toArray(), [8, 4, 3]);
  const bounds = new THREE.Box3().setFromObject(snapshot), center = bounds.getCenter(new THREE.Vector3());
  assert.ok(Math.abs(center.x) < 1e-10 && Math.abs(center.z) < 1e-10 && Math.abs(bounds.min.y) < 1e-10);
  assert.equal(snapshot.children[0].geometry, body.geometry);
});
