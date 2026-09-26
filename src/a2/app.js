import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ARM_STOW_POSE } from './config.js';
import { A2_FOOTPRINT } from './avoidance.js';
import { WAYPOINT_RADIUS } from './navigation.js';
import { worldObstacles, isTraversable, terrainColor } from './terrain.js';
import './style.css';

const BASE = import.meta.env.BASE_URL;
const $ = selector => document.querySelector(selector);
const canvas = $('#scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
const robotViewCanvas = $('#robotView');
const robotRenderer = new THREE.WebGLRenderer({ canvas: robotViewCanvas, antialias: true });
robotRenderer.outputColorSpace = THREE.SRGBColorSpace;
const robotCamera = new THREE.PerspectiveCamera(78, 16 / 9, 0.03, 30);
const cameraPitch = THREE.MathUtils.degToRad(12);
const robotCameraBasis = new THREE.Matrix4().makeBasis(
  new THREE.Vector3(0, -1, 0),
  new THREE.Vector3(Math.sin(cameraPitch), 0, Math.cos(cameraPitch)),
  new THREE.Vector3(-Math.cos(cameraPitch), 0, Math.sin(cameraPitch)),
);
robotCamera.quaternion.setFromRotationMatrix(robotCameraBasis);
let robotCameraReady = false;
const scene = new THREE.Scene();
scene.background = new THREE.Color('#0b1118');
scene.fog = new THREE.Fog('#0b1118', 8, 25);
const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 80);
camera.position.set(2.6, -3.2, 2.1);
camera.layers.enable(1);
const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0, 0.45);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight('#d9edff', '#18202a', 1.7));
const sun = new THREE.DirectionalLight('#fff3dc', 3.2);
sun.position.set(4, -4, 7); sun.castShadow = true;
Object.assign(sun.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5, near: 0.1, far: 20 });
scene.add(sun);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: '#252d35', roughness: 0.9, metalness: 0.05 }));
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
const grid = new THREE.GridHelper(20, 40, '#557086', '#344451'); grid.position.y = 0.002; scene.add(grid);
const robot = new THREE.Group(); robot.rotation.x = -Math.PI / 2; scene.add(robot);

const terrain = new URLSearchParams(location.search).get('terrain') || 'flat';
const obstacleState = new Map(worldObstacles(terrain).map(obstacle => [obstacle.id, obstacle]));
const obstacleMeshes = new Map();
function createObstacleMesh(obstacle) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(obstacle.halfX * 2, obstacle.halfZ * 2, obstacle.halfY * 2), new THREE.MeshStandardMaterial({ color: terrainColor(obstacle), roughness: 0.62, metalness: 0.08 }));
  mesh.position.set(obstacle.x, obstacle.halfZ, -obstacle.y); mesh.castShadow = mesh.receiveShadow = true;
  mesh.userData.obstacleId = obstacle.id;
  mesh.userData.movable = obstacle.movable;
  mesh.userData.shape = `${obstacle.halfX},${obstacle.halfY},${obstacle.halfZ}`;
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.48, 0.50, 48), new THREE.MeshBasicMaterial({ color: '#ffb341', transparent: true, opacity: 0.35, side: THREE.DoubleSide }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = -obstacle.halfZ + 0.004; mesh.add(ring);
  scene.add(mesh); obstacleMeshes.set(obstacle.id, mesh);
}
for (const obstacle of obstacleState.values()) createObstacleMesh(obstacle);

function syncObstacles(objects) {
  const ids = new Set(objects.map(o => o.id));
  for (const [id, mesh] of obstacleMeshes) {
    const o = objects.find(item => item.id === id);
    if (!ids.has(id) || mesh.userData.shape !== `${o.halfX},${o.halfY},${o.halfZ}`) {
      scene.remove(mesh); mesh.traverse(part => { part.geometry?.dispose(); part.material?.dispose(); });
      obstacleMeshes.delete(id); obstacleState.delete(id);
    }
  }
  for (const o of objects) if (!obstacleMeshes.has(o.id)) createObstacleMesh(o);
}

const sensorCone = new THREE.Mesh(new THREE.ConeGeometry(0.48, 1.0, 3, 1, true), new THREE.MeshBasicMaterial({ color: '#65e5a5', transparent: true, opacity: 0.10, side: THREE.DoubleSide, depthWrite: false }));
sensorCone.rotation.z = -Math.PI / 2; sensorCone.position.x = 0.5;
sensorCone.layers.set(1);
const pathPoints = [];
const pathGeometry = new THREE.BufferGeometry();
const pathPositions = new Float32Array(500 * 3);
const pathAttribute = new THREE.BufferAttribute(pathPositions, 3).setUsage(THREE.DynamicDrawUsage);
pathGeometry.setAttribute('position', pathAttribute); pathGeometry.setDrawRange(0, 0);
const pathLine = new THREE.Line(pathGeometry, new THREE.LineBasicMaterial({ color: '#65e5a5', transparent: true, opacity: 0.8 }));
scene.add(pathLine);
const footprintPositions = new Float32Array(12);
const footprintGeometry = new THREE.BufferGeometry();
footprintGeometry.setAttribute('position', new THREE.BufferAttribute(footprintPositions, 3));
const footprintLine = new THREE.LineLoop(footprintGeometry, new THREE.LineBasicMaterial({ color: '#65e5a5', transparent: true, opacity: 0.75 }));
scene.add(footprintLine);
const waypointGroup = new THREE.Group(); scene.add(waypointGroup);
const waypointLine = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineDashedMaterial({ color: '#ffd166', dashSize: 0.12, gapSize: 0.06, transparent: true, opacity: 0.9 }));
scene.add(waypointLine);
const waypoints = [];
const bodyGroups = new Map();
const dark = new THREE.MeshStandardMaterial({ color: '#222b32', roughness: 0.48, metalness: 0.42 });
const shell = new THREE.MeshStandardMaterial({ color: '#cad5dd', roughness: 0.32, metalness: 0.28 });
const accent = new THREE.MeshStandardMaterial({ color: '#ffb341', roughness: 0.4, metalness: 0.18 });

function buildRobotArm() {
  const arm = new THREE.Group(); arm.name = 'single_arm'; arm.position.set(ARM_STOW_POSE.mountX, 0, 0.10);
  arm.scale.setScalar(ARM_STOW_POSE.scale);
  const base = new THREE.Mesh(new THREE.CylinderGeometry(0.12, 0.14, 0.10, 24), dark);
  base.rotation.x = Math.PI / 2; base.position.z = 0.05; arm.add(base);
  const turret = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.09, 0.07, 24), accent);
  turret.rotation.x = Math.PI / 2; turret.position.z = 0.12; arm.add(turret);

  const shoulder = new THREE.Group(); shoulder.name = 'arm_shoulder'; shoulder.position.z = 0.15; shoulder.rotation.y = ARM_STOW_POSE.shoulderPitch; arm.add(shoulder);
  const shoulderJoint = new THREE.Mesh(new THREE.CylinderGeometry(0.075, 0.075, 0.16, 20), dark); shoulder.add(shoulderJoint);
  const upper = new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.11, 0.34), shell); upper.position.z = 0.17; shoulder.add(upper);

  const elbow = new THREE.Group(); elbow.name = 'arm_elbow'; elbow.position.z = 0.34; elbow.rotation.y = ARM_STOW_POSE.elbowPitch; shoulder.add(elbow);
  const elbowJoint = new THREE.Mesh(new THREE.CylinderGeometry(0.066, 0.066, 0.15, 20), accent); elbow.add(elbowJoint);
  const forearm = new THREE.Mesh(new THREE.BoxGeometry(0.09, 0.09, 0.30), dark); forearm.position.z = 0.15; elbow.add(forearm);

  const wrist = new THREE.Group(); wrist.name = 'arm_wrist'; wrist.position.z = 0.30; wrist.rotation.y = ARM_STOW_POSE.wristPitch; elbow.add(wrist);
  const wristJoint = new THREE.Mesh(new THREE.SphereGeometry(0.065, 20, 14), accent); wrist.add(wristJoint);
  const palm = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.16, 0.09), shell); palm.position.z = 0.09; wrist.add(palm);
  for (const side of [-1, 1]) {
    const finger = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.035, 0.17), dark);
    finger.name = side < 0 ? 'gripper_left' : 'gripper_right';
    finger.position.set(0, side * ARM_STOW_POSE.gripperGap, 0.20); finger.rotation.x = side * 0.08; wrist.add(finger);
  }
  arm.traverse(object => { if (object.isMesh) { object.castShadow = true; object.receiveShadow = true; } });
  return arm;
}

function partFor(name) {
  const group = new THREE.Group();
  let mesh;
  if (name === 'base_link') {
    mesh = new THREE.Mesh(new THREE.BoxGeometry(0.67, 0.24, 0.18), shell);
    const nose = new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.27, 0.12), dark); nose.position.x = 0.35; group.add(nose);
    const frontArrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(0.38, 0, 0.15), 0.34, '#65e5a5', 0.12, 0.07);
    frontArrow.traverse(object => object.layers.set(1));
    group.add(frontArrow, buildRobotArm());
  } else if (name.endsWith('_hip')) {
    mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.16, 18), accent); mesh.rotation.x = Math.PI / 2;
  } else if (name.endsWith('_thigh')) {
    mesh = new THREE.Mesh(new THREE.BoxGeometry(0.075, 0.08, 0.275), shell); mesh.position.z = -0.1375;
  } else if (name.endsWith('_calf')) {
    mesh = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.05, 0.275), dark); mesh.position.z = -0.1375;
    const foot = new THREE.Mesh(new THREE.SphereGeometry(0.035, 18, 12), accent); foot.position.z = -0.275; group.add(foot);
  }
  if (mesh) group.add(mesh);
  group.traverse(object => { if (object.isMesh) { object.castShadow = true; object.receiveShadow = true; } });
  return group;
}

const worker = new Worker(new URL('./a2.worker.js', import.meta.url), { type: 'module' });
let bodyNames = [], ready = false, lastPose = null, following = true, editorPending = false;

worker.onmessage = ({ data }) => {
  if (data.type === 'ready') {
    bodyNames = data.bodyNames;
    for (const name of bodyNames) {
      const part = partFor(name); if (!part.children.length) continue;
      bodyGroups.set(name, part); robot.add(part);
      if (name === 'base_link') {
        part.add(sensorCone, robotCamera); robotCamera.position.set(0.43, 0, 0.18); robotCameraReady = true;
      }
    }
    ready = true; $('#status').textContent = 'READY'; $('#status').className = 'ok'; worker.postMessage({ type: 'run' });
  } else if (data.type === 'pose') {
    lastPose = data;
    syncObstacles(data.obstacles || []);
    for (const obstacle of data.obstacles || []) {
      obstacleState.set(obstacle.id, { ...obstacle });
      const mesh = obstacleMeshes.get(obstacle.id);
      if (mesh) mesh.position.set(obstacle.x, obstacle.halfZ, -obstacle.y);
    }
    bodyNames.forEach((name, i) => {
      const part = bodyGroups.get(name); if (!part) return;
      part.position.fromArray(data.xpos, i * 3);
      part.quaternion.set(data.xquat[i * 4 + 1], data.xquat[i * 4 + 2], data.xquat[i * 4 + 3], data.xquat[i * 4]);
    });
    if (editorPending) { updateObstacleState(); populateObstacleEditor(); editorPending = false; }
    updateFootprint(data.base, data.orientation.yaw);
    renderTelemetry(data);
    if (!pathPoints.length || pathPoints.at(-1).distanceToSquared(new THREE.Vector3(data.base[0], 0.015, -data.base[1])) > 0.0004) {
      pathPoints.push(new THREE.Vector3(data.base[0], 0.015, -data.base[1]));
      if (pathPoints.length > 500) pathPoints.shift();
      pathPoints.forEach((point, index) => point.toArray(pathPositions, index * 3));
      pathAttribute.needsUpdate = true; pathGeometry.setDrawRange(0, pathPoints.length);
    }
  } else if (data.type === 'worldChanged') {
    bodyNames = data.bodyNames; selectedObstacle = data.selectedId;
    $('#editStatus').textContent = '변경 완료 · Stand로 정지했습니다. Walk로 재개하세요.';
    editorPending = true;
  } else if (data.type === 'editError') {
    $('#editStatus').textContent = data.message;
  } else if (data.type === 'waypoint' && data.event === 'reached') {
    const reached = waypoints.find(waypoint => !waypoint.reached
      && Math.hypot(waypoint.x - data.waypoint.x, waypoint.y - data.waypoint.y) < 0.01)
      || waypoints.find(waypoint => !waypoint.reached);
    if (reached) { reached.reached = true; reached.reward = data.reward || 0; }
    rebuildWaypoints();
  }
};

worker.onerror = event => { $('#status').textContent = 'ERROR'; $('#status').className = 'fault'; $('#fault').textContent = event.message; };

async function boot() {
  const xml = await fetch(`${BASE}a2/a2.xml`).then(response => {
    if (!response.ok) throw new Error(`A2 model load failed: ${response.status}`);
    return response.text();
  });
  worker.postMessage({ type: 'init', xml, terrain });
}

function command() {
  return {
    vx: Number($('#vx').value), vy: Number($('#vy').value), yawRate: Number($('#yaw').value), frequency: Number($('#frequency').value),
  };
}

function sendCommand() {
  const value = command(); worker.postMessage({ type: 'command', command: value, avoidance: $('#avoidanceEnabled').checked });
  $('#vxValue').textContent = value.vx.toFixed(2); $('#vyValue').textContent = value.vy.toFixed(2);
  $('#yawValue').textContent = value.yawRate.toFixed(2); $('#frequencyValue').textContent = value.frequency.toFixed(1);
}

for (const id of ['vx', 'vy', 'yaw', 'frequency']) $(`#${id}`).addEventListener('input', sendCommand);
for (const button of document.querySelectorAll('[data-mode]')) button.onclick = () => {
  const mode = button.dataset.mode;
  worker.postMessage({ type: 'mode', mode });
  document.querySelectorAll('[data-mode]').forEach(item => item.classList.toggle('active', item === button));
};
$('#reset').onclick = () => { archiveWaypoints(false); worker.postMessage({ type: 'reset' }); };
$('#demo').onclick = () => {
  archiveWaypoints(false); worker.postMessage({ type: 'reset' });
  pathPoints.length = 0; pathGeometry.setDrawRange(0, 0);
  $('#vx').value = '0.60'; $('#vy').value = '0'; $('#yaw').value = '0'; $('#frequency').value = '1.4';
  sendCommand(); worker.postMessage({ type: 'mode', mode: 'walk' });
};
$('#fastWalk').onclick = () => {
  archiveWaypoints();
  $('#vx').value = '0.60'; $('#vy').value = '0'; $('#yaw').value = '0'; $('#frequency').value = '1.4';
  sendCommand(); worker.postMessage({ type: 'mode', mode: 'walk' });
};
$('#resetObstacle').onclick = () => {
  for (const obstacle of worldObstacles(terrain)) moveObstacle(obstacle.id, obstacle.x, obstacle.y);
  selectedObstacle = null; updateObstacleState(); populateObstacleEditor();
};
$('#avoidanceEnabled').onchange = sendCommand;
$('#clearWaypoints').onclick = clearWaypoints;
$('#follow').onchange = event => { following = event.target.checked; };

const raycaster = new THREE.Raycaster(), pointer = new THREE.Vector2();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), groundHit = new THREE.Vector3();
let pointerDown = null, draggingObstacle = null, selectedObstacle = null, dragSurface = null;
addEventListener('pointerdown', event => {
  if (event.target !== canvas && event.target !== robotViewCanvas) return;
  const viewCamera = event.target === robotViewCanvas ? robotCamera : camera;
  setPointerRay(event.clientX, event.clientY, event.target, viewCamera);
  const hit = raycaster.intersectObjects([...obstacleMeshes.values()], false)[0];
  if (hit) {
    pointerDown = null;
    if (hit.object.userData.movable === false) { pointerDown = null; return; }
    draggingObstacle = hit.object.userData.obstacleId;
    selectedObstacle = draggingObstacle;
    populateObstacleEditor();
    dragSurface = { canvas: event.target, camera: viewCamera };
    controls.enabled = false;
    updateObstacleState();
    event.preventDefault(); event.stopPropagation();
    return;
  }
  pointerDown = [event.clientX, event.clientY];
}, { capture: true });
addEventListener('pointermove', event => {
  if (!draggingObstacle) return;
  const point = groundPointFromScreen(event.clientX, event.clientY, dragSurface.canvas, dragSurface.camera);
  if (point) moveObstacle(draggingObstacle, point.x, -point.z);
  event.preventDefault(); event.stopPropagation();
}, { capture: true });
addEventListener('pointerup', event => {
  if (draggingObstacle) {
    draggingObstacle = null; dragSurface = null; controls.enabled = true; updateObstacleState();
    event.preventDefault(); event.stopPropagation();
    return;
  }
  if (event.target === canvas && pointerDown && Math.hypot(event.clientX - pointerDown[0], event.clientY - pointerDown[1]) <= 5) addWaypointFromScreen(event.clientX, event.clientY);
  pointerDown = null;
}, { capture: true });

function setPointerRay(clientX, clientY, sourceCanvas = canvas, viewCamera = camera) {
  const rect = sourceCanvas.getBoundingClientRect();
  pointer.set((clientX - rect.left) / rect.width * 2 - 1, -(clientY - rect.top) / rect.height * 2 + 1);
  viewCamera.updateWorldMatrix(true, false);
  raycaster.setFromCamera(pointer, viewCamera);
}

function groundPointFromScreen(clientX, clientY, sourceCanvas = canvas, viewCamera = camera) {
  setPointerRay(clientX, clientY, sourceCanvas, viewCamera);
  return raycaster.ray.intersectPlane(groundPlane, groundHit) ? groundHit.clone() : null;
}

function addWaypointFromScreen(clientX, clientY) {
  const point = groundPointFromScreen(clientX, clientY);
  if (!point) return false;
  const previous = waypoints.filter(waypoint => !waypoint.reached).at(-1);
  const segmentStart = previous ? { x: previous.x, y: previous.y }
    : { x: lastPose?.base[0] ?? 0, y: lastPose?.base[1] ?? 0 };
  const waypoint = { x: point.x, y: -point.z, reached: false, segmentStart };
  waypoints.push(waypoint); rebuildWaypoints();
  worker.postMessage({ type: 'addWaypoint', waypoint });
  return true;
}

function moveObstacle(id, x, y) {
  const obstacle = obstacleState.get(id);
  const mesh = obstacleMeshes.get(id);
  if (!obstacle || obstacle.movable === false || !mesh || !Number.isFinite(x) || !Number.isFinite(y)) return false;
  obstacle.x = x; obstacle.y = y;
  mesh.position.set(x, obstacle.halfZ, -y);
  worker.postMessage({ type: 'moveObstacle', id, x, y });
  updateObstacleState();
  return true;
}

function updateObstacleState() {
  const label = $('#obstacleState');
  for (const [id, mesh] of obstacleMeshes) mesh.material.emissive.set(id === selectedObstacle ? '#344b60' : '#000000');
  if (!selectedObstacle) {
    label.textContent = 'NONE'; label.className = '';
  } else {
    const obstacle = obstacleState.get(selectedObstacle);
    if (!obstacle) return;
    const action = isTraversable(obstacle) ? '통과' : '회피';
    label.textContent = `${draggingObstacle ? 'MOVING ' : ''}${selectedObstacle} · ${obstacle.halfZ * 200}cm · ${action}`;
    label.className = draggingObstacle ? 'avoid' : 'nav';
  }
}

function populateObstacleEditor() {
  const o = obstacleState.get(selectedObstacle);
  if (o) { $('#obstacleWidth').value = o.halfX * 2; $('#obstacleDepth').value = o.halfY * 2; $('#obstacleHeight').value = o.halfZ * 2; }
  $('#deleteObstacle').disabled = !o; $('#resizeObstacle').disabled = !o;
}
function editedObstacle() {
  const o = obstacleState.get(selectedObstacle);
  return { x: o?.x ?? (lastPose?.base[0] ?? 0) + 2, y: o?.y ?? (lastPose?.base[1] ?? 0) + 2,
    halfX: Number($('#obstacleWidth').value) / 2, halfY: Number($('#obstacleDepth').value) / 2,
    halfZ: Number($('#obstacleHeight').value) / 2, kind: $('#obstacleKind').value };
}
function submitObstacleEdit(action) {
  if (!ready) return;
  const obstacle = editedObstacle();
  if (action !== 'delete' && !['obstacleWidth', 'obstacleDepth', 'obstacleHeight'].every(id => $(`#${id}`).reportValidity())) return;
  if (action === 'create') { obstacle.x = (lastPose?.base[0] ?? 0) + 2; obstacle.y = (lastPose?.base[1] ?? 0) + 2; }
  $('#editStatus').textContent = '적용 중…';
  worker.postMessage({ type: 'editObstacle', action, id: selectedObstacle, obstacle });
}
$('#createObstacle').onclick = () => submitObstacleEdit('create');
$('#resizeObstacle').onclick = () => submitObstacleEdit('resize');
$('#deleteObstacle').onclick = () => submitObstacleEdit('delete');

function clearWaypoints() {
  waypoints.length = 0; rebuildWaypoints(); worker.postMessage({ type: 'clearWaypoints' });
}

function archiveWaypoints(cancelNavigation = true) {
  waypoints.forEach(waypoint => { waypoint.reached = true; });
  rebuildWaypoints();
  if (cancelNavigation) worker.postMessage({ type: 'clearWaypoints' });
}

function rebuildWaypoints() {
  waypointGroup.clear();
  const current = waypoints.find(waypoint => !waypoint.reached);
  waypoints.forEach(waypoint => {
    const color = waypoint.reached ? '#65e5a5' : waypoint === current ? '#ffb341' : '#ffd166';
    const marker = new THREE.Group(); marker.position.set(waypoint.x, 0.012, -waypoint.y);
    const ring = new THREE.Mesh(new THREE.RingGeometry(WAYPOINT_RADIUS * 0.68, WAYPOINT_RADIUS, 32), new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide }));
    ring.rotation.x = -Math.PI / 2; marker.add(ring);
    const pinHeight = waypoint.reached ? 0.08 : 0.22;
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, pinHeight, 12), new THREE.MeshBasicMaterial({ color }));
    pin.position.y = pinHeight / 2; marker.add(pin); waypointGroup.add(marker);
  });
  const pending = waypoints.filter(waypoint => !waypoint.reached);
  const origin = pending[0]?.segmentStart;
  const start = origin ? new THREE.Vector3(origin.x, 0.02, -origin.y) : new THREE.Vector3(0, 0.02, 0);
  waypointLine.geometry.dispose(); waypointLine.geometry = new THREE.BufferGeometry().setFromPoints([start, ...pending.map(point => new THREE.Vector3(point.x, 0.02, -point.y))]);
  waypointLine.computeLineDistances(); waypointLine.visible = pending.length > 0;
}

function updateFootprint(base, bodyYaw) {
  const heading = bodyYaw;
  const forward = [Math.cos(heading), Math.sin(heading)];
  const left = [-forward[1], forward[0]];
  const length = A2_FOOTPRINT.halfLength + A2_FOOTPRINT.margin;
  const width = A2_FOOTPRINT.halfWidth + A2_FOOTPRINT.margin;
  [[1, 1], [1, -1], [-1, -1], [-1, 1]].forEach(([f, l], index) => {
    footprintPositions[index * 3] = base[0] + f * length * forward[0] + l * width * left[0];
    footprintPositions[index * 3 + 1] = 0.014;
    footprintPositions[index * 3 + 2] = -(base[1] + f * length * forward[1] + l * width * left[1]);
  });
  footprintGeometry.attributes.position.needsUpdate = true;
}

const keyState = new Set();
addEventListener('keydown', event => {
  keyState.add(event.code);
  if (event.code === 'Space') { event.preventDefault(); worker.postMessage({ type: 'mode', mode: 'passive' }); }
  updateKeys();
});
addEventListener('keyup', event => { keyState.delete(event.code); updateKeys(); });
function updateKeys() {
  $('#vx').value = keyState.has('KeyW') ? 0.6 : keyState.has('KeyS') ? -0.4 : 0;
  $('#yaw').value = keyState.has('KeyA') ? 0.35 : keyState.has('KeyD') ? -0.35 : 0;
  sendCommand();
}

function renderTelemetry(data) {
  $('#mode').textContent = data.mode.toUpperCase();
  document.querySelectorAll('[data-mode]').forEach(item => item.classList.toggle('active', item.dataset.mode === data.mode));
  $('#time').textContent = data.time.toFixed(1);
  $('#height').textContent = data.base[2].toFixed(3);
  $('#distance').textContent = Math.hypot(data.base[0], data.base[1]).toFixed(2);
  $('#speedState').textContent = `${data.forwardSpeed.toFixed(2)} m/s`;
  $('#assistState').textContent = `${data.forwardServoAcceleration.toFixed(2)} m/s² · ${(data.locomotionStabilityScale * 100).toFixed(0)}% stable`;
  $('#attitude').textContent = `${(data.orientation.roll * 57.3).toFixed(1)}° / ${(data.orientation.pitch * 57.3).toFixed(1)}°`;
  $('#trackingState').textContent = `${data.base[1].toFixed(2)}m / ${(data.orientation.yaw * 57.3).toFixed(1)}°`;
  $('#positionState').textContent = `${data.base[0].toFixed(2)}m / ${data.base[1].toFixed(2)}m`;
  $('#fault').textContent = data.fault || 'none';
  $('#fault').className = data.fault ? 'fault' : '';
  const avoiding = data.avoidance?.active;
  $('#avoidanceState').textContent = avoiding
    ? data.avoidance.strategy?.startsWith('passage')
      ? ({ passage: '통로 직진', 'passage-align': '통로 방향 정렬', 'passage-retreat': '회전 공간 확보', 'passage-blocked': '통로 여유 부족 · 정지' })[data.avoidance.strategy]
      : `${data.avoidance.strategy === 'docking' ? 'DOCKING' : data.avoidance.strategy === 'bypass' ? 'BYPASS' : data.avoidance.strategy === 'arc' ? 'ARC' : 'SIDESTEP'} ${data.avoidance.direction > 0 ? 'LEFT' : 'RIGHT'}` : data.navigation.blocked ? 'GOAL BLOCKED' : 'CLEAR';
  $('#avoidanceState').className = avoiding ? 'avoid' : '';
  const completedWaypoints = waypoints.filter(waypoint => waypoint.reached).length;
  $('#waypointState').textContent = data.navigation?.active
    ? `${data.navigation.remaining} · ${data.navigation.distance.toFixed(2)}m`
    : waypoints.length ? `${completedWaypoints}/${waypoints.length} SAVED` : 'IDLE';
  $('#waypointState').className = data.navigation?.active ? 'nav' : completedWaypoints ? 'complete' : '';
  const rewardRecent = data.reward?.lastValue > 0 && data.time - data.reward.lastAt < 1.2;
  $('#rewardState').textContent = rewardRecent ? `+${data.reward.lastValue} · TOTAL ${data.reward.total}` : `TOTAL ${data.reward?.total || 0}`;
  $('#rewardState').className = rewardRecent ? 'complete' : '';
  $('#footprintState').textContent = avoiding && Number.isFinite(data.avoidance.lateralClearance)
    ? `${data.avoidance.lateralClearance.toFixed(2)}m` : '1.12 × 0.84m';
  $('#footprintState').className = avoiding ? 'avoid' : '';
  sensorCone.material.color.set(avoiding ? '#ff8a3d' : '#65e5a5'); sensorCone.material.opacity = avoiding ? 0.24 : 0.10;
  footprintLine.material.color.set(avoiding ? '#ff8a3d' : '#65e5a5');
  for (const [id, mesh] of obstacleMeshes) {
    const glow = id === draggingObstacle ? '#7a4300' : id === selectedObstacle ? '#402600' : id === data.avoidance?.obstacle && avoiding ? '#6b2500' : '#000000';
    mesh.material.emissive.set(glow);
  }
  const nearest = Math.min(...[...obstacleState.values()].map(obstacle => Math.max(0, Math.hypot(data.base[0] - obstacle.x, data.base[1] - obstacle.y) - Math.hypot(obstacle.halfX, obstacle.halfY))));
  $('#viewRange').textContent = `LiDAR ${nearest.toFixed(2)}m`;
  $('#viewMode').textContent = rewardRecent ? `REWARD +${data.reward.lastValue}` : avoiding ? 'AVOIDING'
    : data.navigation?.recovering ? 'REALIGNING' : data.navigation?.active ? 'TRACKING' : data.mode.toUpperCase();
}

function resize() {
  const width = canvas.clientWidth, height = canvas.clientHeight;
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.setSize(width, height, false);
  camera.aspect = width / height; camera.updateProjectionMatrix();
  const viewWidth = robotViewCanvas.clientWidth, viewHeight = robotViewCanvas.clientHeight;
  robotRenderer.setPixelRatio(Math.min(devicePixelRatio, 1.5)); robotRenderer.setSize(viewWidth, viewHeight, false);
  robotCamera.aspect = viewWidth / viewHeight; robotCamera.updateProjectionMatrix();
}
addEventListener('resize', resize); resize();

function animate() {
  requestAnimationFrame(animate);
  if (following && lastPose) {
    const target = new THREE.Vector3(lastPose.base[0], 0.45, -lastPose.base[1]);
    controls.target.lerp(target, 0.05);
  }
  controls.update(); renderer.render(scene, camera);
  if (robotCameraReady) robotRenderer.render(scene, robotCamera);
}

boot().catch(error => { $('#status').textContent = 'LOAD FAILED'; $('#status').className = 'fault'; $('#fault').textContent = error.message; });
window.__a2 = {
  worker, addWaypointFromScreen, moveObstacle,
  obstacleScreen(id) {
    const mesh = obstacleMeshes.get(id); if (!mesh) return null;
    const point = mesh.position.clone().project(camera), rect = canvas.getBoundingClientRect();
    return { x: rect.left + (point.x + 1) * rect.width / 2, y: rect.top + (1 - point.y) * rect.height / 2 };
  },
  get pose() { return lastPose; }, get waypoints() { return waypoints; }, get obstacles() { return [...obstacleState.values()]; },
};
sendCommand(); animate();
