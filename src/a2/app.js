import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { ARM_STOW_POSE } from './config.js';
import { ARM_ACTIONS, armWorkPose } from './arm-work.js';
import { A2_FOOTPRINT } from './avoidance.js';
import { WAYPOINT_RADIUS } from './navigation.js';
import { worldObstacles, isTraversable, terrainColor } from './terrain.js';
import { actorShape, actorHeading } from './dynamic.js';
import { gamepadCommand, webManualCommand } from './manual.js';
import { buildJetsonBox, buildBodyBranding, renderBeacon, EquipmentBeacon } from './equipment.js';
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
const rearCamera = robotCamera.clone();
rearCamera.quaternion.premultiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI));
let cameraSide = 'front';
const activeRobotCamera = () => cameraSide === 'front' ? robotCamera : rearCamera;
for (const side of ['front', 'rear']) $(`#${side}Camera`).onclick = () => {
  cameraSide = side;
  for (const name of ['front', 'rear']) $(`#${name}Camera`).classList.toggle('active', name === side);
  $('#viewDirection').textContent = `${side.toUpperCase()} · H-FOV 78° (가정)`;
};
let robotCameraReady = false;
const scene = new THREE.Scene();
scene.background = new THREE.Color('#0b1118');
scene.fog = new THREE.Fog('#0b1118', 8, 25);
const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 80);
camera.position.set(2.6, 2.1, 3.2);
camera.layers.enable(1);
const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0.35, 0);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight('#d9edff', '#18202a', 1.7));
const sun = new THREE.DirectionalLight('#fff3dc', 3.2);
sun.position.set(4, 7, 4); sun.castShadow = true;
Object.assign(sun.shadow.camera, { left: -5, right: 5, top: 5, bottom: -5, near: 0.1, far: 20 });
scene.add(sun);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: '#252d35', roughness: 0.9, metalness: 0.05 }));
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
const grid = new THREE.GridHelper(20, 40, '#557086', '#344451'); grid.position.y = 0.002; scene.add(grid);
const robot = new THREE.Group(); robot.rotation.x = -Math.PI / 2; scene.add(robot);
const jetson = buildJetsonBox();
const equipmentBeacon = new EquipmentBeacon();
const brandCanvas = document.createElement('canvas');
brandCanvas.width = 1024; brandCanvas.height = 192;
const brandContext = brandCanvas.getContext('2d');
brandContext.fillStyle = '#142331'; brandContext.fillRect(0, 0, 1024, 192);
brandContext.font = 'bold 142px Arial, sans-serif';
brandContext.textAlign = 'center'; brandContext.textBaseline = 'middle';
brandContext.fillStyle = '#ffffff'; brandContext.fillText('TeamGRIT', 512, 100);
const brandTexture = new THREE.CanvasTexture(brandCanvas);
brandTexture.colorSpace = THREE.SRGBColorSpace;
brandTexture.anisotropy = renderer.capabilities.getMaxAnisotropy();
const bodyBranding = buildBodyBranding(brandTexture);

const terrain = new URLSearchParams(location.search).get('terrain') || 'flat';
const obstacleState = new Map(worldObstacles(terrain).map(obstacle => [obstacle.id, obstacle]));
const obstacleMeshes = new Map();
function createObstacleMesh(obstacle) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(obstacle.halfX * 2, obstacle.halfZ * 2, obstacle.halfY * 2), new THREE.MeshStandardMaterial({ color: terrainColor(obstacle), roughness: 0.62, metalness: 0.08 }));
  mesh.position.set(obstacle.x, obstacle.halfZ, -obstacle.y); mesh.castShadow = mesh.receiveShadow = true;
  mesh.rotation.y = actorHeading(obstacle);
  mesh.userData.obstacleId = obstacle.id;
  mesh.userData.movable = obstacle.movable;
  mesh.userData.shape = `${obstacle.halfX},${obstacle.halfY},${obstacle.halfZ}`;
  if (obstacle.kind === 'person') {
    mesh.material.transparent = true; mesh.material.opacity = 0.18;
    const torso = new THREE.Mesh(new THREE.CylinderGeometry(obstacle.halfX * 0.7, obstacle.halfX * 0.8, obstacle.halfZ, 12), new THREE.MeshStandardMaterial({ color: '#49b7ef' }));
    torso.name = 'person_torso'; mesh.add(torso);
    const head = new THREE.Mesh(new THREE.SphereGeometry(Math.min(obstacle.halfX, obstacle.halfZ * 0.23), 12, 10), new THREE.MeshStandardMaterial({ color: '#f1c49b' }));
    head.position.y = obstacle.halfZ * 0.72; mesh.add(head);
    for (const sign of [-1, 1]) { const leg = new THREE.Mesh(new THREE.BoxGeometry(obstacle.halfX * 0.45, obstacle.halfZ * 0.7, obstacle.halfY), new THREE.MeshStandardMaterial({ color: '#24394d' })); leg.position.set(sign * obstacle.halfX * 0.45, -obstacle.halfZ * 0.6, 0); mesh.add(leg); }
  } else if (obstacle.kind === 'car') {
    mesh.material.color.set('#e6c347');
    const front = new THREE.Mesh(new THREE.BoxGeometry(0.04, obstacle.halfZ * 0.25, obstacle.halfY * 1.5), new THREE.MeshStandardMaterial({ color: '#fff5bf', emissive: '#594f2a' }));
    front.position.x = obstacle.halfX + 0.01; mesh.add(front);
    const cabin = new THREE.Mesh(new THREE.BoxGeometry(obstacle.halfX, obstacle.halfZ * 0.6, obstacle.halfY * 1.5), new THREE.MeshStandardMaterial({ color: '#244458' }));
    cabin.position.y = obstacle.halfZ * 0.3; mesh.add(cabin);
    for (const x of [-1, 1]) for (const z of [-1, 1]) { const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.18, 0.18, 0.12, 12), new THREE.MeshStandardMaterial({ color: '#161a20' })); wheel.rotation.x = Math.PI / 2; wheel.position.set(x * obstacle.halfX * 0.65, -obstacle.halfZ * 0.7, z * obstacle.halfY); mesh.add(wheel); }
  }
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.48, 0.50, 48), new THREE.MeshBasicMaterial({ color: '#ffb341', transparent: true, opacity: 0.35, side: THREE.DoubleSide }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = -obstacle.halfZ + 0.004; mesh.add(ring);
  scene.add(mesh); obstacleMeshes.set(obstacle.id, mesh);
}
for (const obstacle of obstacleState.values()) createObstacleMesh(obstacle);

function syncObstacles(objects) {
  const ids = new Set(objects.map(o => o.id));
  if (selectedObstacle && !ids.has(selectedObstacle)) { selectedObstacle = null; editorPending = true; }
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
let waypointSerial = 0;
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

  // PiPER-X-inspired six-axis visual chain, not calibrated manufacturer kinematics.
  // J1 base yaw, J2 shoulder, J3 elbow, J4 forearm roll, J5 wrist pitch, J6 tool roll.
  const forearmRoll = new THREE.Group(); forearmRoll.name = 'arm_forearm_roll'; forearmRoll.position.z = 0.30; elbow.add(forearmRoll);
  const rollHousing = new THREE.Mesh(new THREE.CylinderGeometry(0.058, 0.058, 0.09, 20), shell);
  rollHousing.rotation.x = Math.PI / 2; forearmRoll.add(rollHousing);
  const wrist = new THREE.Group(); wrist.name = 'arm_wrist'; wrist.rotation.y = ARM_STOW_POSE.wristPitch; forearmRoll.add(wrist);
  const wristJoint = new THREE.Mesh(new THREE.SphereGeometry(0.065, 20, 14), accent); wrist.add(wristJoint);
  const toolRoll = new THREE.Group(); toolRoll.name = 'arm_tool_roll'; wrist.add(toolRoll);
  const toolFlange = new THREE.Mesh(new THREE.CylinderGeometry(0.052, 0.052, 0.04, 20), dark);
  toolFlange.rotation.x = Math.PI / 2; toolFlange.position.z = 0.045; toolRoll.add(toolFlange);
  const palm = new THREE.Mesh(new THREE.BoxGeometry(0.13, 0.16, 0.09), shell); palm.position.z = 0.09; toolRoll.add(palm);
  for (const side of [-1, 1]) {
    const finger = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.035, 0.17), dark);
    finger.name = side < 0 ? 'gripper_left' : 'gripper_right';
    finger.position.set(0, side * ARM_STOW_POSE.gripperGap, 0.20); finger.rotation.x = side * 0.08; toolRoll.add(finger);
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
    group.add(frontArrow, buildRobotArm(), jetson.group, bodyBranding);
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
let manualMode = false, manualStopped = false, lastManualSend = 0, hadGamepad = false;
let editorMode = false, savedView = null;
function fitEditor() {
  const points = [{ x: lastPose?.base[0] || 0, y: lastPose?.base[1] || 0 }, ...waypoints, ...obstacleState.values()];
  const xs = points.map(p => p.x), ys = points.map(p => p.y);
  const x = (Math.min(...xs) + Math.max(...xs)) / 2, y = (Math.min(...ys) + Math.max(...ys)) / 2;
  const span = Math.max(5, Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys));
  controls.target.set(x, 0, -y); camera.position.set(x, Math.min(65, span * 2.4), -y + .001); controls.update();
}
function setEditor(enabled) {
  if (enabled === editorMode || !ready) return;
  if (enabled) savedView = { position: camera.position.clone(), target: controls.target.clone(), following };
  editorMode = enabled; setManualMode(false);
  worker.postMessage({ type: 'routeEditor', enabled });
  for (const id of ['vx', 'vy', 'yaw']) $(`#${id}`).value = '0';
  $('#driveTools').hidden = enabled; $('#driveDetails').hidden = enabled; $('#editorTools').hidden = !enabled;
  document.body.classList.toggle('editing-route', enabled);
  $('#editView').classList.toggle('active', enabled); $('#driveView').classList.toggle('active', !enabled);
  controls.enableRotate = !enabled; controls.mouseButtons.LEFT = enabled ? THREE.MOUSE.PAN : THREE.MOUSE.ROTATE;
  following = enabled ? false : savedView?.following ?? true;
  if (enabled) fitEditor();
  else if (savedView) { camera.position.copy(savedView.position); controls.target.copy(savedView.target); controls.update(); }
  rebuildWaypoints(); resize();
}
$('#editView').onclick = () => setEditor(true);
$('#driveView').onclick = () => setEditor(false);
$('#editorFit').onclick = fitEditor;
$('#editorStop').onclick = () => emergencyStop();
function syncEditedRoute() {
  let start = { x: lastPose?.base[0] || 0, y: lastPose?.base[1] || 0 };
  for (const w of waypoints.filter(w => !w.reached)) { w.segmentStart = start; start = { x: w.x, y: w.y }; }
  worker.postMessage({ type: 'editRoute', waypoints: waypoints.filter(w => !w.reached) }); rebuildWaypoints();
}
let bodyNames = [], ready = false, lastPose = null, following = true, editorPending = false;

worker.onmessage = ({ data }) => {
  if (data.type === 'ready') {
    bodyNames = data.bodyNames;
    for (const name of bodyNames) {
      const part = partFor(name); if (!part.children.length) continue;
      bodyGroups.set(name, part); robot.add(part);
      if (name === 'base_link') {
        part.add(sensorCone, robotCamera, rearCamera); robotCamera.position.set(0.43, 0, 0.18);
        rearCamera.position.set(-0.36, 0, 0.08); robotCameraReady = true;
      }
    }
    ready = true; $('#status').textContent = 'READY'; $('#status').className = 'ok'; worker.postMessage({ type: 'run' });
  } else if (data.type === 'pose') {
    lastPose = data;
    const armPose = data.armWork?.pose || armWorkPose();
    const arm = bodyGroups.get('base_link')?.getObjectByName('single_arm');
    if (arm) {
      arm.rotation.z = armPose.yaw;
      arm.getObjectByName('arm_shoulder').rotation.y = armPose.shoulder;
      arm.getObjectByName('arm_elbow').rotation.y = armPose.elbow;
      arm.getObjectByName('arm_wrist').rotation.y = armPose.wrist;
    }
    equipmentBeacon.receive(data, performance.now() / 1000);
    syncObstacles(data.obstacles || []);
    for (const obstacle of data.obstacles || []) {
      obstacleState.set(obstacle.id, { ...obstacle });
      const mesh = obstacleMeshes.get(obstacle.id);
      if (mesh) { mesh.position.set(obstacle.x, obstacle.halfZ, -obstacle.y); mesh.rotation.y = actorHeading(obstacle); }
      const torso = mesh?.getObjectByName('person_torso');
      if (torso) torso.material.color.set(obstacle.yielding === 'waiting' ? '#ffc857'
        : ['sidestep', 'returning'].includes(obstacle.yielding) ? '#52e3bd' : '#49b7ef');
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
    bodyNames = data.bodyNames;
    if (!data.automatic) { selectedObstacle = data.selectedId; $('#editStatus').textContent = '변경 완료 · Stand로 정지했습니다. Walk로 재개하세요.'; }
    editorPending = true;
  } else if (data.type === 'editError') {
    equipmentBeacon.event('장애물 편집 거부', performance.now() / 1000);
    $('#editStatus').textContent = data.message;
  } else if (data.type === 'waypoint' && data.event === 'work-start') {
    equipmentBeacon.event('경로점 팔 작업', performance.now() / 1000);
    rebuildWaypoints();
  } else if (data.type === 'waypoint' && data.event === 'reached') {
    equipmentBeacon.event('경로점 도착', performance.now() / 1000);
    const reached = waypoints.find(waypoint => data.waypoint.id && waypoint.id === data.waypoint.id) || waypoints.find(waypoint => !waypoint.reached
      && Math.hypot(waypoint.x - data.waypoint.x, waypoint.y - data.waypoint.y) < 0.01)
      || waypoints.find(waypoint => !waypoint.reached);
    if (reached) { reached.reached = true; reached.reward = data.reward || 0; }
    rebuildWaypoints();
  }
};

worker.onerror = event => { equipmentBeacon.error = event.message || 'worker 오류'; $('#status').textContent = 'ERROR'; $('#status').className = 'fault'; $('#fault').textContent = event.message; };

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
  if (manualMode) return;
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
$('#personYield').onchange = () => worker.postMessage({ type: 'personYield', enabled: $('#personYield').checked });
$('#clearWaypoints').onclick = clearWaypoints;
$('#follow').onchange = event => { following = event.target.checked; };

const raycaster = new THREE.Raycaster(), pointer = new THREE.Vector2();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), groundHit = new THREE.Vector3();
let pointerDown = null, draggingObstacle = null, selectedObstacle = null, dragSurface = null;
addEventListener('pointerdown', event => {
  if (event.target !== canvas && event.target !== robotViewCanvas) return;
  if (!editorMode || event.button !== 0) return;
  const viewCamera = event.target === robotViewCanvas ? activeRobotCamera() : camera;
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
  if (!editorMode) return false;
  const point = groundPointFromScreen(clientX, clientY);
  if (!point) return false;
  const previous = waypoints.filter(waypoint => !waypoint.reached).at(-1);
  const segmentStart = previous ? { x: previous.x, y: previous.y }
    : { x: lastPose?.base[0] ?? 0, y: lastPose?.base[1] ?? 0 };
  const waypoint = { id: `waypoint_${++waypointSerial}`, action: $('#waypointAction').value, x: point.x, y: -point.z, reached: false, segmentStart };
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
$('#obstacleKind').onchange = () => {
  const kind = $('#obstacleKind').value;
  if (kind === 'person' || kind === 'car') {
    const o = actorShape(kind); $('#obstacleWidth').value = o.halfX * 2; $('#obstacleDepth').value = o.halfY * 2; $('#obstacleHeight').value = o.halfZ * 2;
  }
};
$('#randomActors').onchange = event => worker.postMessage({ type: 'randomActors', enabled: event.target.checked });
$('#autoRecover').onchange = event => worker.postMessage({ type: 'autoRecover', enabled: event.target.checked });

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
  $('#waypointActions').replaceChildren();
  const current = waypoints.find(waypoint => !waypoint.reached);
  waypoints.forEach((waypoint, index) => {
    const label = document.createElement('label');
    label.textContent = `${index + 1}. (${waypoint.x.toFixed(1)}, ${waypoint.y.toFixed(1)})${waypoint.reached ? ' 완료' : ''} `;
    const select = document.createElement('select'); select.dataset.waypointId = waypoint.id;
    for (const [value, text] of Object.entries(ARM_ACTIONS)) select.add(new Option(text, value));
    select.value = waypoint.action || 'none'; select.disabled = waypoint.reached;
    select.onchange = () => { waypoint.action = select.value; worker.postMessage({ type: 'waypointAction', id: waypoint.id, action: waypoint.action }); rebuildWaypoints(); };
    label.append(select); $('#waypointActions').append(label);
    if (editorMode && !waypoint.reached) {
      const row = document.createElement('div'); row.className = 'waypoint-edit';
      for (const axis of ['x', 'y']) {
        const input = document.createElement('input'); input.type = 'number'; input.step = '.1'; input.value = waypoint[axis].toFixed(2);
        input.setAttribute('aria-label', `경로점 ${index + 1} ${axis.toUpperCase()} 좌표`);
        input.onchange = () => { const value = input.valueAsNumber; if (Number.isFinite(value)) { waypoint[axis] = value; syncEditedRoute(); } else input.value = waypoint[axis]; };
        row.append(input);
      }
      for (const [title, offset] of [['앞 순서', -1], ['뒤 순서', 1], ['삭제', 0]]) {
        const button = document.createElement('button'); button.textContent = title;
        button.setAttribute('aria-label', `경로점 ${index + 1} ${title}`);
        button.disabled = offset !== 0 && (!waypoints[index + offset] || waypoints[index + offset].reached);
        button.onclick = () => {
          if (!offset) waypoints.splice(index, 1);
          else [waypoints[index], waypoints[index + offset]] = [waypoints[index + offset], waypoints[index]];
          syncEditedRoute();
        }; row.append(button);
      }
      $('#waypointActions').append(row);
    }
    const color = waypoint.reached ? '#65e5a5' : waypoint.action && waypoint.action !== 'none' ? '#c994ff' : waypoint === current ? '#ffb341' : '#ffd166';
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
let pointerDirection = '';
for (const button of document.querySelectorAll('[data-drive]')) {
  button.style.touchAction = 'none';
  button.onpointerdown = event => {
    if (event.button !== 0 || !manualMode || manualStopped) return;
    event.preventDefault(); button.setPointerCapture(event.pointerId); pointerDirection = button.dataset.drive;
  };
  const release = () => { pointerDirection = ''; worker.postMessage({ type: 'manualCommand', held: false, command: {} }); };
  button.onpointerup = release; button.onpointercancel = release; button.onlostpointercapture = release;
}
$('#manualArmWork').onclick = () => { keyState.clear(); pointerDirection = ''; worker.postMessage({ type: 'manualArm', action: $('#manualArmAction').value }); };
$('#manualArmHome').onclick = () => worker.postMessage({ type: 'manualArm', action: 'home' });
addEventListener('keydown', event => {
  if (event.code === 'Space') { event.preventDefault(); emergencyStop(); return; }
  if (editorMode) return;
  if (['INPUT', 'SELECT', 'TEXTAREA'].includes(event.target.tagName)) return;
  if (manualMode && event.code.startsWith('Arrow')) event.preventDefault();
  keyState.add(event.code);
  if (manualMode) return;
  updateKeys();
});
addEventListener('keyup', event => { keyState.delete(event.code); if (!editorMode) updateKeys(); });
function updateKeys() {
  if (manualMode) return;
  $('#vx').value = keyState.has('KeyW') ? 0.6 : keyState.has('KeyS') ? -0.4 : 0;
  $('#yaw').value = keyState.has('KeyA') ? 0.35 : keyState.has('KeyD') ? -0.35 : 0;
  sendCommand();
}

function emergencyStop() {
  equipmentBeacon.event('긴급 정지', performance.now() / 1000);
  manualStopped = true; keyState.clear(); pointerDirection = ''; worker.postMessage({ type: 'emergencyStop' });
  $('#manualStatus').textContent = '비상 정지 · Manual Mode 버튼을 다시 눌러 재활성화하세요.';
}
function setManualMode(enabled) {
  $('#manualControls').open = enabled;
  manualMode = enabled; manualStopped = false; hadGamepad = false; keyState.clear(); pointerDirection = '';
  worker.postMessage({ type: 'manualMode', enabled });
  for (const id of ['demo', 'fastWalk', 'vx', 'vy', 'yaw']) $(`#${id}`).disabled = enabled;
  for (const button of document.querySelectorAll('[data-mode]')) button.disabled = false;
  $('#manualMode').classList.toggle('active', enabled);
  $('#manualStatus').textContent = enabled ? '수동 · Walk 선택 후 RB/Shift를 놓았다 누르고 조종하세요.' : '자동 모드 · Walk를 눌러 경로를 재개하세요.';
}
$('#manualMode').onclick = () => setManualMode(true);
$('#automaticMode').onclick = () => setManualMode(false);
$('#emergencyStop').onclick = emergencyStop;
addEventListener('blur', () => { if (manualMode) emergencyStop(); });
document.addEventListener('visibilitychange', () => { if (document.hidden && manualMode) emergencyStop(); });
addEventListener('gamepaddisconnected', () => { if (manualMode) emergencyStop(); });
function pollManual() {
  if (!ready || performance.now() - lastManualSend < 50) return;
  lastManualSend = performance.now();
  const pad = Array.from(navigator.getGamepads?.() || []).find(p => p?.connected);
  if (pad?.mapping === 'standard' && pad.buttons[1]?.pressed) { emergencyStop(); return; }
  if (!manualMode || manualStopped) return;
  if (hadGamepad && !pad) { emergencyStop(); return; }
  hadGamepad = !!pad;
  let input;
  const webInput = webManualCommand(keyState, pointerDirection);
  if (pad && !webInput.held) {
    if (pad.mapping !== 'standard') { $('#manualStatus').textContent = '비표준 조종기: 매핑 설정 필요 · 키보드 사용 시 조종기를 분리하세요.'; worker.postMessage({ type: 'manualCommand', held: false, command: {} }); return; }
    input = gamepadCommand(pad);
    $('#manualStatus').textContent = `${pad.id} · Walk 선택 후 RB를 놓았다 누르고 조종`;
  } else {
    input = webInput;
    $('#manualStatus').textContent = 'Walk: 버튼/화살표를 누르면 이동, 놓으면 정지 · Shift+W/S 전후, A/D 회전, Q/E 옆걸음';
  }
  if (input.emergency) { emergencyStop(); return; }
  worker.postMessage({ type: 'manualCommand', ...input });
}

function renderTelemetry(data) {
  const work = data.armWork;
  for (const button of document.querySelectorAll('[data-drive]')) button.disabled = !manualMode || manualStopped || data.mode !== 'walk' || !!work?.active;
  $('#manualArmWork').disabled = !manualMode || manualStopped || data.mode !== 'stand' || !!data.fault || !!work?.active;
  $('#manualArmHome').disabled = !manualMode || manualStopped || !work?.active;
  $('#armWorkState').textContent = work?.active ? `${ARM_ACTIONS[work.action]} · ${{ settling: '정착', extending: '뻗기', working: '작업', folding: '접기' }[work.stage]}` : '접힘';
  for (const select of document.querySelectorAll('#waypointActions select')) {
    select.disabled = waypoints.find(w => w.id === select.dataset.waypointId)?.reached || (work?.active && work.id === select.dataset.waypointId);
  }
  const people = (data.obstacles || []).filter(o => o.kind === 'person');
  $('#pedestrianState').textContent = `양보 ${people.filter(o => o.yielding === 'sidestep').length} · 복귀 ${people.filter(o => o.yielding === 'returning').length} · 대기 ${people.filter(o => o.yielding === 'waiting').length}`;
  $('#perceptionHealth').textContent = data.perception?.health === 'ok' ? '가상 센서 입력 정상 · 위험 추적 10Hz'
    : data.perception?.health === 'degraded' ? '영상 입력 누락/지연 · 거리만 사용'
      : '센서 입력 불명 · 안전한 공간으로 판단하지 않음';
  for (const side of ['front', 'rear']) {
    const risk = data.perception?.[side];
    const status = { clear: '여유', near: '근접', warning: '접근 위험', critical: '충돌 임박', unknown: '입력 불명' }[risk?.level] || '초기화';
    $(`#${side}Risk`).textContent = `${side === 'front' ? '전방' : '후방'}: ${status}`
      + (risk?.clearance != null ? ` · ${risk.clearance.toFixed(2)}m` : '')
      + (risk?.ttc != null ? ` · TTC ${risk.ttc.toFixed(1)}s` : '')
      + (risk?.kind && risk.kind !== 'unknown' ? ` · ${risk.kind}` : '');
    $(`#${side}Risk`).className = risk?.level === 'critical' ? 'fault' : risk?.level === 'warning' ? 'avoid' : '';
  }
  $('#recoveryState').textContent = data.recovery === 'waiting' ? '복구 대기 · 주변 장애물이 지나갈 때까지 정지'
    : data.recovery ? '시뮬레이션 자세 보조 복구 중 · 실제 관절 자력 기상 아님'
      : '옆넘어짐 시 주변 공간 확보 후 3초 자세 보조 · 최대 2회 · Passive로 취소';
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
    ? data.avoidance.strategy === 'rear-buffer-departure' ? '후방 여유 확보 · 저속 전진'
      : data.avoidance.strategy === 'dynamic-dodge' ? `이동체 회피 · ${data.avoidance.direction > 0 ? '왼쪽' : '오른쪽'}`
      : data.avoidance.strategy === 'dynamic-retreat' ? '이동체 회피 · 짧은 후퇴'
      : data.avoidance.strategy === 'detour-blocked' ? '우회 경로 없음 · 정지'
      : data.avoidance.strategy === 'detour' ? '밀집 장애물 우회'
      : data.avoidance.strategy === 'dynamic-escape' ? '후방 접근 · 전진 탈출' : data.avoidance.strategy === 'dynamic-wait' ? (data.avoidance.blocked ? '이동체 · 안전 회피 경로 없음' : '이동 장애물 대기') : data.avoidance.strategy?.startsWith('passage')
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
  const clearance = data.perception?.[cameraSide]?.clearance;
  $('#viewRange').textContent = clearance == null ? 'SIM 여유거리 --' : `SIM 여유거리 ${clearance.toFixed(2)}m`;
  $('#viewMode').textContent = rewardRecent ? `REWARD +${data.reward.lastValue}` : avoiding ? 'AVOIDING'
    : data.navigation?.recovering ? 'REALIGNING' : data.navigation?.active ? 'TRACKING' : data.mode.toUpperCase();
}

function resize() {
  const width = canvas.clientWidth, height = canvas.clientHeight;
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); renderer.setSize(width, height, false);
  camera.aspect = width / height; camera.updateProjectionMatrix();
  const viewWidth = robotViewCanvas.clientWidth, viewHeight = robotViewCanvas.clientHeight;
  robotRenderer.setPixelRatio(Math.min(devicePixelRatio, 1.5)); robotRenderer.setSize(viewWidth, viewHeight, false);
  robotCamera.aspect = viewWidth / viewHeight;
  robotCamera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(Math.tan(THREE.MathUtils.degToRad(39)) / robotCamera.aspect));
  robotCamera.updateProjectionMatrix();
  rearCamera.aspect = robotCamera.aspect; rearCamera.fov = robotCamera.fov; rearCamera.updateProjectionMatrix();
}
addEventListener('resize', resize); resize();

function animate() {
  requestAnimationFrame(animate);
  pollManual();
  const beacon = equipmentBeacon.sample(performance.now() / 1000, { ready, stopped: manualMode && manualStopped });
  renderBeacon(jetson.lamps, beacon);
  $('#beaconState').textContent = beacon.label;
  $('#beaconState').dataset.state = beacon.mode;
  $('#beaconState').className = beacon.mode === 'error' ? 'fault' : beacon.mode === 'event' ? 'avoid' : '';
  if (following && lastPose) {
    const target = new THREE.Vector3(lastPose.base[0], 0.45, -lastPose.base[1]);
    controls.target.lerp(target, 0.05);
  }
  controls.update(); renderer.render(scene, camera);
  if (robotCameraReady) robotRenderer.render(scene, activeRobotCamera());
}

boot().catch(error => { equipmentBeacon.error = error.message; $('#status').textContent = 'LOAD FAILED'; $('#status').className = 'fault'; $('#fault').textContent = error.message; });
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
