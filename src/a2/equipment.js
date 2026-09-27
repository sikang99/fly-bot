import * as THREE from 'three';

// Visual enclosure dimensions, not measured hardware mass/collision properties.
export const JETSON_BOX = Object.freeze({ length: 0.22, width: 0.21, height: 0.11, x: -0.21, z: 0.165 });

export function buildBodyBranding(texture) {
  const group = new THREE.Group(); group.name = 'teamgrit_body_labels';
  const material = new THREE.MeshBasicMaterial({ map: texture, toneMapped: false });
  for (const side of [-1, 1]) {
    const label = new THREE.Mesh(new THREE.PlaneGeometry(.42, .078), material);
    label.name = side > 0 ? 'teamgrit_left' : 'teamgrit_right';
    label.position.set(0, side * .121, .018);
    // Both faces have world/body +Z as text up and an outward normal.
    // Reverse the local text-right axis on the opposite side, never mirror UVs.
    label.quaternion.setFromRotationMatrix(new THREE.Matrix4().makeBasis(
      new THREE.Vector3(-side, 0, 0), new THREE.Vector3(0, 0, 1), new THREE.Vector3(0, side, 0)));
    group.add(label);
  }
  return group;
}

export function buildJetsonBox() {
  const group = new THREE.Group(); group.name = 'jetson_orin_nx_16gb';
  group.position.set(JETSON_BOX.x, 0, JETSON_BOX.z);
  const metal = new THREE.MeshStandardMaterial({ color: '#344657', metalness: .6, roughness: .4 });
  const black = new THREE.MeshStandardMaterial({ color: '#101820', metalness: .35, roughness: .55 });
  const addBox = (name, size, position, material) => {
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), material);
    mesh.name = name; mesh.position.set(...position); group.add(mesh); return mesh;
  };
  addBox('jetson_enclosure', [JETSON_BOX.length, JETSON_BOX.width, JETSON_BOX.height], [0, 0, 0], metal);
  for (const y of [-.075, .075]) addBox('mount_rail', [.20, .025, .02], [0, y, -.065], black);
  for (let i = 0; i < 7; i++) addBox('cooling_fin', [.009, .17, .009], [-.075 + i * .025, 0, .0595], black);
  for (const y of [-.06, 0, .06]) addBox('rear_port', [.005, .036, .02], [-.112, y, -.01], black);
  addBox('jetson_badge', [.085, .003, .022], [.015, -.107, .008],
    new THREE.MeshStandardMaterial({ color: '#8dc63f', emissive: '#163000', roughness: .5 }));
  addBox('beacon_base', [.06, .145, .02], [-.055, 0, .079], black);
  // Visual mushroom E-stop, separate from the rear warning lamps.
  // This mesh is not a hardware safety circuit or an interactive stop control.
  const stop = new THREE.Group(); stop.name = 'jetson_emergency_stop';
  stop.position.set(.055, 0, .065); group.add(stop);
  for (const [name, radius, height, z, color] of [
    ['estop_yellow_collar', .037, .008, .004, '#ffd229'],
    ['estop_stem', .016, .012, .014, '#20252b'],
    ['estop_red_mushroom', .028, .021, .0305, '#e61922'],
  ]) {
    const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, height, 32),
      new THREE.MeshStandardMaterial({ color, roughness: .36 }));
    mesh.name = name; mesh.rotation.x = Math.PI / 2; mesh.position.z = z; stop.add(mesh);
  }
  const lamps = {};
  for (const [name, color, y] of [['red', '#ff2028', .039], ['green', '#23ff70', -.039]]) {
    const material = new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0, roughness: .22 });
    const lens = new THREE.Mesh(new THREE.CylinderGeometry(.024, .029, .045, 20), material);
    lens.name = 'beacon_' + name; lens.rotation.x = Math.PI / 2;
    lens.position.set(-.055, y, .11); group.add(lens); lamps[name] = lens;
  }
  group.traverse(object => { if (object.isMesh) object.castShadow = object.receiveShadow = true; });
  return { group, lamps };
}

export function renderBeacon(lamps, state) {
  for (const color of ['red', 'green']) {
    lamps[color].material.emissiveIntensity = state[color] ? 3 : 0;
    lamps[color].material.color.set(state[color] ? (color === 'red' ? '#ff3038' : '#35ff80') : (color === 'red' ? '#400b10' : '#083a1c'));
  }
}

// All timers use wall seconds so worker failure cannot freeze a blinking error lamp.
export class EquipmentBeacon {
  constructor() { this.reset(); }
  reset() { this.pose = null; this.lastAt = -Infinity; this.eventUntil = -Infinity; this.eventName = ''; this.movingUntil = -Infinity; this.error = ''; }
  event(name, now) { this.eventName = name; this.eventUntil = now + 3; }
  receive(pose, now) {
    if (this.pose && pose.time < this.pose.time) this.reset();
    if (pose.reward?.total > (this.pose?.reward?.total ?? 0)) this.event('경로점 도착', now);
    const dt = pose.time - (this.pose?.time ?? pose.time);
    const speed = dt > 0 ? Math.hypot(pose.base[0] - this.pose.base[0], pose.base[1] - this.pose.base[1]) / dt : Math.abs(pose.forwardSpeed || 0);
    const angle = this.pose ? pose.orientation.yaw - this.pose.orientation.yaw : 0;
    const yawSpeed = dt > 0 ? Math.abs(Math.atan2(Math.sin(angle), Math.cos(angle))) / dt : 0;
    if (pose.mode === 'walk' && (speed > .025 || yawSpeed > .04)) this.movingUntil = now + .25;
    this.pose = pose; this.lastAt = now;
    if (['warning', 'critical'].includes(pose.perception?.rear?.level)) this.event('후방 위험 감지 (SIM)', now);
    else if (['warning', 'critical'].includes(pose.perception?.front?.level)) this.event('전방 위험 감지 (SIM)', now);
    else if (pose.avoidance?.active) this.event('장애물 감지·회피', now);
    else if (pose.navigation?.blocked) this.event('경로 차단', now);
    else if (pose.recovery) this.event('자세 복구', now);
  }
  sample(now, { ready = false, stopped = false } = {}) {
    const fault = this.error || this.pose?.fault || (ready && now - this.lastAt > 1 ? '상태 통신 끊김' : '');
    if (fault) return { mode: 'error', red: true, green: false, label: '적색 고정 · ' + fault };
    if (stopped || now < this.eventUntil) return { mode: 'event', red: Math.floor(now * 4) % 2 === 0, green: false,
      label: '적색 점멸 · ' + (stopped ? '긴급 정지' : this.eventName) };
    if (this.pose?.mode === 'walk' && now < this.movingUntil) {
      const red = Math.floor(now * 2) % 2 === 0;
      return { mode: 'moving', red, green: !red, label: '이동 · 적녹 교대' };
    }
    return { mode: 'idle', red: false, green: false, label: ready ? '대기 · 소등' : '초기화 · 소등' };
  }
}
