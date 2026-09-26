import loadMujoco from '@mujoco/mujoco';
import { A2WalkingController } from './controller.js';
import { A2_OBSTACLES, JOINT_ORDER, VELOCITY_ASSIST_PROFILE, accelerationLimitedVelocity, lowPass, sanitizeCommand, tractionStabilityScale, updateStabilityEnvelope } from './config.js';
import { A2_FOOTPRINT, obstacleRelativeGeometry, planNearbyDocking, planObstacleAvoidance, waypointRouteClear } from './avoidance.js';
import { WAYPOINT_PROGRESS_EPSILON, alignmentYawRate, assessWaypointProgress, planSegmentCommand, planWaypointRecovery, waypointNeedsRecovery } from './navigation.js';
import { buildA2WorldXml } from './model.js';
import { planPassage } from './passage.js';
import { isTraversable, worldObstacles } from './terrain.js';

let mj, model, data, controller, running = false, timer = null, lastReal = 0, simBudget = 0, lastPose = 0;
let jointQ = {}, jointV = {}, actuator = {}, bodyNames = [];
let requestedCommand = null, avoidanceEnabled = true, avoidance = { active: false, direction: 0, clearance: Infinity };
let waypoints = [], navigation = { active: false, remaining: 0, distance: Infinity, headingError: 0 };
let reward = { total: 0, lastValue: 0, lastAt: -1e9 };
let alignmentAnchor = null;
let cruiseHeading = null;
let assistSpeedScale = 0.75;
let filteredForwardSpeed = 0;
let forwardServoAcceleration = 0;
let servoWorldVelocity = null;
let servoForwardVelocity = 0, servoLeftVelocity = 0;
let locomotionStabilityScale = 1;
let obstacles = A2_OBSTACLES.map(obstacle => ({ ...obstacle })), obstacleMocap = {};
let sourceXml, terrainName, objectSerial = 0;

globalThis.onmessage = async ({ data: message }) => {
  if (message.type === 'init') await init(message);
  else if (message.type === 'mode') controller?.setMode(message.mode);
  else if (message.type === 'command') { requestedCommand = sanitizeCommand(message.command); avoidanceEnabled = message.avoidance !== false; controller?.setCommand(requestedCommand); }
  else if (message.type === 'addWaypoint') {
    waypoints.push({ x: message.waypoint.x, y: message.waypoint.y, tracking: false,
      bestDistance: Infinity, lastProgressAt: data?.time ?? 0, recovering: false });
    controller?.setMode('walk');
  }
  else if (message.type === 'clearWaypoints') { waypoints = []; navigation = { active: false, remaining: 0, distance: Infinity, headingError: 0 }; reward = { total: 0, lastValue: 0, lastAt: -1e9 }; alignmentAnchor = null; controller?.setMode('stand'); }
  else if (message.type === 'moveObstacle') moveObstacle(message.id, message.x, message.y);
  else if (message.type === 'editObstacle') editObstacle(message);
  else if (message.type === 'run') start();
  else if (message.type === 'pause') stop();
  else if (message.type === 'reset') reset();
};

async function init(message) {
  sourceXml = message.xml; terrainName = message.terrain;
  obstacles = worldObstacles(message.terrain);
  obstacleMocap = {};
  mj = await loadMujoco();
  model = mj.MjModel.from_xml_string(buildA2WorldXml(message.xml, message.terrain));
  data = new mj.MjData(model);
  controller = new A2WalkingController();
  bodyNames = Array.from({ length: model.nbody }, (_, i) => model.body(i).name);
  for (const name of JOINT_ORDER) {
    const joint = model.jnt(`${name}_joint`);
    jointQ[name] = model.jnt_qposadr[joint.id];
    jointV[name] = model.jnt_dofadr[joint.id];
    actuator[name] = model.actuator(name).id;
  }
  for (const obstacle of obstacles) obstacleMocap[obstacle.id] = model.body_mocapid[model.body(`obstacle_${obstacle.id}`).id];
  reset();
  postMessage({ type: 'ready', bodyNames, timestep: model.opt.timestep });
  postPose();
}

function reset() {
  if (!data) return;
  mj.mj_resetData(model, data);
  data.qpos[0] = 0; data.qpos[1] = 0; data.qpos[2] = 0.62;
  data.qpos[3] = 1; data.qpos[4] = 0; data.qpos[5] = 0; data.qpos[6] = 0;
  for (const name of JOINT_ORDER) data.qpos[jointQ[name]] = name.endsWith('_hip') ? (name[1] === 'R' ? 0.1 : -0.1) : name.endsWith('_thigh') ? 0.9 : -1.8;
  controller = new A2WalkingController();
  controller.setMode('stand');
  requestedCommand = null;
  waypoints = [];
  navigation = { active: false, remaining: 0, distance: Infinity, headingError: 0 };
  reward = { total: 0, lastValue: 0, lastAt: -1e9 };
  alignmentAnchor = null;
  cruiseHeading = null;
  assistSpeedScale = 0.75;
  filteredForwardSpeed = 0;
  forwardServoAcceleration = 0;
  servoWorldVelocity = null;
  servoForwardVelocity = 0; servoLeftVelocity = 0;
  locomotionStabilityScale = 1;
  for (const obstacle of obstacles) applyObstaclePose(obstacle);
  avoidance = { active: false, direction: 0, clearance: Infinity };
  mj.mj_forward(model, data);
  postPose();
}

function start() {
  if (running || !data) return;
  running = true; lastReal = performance.now(); loop();
}

function stop() {
  running = false; clearTimeout(timer);
}

function quatToEuler(w, x, y, z) {
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
}

function controlStep(dt) {
  const position = {}, velocity = {};
  for (const name of JOINT_ORDER) { position[name] = data.qpos[jointQ[name]]; velocity[name] = data.qvel[jointV[name]]; }
  const orientation = quatToEuler(data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]);
  controller.checkSafety({ ...orientation, height: data.qpos[2] });
  const measuredStability = tractionStabilityScale({ ...orientation, height: data.qpos[2], verticalSpeed: data.qvel[2] });
  locomotionStabilityScale = updateStabilityEnvelope(locomotionStabilityScale, measuredStability,
    VELOCITY_ASSIST_PROFILE.stabilityRecoveryRate, dt);
  controller.setMotionScale(locomotionStabilityScale);
  updateAvoidance(orientation.yaw, updateNavigation(orientation.yaw));
  const target = controller.targets(dt);
  applyPlanarVelocityAssist(orientation, dt);
  const torque = controller.torques(position, velocity, target);
  for (const name of JOINT_ORDER) data.ctrl[actuator[name]] = torque[name];
  return { position, target, orientation };
}

function applyPlanarVelocityAssist(orientation, dt) {
  data.qfrc_applied.fill(0);
  if (controller.mode !== 'walk') { servoWorldVelocity = null; servoForwardVelocity = 0; servoLeftVelocity = 0; return; }
  const command = controller.gaitCommand, yaw = orientation.yaw;
  const forwardX = Math.cos(yaw), forwardY = Math.sin(yaw);
  const leftX = -forwardY, leftY = forwardX;
  const targetSpeedScale = avoidance.active ? VELOCITY_ASSIST_PROFILE.avoidanceScale : VELOCITY_ASSIST_PROFILE.cruiseScale;
  const speedScaleStep = 0.03 * dt;
  assistSpeedScale += Math.max(-speedScaleStep, Math.min(speedScaleStep, targetSpeedScale - assistSpeedScale));
  const speedScale = assistSpeedScale;
  let targetForwardVelocity = speedScale * command.vx;
  let targetLeftVelocity = Math.max(-VELOCITY_ASSIST_PROFILE.maxLateralServoSpeed,
    Math.min(VELOCITY_ASSIST_PROFILE.maxLateralServoSpeed, speedScale * command.vy));
  if (alignmentAnchor) {
    const correctionX = Math.max(-0.2, Math.min(0.2, (alignmentAnchor[0] - data.qpos[0]) * 1.8));
    const correctionY = Math.max(-0.2, Math.min(0.2, (alignmentAnchor[1] - data.qpos[1]) * 1.8));
    targetForwardVelocity = correctionX * forwardX + correctionY * forwardY;
    targetLeftVelocity = correctionX * leftX + correctionY * leftY;
  }
  targetForwardVelocity *= locomotionStabilityScale;
  targetLeftVelocity *= locomotionStabilityScale;
  const previousForwardVelocity = servoForwardVelocity;
  servoForwardVelocity = accelerationLimitedVelocity(servoForwardVelocity, targetForwardVelocity,
    VELOCITY_ASSIST_PROFILE.velocityServoGain, VELOCITY_ASSIST_PROFILE.maxPlanarAcceleration * locomotionStabilityScale,
    VELOCITY_ASSIST_PROFILE.maxPlanarDeceleration, dt);
  servoLeftVelocity = accelerationLimitedVelocity(servoLeftVelocity, targetLeftVelocity,
    VELOCITY_ASSIST_PROFILE.velocityServoGain, VELOCITY_ASSIST_PROFILE.maxPlanarAcceleration * locomotionStabilityScale,
    VELOCITY_ASSIST_PROFILE.maxPlanarDeceleration, dt);
  servoWorldVelocity = [servoForwardVelocity * forwardX + servoLeftVelocity * leftX,
    servoForwardVelocity * forwardY + servoLeftVelocity * leftY];
  forwardServoAcceleration = (servoForwardVelocity - previousForwardVelocity) / dt;
  data.qfrc_applied[3] = Math.max(-60, Math.min(60, -orientation.roll * 100 - data.qvel[3] * 16));
  data.qfrc_applied[4] = Math.max(-70, Math.min(70, -orientation.pitch * 110 - data.qvel[4] * 18));
  const heldHeading = avoidance.active
    ? avoidance.strategy === 'arc' ? null : avoidance.heading
    : cruiseHeading;
  const headingError = Number.isFinite(heldHeading)
    ? Math.atan2(Math.sin(heldHeading - yaw), Math.cos(heldHeading - yaw)) : 0;
  const desiredYawRate = navigation.aligning && !avoidance.active
    ? alignmentYawRate(navigation.headingError, locomotionStabilityScale)
    : Number.isFinite(heldHeading) ? Math.max(-0.3, Math.min(0.3, headingError * 1.8)) : command.yawRate;
  data.qfrc_applied[5] = Math.max(-45, Math.min(45, (desiredYawRate - data.qvel[5]) * 180));
}

function updateNavigation(yaw) {
  while (waypoints.length) {
    const current = waypoints[0];
    current.segmentStart ||= { x: data.qpos[0], y: data.qpos[1] };
    let plan = planSegmentCommand({ x: data.qpos[0], y: data.qpos[1], heading: yaw }, current.segmentStart, current, { tracking: current.tracking });
    const progress = assessWaypointProgress(plan.distance);
    if (plan.arrived && avoidanceEnabled && obstacles.some(o => !isTraversable(o)
      && Math.abs(current.x - o.x) <= o.halfX && Math.abs(current.y - o.y) <= o.halfY)) {
      navigation = { active: true, blocked: true, remaining: waypoints.length, distance: plan.distance, target: current };
      avoidance = { active: false, direction: 0, clearance: Infinity };
      alignmentAnchor = [data.qpos[0], data.qpos[1]];
      return sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 });
    }
    if (!plan.arrived) {
      // Detours need not reduce waypoint distance. Do not start recovery while
      // avoidance owns steering; its held heading would prevent recovery from
      // finishing while the recovery anchor prevents the detour from moving.
      if (avoidance.active) {
        current.recovering = false;
        current.bestDistance = plan.distance;
        current.lastProgressAt = data.time;
      }
      if (!current.tracking && !plan.aligning) {
        current.tracking = true; current.bestDistance = plan.distance; current.lastProgressAt = data.time;
      } else if (current.tracking) {
        if (plan.distance < current.bestDistance - WAYPOINT_PROGRESS_EPSILON) {
          current.bestDistance = plan.distance; current.lastProgressAt = data.time;
        }
        const recovery = planWaypointRecovery(plan.headingError);
        const stalled = !avoidance.active && waypointNeedsRecovery(plan.distance, current.bestDistance, data.time - current.lastProgressAt);
        if ((current.recovering || stalled) && !recovery.aligned) {
          current.recovering = true;
          plan = { ...plan, aligning: true, command: recovery.command };
        } else if (current.recovering) {
          current.recovering = false; current.bestDistance = plan.distance; current.lastProgressAt = data.time;
        }
      }
      if (plan.aligning && !alignmentAnchor) alignmentAnchor = [data.qpos[0], data.qpos[1]];
      else if (!plan.aligning) { alignmentAnchor = null; current.tracking = true; }
      navigation = { active: true, aligning: plan.aligning, recovering: current.recovering,
        remaining: waypoints.length, target: current, distance: plan.distance, headingError: plan.headingError };
      return plan.command;
    }
    alignmentAnchor = null;
    const reached = waypoints.shift();
    if (waypoints.length) waypoints[0].segmentStart = { x: reached.x, y: reached.y };
    avoidance = { active: false, direction: 0, clearance: Infinity };
    if (progress.reward) reward = { total: reward.total + progress.reward, lastValue: progress.reward, lastAt: data.time };
    postMessage({ type: 'waypoint', event: 'reached', waypoint: reached, reward: progress.reward, totalReward: reward.total });
  }
  if (navigation.active) controller.setMode('stand');
  alignmentAnchor = null;
  navigation = { active: false, remaining: 0, distance: Infinity, headingError: 0 };
  return null;
}

function updateAvoidance(yaw, navigationCommand) {
  const command = navigationCommand || requestedCommand || controller.command;
  const straightCruise = !navigationCommand && command.vx > 0.05 && Math.abs(command.vy) < 0.05 && Math.abs(command.yawRate) < 0.03;
  if (straightCruise && !Number.isFinite(cruiseHeading)) cruiseHeading = yaw;
  else if (!straightCruise) cruiseHeading = null;
  const pose = { x: data.qpos[0], y: data.qpos[1], yaw };
  const target = waypoints[0];
  if (navigation.blocked) { controller.setCommand(command); return; }
  const passage = avoidanceEnabled && planPassage(pose, target, obstacles);
  if (passage) {
    avoidance = { active: true, strategy: passage.strategy, heading: passage.heading, direction: 0, clearance: Infinity };
    if (passage.strategy === 'passage-align' || passage.strategy === 'passage-blocked') alignmentAnchor ||= [pose.x, pose.y];
    else alignmentAnchor = null;
    navigation.aligning = false; navigation.recovering = false;
    target.recovering = false; target.bestDistance = navigation.distance; target.lastProgressAt = data.time;
    controller.setCommand(passage.command);
    return;
  }
  const docking = avoidanceEnabled && planNearbyDocking(pose, target, obstacles);
  if (docking) {
    const heading = avoidance.strategy === 'docking' ? avoidance.heading : yaw;
    avoidance = { active: true, strategy: 'docking', heading, direction: Math.sign(docking.vy), clearance: Infinity };
    alignmentAnchor = null;
    navigation.aligning = false; navigation.recovering = false;
    target.recovering = false; target.bestDistance = navigation.distance; target.lastProgressAt = data.time;
    controller.setCommand(docking);
    return;
  }
  // A detour is subordinate to the waypoint, never an independent cruise.
  if (avoidanceEnabled && target && waypointRouteClear(pose, target, obstacles)) {
    if (avoidance.active) {
      target.tracking = false; target.recovering = false;
      avoidance = { active: false, direction: 0, clearance: Infinity };
      controller.setCommand(updateNavigation(yaw));
    } else controller.setCommand(command);
    return;
  }
  const plan = planObstacleAvoidance(pose, command, obstacles, target);
  const latchedObstacle = obstacles.find(obstacle => obstacle.id === avoidance.obstacle);
  const latchedGeometry = latchedObstacle ? obstacleRelativeGeometry(pose, latchedObstacle) : null;
  const manualStop = !navigation.active && command.vx <= 0.05;
  // Finish lateral clearance before resuming forward motion, even if the
  // instantaneous planner would switch back to an arc near the edge.
  if (avoidanceEnabled && avoidance.active && avoidance.strategy === 'sidestep'
    && !manualStop && latchedGeometry && !latchedGeometry.behind
    && latchedGeometry.lateralClearance < A2_FOOTPRINT.bypassMargin) {
    Object.assign(plan, { active: true, strategy: 'sidestep', direction: avoidance.direction,
      obstacle: latchedObstacle.id, clearance: latchedGeometry.clearance,
      lateralClearance: latchedGeometry.lateralClearance, collisionRisk: true,
      command: sanitizeCommand({ ...command, vx: 0, vy: avoidance.direction * 0.38, yawRate: 0 }) });
  }
  const bypassing = avoidanceEnabled && avoidance.active && !plan.active && !manualStop
    && latchedGeometry && !latchedGeometry.behind;
  const active = avoidanceEnabled && (plan.active || bypassing);
  const sameObstacle = avoidance.obstacle === plan.obstacle;
  const direction = avoidance.active && active && (sameObstacle || bypassing) ? avoidance.direction : plan.direction;
  const heading = plan.active && plan.strategy === 'arc' ? yaw
    : avoidance.active && active && (sameObstacle || bypassing) && Number.isFinite(avoidance.heading) ? avoidance.heading : yaw;
  const plannedCommand = bypassing
    ? sanitizeCommand({ ...command, vx: Math.min(0.42, Math.max(0.32, command.vx)), vy: 0, yawRate: 0 })
    : plan.command;
  avoidance = { active, strategy: bypassing ? 'bypass' : plan.strategy, direction, heading,
    clearance: bypassing ? latchedGeometry.clearance : plan.clearance,
    lateralClearance: bypassing ? latchedGeometry.lateralClearance : plan.lateralClearance,
    collisionRisk: !bypassing && plan.collisionRisk,
    obstacle: bypassing ? latchedObstacle.id : plan.obstacle };
  if (active) {
    alignmentAnchor = null;
    navigation.aligning = false;
    navigation.recovering = false;
    if (waypoints.length) {
      waypoints[0].recovering = false;
      waypoints[0].bestDistance = navigation.distance;
      waypoints[0].lastProgressAt = data.time;
    }
  }
  const avoidanceCommand = { ...plannedCommand, vy: direction * Math.abs(plannedCommand.vy), yawRate: direction * Math.abs(plannedCommand.yawRate) };
  controller.setCommand(avoidance.active ? avoidanceCommand : command);
}

function moveObstacle(id, x, y) {
  const obstacle = obstacles.find(item => item.id === id);
  if (!obstacle || obstacle.movable === false || !Number.isFinite(x) || !Number.isFinite(y)) return;
  obstacle.x = x; obstacle.y = y; applyObstaclePose(obstacle);
  mj.mj_forward(model, data);
  postPose();
}

function editObstacle(message) {
  if (!model) return;
  let next = obstacles.map(o => ({ ...o }));
  const index = next.findIndex(o => o.id === message.id);
  if (message.action === 'delete') {
    if (index < 0) return;
    next.splice(index, 1);
  } else {
    const o = message.obstacle;
    if (!o || !['x', 'y', 'halfX', 'halfY', 'halfZ'].every(k => Number.isFinite(o[k]))
      || o.halfX < 0.05 || o.halfX > 2 || o.halfY < 0.05 || o.halfY > 2 || o.halfZ < 0.005 || o.halfZ > 1
      || Math.abs(o.x) > 18 || Math.abs(o.y) > 18) { postMessage({ type: 'editError', message: '크기 또는 위치가 허용 범위를 벗어났습니다.' }); return; }
    if (message.action === 'create') {
      if (next.length >= 32) { postMessage({ type: 'editError', message: '장애물은 최대 32개입니다.' }); return; }
      next.push({ id: `custom_${++objectSerial}`, x: o.x, y: o.y, halfX: o.halfX, halfY: o.halfY, halfZ: o.halfZ, kind: o.kind === 'step' ? 'step' : 'box', movable: true });
    } else if (message.action === 'resize' && index >= 0) next[index] = { ...next[index], halfX: o.halfX, halfY: o.halfY, halfZ: o.halfZ };
    else return;
  }
  // Recompile to refresh collision broad-phase bounds as well as geom sizes.
  // Free joints and actuators are unchanged by mocap-only obstacle edits.
  let replacement, replacementData;
  try {
    replacement = mj.MjModel.from_xml_string(buildA2WorldXml(sourceXml, terrainName, next));
    replacementData = new mj.MjData(replacement);
  } catch (error) { replacement?.delete(); postMessage({ type: 'editError', message: String(error) }); return; }
  replacementData.qpos.set(data.qpos); replacementData.qvel.set(data.qvel); replacementData.time = data.time;
  data.delete(); model.delete(); model = replacement; data = replacementData; obstacles = next;
  bodyNames = Array.from({ length: model.nbody }, (_, i) => model.body(i).name);
  obstacleMocap = {};
  for (const o of obstacles) { obstacleMocap[o.id] = model.body_mocapid[model.body(`obstacle_${o.id}`).id]; applyObstaclePose(o); }
  controller.setMode('stand'); requestedCommand = null; servoWorldVelocity = null;
  servoForwardVelocity = 0; servoLeftVelocity = 0; alignmentAnchor = null;
  avoidance = { active: false, direction: 0, clearance: Infinity };
  for (const waypoint of waypoints) { waypoint.tracking = false; waypoint.recovering = false; }
  mj.mj_forward(model, data);
  postMessage({ type: 'worldChanged', bodyNames, selectedId: message.action === 'create' ? next.at(-1).id : message.action === 'delete' ? null : message.id });
  postPose();
}

function applyObstaclePose(obstacle) {
  if (!data || obstacleMocap[obstacle.id] == null) return;
  const address = obstacleMocap[obstacle.id] * 3;
  data.mocap_pos[address] = obstacle.x; data.mocap_pos[address + 1] = obstacle.y; data.mocap_pos[address + 2] = obstacle.halfZ;
}

export function step() {
  const beforeX = data.qpos[0], beforeY = data.qpos[1];
  controlStep(model.opt.timestep);
  mj.mj_step(model, data);
  if (controller.mode === 'walk' && servoWorldVelocity) {
    data.qpos[0] += servoWorldVelocity[0] * model.opt.timestep;
    data.qpos[1] += servoWorldVelocity[1] * model.opt.timestep;
    mj.mj_forward(model, data);
  }
  const yaw = quatToEuler(data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]).yaw;
  const measuredForwardSpeed = ((data.qpos[0] - beforeX) * Math.cos(yaw) + (data.qpos[1] - beforeY) * Math.sin(yaw)) / model.opt.timestep;
  filteredForwardSpeed = lowPass(filteredForwardSpeed, measuredForwardSpeed, VELOCITY_ASSIST_PROFILE.velocityFilterTimeConstant, model.opt.timestep);
}

export function postPose() {
  if (!data) return;
  lastPose = performance.now();
  const xpos = Float32Array.from(data.xpos);
  const xquat = Float32Array.from(data.xquat);
  const orientation = quatToEuler(data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]);
  const forwardSpeedInstantaneous = data.qvel[0] * Math.cos(orientation.yaw) + data.qvel[1] * Math.sin(orientation.yaw);
  postMessage({ type: 'pose', time: data.time, xpos, xquat, base: [data.qpos[0], data.qpos[1], data.qpos[2]], orientation,
    velocity: [data.qvel[0], data.qvel[1], data.qvel[2]], forwardSpeed: filteredForwardSpeed, forwardSpeedInstantaneous,
    forwardServoAcceleration, assistSpeedScale, locomotionStabilityScale,
    mode: controller.mode, command: controller.command, requestedCommand, avoidance, navigation, reward, obstacles, fault: controller.fault }, [xpos.buffer, xquat.buffer]);
}

function loop() {
  if (!running) return;
  const now = performance.now();
  simBudget += Math.min(50, now - lastReal) / 1000;
  lastReal = now;
  const began = performance.now();
  while (simBudget >= model.opt.timestep && performance.now() - began < 10) {
    step(); simBudget -= model.opt.timestep;
  }
  if (now - lastPose > 1000 / 30) postPose();
  timer = setTimeout(loop, 0);
}
