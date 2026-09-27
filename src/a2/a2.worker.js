import loadMujoco from '@mujoco/mujoco';
import { A2WalkingController } from './controller.js';
import { A2_OBSTACLES, JOINT_ORDER, VELOCITY_ASSIST_PROFILE, accelerationLimitedVelocity, lowPass, sanitizeCommand, tractionStabilityScale, updateStabilityEnvelope } from './config.js';
import { A2_FOOTPRINT, obstacleRelativeGeometry, planNearbyDocking, planObstacleAvoidance, waypointRouteClear } from './avoidance.js';
import { WAYPOINT_PROGRESS_EPSILON, alignmentYawRate, assessWaypointProgress, planSegmentCommand, planWaypointRecovery, waypointNeedsRecovery } from './navigation.js';
import { buildA2WorldXml } from './model.js';
import { planPassage } from './passage.js';
import { LocalDetour } from './detour.js';
const localDetour = new LocalDetour();
import { rearEscapeSafe, rearBufferDeparture } from './dynamic-safety.js';
import { EncounterPlanner, ENCOUNTER } from './encounter.js';
import { ArmWork, armAction } from './arm-work.js';
const armWork = new ArmWork();
const encounterPlanner = new EncounterPlanner();
import { ManualControl } from './manual.js';
import { pedestrianState, stepPedestrian } from './pedestrian.js';
const pedestrians = new Map();
let personYield = true;
function clearPedestrians() {
  for (const o of obstacles) {
    const state = pedestrians.get(o.id);
    if (state) { o.vx = state.cruiseVx; o.vy = state.cruiseVy; }
    delete o.yielding;
  }
  pedestrians.clear();
}
import { RiskTracker, simulatedSensorFrame, PERCEPTION_PROFILE } from './perception.js';
const riskTracker = new RiskTracker();
let perception = null, perceptionAt = -Infinity;
const manualControl = new ManualControl();
import { isDynamic, crossingActor, actorHeading, obstacleWorldBounds } from './dynamic.js';
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
let filteredLeftSpeed = 0;
let forwardServoAcceleration = 0;
let servoWorldVelocity = null;
let servoForwardVelocity = 0, servoLeftVelocity = 0;
let locomotionStabilityScale = 1;
let obstacles = A2_OBSTACLES.map(obstacle => ({ ...obstacle })), obstacleMocap = {};
let sourceXml, terrainName, objectSerial = 0;
let randomActors = false, nextActorAt = 0, randomSeed = 12345;
let recovery = null, recoveryAttempts = 0, autoRecover = true;
function randomActorValue() { randomSeed = (1664525 * randomSeed + 1013904223) >>> 0; return randomSeed / 4294967296; }

globalThis.onmessage = async ({ data: message }) => {
  if (message.type === 'init') await init(message);
  else if (message.type === 'mode') {
    if (armWork.task) armWork.cancel();
    if (message.mode !== 'walk') encounterPlanner.reset();
    if (manualControl.enabled) {
      manualControl.setMode(message.mode);
      controller?.setMode(manualControl.latched ? 'passive' : manualControl.mode);
    } else controller?.setMode(message.mode);
    if (message.mode === 'passive') recovery = null;
  }
  else if (message.type === 'manualMode') {
    armWork.cancel();
    encounterPlanner.reset();
    manualControl.enter(!!message.enabled); recovery = null; alignmentAnchor = null; cruiseHeading = null;
    servoForwardVelocity = 0; servoLeftVelocity = 0; servoWorldVelocity = null;
    controller?.setMode('stand'); avoidance = { active: false, direction: 0, clearance: Infinity };
    for (const target of waypoints) { target.tracking = false; target.recovering = false; }
  }
  else if (message.type === 'manualArm') {
    if (!manualControl.enabled || manualControl.latched) return;
    if (message.action === 'home') { armWork.cancel(); return; }
    if (manualControl.mode !== 'stand' || controller?.fault || armWork.task || !data) return;
    armWork.start({ id: '__manual_arm', action: message.action }, data.time);
  }
  else if (message.type === 'manualCommand') manualControl.receive(message, performance.now());
  else if (message.type === 'emergencyStop') { armWork.cancel(); manualControl.stop(); recovery = null; controller?.setMode('passive'); servoWorldVelocity = null; }
  else if (message.type === 'autoRecover') { autoRecover = !!message.enabled; if (!autoRecover) recovery = null; }
  else if (message.type === 'command') { requestedCommand = sanitizeCommand(message.command); avoidanceEnabled = message.avoidance !== false; controller?.setCommand(requestedCommand); }
  else if (message.type === 'addWaypoint') {
    waypoints.push({ id: message.waypoint.id, action: armAction(message.waypoint.action), x: message.waypoint.x, y: message.waypoint.y, tracking: false,
      bestDistance: Infinity, lastProgressAt: data?.time ?? 0, recovering: false });
    if (!manualControl.enabled && !armWork.task) controller?.setMode('walk');
  }
  else if (message.type === 'waypointAction') {
    const waypoint = waypoints.find(w => w.id === message.id);
    if (waypoint && armWork.task?.id !== message.id) waypoint.action = armAction(message.action);
  }
  else if (message.type === 'clearWaypoints') { armWork.cancel(); waypoints = []; navigation = { active: false, remaining: 0, distance: Infinity, headingError: 0 }; reward = { total: 0, lastValue: 0, lastAt: -1e9 }; alignmentAnchor = null; controller?.setMode('stand'); }
  else if (message.type === 'moveObstacle') moveObstacle(message.id, message.x, message.y);
  else if (message.type === 'editObstacle') editObstacle(message);
  else if (message.type === 'randomActors') { randomActors = !!message.enabled; randomSeed = Number.isFinite(message.seed) ? message.seed >>> 0 : Date.now() >>> 0; nextActorAt = (data?.time ?? 0) + 2; }
  else if (message.type === 'personYield') { clearPedestrians(); personYield = !!message.enabled; }
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
  armWork.cancel();
  filteredLeftSpeed = 0;
  encounterPlanner.reset();
  localDetour.reset();
  clearPedestrians();
  riskTracker.reset(); perception = null; perceptionAt = -Infinity;
  if (!data) return;
  manualControl.enter(manualControl.enabled);
  recovery = null; recoveryAttempts = 0;
  mj.mj_resetData(model, data);
  nextActorAt = 2;
  for (const o of obstacles) if (isDynamic(o)) o.born = 0;
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
  const wasWalking = controller.mode === 'walk';
  controller.checkSafety({ ...orientation, height: data.qpos[2] });
  if (controller.fault) armWork.cancel();
  if (autoRecover && !manualControl.enabled && wasWalking && controller.fault && recoveryAttempts < 2 && Math.abs(orientation.roll) > 0.65) {
    recovery = { stage: 'waiting', started: data.time, yaw: orientation.yaw, resume: true };
    recoveryAttempts++;
  }
  if (recovery) { data.ctrl.fill(0); servoWorldVelocity = null; return; }
  const measuredStability = tractionStabilityScale({ ...orientation, height: data.qpos[2], verticalSpeed: data.qvel[2] });
  locomotionStabilityScale = updateStabilityEnvelope(locomotionStabilityScale, measuredStability,
    VELOCITY_ASSIST_PROFILE.stabilityRecoveryRate, dt);
  controller.setMotionScale(locomotionStabilityScale);
  if (manualControl.enabled) {
    const value = manualControl.sample(performance.now());
    // Passive naturally lowers the body; allow Stand to raise it again.
    // Walking faults and unsafe tilt still latch the emergency stop.
    if (controller.fault && (manualControl.mode === 'walk'
      || (manualControl.mode === 'stand' && controller.fault === 'body tilt above safety limit'))) manualControl.stop();
    controller.setMode(manualControl.latched ? 'passive' : manualControl.mode);
    controller.setCommand(manualControl.latched || manualControl.mode !== 'walk' ? { vx: 0, vy: 0, yawRate: 0 } : value);
    navigation = { active: false, remaining: waypoints.length, distance: Infinity, headingError: 0 };
    avoidance = { active: false, direction: 0, clearance: Infinity };
    if (armWork.task && armWork.sample(data.time).done) armWork.cancel();
  } else if (armWork.task) {
    controller.setMode('stand'); controller.setCommand({ vx: 0, vy: 0, yawRate: 0 });
    servoWorldVelocity = null;
    avoidance = { active: false, direction: 0, clearance: Infinity };
    if (armWork.sample(data.time).done) {
      if (waypoints[0]) waypoints[0].actionDone = true;
      armWork.cancel(); controller.setMode('walk');
    }
  } else updateAvoidance(orientation.yaw, updateNavigation(orientation.yaw));
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
  const escaping = avoidance.strategy === 'dynamic-escape';
  // Preview-only straight cruise. Keep the tested joint gait unchanged and
  // compensate its measured drag with bounded velocity feedback, not larger
  // open-loop strides. Reserve distance for braking before turns/obstacles.
  const clearAhead = obstacles.every(o => {
    const g = obstacleRelativeGeometry({ x: data.qpos[0], y: data.qpos[1], yaw }, o);
    return g.behind || g.clearance > VELOCITY_ASSIST_PROFILE.openStraightBrakeDistance
      || g.lateralClearance > 0.4;
  });
  const fastStraight = !manualControl.enabled && !avoidance.active && !alignmentAnchor && clearAhead
    && command.vx > 0.55 && Math.abs(command.vy) < 0.06 && Math.abs(command.yawRate) < 0.06
    && (!navigation.active || navigation.distance > VELOCITY_ASSIST_PROFILE.openStraightBrakeDistance);
  if (fastStraight) targetForwardVelocity = Math.max(0, Math.min(1.22,
    VELOCITY_ASSIST_PROFILE.openStraightSpeed + 0.15
      + 0.5 * (VELOCITY_ASSIST_PROFILE.openStraightSpeed - filteredForwardSpeed)));
  if (escaping) targetForwardVelocity = 1.08;
  const lateralLimit = avoidance.strategy === 'dynamic-dodge' ? ENCOUNTER.lateralSpeed : VELOCITY_ASSIST_PROFILE.maxLateralServoSpeed;
  let targetLeftVelocity = Math.max(-lateralLimit, Math.min(lateralLimit, speedScale * command.vy));
  if (avoidance.strategy === 'dynamic-dodge') {
    // Preview-only feedback compensates lateral gait drag; predictions use
    // measured net motion, not the faster root-position assist command.
    const desired = Math.sign(command.vy) * ENCOUNTER.lateralSpeed;
    targetLeftVelocity = Math.max(-.5, Math.min(.5, desired + .7 * (desired - filteredLeftSpeed)));
  }
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
    VELOCITY_ASSIST_PROFILE.velocityServoGain, (escaping ? 0.6 : VELOCITY_ASSIST_PROFILE.maxPlanarAcceleration) * locomotionStabilityScale,
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
    if (plan.arrived && avoidanceEnabled && obstacles.map(obstacleWorldBounds).some(o => !isTraversable(o)
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
    if (current.action !== 'none' && current.action && !current.actionDone) {
      if (controller.mode === 'walk') {
        if (!current.arrivalRewarded && progress.reward) {
          reward = { total: reward.total + progress.reward, lastValue: progress.reward, lastAt: data.time };
          current.arrivalRewarded = true;
        }
        armWork.start(current, data.time); controller.setMode('stand');
        navigation = { active: true, working: true, remaining: waypoints.length, target: current, distance: plan.distance };
        postMessage({ type: 'waypoint', event: 'work-start', waypoint: current });
      }
      return sanitizeCommand({ vx: 0, vy: 0, yawRate: 0 });
    }
    alignmentAnchor = null;
    const reached = waypoints.shift();
    if (waypoints.length) waypoints[0].segmentStart = { x: reached.x, y: reached.y };
    avoidance = { active: false, direction: 0, clearance: Infinity };
    const value = reached.arrivalRewarded ? 0 : progress.reward;
    if (value) reward = { total: reward.total + value, lastValue: value, lastAt: data.time };
    postMessage({ type: 'waypoint', event: 'reached', waypoint: reached, reward: value, totalReward: reward.total });
  }
  if (navigation.active) controller.setMode('stand');
  alignmentAnchor = null;
  navigation = { active: false, remaining: 0, distance: Infinity, headingError: 0 };
  return null;
}

function updateAvoidance(yaw, navigationCommand) {
  if (armWork.task) { avoidance = { active: false, direction: 0, clearance: Infinity }; controller.setCommand({ vx: 0, vy: 0, yawRate: 0 }); return; }
  const command = navigationCommand || requestedCommand || controller.command;
  const straightCruise = !navigationCommand && command.vx > 0.05 && Math.abs(command.vy) < 0.05 && Math.abs(command.yawRate) < 0.03;
  if (straightCruise && !Number.isFinite(cruiseHeading)) cruiseHeading = yaw;
  else if (!straightCruise) cruiseHeading = null;
  const pose = { x: data.qpos[0], y: data.qpos[1], yaw };
  const target = waypoints[0];
  if (navigation.blocked) { controller.setCommand(command); return; }
  if (controller.mode === 'walk' && rearEscapeSafe(pose, obstacles, filteredForwardSpeed)) {
    encounterPlanner.reset();
    avoidance = { active: true, strategy: 'dynamic-escape', heading: yaw, direction: 0, clearance: Infinity };
    alignmentAnchor = null; navigation.aligning = false; navigation.recovering = false;
    if (target) { target.recovering = false; target.lastProgressAt = data.time; target.bestDistance = navigation.distance; }
    controller.setCommand({ vx: 0.6, vy: 0, yawRate: 0 }); return;
  }
  const encounter = controller.mode === 'walk' && encounterPlanner.plan(pose, command, obstacles, data.time,
    { vx: filteredForwardSpeed * Math.cos(yaw) - filteredLeftSpeed * Math.sin(yaw),
      vy: filteredForwardSpeed * Math.sin(yaw) + filteredLeftSpeed * Math.cos(yaw) }, target);
  if (encounter) {
    avoidance = { ...encounter, active: true, clearance: 0 };
    alignmentAnchor = encounter.strategy === 'dynamic-wait' ? (alignmentAnchor || [pose.x, pose.y]) : null;
    navigation.aligning = false; navigation.recovering = false;
    localDetour.reset();
    if (target) { target.recovering = false; target.lastProgressAt = data.time; target.bestDistance = navigation.distance; }
    controller.setCommand(encounter.command); return;
  }
  if (avoidanceEnabled && rearBufferDeparture(pose, target, obstacles)) {
    avoidance = { active: true, strategy: 'rear-buffer-departure', heading: yaw, direction: 0, clearance: Infinity };
    localDetour.reset(); alignmentAnchor = null;
    navigation.aligning = false; navigation.recovering = false;
    controller.setCommand({ vx: .16, vy: 0, yawRate: 0 }); return;
  }
  const passage = avoidanceEnabled && planPassage(pose, target, obstacles);
  const crowded = obstacles.filter(o => !isTraversable(o) && !isDynamic(o)
    && Math.hypot(o.x - pose.x, o.y - pose.y) < 2 + Math.hypot(o.halfX, o.halfY)).length >= 2;
  const rotationTight = navigation.aligning && !waypointRouteClear(pose, pose,
    obstacles.filter(o => !isDynamic(o)));
  const detour = localDetour.plan(pose, target, obstacles, data.time,
    avoidanceEnabled && (localDetour.route || passage?.strategy === 'passage-blocked' || (!passage && (crowded || rotationTight))));
  if (detour) {
    avoidance = { active: true, strategy: detour.strategy, heading: detour.heading, direction: 0, clearance: Infinity };
    alignmentAnchor = detour.strategy === 'detour-blocked' ? (alignmentAnchor || [pose.x, pose.y]) : null;
    navigation.aligning = false; navigation.recovering = false;
    controller.setCommand(detour.command); return;
  }
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
      // The released route was checked toward the target, not toward a point
      // on the old segment which can lie across the obstacle just passed.
      if (avoidance.strategy === 'detour' || avoidance.strategy?.startsWith('dynamic-')) target.segmentStart = { x: pose.x, y: pose.y };
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
  const walking = pedestrians.get(id);
  if (walking) { obstacle.vx = walking.cruiseVx; obstacle.vy = walking.cruiseVy; pedestrians.delete(id); }
  obstacle.x = x; obstacle.y = y; applyObstaclePose(obstacle);
  mj.mj_forward(model, data);
  postPose();
}

function editObstacle(message) {
  if (!message.automatic && armWork.task) armWork.cancel();
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
      || Math.abs(o.x) > (message.automatic ? 1000 : 18) || Math.abs(o.y) > (message.automatic ? 1000 : 18)) { postMessage({ type: 'editError', message: '크기 또는 위치가 허용 범위를 벗어났습니다.' }); return; }
    if (message.action === 'create') {
      if (next.length >= 32) { postMessage({ type: 'editError', message: '장애물은 최대 32개입니다.' }); return; }
      const kind = ['step', 'person', 'car'].includes(o.kind) ? o.kind : 'box';
      next.push({ id: `custom_${++objectSerial}`, x: o.x, y: o.y, halfX: o.halfX, halfY: o.halfY, halfZ: o.halfZ, kind, movable: true,
        ...(isDynamic({ kind }) ? { vx: Number.isFinite(o.vx) ? o.vx : 0, vy: Number.isFinite(o.vy) ? o.vy : kind === 'person' ? -0.6 : -1.2,
          born: data.time, automatic: !!message.automatic } : {}) });
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
  if (!message.automatic) {
  if (manualControl.enabled) manualControl.stop();
  controller.setMode('stand'); requestedCommand = null; servoWorldVelocity = null;
  servoForwardVelocity = 0; servoLeftVelocity = 0; alignmentAnchor = null;
  avoidance = { active: false, direction: 0, clearance: Infinity };
  for (const waypoint of waypoints) { waypoint.tracking = false; waypoint.recovering = false; }
  }
  mj.mj_forward(model, data);
  postMessage({ type: 'worldChanged', bodyNames, automatic: !!message.automatic, selectedId: message.action === 'create' ? next.at(-1).id : message.action === 'delete' ? null : message.id });
  postPose();
}

function applyObstaclePose(obstacle) {
  if (!data || obstacleMocap[obstacle.id] == null) return;
  const address = obstacleMocap[obstacle.id] * 3;
  data.mocap_pos[address] = obstacle.x; data.mocap_pos[address + 1] = obstacle.y; data.mocap_pos[address + 2] = obstacle.halfZ;
  const q = obstacleMocap[obstacle.id] * 4, yaw = actorHeading(obstacle);
  data.mocap_quat[q] = Math.cos(yaw / 2); data.mocap_quat[q + 1] = 0; data.mocap_quat[q + 2] = 0; data.mocap_quat[q + 3] = Math.sin(yaw / 2);
}

export function step() {
  updateDynamicObstacles();
  if (data.time - perceptionAt >= 1 / PERCEPTION_PROFILE.hz) {
    const yaw = quatToEuler(data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]).yaw;
    perception = riskTracker.update(simulatedSensorFrame({ x: data.qpos[0], y: data.qpos[1], yaw },
      obstacles.filter(o => !isTraversable(o)), data.time));
    perceptionAt = data.time;
  }
  if (recovery) { assistedRecoveryStep(); return; }
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
  const measuredLeftSpeed = (-(data.qpos[0] - beforeX) * Math.sin(yaw) + (data.qpos[1] - beforeY) * Math.cos(yaw)) / model.opt.timestep;
  filteredLeftSpeed = lowPass(filteredLeftSpeed, measuredLeftSpeed, VELOCITY_ASSIST_PROFILE.velocityFilterTimeConstant, model.opt.timestep);
}

function assistedRecoveryStep() {
  const dt = model.opt.timestep;
  const pose = { x: data.qpos[0], y: data.qpos[1] };
  const spaceClear = [0, 0.5, 1].every(t => waypointRouteClear(pose, pose,
    obstacles.map(o => ({ ...o, x: o.x + (o.vx || 0) * t, y: o.y + (o.vy || 0) * t }))));
  data.ctrl.fill(0); data.qfrc_applied.fill(0); servoWorldVelocity = null;
  if (!spaceClear) { recovery.stage = 'waiting'; data.time += dt; return; }
  if (recovery.stage === 'waiting') {
    recovery.stage = 'assisted-righting'; recovery.started = data.time;
    recovery.q = Array.from(data.qpos); recovery.height = data.qpos[2];
  }
  const t = Math.min(1, (data.time - recovery.started) / 3), blend = t * t * (3 - 2 * t);
  const targetQ = [Math.cos(recovery.yaw / 2), 0, 0, Math.sin(recovery.yaw / 2)];
  const sign = targetQ.reduce((sum, v, i) => sum + v * recovery.q[i + 3], 0) < 0 ? -1 : 1;
  const q = targetQ.map((v, i) => recovery.q[i + 3] * (1 - blend) + sign * v * blend);
  const norm = Math.hypot(...q);
  q.forEach((v, i) => { data.qpos[i + 3] = v / norm; });
  data.qpos[2] = recovery.height * (1 - blend) + 0.40 * blend;
  for (const name of JOINT_ORDER) {
    const nominal = name.endsWith('_hip') ? (name[1] === 'R' ? 0.1 : -0.1) : name.endsWith('_thigh') ? 0.9 : -1.8;
    data.qpos[jointQ[name]] = recovery.q[jointQ[name]] * (1 - blend) + nominal * blend;
  }
  data.qvel.fill(0); data.time += dt; mj.mj_forward(model, data);
  if (t === 1) {
    recovery = null; controller = new A2WalkingController(); controller.setMode(waypoints.length ? 'walk' : 'stand');
    servoForwardVelocity = 0; servoLeftVelocity = 0; locomotionStabilityScale = 0.5; alignmentAnchor = null;
    avoidance = { active: false, direction: 0, clearance: Infinity };
    for (const target of waypoints) { target.tracking = false; target.recovering = false; target.bestDistance = Infinity; target.lastProgressAt = data.time; }
  }
}

// Deterministic Node regression fixture; deliberately not exposed as a worker
// message or browser control. Exercises the same safety/recovery step as falls.
export function simulateSideFallForTest() {
  data.qpos[2] = 0.2; data.qpos[3] = Math.SQRT1_2; data.qpos[4] = Math.SQRT1_2;
  data.qpos[5] = 0; data.qpos[6] = 0; data.qvel.fill(0); controller.setMode('walk');
  mj.mj_forward(model, data);
}

function updateDynamicObstacles() {
  for (const id of pedestrians.keys()) if (!obstacles.some(o => o.id === id)) pedestrians.delete(id);
  const expired = obstacles.find(o => o.automatic && data.time - o.born > 12);
  if (expired) editObstacle({ action: 'delete', id: expired.id, automatic: true });
  if (randomActors && !manualControl.enabled && controller.mode === 'walk' && data.time >= nextActorAt) {
    nextActorAt = data.time + 8 + randomActorValue() * 6;
    const actor = crossingActor({ x: data.qpos[0], y: data.qpos[1] }, waypoints[0], randomActorValue);
    if (actor && obstacles.length < 32 && !obstacles.some(o => Math.hypot(o.x - actor.x, o.y - actor.y) < Math.hypot(o.halfX, o.halfY) + 1.2)) {
      editObstacle({ action: 'create', obstacle: actor, automatic: true });
    }
  }
  for (const o of obstacles.filter(isDynamic)) {
    const reversing = !o.automatic && Math.floor((data.time - o.born) / 6) !== Math.floor((data.time + model.opt.timestep - o.born) / 6);
    if (personYield && o.kind === 'person') {
      if (!pedestrians.has(o.id)) pedestrians.set(o.id, pedestrianState(o));
      const state = pedestrians.get(o.id);
      if (reversing) { state.cruiseVx *= -1; state.cruiseVy *= -1; state.active = false; state.side = 0; }
      const yaw = quatToEuler(data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]).yaw;
      Object.assign(o, stepPedestrian(o, state, { x: data.qpos[0], y: data.qpos[1], yaw,
        vx: servoWorldVelocity?.[0] ?? data.qvel[0], vy: servoWorldVelocity?.[1] ?? data.qvel[1] }, obstacles, model.opt.timestep));
    } else {
      if (reversing) { o.vx *= -1; o.vy *= -1; }
      o.x += o.vx * model.opt.timestep; o.y += o.vy * model.opt.timestep;
    }
    applyObstaclePose(o);
  }
}

export function postPose() {
  if (!data) return;
  lastPose = performance.now();
  const xpos = Float32Array.from(data.xpos);
  const xquat = Float32Array.from(data.xquat);
  const orientation = quatToEuler(data.qpos[3], data.qpos[4], data.qpos[5], data.qpos[6]);
  const forwardSpeedInstantaneous = data.qvel[0] * Math.cos(orientation.yaw) + data.qvel[1] * Math.sin(orientation.yaw);
  postMessage({ type: 'pose', time: data.time, xpos, xquat, base: [data.qpos[0], data.qpos[1], data.qpos[2]], orientation,
    velocity: [data.qvel[0], data.qvel[1], data.qvel[2]], forwardSpeed: filteredForwardSpeed, forwardSpeedInstantaneous, perception,
    forwardServoAcceleration, assistSpeedScale, locomotionStabilityScale,
    armWork: armWork.sample(data.time), recovery: recovery?.stage ?? null, mode: controller.mode, command: controller.command, requestedCommand, avoidance, navigation, reward, obstacles, fault: controller.fault }, [xpos.buffer, xquat.buffer]);
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
