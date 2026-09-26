import test from 'node:test';
import assert from 'node:assert/strict';
import { A2WalkingController, footCycle } from '../src/a2/controller.js';
import { A2_GAIT_PROFILE, A2_LIMITS, A2_OBSTACLES, ARM_STOW_POSE, JOINT_ORDER, JOINTS, VELOCITY_ASSIST_PROFILE, accelerationLimitedVelocity, lowPass, sanitizeCommand, tractionStabilityScale, updateStabilityEnvelope } from '../src/a2/config.js';
import { buildA2WorldXml } from '../src/a2/model.js';
import { A2_FOOTPRINT, obstacleRelativeGeometry, planObstacleAvoidance, waypointRouteClear } from '../src/a2/avoidance.js';
import { WAYPOINT_DIVERGENCE_LIMIT, WAYPOINT_RADIUS, WAYPOINT_STALL_TIMEOUT, assessWaypointProgress, planWaypointCommand, planWaypointRecovery, waypointDistanceFromBodyCenter, waypointNeedsRecovery } from '../src/a2/navigation.js';
import fs from 'node:fs';
import { planNearbyDocking } from '../src/a2/avoidance.js';
test('near-obstacle docking keeps heading and prioritizes safe lateral approach', () => {
  const pose = { x: 1.5, y: -0.3, yaw: 0 }, target = { x: 2, y: 0 };
  const obstacles = [{ id: 'box', x: 2, y: 0.75, halfX: 0.2, halfY: 0.2 }];
  const command = planNearbyDocking(pose, target, obstacles);
  assert.equal(command.vx, 0); assert.equal(command.yawRate, 0); assert.ok(command.vy > 0);
  assert.equal(planNearbyDocking(pose, { x: 2, y: 0.7 }, obstacles), null);
  assert.equal(planNearbyDocking(pose, { x: 5, y: 0 }, obstacles), null);
});
import { segmentAim, planSegmentCommand } from '../src/a2/navigation.js';
test('segment tracking brings the body centre back to the line without rewarding the lookahead', () => {
  const start = { x: 0, y: 0 }, end = { x: 4, y: 0 };
  for (const y of [-0.3, 0.3]) {
    const pose = { x: 1, y, heading: 0 };
    const aim = segmentAim(pose, start, end);
    assert.equal(aim.y, 0);
    assert.ok(Math.abs(aim.x - 1.55) < 1e-9);
    const plan = planSegmentCommand(pose, start, end, { tracking: true });
    assert.ok(plan.command.vy * y < 0);
    assert.ok(plan.command.yawRate * y < 0);
    assert.equal(plan.arrived, false);
    assert.equal(plan.distance, Math.hypot(3, y));
  }
  assert.equal(planSegmentCommand({ x: 4, y: 0.03, heading: 0 }, start, end, { tracking: true }).arrived, true);
});
import { alignmentYawRate } from '../src/a2/navigation.js';
test('in-place yaw assist is faster but slows near alignment and under instability', () => {
  assert.equal(alignmentYawRate(Math.PI / 2), 0.7);
  assert.equal(alignmentYawRate(-Math.PI / 2), -0.7);
  assert.ok(alignmentYawRate(0.15) < 0.3);
  assert.equal(alignmentYawRate(2, 0.5), 0.35);
  assert.equal(alignmentYawRate(2, 0), 0);
});
test('avoidance respects target distance and picks the shorter target-facing side', () => {
  const pose = { x: 0, y: 0, yaw: 0 };
  const obstacles = [{ id: 'box', x: 1.8, y: 0, halfX: 0.2, halfY: 0.2 }];
  assert.equal(waypointRouteClear(pose, { x: 0.6, y: 0 }, obstacles), true);
  assert.equal(waypointRouteClear(pose, { x: 4, y: 0 }, obstacles), false);
  for (const sign of [-1, 1]) {
    const plan = planObstacleAvoidance(pose, { vx: 0.6 }, obstacles, { x: 4, y: sign * 2 });
    assert.equal(plan.direction, sign);
  }
  assert.equal(waypointRouteClear({ x: 0, y: 2 }, { x: 4, y: 2 }, obstacles), true);
});
import { worldObstacles, isTraversable, terrainColor } from '../src/a2/terrain.js';

test('rough terrain shares all collision obstacles with rendering and navigation', () => {
  const obstacles = worldObstacles('rough');
  const xml = buildA2WorldXml(fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8'), 'rough');
  for (const obstacle of obstacles) assert.ok(xml.includes(`name="${obstacle.id}"`));
  assert.ok(obstacles.every(o => o.movable));
  const low = obstacles.find(o => o.id === 'step_left');
  const high = obstacles.find(o => o.id === 'step_right');
  assert.equal(isTraversable(low), true);
  assert.equal(isTraversable(high), false);
  assert.notEqual(terrainColor(low), terrainColor(high));
  const pose = { x: 0, y: 0, yaw: 0 }, command = { vx: 0.6 };
  assert.equal(planObstacleAvoidance(pose, command, [{ ...low, x: 1, y: 0 }]).active, false);
  assert.equal(planObstacleAvoidance(pose, command, [{ ...high, x: 1, y: 0 }]).active, true);
  assert.equal(worldObstacles('flat').length, A2_OBSTACLES.length);
});

test('A2 uses the SDK-compatible 12 joint order', () => {
  assert.equal(JOINT_ORDER.length, 12);
  assert.deepEqual(JOINT_ORDER.slice(0, 3), ['FR_hip', 'FR_thigh', 'FR_calf']);
  assert.deepEqual(JOINT_ORDER.slice(-3), ['RL_hip', 'RL_thigh', 'RL_calf']);
});

test('single arm starts in a compact fixed storage pose', () => {
  assert.ok(ARM_STOW_POSE.mountX > 0, 'arm must be mounted above the +X front legs');
  assert.equal(ARM_STOW_POSE.scale, 0.75);
  assert.ok(ARM_STOW_POSE.shoulderPitch < -1.2);
  assert.ok(ARM_STOW_POSE.elbowPitch > 2.5);
  assert.ok(ARM_STOW_POSE.gripperGap <= 0.05);
});

test('A2 navigation uses the official +X front convention', () => {
  assert.ok(A2_OBSTACLES[0].x > 0);
  const ahead = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: 1, y: 0 });
  assert.equal(ahead.arrived, false); assert.ok(ahead.command.vx > 0); assert.ok(Math.abs(ahead.command.yawRate) < 1e-9);
});

test('commands and joint targets remain inside safety limits', () => {
  const command = sanitizeCommand({ vx: 99, vy: -99, yawRate: 4, frequency: 50 });
  assert.deepEqual(command, { vx: 0.6, vy: -0.5, yawRate: 0.35, frequency: 1.4 });
  const controller = new A2WalkingController(); controller.setMode('walk'); controller.setCommand(command);
  for (let i = 0; i < 500; i++) {
    const targets = controller.targets(0.002);
    for (const name of JOINT_ORDER) assert.ok(targets[name] >= JOINTS[name].min && targets[name] <= JOINTS[name].max, name);
  }
});

test('traction is derated before the fall safety boundary', () => {
  assert.equal(tractionStabilityScale({ roll: 0, pitch: 0, height: 0.32 }), 1);
  assert.equal(tractionStabilityScale({ roll: 0.28, pitch: 0, height: 0.32 }), 0);
  assert.equal(tractionStabilityScale({ roll: 0, pitch: 0, height: 0.25 }), 0);
  assert.equal(tractionStabilityScale({ roll: 0, pitch: 0, height: 0.32, verticalSpeed: -0.45 }), 0);
  assert.ok(tractionStabilityScale({ roll: 0.2, pitch: 0, height: 0.32 }) > 0);
});

test('stability envelope brakes immediately and recovers gradually', () => {
  assert.equal(updateStabilityEnvelope(1, 0.2, 0.5, 0.1), 0.2);
  assert.equal(updateStabilityEnvelope(0.2, 1, 0.5, 0.1), 0.25);
});

test('forward speed filtering rejects gait-cycle spikes', () => {
  let speed = 0;
  for (let i = 0; i < 175; i++) speed = lowPass(speed, 0.4, 0.35, 0.002);
  assert.ok(speed > 0.24 && speed < 0.27);
  const afterSpike = lowPass(speed, -0.8, 0.35, 0.002);
  assert.ok(afterSpike > 0.23);
});

test('simulation velocity servo respects acceleration and braking limits', () => {
  assert.ok(Math.abs(accelerationLimitedVelocity(0, 0.5, 2, 0.35, 0.8, 0.1) - 0.035) < 1e-9);
  assert.ok(Math.abs(accelerationLimitedVelocity(0.5, 0, 2, 0.35, 0.8, 0.1) - 0.42) < 1e-9);
});

test('diagonal legs share a trot phase', () => {
  const controller = new A2WalkingController(); controller.setMode('walk'); controller.setCommand({ vx: 0.7 });
  const targets = controller.targets(0.05);
  assert.equal(targets.FR_thigh, targets.RL_thigh);
  assert.equal(targets.FL_thigh, targets.RR_thigh);
  assert.notEqual(targets.FR_thigh, targets.FL_thigh);
});

test('speed gait keeps four-foot support overlap between diagonal swings', () => {
  const duty = 0.58;
  assert.equal(footCycle(Math.PI * 2 * 0.45, duty).swing, false);
  assert.equal(footCycle(Math.PI * 2 * 0.95, duty).swing, false);
  assert.equal(footCycle(Math.PI * 2 * 0.2, duty).swing, true);
  assert.equal(footCycle(Math.PI * 2 * 0.7, duty).swing, false);
});

test('positive walking command has a forward swing and backward stance sweep', () => {
  const swingStart = footCycle(0);
  const touchdown = footCycle(Math.PI * 2 * (1 - A2_GAIT_PROFILE.stanceDutyFactor));
  const stanceEnd = footCycle(Math.PI * 2 - 1e-5);
  assert.ok(touchdown.foreAft > swingStart.foreAft);
  assert.ok(stanceEnd.foreAft < touchdown.foreAft);
});

test('stability supervisor shortens gait motion before a fall', () => {
  const full = new A2WalkingController(); full.setMode('walk'); full.gaitCommand = { vx: 0.6, vy: 0.3, yawRate: 0, frequency: 1.4 };
  const guarded = new A2WalkingController(); guarded.setMode('walk'); guarded.gaitCommand = { ...full.gaitCommand }; guarded.setMotionScale(0.25);
  full.phase = guarded.phase = Math.PI / 2;
  const fullTargets = full.targets(0), guardedTargets = guarded.targets(0);
  assert.ok(Math.abs(guardedTargets.FR_thigh - JOINTS.FR_thigh.nominal) < Math.abs(fullTargets.FR_thigh - JOINTS.FR_thigh.nominal));
  assert.ok(Math.abs(guardedTargets.FR_hip - JOINTS.FR_hip.nominal) < Math.abs(fullTargets.FR_hip - JOINTS.FR_hip.nominal));
});

test('foot targets keep a continuous velocity through touchdown', () => {
  const controller = new A2WalkingController(); controller.setMode('walk');
  controller.gaitCommand = { vx: 0.6, vy: 0, yawRate: 0, frequency: 1.4 };
  const sample = phase => { controller.phase = phase; return controller.targets(0); };
  const touchdown = Math.PI * 2 * (1 - A2_GAIT_PROFILE.stanceDutyFactor);
  const epsilon = 1e-4, before = sample(touchdown - epsilon), at = sample(touchdown), after = sample(touchdown + epsilon);
  for (const joint of ['FR_thigh', 'FR_calf']) {
    const leftVelocity = (at[joint] - before[joint]) / epsilon;
    const rightVelocity = (after[joint] - at[joint]) / epsilon;
    assert.ok(Math.abs(leftVelocity - rightVelocity) < 0.01, `${joint} touchdown velocity jump`);
  }
});

test('fall detection forces passive mode and zero torque', () => {
  const controller = new A2WalkingController(); controller.setMode('walk');
  assert.equal(controller.checkSafety({ height: 0.1, roll: 0, pitch: 0 }), false);
  const zeros = controller.torques(Object.fromEntries(JOINT_ORDER.map(n => [n, 0])), Object.fromEntries(JOINT_ORDER.map(n => [n, 0])), controller.targets(0.002));
  assert.ok(Object.values(zeros).every(value => value === 0));
});

test('browser MJCF keeps A2 physics while removing unavailable visual meshes', () => {
  const source = fs.readFileSync(new URL('../public/a2/a2.xml', import.meta.url), 'utf8');
  const xml = buildA2WorldXml(source, 'rough');
  assert.match(xml, /model="a2"/); assert.match(xml, /mass="19\.651"/); assert.match(xml, /name="FR_hip_joint"/);
  assert.match(xml, /name="floor"/); assert.match(xml, /name="obstacle_demo_box" mocap="true"/); assert.match(xml, /name="demo_box"/); assert.match(xml, /name="step_left"/); assert.doesNotMatch(xml, /type="mesh"/);
});

test('obstacle avoidance sidesteps a box without a sharp turn', () => {
  assert.ok(Math.abs((A2_FOOTPRINT.halfLength + A2_FOOTPRINT.margin) * 2 - 1.12) < 1e-9);
  assert.ok(Math.abs((A2_FOOTPRINT.halfWidth + A2_FOOTPRINT.margin) * 2 - 0.84) < 1e-9);
  const plan = planObstacleAvoidance({ x: 0, y: 0, yaw: 0 }, { vx: 0.7, vy: 0, yawRate: 0, frequency: 1.7 });
  assert.equal(plan.active, true); assert.equal(plan.obstacle, 'demo_box'); assert.ok(Math.abs(plan.direction) === 1);
  assert.equal(plan.strategy, 'sidestep'); assert.equal(Math.sign(plan.command.vy), plan.direction);
  assert.equal(plan.command.yawRate, 0); assert.equal(plan.collisionRisk, true); assert.equal(plan.command.vx, 0);
  const clear = planObstacleAvoidance({ x: 0, y: 2, yaw: 0 }, { vx: 0.7 });
  assert.equal(clear.active, false); assert.equal(clear.command.vx, 0.6);
  const bodyWouldClip = planObstacleAvoidance({ x: 0, y: 0.6, yaw: 0 }, { vx: 0.7 });
  assert.equal(bodyWouldClip.active, true); assert.ok(bodyWouldClip.lateralClearance < 0);
  const bodyClears = planObstacleAvoidance({ x: 0, y: 0.9, yaw: 0 }, { vx: 0.7 });
  assert.equal(bodyClears.active, false);
  const behindButClose = planObstacleAvoidance({ x: 1.1, y: 0, yaw: 0 }, { vx: 0.6 });
  assert.equal(behindButClose.active, false); assert.ok(behindButClose.command.vx > 0);
  const turnAround = planObstacleAvoidance({ x: 0, y: 0, yaw: 0 }, { vx: 0.6 }, [{ ...A2_OBSTACLES[0], x: 1.6 }]);
  assert.equal(turnAround.strategy, 'arc'); assert.ok(turnAround.command.vx > 0.25); assert.ok(Math.abs(turnAround.command.yawRate) > 0.15);
});

test('obstacle is not considered passed until it clears the full rear footprint', () => {
  const obstacle = A2_OBSTACLES[0];
  assert.equal(obstacleRelativeGeometry({ x: 1.2, y: 0.9, yaw: 0 }, obstacle).behind, false);
  assert.equal(obstacleRelativeGeometry({ x: 1.58, y: 0.9, yaw: 0 }, obstacle).behind, true);
});

test('waypoint navigation generates bounded steering and arrival', () => {
  const ahead = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: 2, y: 0 });
  assert.equal(ahead.arrived, false); assert.equal(ahead.command.vx, A2_GAIT_PROFILE.cruiseSpeed); assert.ok(Math.abs(ahead.command.yawRate) < 1e-9);
  const side = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: 0, y: 1 });
  assert.equal(side.arrived, false); assert.ok(Math.abs(side.command.yawRate) <= A2_LIMITS.maxYawRate);
  assert.equal(side.aligning, true);
  assert.equal(side.command.vx, 0); assert.equal(side.command.vy, 0);
  const diagonal = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: 1, y: 0.5 });
  assert.equal(diagonal.aligning, true); assert.equal(diagonal.command.vx, 0);
  const trackedDiagonal = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: 1, y: 0.5 }, { tracking: true });
  assert.equal(trackedDiagonal.aligning, false); assert.ok(trackedDiagonal.command.vx > 0);
  const badlyTurned = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: 0, y: 1 }, { tracking: true });
  assert.equal(badlyTurned.aligning, false); assert.ok(badlyTurned.command.vx > 0);
  const shallow = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: 1, y: 0.05 });
  assert.equal(shallow.aligning, false); assert.ok(shallow.command.vx > 0); assert.ok(shallow.command.vy > 0);
  const reached = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: WAYPOINT_RADIUS, y: 0 });
  assert.equal(reached.arrived, true); assert.equal(reached.command.vx, 0);
  const bodyOverlapsButCentreMisses = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: WAYPOINT_RADIUS + 0.01, y: 0 });
  assert.equal(bodyOverlapsButCentreMisses.arrived, false);
  const approaching = planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x: 0.16, y: 0 });
  assert.ok(approaching.command.vx > 0); assert.ok(approaching.command.vx < A2_GAIT_PROFILE.cruiseSpeed);
  assert.ok(Math.abs(waypointDistanceFromBodyCenter({ x: 1, y: 2 }, { x: 1.06, y: 2.08 }) - 0.1) < 1e-9);
});

test('waypoint speed profile is monotonic from precision approach to stable cruise', () => {
  const distances = [0.05, 0.07, 0.10, 0.15, 0.25, 0.40, 0.80, 1.50, 3.00];
  const plans = distances.map(x => planWaypointCommand({ x: 0, y: 0, heading: 0 }, { x, y: 0 }, { tracking: true }));
  assert.equal(plans[0].arrived, true); assert.equal(plans[0].command.vx, 0);
  for (let index = 2; index < plans.length; index++) {
    assert.ok(plans[index].command.vx >= plans[index - 1].command.vx, `${distances[index]}m speed regressed`);
  }
  assert.ok(plans[1].command.vx > 0 && plans[1].command.vx < 0.2);
  assert.equal(plans[3].command.vx, 0.15 * 1.2);
  assert.ok(plans[5].command.vx <= 0.22, 'terminal centering must remain slow');
  for (const index of [6, 7, 8]) assert.equal(plans[index].command.vx, A2_GAIT_PROFILE.cruiseSpeed);
});

test('terminal centering corrects overshoot without turning back into terrain', () => {
  const plan = planWaypointCommand({ x: 0.1, y: -0.1, heading: 0 }, { x: 0, y: 0 }, { tracking: true });
  assert.equal(plan.centering, true);
  assert.equal(plan.command.yawRate, 0);
  assert.ok(plan.command.vx < 0 && plan.command.vy > 0);
  assert.ok(Math.abs(plan.command.vx) <= 0.22 && Math.abs(plan.command.vy) <= 0.20);
});

test('waypoint pursuit converges across distances and every principal direction without a mid-route stop', () => {
  const dt = 0.02;
  for (const distance of [0.25, 0.80, 1.50, 3.00, 5.00]) {
    for (const degrees of [-180, -135, -90, -45, 0, 45, 90, 135, 180]) {
      const angle = degrees * Math.PI / 180;
      const waypoint = { x: distance * Math.cos(angle), y: distance * Math.sin(angle) };
      const pose = { x: 0, y: 0, heading: 0 };
      let tracking = false, arrived = false;
      for (let step = 0; step < 1000; step++) {
        const plan = planWaypointCommand(pose, waypoint, { tracking });
        if (plan.arrived) { arrived = true; break; }
        if (tracking) {
          assert.equal(plan.aligning, false, `${distance}m/${degrees}deg stopped after route acquisition`);
          assert.ok(plan.command.vx > 0, `${distance}m/${degrees}deg lost forward motion`);
        } else if (!plan.aligning) tracking = true;
        const command = plan.command;
        pose.heading = Math.atan2(Math.sin(pose.heading + command.yawRate * dt), Math.cos(pose.heading + command.yawRate * dt));
        pose.x += (command.vx * Math.cos(pose.heading) - command.vy * Math.sin(pose.heading)) * dt;
        pose.y += (command.vx * Math.sin(pose.heading) + command.vy * Math.cos(pose.heading)) * dt;
      }
      assert.equal(arrived, true, `${distance}m/${degrees}deg failed to arrive`);
    }
  }
});

test('waypoint progress watchdog realigns a diverging or stalled route and then releases it', () => {
  assert.equal(waypointNeedsRecovery(1.0 + WAYPOINT_DIVERGENCE_LIMIT + 0.01, 1.0, 0.2), true);
  assert.equal(waypointNeedsRecovery(1.0, 1.0, WAYPOINT_STALL_TIMEOUT + 0.01), true);
  assert.equal(waypointNeedsRecovery(1.05, 1.0, 0.5), false);
  const left = planWaypointRecovery(Math.PI / 2);
  assert.equal(left.aligned, false); assert.equal(left.command.vx, 0); assert.ok(left.command.yawRate > 0);
  const ready = planWaypointRecovery(0.1);
  assert.equal(ready.aligned, true); assert.equal(ready.command.yawRate, 0);
});

test('waypoint rewards exactly once when the body centre reaches it', () => {
  const outside = assessWaypointProgress(WAYPOINT_RADIUS + 0.001);
  assert.equal(outside.arrived, false); assert.equal(outside.reward, 0);
  const arrival = assessWaypointProgress(WAYPOINT_RADIUS);
  assert.equal(arrival.arrived, true); assert.equal(arrival.reward, 1);
});
