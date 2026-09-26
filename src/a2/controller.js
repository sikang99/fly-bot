import { A2_GAIT_PROFILE, A2_LIMITS, DEFAULT_COMMAND, JOINT_ORDER, JOINTS, clamp, sanitizeCommand } from './config.js';

const PHASE_OFFSET = { FR: 0, FL: Math.PI, RR: Math.PI, RL: 0 };

export function footCycle(phase, stanceDutyFactor = A2_GAIT_PROFILE.stanceDutyFactor) {
  const cycle = ((phase / (Math.PI * 2)) % 1 + 1) % 1;
  const swingFraction = 1 - stanceDutyFactor;
  if (cycle < swingFraction) {
    const progress = cycle / swingFraction;
    return { swing: true, foreAft: -0.5 * Math.cos(Math.PI * progress), lift: Math.sin(Math.PI * progress) ** 2 };
  }
  const progress = (cycle - swingFraction) / stanceDutyFactor;
  return { swing: false, foreAft: 0.5 * Math.cos(Math.PI * progress), lift: 0 };
}

export class A2WalkingController {
  constructor() {
    this.mode = 'passive';
    this.command = { ...DEFAULT_COMMAND };
    this.gaitCommand = { ...DEFAULT_COMMAND };
    this.phase = 0;
    this.fault = null;
    this.motionScale = 1;
  }

  setMode(mode) {
    if (!['passive', 'stand', 'walk'].includes(mode)) throw new Error(`unknown A2 mode: ${mode}`);
    this.mode = mode;
    if (mode !== 'walk') this.command = { ...this.command, vx: 0, vy: 0, yawRate: 0 };
    if (mode === 'passive') this.phase = 0;
  }

  setCommand(command) {
    this.command = sanitizeCommand({ ...this.command, ...command });
  }

  setMotionScale(scale) {
    this.motionScale = clamp(scale, 0, 1);
  }

  checkSafety(state) {
    const tilt = Math.max(Math.abs(state.roll || 0), Math.abs(state.pitch || 0));
    if ((state.height ?? 1) < A2_LIMITS.minBaseHeight) this.fault = 'base height below safety limit';
    else if (tilt > A2_LIMITS.maxTilt) this.fault = 'body tilt above safety limit';
    else this.fault = null;
    if (this.fault) this.mode = 'passive';
    return !this.fault;
  }

  targets(dt) {
    const blend = 1 - Math.exp(-dt / A2_GAIT_PROFILE.commandTimeConstant);
    for (const key of ['vx', 'vy', 'yawRate', 'frequency']) this.gaitCommand[key] += (this.command[key] - this.gaitCommand[key]) * blend;
    const gait = {
      ...this.gaitCommand,
      vx: this.gaitCommand.vx * this.motionScale,
      vy: this.gaitCommand.vy * this.motionScale,
      yawRate: this.gaitCommand.yawRate * this.motionScale,
    };
    if (this.mode === 'walk') this.phase = (this.phase + Math.PI * 2 * gait.frequency * dt) % (Math.PI * 2);
    const motion = Math.max(Math.abs(gait.vx), Math.abs(gait.vy), Math.abs(gait.yawRate));
    const legTargets = {};
    if (this.mode === 'walk') for (const leg of Object.keys(PHASE_OFFSET)) {
      const p = (this.phase + PHASE_OFFSET[leg]) % (Math.PI * 2);
      const cycle = footCycle(p);
      const sideSign = leg[1] === 'L' ? 1 : -1;
      const turnScale = clamp(1 - sideSign * gait.yawRate * 1.6, 0.55, 1.45);
      const strideDemand = Math.abs(gait.vx) + 0.2 * Math.abs(gait.yawRate);
      const stepLength = clamp(0.34 * strideDemand * turnScale, 0, A2_GAIT_PROFILE.maxStepLength);
      const direction = Math.sign(gait.vx || 1);
      const footX = direction * cycle.foreAft * stepLength;
      const motionScale = clamp(motion / A2_GAIT_PROFILE.cruiseSpeed, 0, 1);
      const stanceZ = -0.34 + A2_GAIT_PROFILE.speedCrouch * motionScale;
      const footZ = stanceZ + A2_GAIT_PROFILE.swingHeight * cycle.lift * motionScale;
      legTargets[leg] = legIk(footX, footZ);
    }
    const targets = {};

    for (const name of JOINT_ORDER) {
      const joint = JOINTS[name];
      const sideSign = joint.leg[1] === 'L' ? 1 : -1;
      let value = joint.nominal;

      if (joint.type === 'hip') value += 0.08 * gait.vy + 0.20 * sideSign * gait.yawRate;
      if (joint.type === 'thigh') value = legTargets[joint.leg]?.thigh ?? value;
      if (joint.type === 'calf') value = legTargets[joint.leg]?.calf ?? value;
      targets[name] = clamp(value, joint.min + 0.04, joint.max - 0.04);
    }
    return targets;
  }

  torques(position, velocity, targets) {
    if (this.mode === 'passive') return Object.fromEntries(JOINT_ORDER.map(name => [name, 0]));
    return Object.fromEntries(JOINT_ORDER.map(name => {
      const joint = JOINTS[name];
      const torque = joint.kp * (targets[name] - position[name]) - joint.kd * velocity[name];
      return [name, clamp(torque, -joint.effort, joint.effort)];
    }));
  }
}

function legIk(x, z) {
  const upper = 0.275, lower = 0.275;
  const radius2 = x * x + z * z;
  const cosine = clamp((radius2 - upper * upper - lower * lower) / (2 * upper * lower), -1, 1);
  const calf = -Math.acos(cosine);
  const thigh = Math.atan2(-x, -z) - Math.atan2(lower * Math.sin(calf), upper + lower * Math.cos(calf));
  return { thigh, calf };
}
