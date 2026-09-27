export const LEG_ORDER = ['FR', 'FL', 'RR', 'RL'];
export const JOINT_TYPES = ['hip', 'thigh', 'calf'];
export const JOINT_ORDER = LEG_ORDER.flatMap(leg => JOINT_TYPES.map(type => `${leg}_${type}`));

const HIP = { min: -1.01, max: 1.01, effort: 120, kp: 100, kd: 6 };
const FRONT_THIGH = { min: -2.34, max: 3.15, effort: 120, kp: 100, kd: 6 };
const REAR_THIGH = { min: -1.56, max: 3.94, effort: 120, kp: 100, kd: 6 };
const CALF = { min: -2.77, max: -0.54, effort: 180, kp: 150, kd: 9 };

export const JOINTS = Object.fromEntries(JOINT_ORDER.map(name => {
  const [leg, type] = name.split('_');
  const limits = type === 'hip' ? HIP : type === 'calf' ? CALF : leg[0] === 'F' ? FRONT_THIGH : REAR_THIGH;
  const nominal = type === 'hip' ? (leg[1] === 'R' ? 0.1 : -0.1) : type === 'thigh' ? 0.9 : -1.8;
  return [name, { name, leg, type, nominal, ...limits }];
}));

export const A2_LIMITS = Object.freeze({
  maxLinearX: 0.6,
  maxLinearY: 0.5,
  maxYawRate: 0.35,
  maxTilt: 0.9,
  minBaseHeight: 0.24,
  commandTimeoutMs: 500,
});

export const DEFAULT_COMMAND = Object.freeze({ vx: 0, vy: 0, yawRate: 0, frequency: 1.4 });
export const A2_GAIT_PROFILE = Object.freeze({
  cruiseSpeed: 0.6,
  commandTimeConstant: 0.4,
  minCadence: 1.4,
  maxCadence: 1.4,
  maxStepLength: 0.28,
  swingHeight: 0.065,
  stanceDutyFactor: 0.58,
  speedCrouch: 0.015,
});

// Browser-preview locomotion assistance. It is deliberately separate from the
// joint controller and must never be translated into a hardware command.
export const VELOCITY_ASSIST_PROFILE = Object.freeze({
  openStraightSpeed: 1.0,
  openStraightBrakeDistance: 2.5,
  cruiseScale: 0.80,
  avoidanceScale: 0.65,
  velocityFilterTimeConstant: 0.35,
  tiltDerateStart: 0.12,
  tiltDerateStop: 0.28,
  heightDerateStart: 0.285,
  heightDerateStop: 0.25,
  descentDerateStart: -0.18,
  descentDerateStop: -0.45,
  maxPlanarAcceleration: 0.25,
  maxPlanarDeceleration: 0.60,
  velocityServoGain: 2.0,
  maxLateralServoSpeed: 0.16,
  stabilityRecoveryRate: 0.50,
});

// Fixed visual storage pose. The arm stays folded until a later manipulation
// controller explicitly owns these joints.
export const ARM_STOW_POSE = Object.freeze({
  mountX: 0.20,
  scale: 0.75,
  shoulderPitch: -1.38,
  elbowPitch: 2.72,
  wristPitch: 0.08,
  gripperGap: 0.045,
});

export const A2_OBSTACLES = Object.freeze([
  Object.freeze({ id: 'demo_box', x: 0.85, y: 0, halfX: 0.16, halfY: 0.28, halfZ: 0.16 }),
]);

export function clamp(value, min, max) {
  return Math.max(min, Math.min(max, Number.isFinite(value) ? value : 0));
}

export function tractionStabilityScale({ roll = 0, pitch = 0, height = 1, verticalSpeed = 0 }) {
  const tilt = Math.max(Math.abs(roll), Math.abs(pitch));
  const tiltScale = 1 - clamp((tilt - VELOCITY_ASSIST_PROFILE.tiltDerateStart)
    / (VELOCITY_ASSIST_PROFILE.tiltDerateStop - VELOCITY_ASSIST_PROFILE.tiltDerateStart), 0, 1);
  const heightScale = clamp((height - VELOCITY_ASSIST_PROFILE.heightDerateStop)
    / (VELOCITY_ASSIST_PROFILE.heightDerateStart - VELOCITY_ASSIST_PROFILE.heightDerateStop), 0, 1);
  const descentScale = clamp((verticalSpeed - VELOCITY_ASSIST_PROFILE.descentDerateStop)
    / (VELOCITY_ASSIST_PROFILE.descentDerateStart - VELOCITY_ASSIST_PROFILE.descentDerateStop), 0, 1);
  return Math.min(tiltScale, heightScale, descentScale);
}

export function lowPass(previous, sample, timeConstant, dt) {
  return previous + (sample - previous) * (1 - Math.exp(-dt / timeConstant));
}

export function accelerationLimitedVelocity(current, target, gain, acceleration, deceleration, dt) {
  const requestedAcceleration = (target - current) * gain;
  const limit = Math.abs(target) > Math.abs(current) ? acceleration : deceleration;
  return current + clamp(requestedAcceleration, -limit, limit) * dt;
}

export function updateStabilityEnvelope(previous, measured, recoveryRate, dt) {
  const safeMeasured = clamp(measured, 0, 1);
  return safeMeasured < previous ? safeMeasured : Math.min(safeMeasured, previous + recoveryRate * dt);
}

export function sanitizeCommand(command = {}) {
  return {
    vx: clamp(command.vx, -A2_LIMITS.maxLinearX, A2_LIMITS.maxLinearX),
    vy: clamp(command.vy, -A2_LIMITS.maxLinearY, A2_LIMITS.maxLinearY),
    yawRate: clamp(command.yawRate, -A2_LIMITS.maxYawRate, A2_LIMITS.maxYawRate),
    frequency: clamp(command.frequency ?? DEFAULT_COMMAND.frequency, A2_GAIT_PROFILE.minCadence, A2_GAIT_PROFILE.maxCadence),
  };
}
