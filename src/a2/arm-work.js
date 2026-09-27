import { ARM_STOW_POSE } from './config.js';

export const ARM_ACTIONS = Object.freeze({ none: '이동만', front: '팔 앞으로', left: '팔 몸 왼쪽', right: '팔 몸 오른쪽' });
export const armAction = value => Object.hasOwn(ARM_ACTIONS, value) ? value : 'none';
const smooth = t => { t = Math.max(0, Math.min(1, t)); return t * t * (3 - 2 * t); };

// Visual work sequence only: no manipulator mass, contact, grasp or SDK output.
export function armWorkPose(action = 'none', seconds = 0) {
  const amount = action === 'none' ? 0 : seconds < 1 ? 0 : seconds < 3 ? smooth((seconds - 1) / 2)
    : seconds < 5 ? 1 : 1 - smooth((seconds - 5) / 2);
  return { yaw: (action === 'left' ? Math.PI / 2 : action === 'right' ? -Math.PI / 2 : 0) * amount,
    shoulder: ARM_STOW_POSE.shoulderPitch + (Math.PI / 2 - ARM_STOW_POSE.shoulderPitch) * amount,
    elbow: ARM_STOW_POSE.elbowPitch * (1 - amount), wrist: ARM_STOW_POSE.wristPitch * (1 - amount) };
}

export class ArmWork {
  constructor() { this.cancel(); }
  cancel() { this.task = null; }
  start(waypoint, now) {
    if (armAction(waypoint.action) === 'none') return false;
    this.task = { id: waypoint.id, action: armAction(waypoint.action), started: now }; return true;
  }
  sample(now) {
    if (!this.task) return { active: false, stage: 'stowed', pose: armWorkPose() };
    const seconds = Math.max(0, now - this.task.started);
    return { ...this.task, active: true, done: seconds >= 7,
      stage: seconds < 1 ? 'settling' : seconds < 3 ? 'extending' : seconds < 5 ? 'working' : 'folding',
      pose: armWorkPose(this.task.action, seconds) };
  }
}
