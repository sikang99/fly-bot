// Experimental telemetry encoding, NOT an established biological mapping or controller.
export const LINK_GROUPS = { motion: 'vnc_sensory', turn: 'ascending_neuron', obstacle: 'visual_projection', arm: 'descending_neuron' };
const unit = x => Math.max(0, Math.min(1, Number.isFinite(x) ? x : 0));
export function telemetryDrives(pose) {
  if (!pose || pose.fault || pose.mode === 'passive') return { motion: 0, turn: 0, obstacle: 0, arm: 0 };
  return {
    motion: 120 * unit(Math.hypot(pose.forwardSpeed || 0, pose.lateralSpeed || 0) / .6),
    turn: 100 * unit(Math.abs(pose.command?.yawRate || 0) / .35),
    obstacle: pose.avoidance?.active ? 100 : 0,
    arm: pose.armWork?.active ? 100 : 0,
  };
}
export function selectLinkGroups(data, cap = 64) {
  return Object.fromEntries(Object.entries(LINK_GROUPS).map(([key, name]) => {
    const ids = [];
    for (let i = 0; i < data.N; i++) if (data.meta.superclasses[data.superclass[i]] === name) ids.push(i);
    const count = Math.min(cap, ids.length);
    return [key, Array.from({ length: count }, (_, i) => ids[Math.floor(i * ids.length / count)])];
  }));
}
