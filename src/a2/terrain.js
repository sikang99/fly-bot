import { A2_OBSTACLES } from './config.js';

// One geometry source for physics, rendering and navigation. Heights are half
// extents. Traversability is a conservative preview-controller limit, not an
// A2 hardware specification.
export const MAX_TRAVERSABLE_HEIGHT = 0.02;
export function isTraversable(obstacle) {
  return obstacle.kind === 'step' && obstacle.halfZ * 2 <= MAX_TRAVERSABLE_HEIGHT;
}
export function terrainColor(obstacle) {
  return isTraversable(obstacle) ? '#36c98f' : '#ef694e';
}
export function worldObstacles(terrain = 'flat') {
  return [
    ...A2_OBSTACLES.map(obstacle => ({ ...obstacle, movable: true })),
    ...(terrain === 'rough' ? [
      { id: 'step_left', kind: 'step', x: 1.5, y: 0.85, halfX: 0.45, halfY: 0.7, halfZ: 0.01, movable: true },
      { id: 'step_right', kind: 'step', x: 2.5, y: -0.85, halfX: 0.45, halfY: 0.7, halfZ: 0.08, movable: true },
    ] : []),
  ];
}
