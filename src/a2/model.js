// The checked-in MJCF is Unitree's official A2 model. The browser renderer uses
// procedural geometry, so visual STL references are removed before MuJoCo compiles it.
// Masses, inertias, collision shapes, joint limits, actuators and sensors remain intact.
import { worldObstacles } from './terrain.js';

export function buildA2WorldXml(source, terrain = 'flat', objects = worldObstacles(terrain)) {
  let xml = source
    .replace(/\s*<mesh\b[^>]*\/>/g, '')
    .replace(/\s*<geom\b(?=[^>]*\btype="mesh")[^>]*\/>/g, '')
    .replace('<option gravity="0 0 -9.81"/>', '<option gravity="0 0 -9.81" timestep="0.002" integrator="implicitfast"/>')
    .replace('<body name="base_link" pos="-1 1 0.7">', '<body name="base_link" pos="0 0 0.62">');

  const obstacles = objects.map(o => `<body name="obstacle_${o.id}" mocap="true" pos="${o.x} ${o.y} ${o.halfZ}"><geom name="${o.id}" type="box" size="${o.halfX} ${o.halfY} ${o.halfZ}" rgba=".92 .49 .16 1" friction="1.1"/></body>`).join('');
  const world = `<geom name="floor" type="plane" size="20 20 .1" rgba=".18 .21 .24 1" friction="1.1 .01 .001"/>${obstacles}`;
  xml = xml.replace('<worldbody>', `<worldbody>${world}`);
  return xml;
}
