# A2 simulator regression

Run `npm run test:environments` for the focused obstacle suite, or `npm test`
for all regression tests. No browser, real robot or gamepad is needed.
The checked-in GitHub Actions workflow runs all tests and the build on pushes
and pull requests once these files are pushed. It has not been run remotely yet.

## Reproducible coverage

- 24 geometric buffer-exit cases: four headings × front/rear × 3/6/9cm separation.
- 24 MuJoCo rough-terrain cases: mirrored layouts, ±12cm position offsets,
  opposite-side targets, 3/5/8/12m target distances and two box sizes.
- Existing focused suites cover the reported screen pose, two-box detours,
  a narrow corridor, rear proximity, approaching actors and assisted recovery.
- Low-step tests require 1cm and 2cm physical steps to be crossed without
  obstacle avoidance, and verify that low terrain does not become a solid box
  in mixed-obstacle planning. The >2cm threshold is also checked.
- The full suite also checks waypoint queues, manual stop handling, dynamic
  actors, pedestrian yielding and 1m/s preview cruise.
- Twelve moving-encounter physical cases plus geometric prediction tests check
  lateral avoidance, limited reverse, blocked escape, multiple actors and
  recovery of waypoint progress before actors expire. See `A2_DYNAMIC_ENCOUNTERS.md`.

Each physical matrix case must reach the target with its body centre within
20cm and receive exactly one reward. Faults, automatic recovery, 20 seconds
without 8cm displacement, routes longer than 25m, and the 180-second simulation
budget are failures. Motion in circles cannot pass just by avoiding the stall
limit: arrival and route-length assertions are also required.

Failure messages contain the scenario parameters, final position, command,
avoidance state and navigation state. Subtest names contain the parameters;
rerun the file to reproduce them. Durations, travelled distance and peak tilt
are printed on successful runs. Cases are deterministic rather than time-seeded.
Do not remove failed scenarios or weaken their completion assertions to pass.

## Defects found by this matrix (2026-09-27)

1. Advancing a route corner within the waypoint tolerance could cut into an
   obstacle buffer. Route advancement now requires a clear connecting segment.
2. A level-ground detour could be released early using a different obstacle
   set. Initially fixed by preserving that set; subsequently removed because
   converting traversable steps to boxes caused unnecessary avoidance.
3. A goal that fits the fixed rectangular footprint but not the turning circle
   could reject every route from the outset. The local search now supports a
   checked, fixed-heading final approach within 1.2m.
4. Slow 20cm-cell tracking stalled on low steps. Checked lookahead up to 80cm
   and a 0.45 forward command cap (not measured m/s) maintain progress. The
   original gait, terrain collisions and height threshold are unchanged.
5. After a detour, the old segment's lookahead point could cross an obstacle
   despite a clear route to the real target. Detour release now anchors the
   return segment at the current pose, matching the route actually checked.
   In-progress low-step crossings retain heading until clear where practical.

## Scope and limitations

The matrix uses the real browser worker/controller in headless MuJoCo, including
the existing preview translation servo. Geometric cases vary initial heading;
physical cases start from the normal settled pose. This is not hardware safety
validation, a proof that every possible arrangement is traversable, or a direct
measurement of contact forces. Screenshots alone do not capture edited obstacle
coordinates. Retain newly reported configurations as additional regression cases.
