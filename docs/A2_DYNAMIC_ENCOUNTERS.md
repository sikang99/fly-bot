# Moving-obstacle encounters (simulation)

The encounter planner runs at up to 20Hz, predicts four seconds of constant
actor velocity, and checks relative swept segments every 0.1 seconds. Segment
checks include the full rectangular body/foot envelope and planning margin;
fast actors crossing between samples are not checked only at their endpoints.
Cars retain their lengthwise travel orientation through the existing geometry.

When the planned motion or remaining stationary is threatened, the candidates
are left/right translation, shallow forward-left/right translation, a short
reverse, and waiting. Feasible side steps have priority over reverse, which has
priority over waiting. Every candidate checks static and moving obstacles.
Side preference persists while safe to reduce oscillation. Heading is held;
return to waypoint navigation requires 0.6 seconds of clear prediction.
The existing forward escape for a rear approach remains higher priority.

Reverse planning has a 0.45m episode budget and slows as that budget is used.
Physical settling is not an exact distance clamp; tests bound rearward motion
to 0.7m from the scenario's origin, including initial settling. Backward travel
is never chosen without checking rear occupancy and predicted actor motion.
When every candidate is unsafe, the UI reports no safe escape and commands
zero motion. This cannot prevent an external actor from striking a trapped robot.

## Locomotion boundaries

The joint command limits, gait frequency, tilt protection and acceleration
limits are unchanged. Dynamic side-stepping targets 0.30m/s in the browser's
existing translation-assist layer. Bounded lateral feedback (assist at most
0.50m/s) compensates gait drag; prediction uses measured filtered net forward
and lateral velocity instead of assuming the assist equals actual motion.
Actual speed varies with stance and stability derating. These are simulation
settings, not validated Unitree SDK commands or a deployable hardware policy.
Manual mode remains outside autonomous planning; Passive/Stand clear encounter
state, and existing emergency-stop handling is retained.

## Reproduction and limitations

Run `node --test test/a2-encounter.test.js` or `npm run test:environments`.
Twelve physical scenarios cover head-on people, faster people, a person appearing
during cruise, two people, blocked left/right sides, a corridor requiring reverse,
a head-on car with warning distance, and mirrored near/far crossing cars.
Pedestrian cooperation is disabled. Tests require the requested behaviour,
no fault/recovery or sampled geometric body-envelope contact, resuming before
actors expire, and body-centred waypoint arrival with one reward. They also
exercise unsafe rear space, exhausted reverse budget, heading rotation,
side preference, and high-speed inter-sample crossings in geometric tests.

Prediction assumes constant actor velocity over the horizon. It is not a
guarantee against sudden acceleration, sensor latency, occlusion, a spawn inside
the robot, or insufficient escape space. Simulation uses exact scene geometry,
not real camera/LiDAR measurements. Contact checks in tests are geometric, not
measured contact forces. Real A2 deployment requires separate sensing, SDK,
balance-controller and safety validation.
