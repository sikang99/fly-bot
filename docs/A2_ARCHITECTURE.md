# Unitree A2 walking environment

## Scope

This repository now provides a browser-based baseline walking environment for the Unitree A2. It is intended for simulation, controller experiments and command-interface development. It is not a certified controller and must not be connected to a physical robot at joint-torque level without an independent safety review.

## Data flow

```text
UI velocity command
  -> command clamp
  -> forward obstacle corridor check
  -> footprint-aware lateral bypass and heading hold
  -> diagonal-trot phase generator
  -> 12 joint position targets
  -> bounded PD torque controller
  -> MuJoCo A2 plant
  -> pose and safety telemetry
```

The MuJoCo plant is compiled from Unitree's official A2 MJCF. Browser rendering is procedural, so only visual mesh declarations are removed. Masses, inertias, collision shapes, joint limits, effort limits, actuators and sensors are retained.

## Forward-axis convention

The official MJCF places `FR/FL` hips at `x=+0.25944m` and `RR/RL` hips at `x=-0.25944m`, so A2 forward is world/body `+X` at the reset pose. The procedural nose, green front arrow, camera, LiDAR cone, waypoint heading and obstacle planner all use that same convention. Positive `vx` means forward. World X/Y telemetry makes the sign visible during testing.

## Arm morphology prototype

A fixed single-arm visual prototype is mounted on the `+X` front section of `base_link`, above the FL/FR leg line as shown in the morphology reference. It includes a 0.75-scale base yaw stage, shoulder, elbow, wrist and two-finger gripper. Its initial and current default is a compact pose folded rearward over the body; it remains fixed there during walking until a later manipulation controller explicitly takes ownership. The prototype follows the robot pose without changing MuJoCo mass or walking dynamics. The intended biological translation is documented in `docs/MORPHOLOGICAL_MAPPING.md`: T1 bilateral foreleg channels map to arm common/differential commands, T2 maps to FL/FR, and T3 maps to RL/RR. Physical arm inertias, joints, collision and control remain a separate implementation stage.

## Joint convention

The controller uses the SDK order below:

```text
FR hip, thigh, calf
FL hip, thigh, calf
RR hip, thigh, calf
RL hip, thigh, calf
```

The nominal standing pose is `hip = +/-0.1`, `thigh = 0.9`, `calf = -1.8` radians. Hip and thigh torque commands are limited to 120 Nm and calf commands to 180 Nm, matching the official MJCF.

## Safety boundary

- `Passive` always writes zero torque.
- Commands are clamped to the validated simulation envelope: `vx +/-0.6 m/s`, `vy +/-0.5 m/s`, `yaw +/-0.35 rad/s`. The trot cadence is fixed at 1.4Hz. This is a simulation limit, not permission to use the same command envelope on hardware.
- A base height below 0.24 m or roll/pitch above 0.9 rad switches the controller to `Passive`.
- Space triggers `Passive` in the browser.
- The demonstration box is represented as a MuJoCo mocap body and a synchronized Three.js mesh. It can be dragged in either the external view or the A2 camera view, while the worker continuously uses its current position for collision and avoidance planning.
- Avoidance uses a swept A2 footprint rather than a point or only `base_link`: a 0.92m × 0.64m body/leg envelope plus 0.10m on every side. Obstacle half-extents are projected into the robot frame, so the test works while the robot is turning. Avoidance begins up to 1.65m ahead and remains active until the obstacle is behind the complete footprint.
- The green/orange ground rectangle visualizes the inflated 1.12m × 0.84m footprint used by the planner. Telemetry reports the lateral corridor clearance while avoidance is active.
- A new avoidance is acquired only when the obstacle's leading edge is actually in front of the robot. An obstacle beside or behind the body cannot replace a clear positive-`vx` command, preventing false avoidance from looking like reverse walking. With usable forward clearance the planner selects `ARC`: it keeps positive forward speed, adds a bounded yaw command toward the chosen free side, and uses only a small lateral bias. If clearance falls below 0.35m while the side corridor remains blocked, it switches to `SIDESTEP`, caps forward demand at 0.05m/s and opens the side corridor without rotating. The preview lateral servo is capped at 0.16m/s. Once the side corridor opens, the state changes to `BYPASS`: lateral velocity returns to zero, the selected side and current bypass heading remain latched, and forward motion continues without accepting path-return steering. `CLEAR` is allowed only after the obstacle's far edge is behind the complete rear footprint and margin. The original cruise heading or waypoint planner then performs a single path return.
- A pre-fall stability supervisor monitors roll, pitch, base height and downward velocity. Deterioration immediately scales both the IK stride and preview acceleration toward zero; recovery is limited to 50% per second so a single good gait phase cannot restore full stride abruptly. Bounded roll/pitch damping remains active throughout `Walk`.
- The command layer is separate from the physics worker so that a real-robot adapter can use the same bounded velocity command without reusing browser torque output.

For a physical A2, use Unitree SDK2's A2 `SportClient` for the initial integration. Do not forward the simulator's raw torque array to the robot. Add a dead-man signal, network watchdog, physical emergency stop, operating-zone checks and on-robot command timeout before any hardware test.

## Camera and LiDAR perception adapter

The fly's paired compound-eye input cannot be replaced directly with A2 camera pixels or LiDAR points. Their dimensions, field of view and meaning differ. A perception adapter should convert A2 sensor frames into a small robot-centric representation:

```text
HD camera -> left/right image sectors -> optical flow, target bearing, looming
front/rear LiDAR -> range sectors -> free space, obstacle bearing, clearance, closing speed
                                    |
                                    v
                      fused egocentric perception frame
                                    |
                   gait command / avoidance / safety stop
```

For connectome experiments, the camera image can be resampled onto the existing left/right ommatidia directions and passed through flyvis. LiDAR-derived looming and proximity signals can supplement the corresponding visual and touch channels. For practical A2 navigation, the preferred path bypasses the biological photoreceptor representation and feeds the fused perception frame to the local planner. Emergency stopping remains independent of both paths.

## Running

```bash
npm ci
npm run dev
```

Open `/a2.html`. Add `?terrain=rough` to place two low obstacles in the physics scene.
Use **Obstacle avoidance demo** to reset the robot, start a forward walk and visualize the sensor corridor, active avoidance state and traveled path.
Drag the orange box in either camera view to create a new avoidance case, and use **Reset obstacle** to restore its starting position. **Stable cruise +X** applies the validated `0.6 m/s`, `1.4 Hz` command profile. Its measured browser-preview speed is approximately `0.42 m/s`. Tune and validate a separate conservative profile before any hardware experiment.

## Smoothed gait

The default cadence is fixed at 1.4Hz so increasing a UI command cannot simultaneously increase stride rate and destabilize the trot. Each diagonal pair uses a 42% swing / 58% stance cycle. The two 8% overlap windows put all four feet in stance between diagonal swings, providing a support transition that the earlier 50/50 trot lacked. Foot fore-aft motion follows a piecewise cosine trajectory and swing height follows a squared-sine trajectory, giving zero velocity at lift-off and touchdown instead of an abrupt velocity change. Velocity and yaw commands pass through a 0.4-second first-order transition. Step length is capped at 0.28m, turn-induced left/right stride asymmetry is bounded, and swing clearance is capped at 6.5cm. At cruise demand the stance target shortens by up to 1.5cm, producing a slightly crouched posture with more joint travel available for impact absorption. Joint derivative gains damp impact oscillation.

The open-loop IK trot still computes joint motion, contact, body height and attitude in MuJoCo. For a stable browser demonstration, planar route progress is handled separately by an acceleration-limited preview translation servo. It follows the filtered command, ramps at no more than `0.25 m/s²`, brakes at up to `0.60 m/s²`, limits lateral bypass speed to `0.16 m/s`, and derates before the fall boundary when tilt, height or downward speed deteriorates. The same stability envelope now scales the velocity target itself, not only acceleration and stride, so an already-fast robot decelerates when its support posture deteriorates. Stable cruise uses 80% of the bounded command and avoidance uses 65%. A captured heading reference keeps zero-yaw cruise aligned with the original `+X` direction. This servo translates the simulated root without adding angular momentum, so it is visualization scaffolding rather than a dynamics-faithful actuator or a command for hardware. A physical A2 must use the SDK2 SportClient velocity controller and Unitree's closed-loop locomotion stack instead.

The acceptance test for this preview profile is an obstacle bypass followed by at least 60 simulated seconds of continuous walking: no safety fault, measured forward speed `0.35–0.50 m/s`, base height above `0.24m`, and straight-cruise yaw error below 3 degrees. With the quadruped-adaptive gait, the validated run reached 68.2 seconds at `0.38–0.40 m/s`; the final sample had base height `0.302m`, roll `-0.5°`, pitch `-6.1°`, and no safety fault.

## Mouse waypoints

Clicking the simulation floor appends a waypoint in world coordinates. Arrival distance is measured only from the `base_link` centre projected onto the ground—not the nose, feet, sensor cone, or swept footprint—and requires that centre to enter a 0.06m radius around the waypoint. With no obstacle override, the waypoint follower uses the verified stable 0.60m/s gait command (about 0.48m/s preview velocity) over the open part of the route, then follows a deceleration-limited stopping profile so the body centre does not overshoot the point. Entering the centre radius completes the waypoint and awards `+1` exactly once. The cumulative reward and a short reward pulse are shown in telemetry and the robot-camera HUD. After each arrival the completed marker remains green and control immediately transfers to the next queued waypoint; when the queue is empty the robot returns to `Stand`. For a route's first target more than about 20 degrees away it performs one initial in-place alignment at up to the configured safe yaw limit of 0.35rad/s instead of entering a blind forward arc. Once forward tracking starts, ordinary corrections and subsequent queued waypoints use continuous forward arcs. A progress watchdog records the best distance: if distance grows by more than 0.18m or fails to improve by 0.02m for 1.5 seconds, `REALIGNING` temporarily suppresses translation, points the body within about 10 degrees of the target, and automatically resumes forward tracking. A temporary planar anchor prevents drift during either bounded alignment. Obstacle avoidance is applied after waypoint navigation, so its command override has priority.

## Robot camera view

A second renderer uses a 78-degree perspective camera attached to the front of `base_link`. It renders the same terrain, obstacle and waypoint objects as the external view, so the inset follows the simulated body pose without duplicating world state. The HUD reports navigation/avoidance state and the nearest mapped obstacle range. This is currently a rendered RGB simulation view; later sensor work should replace or augment it with the physical A2 camera stream and timestamped LiDAR sectors.

### Navigation and avoidance ownership

`passage.js` detects overlapping pairs of axis-aligned solid boxes with 0.86–1.35m free width. Passage control precedes docking: align the long body axis to the corridor, then translate slowly along its centre at up to 0.24m/s command. Rotation sweeps are checked at 25 headings using the conservative footprint. If rotation is unsafe, bounded fore/aft pocket searches may release room; otherwise stop and display the blocked state. This geometry heuristic supports editor-box corridors, not arbitrary maze planning. A physical-worker regression verifies a 1m-wide, 2m-long passage with lateral error under 0.1m, heading error under 0.16rad and final waypoint reward.

Near-obstacle docking activates within 1.2m of a waypoint near a blocking object if translation at the current heading is clear. It uses a heading-specific conservative swept body envelope instead of the rotation envelope. Safe lateral correction takes priority, followed by bounded fore/aft correction (0.12m/s command) and lateral correction (0.22m/s command), without yaw. Existing 0.10m clearance remains. Targets inside blocking geometry are not rewarded and show GOAL BLOCKED at arrival. This is not a full narrow-space reachability solver.

Body-centre segment tracking fixes each route leg between its start and endpoint. It projects the body centre onto that segment and follows a 0.55m lookahead point, returning toward the line after lateral displacement instead of continually shortcutting directly to the endpoint. The last 0.45m uses endpoint centering. Distance, arrival and rewards always refer to the true waypoint, never the intermediate aim. Obstacle avoidance still overrides segment tracking. Rendered route origins are fixed too.

In-place waypoint alignment uses a dedicated preview yaw assist capped at 0.70rad/s with proportional gain 1.6 and stability derating. This is enabled only during navigation alignment without obstacle avoidance. Moving turns and joint gait commands retain their 0.35rad/s cap. The existing yaw torque bound and planar alignment anchor remain active; speed tapers near the requested heading.

Waypoint-aware avoidance now scores the two lateral clearance positions using travel to the clearance point plus remaining distance to the active waypoint. A segment/AABB test, inflated by a rotation-safe body envelope, checks the route all the way to that waypoint. A clear route releases the avoidance latch and reacquires target heading in place, instead of cruising until the old obstacle is behind. Obstacles beyond the waypoint no longer trigger an unnecessary detour. Reaching a waypoint clears old avoidance state; switching obstacle IDs does not inherit the previous obstacle's direction or held heading. This is a local detour heuristic, not a globally shortest-path planner for arbitrary mazes.

Each queued waypoint now starts a fresh segment acquisition: hold position and align within 0.12rad before translating. Already aligned segments start immediately. This supersedes the earlier continuous-arc queue transition, which could wander far from short or reversing routes. Square, reversal and short-zigzag physics tests require the body centre to stay within 0.25m of each clear segment. Nearby-obstacle sidesteps request zero forward velocity and zero yaw, and remain latched until the lateral footprint plus bypass margin clears the obstacle; only then may forward bypass resume. Filtered motion still decelerates rather than teleporting to a stop.

While obstacle avoidance is active, waypoint recovery is suspended, its progress baseline is refreshed, and its planar alignment anchor is cleared. Avoidance owns both translation and heading until the detour ends. Without this arbitration, recovery pins position while bypass holds heading: neither can finish. This was reproduced with a box at (2, 0) and waypoint (5, 0), leaving the robot at approximately (1.09, 0.60) with yaw 10 degrees for over 40 simulation seconds.

`test/a2-physics.test.js` runs the actual worker and MuJoCo model in Node using the same control and physics steps as the browser. It verifies 45 clear distance/bearing cases, nine obstacle placements around the reproduced failure, and a five-waypoint zigzag. Checks include arrival from the body centre, rewards, passive/fault states, final standing, straight-route yaw, and exclusive control ownership. These physics tests complement the ideal kinematic planner tests; planner convergence alone does not validate the integrated simulator.

### Rough-terrain consistency and terminal approach

`terrain.js` supplies the same obstacles to MJCF generation, rendering, and navigation. With `?terrain=rough`, the green 2cm platform is traversable and the red-orange 16cm platform must be bypassed. The former 8cm platform was reduced to a tested 2cm preview threshold; this is not an A2 hardware capability specification. Low platforms retain physical collision geometry but are excluded from avoidance. Both platforms can be selected and dragged like the box; physics mocap positions and render positions move together. Reset obstacles restores every obstacle. Six rough routes, three relocated low-step crossings and one relocated high-step detour supplement the 55 flat scenarios. Originally the platforms existed only as invisible collision geometry and caused a stall near x=1.24m.

Within 0.45m of a tracked waypoint, the follower holds heading and uses bounded local translation (including a small reverse correction after overshoot) to center the body. This avoids large return turns into nearby terrain. Arrival still requires a body-centre error at most 0.06m, with one reward per target. Outside that zone, forward pursuit and obstacle avoidance remain active. These are preview-servo tests, not hardware locomotion certification.

## Obstacle editor

The sidebar creates boxes or steps, resizes selected objects and deletes them. Dimensions are full metres (XY 0.1–4m, height 0.01–2m), with a 32-object cap. New objects appear at robot XY + (2,2), then can be dragged. Edits rebuild the MuJoCo model atomically to refresh collision bounds; mocap-only changes preserve joint layout, robot qpos/qvel, simulation time and waypoint queue. Locomotion enters Stand and requires Walk to resume. Rendering is reconciled from the worker's authoritative obstacle list. Step traversability/color are recalculated from height. Changes are session-only and reload restores defaults; Reset simulation retains edited objects.

## Sources

- Unitree A2 product page: <https://www.unitree.com/A2/>
- Unitree SDK2: <https://github.com/unitreerobotics/unitree_sdk2>
- Unitree MuJoCo: <https://github.com/unitreerobotics/unitree_mujoco>
- Unitree RL MjLab A2 configuration: <https://github.com/unitreerobotics/unitree_rl_mjlab>
