# Fly-to-A2 morphological mapping

## Target morphology

The target platform is a quadruped with one arm and a gripper mounted above the front-leg line. The current browser model places that base on the `+X` front section of `base_link` and renders a base yaw stage, shoulder, elbow, wrist and two-finger gripper as a visual prototype. The arm starts and remains folded rearward over the body during walking; later manipulation work will add the only path that can deploy it. It follows `base_link` but has no mass, collision or actuator coupling yet, so it cannot destabilize the walking baseline while the neural mapping is being designed.

## Translation matrix

| Fly motor domain | Robot target | Initial translation |
| --- | --- | --- |
| T1 left/right forelegs | Single arm | Sum drives reach/lift and gripper intent; left-right difference drives arm yaw or wrist orientation. |
| T2 left/right middle legs | FL/FR quadruped legs | Preserve left-right support and propulsion timing for the front pair. |
| T3 left/right hind legs | RL/RR quadruped legs | Preserve strong propulsion, rear support and jump-like extension for the rear pair. |
| T1-T2-T3 interneuron coupling | Arm/leg coordination | Keep the arm retracted during fast trot, permit reach during stable support, and compensate arm motion with stance control. |

## Implementation stages

1. Visual morphology: fixed 4-DOF arm and gripper on `base_link`.
2. Physical morphology: add arm links, inertias, joint limits and collision groups to MJCF.
3. Safe arm controller: bounded position targets with a separate emergency-stop and self-collision checks.
4. Neural adapter: convert T1 bilateral activity into common-mode and differential arm channels.
5. Whole-body coordination: gate manipulation by support phase and compensate the arm payload in the gait controller.

The mapping is functional rather than a claim that insect and robot joints are anatomically equivalent. The adapter must preserve signal relationships while enforcing the robot's mechanical and safety constraints.
