# Course simulation models

Everything the course flies is **vendored here**. The simulation does not load a single asset
from whatever PX4 happens to ship, so a PX4 version bump cannot silently change the aircraft,
the camera, or the world out from under the course material.

| Asset | What it is |
|---|---|
| `course_x500_base` | Airframe body, motors, props, meshes and textures |
| `course_x500` | The x500 quadrotor, composed from `course_x500_base` |
| `course_gimbal` | The gimbal: CGO3 kinematics, A8-mini-like optics, joint state feedback |
| `x500_course_gimbal` | The course aircraft: `course_x500` + `course_gimbal` |
| `course_target_vehicle` | The ground target the drone follows |
| `../worlds/course_world.sdf` | The world the course flies in |

Every `model://` reference inside these resolves to another `course_*` asset in this repo, and
**the image build asserts it** — if anything ever points back at a PX4 asset, the build fails
rather than the lab.

`course_target_vehicle` uses the mesh and textures of the **"Pickup"** model from Gazebo Fuel,
by Nate Koenig (Open Robotics). The body was made dynamic, given a box collision instead of the
full mesh, and fitted with velocity control and an odometry publisher. **Its licence is not
stated in the model metadata and should be confirmed before this repository is made public** --
most Open Robotics Fuel models are CC-BY 4.0, but that has not been verified here.

It is a textured vehicle mesh rather than a box primitive for one reason: it has to be
recognisable to YOLO, and COCO has no "box" class.

`course_x500_base`, `course_x500` and `course_world.sdf` are derived from PX4's Gazebo models,
BSD-3-Clause, copyright 2022 PX4 Autopilot for Drones. See `LICENSE-PX4-ASSETS`. They are
renamed rather than kept under their original names so that they cannot be confused with, or
shadowed by, PX4's own copies on the resource path.

## Gimbal changes

`course_gimbal` is PX4's CGO3 gimbal with three changes, plus one deliberate non-change.

**Camera field of view: 2.0 rad -> 1.414 rad (114.6 deg -> 81 deg).** PX4's stock gimbal
carries a near-fisheye lens. The course spends a whole session on the relationship between
altitude, look angle and position error, and on how far away a target can still be detected --
all of which are meaningless if the simulated optics do not resemble the SIYI A8 mini the
students will actually fly.

**Joint state feedback added.** A `JointStatePublisher` now reports the three gimbal joint
angles. Without it the only available gimbal angle is the autopilot's own report, which leaves
no way to check the TF tree against what the gimbal actually did.

**The joint controllers can now actually integrate.** The stock model sets `i_max` and `i_min`
to `0`, which clamps the integral term to zero and makes the configured `i_gain` dead code. The
result is a standing pointing error under gravity: commanded -45.00 deg, measured -42.6 deg.
That is 2.4 deg, and at 28 m slant range 2.4 deg is **1.2 m on the ground** -- larger than the
entire geolocation error budget the course teaches. With the limits opened to +/-1.0 the residual
drops to roughly 0.3 deg on a single-axis move and 1.1-1.4 deg on a combined pitch+yaw move
(yaw has a lower P gain and settles more slowly), with no oscillation. Better, not perfect:
if the tracking lab ever needs tighter pointing, the gains are the place to look.

(The TF tree is built from *measured* joint angles, so geolocation was never biased by this --
but a gimbal that cannot hold the angle it was told to hold makes the tracking lab feel broken.)

**Meshes actually come from this repo.** The original model referenced
`model://gimbal/meshes/...`, so a copied-and-renamed gimbal silently kept loading PX4's meshes
and the ones sitting in this repository were dead weight. They now resolve to
`model://course_gimbal/meshes/...`.

**`3.14` replaced with pi to full precision.** The stock model rotates the gimbal mount and the
camera sensor by a literal `3.14`, which is 179.909 deg -- a built-in 0.09 deg error. In a course
that teaches an angular error budget where 1 deg costs half a metre at 28 m range, shipping a
tenth of a degree of avoidable error in the model itself is not defensible.

## Deliberately NOT changed

**The kinematic conventions.** The mount is rotated 180 deg about z, and the camera sensor is
rotated another 180 deg inside `camera_link`. The two cancel, so the camera does look forward --
but it means `camera_link`'s own +x axis points *backwards*.

It is tempting to "clean this up". Do not. PX4's gimbal control assumes these joint sign
conventions, and flipping the mount silently inverts commanded yaw and roll relative to the
airframe. The frames are instead handled explicitly in the URDF, where they are visible and
commented, rather than hidden in a model edit that breaks control.
