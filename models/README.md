# Course simulation models

`x500_course_gimbal` is the aircraft the course flies. It is PX4's `x500_gimbal`
with two changes, plus one deliberate non-change.

## Changed

**Camera field of view: 2.0 rad -> 1.414 rad (114.6 deg -> 81 deg).** PX4's stock gimbal
carries a near-fisheye lens. The course spends a whole session on the relationship between
altitude, look angle and position error, and on how far away a target can still be detected --
all of which are meaningless if the simulated optics do not resemble the SIYI A8 mini the
students will actually fly.

**Joint state feedback added.** A `JointStatePublisher` now reports the three gimbal joint
angles. Without it the only available gimbal angle is the autopilot's own report, which leaves
no way to check the TF tree against what the gimbal actually did.

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
