# Progress

Running status of the course simulation environment. Newest work at the top.
Detailed findings and every gotcha live in [`docs/spike-notes.md`](docs/spike-notes.md).

**Repo:** `git@github.com:mzahana/drone-course-sim.git`

---

## Where things stand

| Piece | Status |
|---|---|
| ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17 + MAVROS in one image | **done**, validated |
| Self-contained simulation assets (airframe, gimbal, world) | **done**, asserted at build time |
| Camera into ROS (`/camera/image_raw`, `camera_info`) | **done**, ~25 Hz |
| TF tree `base_link -> camera_optical_frame` | **done**, asserted by `course verify` |
| Gimbal control from ROS topics | **done**, angle + rate + watchdog |
| Ground truth for scoring | **blocked** — bridge loses link names |
| YOLO11n node on the camera stream | not started |
| Moving ground target + scenario | not started |
| Scoring script | not started |
| Lab skeletons and solutions | not started |

**Image:** ~3.4 GB to download, ~13 GB on disk.

---

## Done

### Gimbal ROS command interface
Student code points the gimbal over topics, never services:

| Topic | Type | Use |
|---|---|---|
| `/gimbal/cmd/angle` | `Vector3Stamped` (rad) | absolute — "look there" |
| `/gimbal/cmd/rate` | `Vector3Stamped` (rad/s) | closed-loop tracking |
| `/gimbal/attitude` | `Vector3Stamped` (rad) | measured, from the gimbal's own joints |
| `/gimbal/saturated` | `Vector3Stamped` | 1.0 per axis against a limit |

Verified on a clean image: an angle command of -0.7854 rad settles at -0.780 measured; a 20 deg/s rate command
slews and the watchdog freezes the gimbal 0.31 s after commands stop. Travel limits default to
the A8 mini (pitch -90/+25, yaw +/-135) rather than to what the simulated joints could do.

**Why not MAVROS.** Two reasons, and the first is the important one. On the real aircraft the
A8 mini hangs off the *onboard computer* over Ethernet — PX4 never sees the commands — so
commanding it from ROS is the faithful arrangement, not a simulation shortcut. Second, MAVROS
exposes gimbal control as *services*, which block the executor on every call and are unusable in
a 30 Hz pixel-error loop.

(`/mavros/gimbal_control/manager/pitchyaw` returns DENIED regardless of control acquisition.
That is now moot rather than blocking, and is left documented in the spike notes.)

### Self-contained simulation assets
`models/` carries the whole aircraft — `course_x500_base`, `course_x500`, `course_gimbal`,
`x500_course_gimbal` — plus `worlds/course_world.sdf`. Nothing is inherited from whatever PX4
ships, so a PX4 bump cannot silently change the aircraft, the camera or the world underneath the
course material. The image build **asserts** it: any `model://` reference that is not a
`course_*` asset fails the build.

Three fixes went in with it:
- FOV 2.0 -> 1.414 rad. The stock lens is a 114.6 deg fisheye; the A8 mini is ~81.
- Joint controllers can now integrate (`i_max`/`i_min` were pinned to 0, making `i_gain` dead
  code). Standing pointing error went from 2.4 deg to
  0.3 deg single-axis / 1.1-1.4 deg on a combined move — 2.4 deg is 1.2 m on the ground at
  28 m slant range. Better, not perfect; the gains are the place to look if tracking needs more.
- Literal `3.14` replaced with pi: a built-in 0.09 deg error.

### Verified camera frame
`course verify` asserts optical z = forward, x = right, y = down, and catches regressions.
The two original bugs: the mount offset was 0.26 m when it should be 0.02 m, and the rotation
composition was wrong. Both were fixed by reading link poses out of a *running* simulation
rather than deriving three chained 180 deg rotations on paper.

---

## Next

1. **Ground truth for scoring.** Bridging `gz.msgs.Pose_V` to `TFMessage` drops `frame_id`, so
   link names are lost. Needs a small node reading `/world/course_world/pose/info` directly.
2. **YOLO11n node** against `/camera/image_raw`.
3. **The scenario**: a driving ground target on a scripted route, with difficulty tiers.
4. **Scoring script** reading ground truth, plus the detector-blackout injector.
5. **MAVROS TF**: `map -> base_link` is not published yet; geolocation needs it.

---

## Known open issues

- Ground truth link names lost in the bridge (blocks scoring).
- `map -> base_link` not yet published by MAVROS.
- `/mavros/gimbal_control/manager/pitchyaw` returns DENIED — worked around, not fixed.
