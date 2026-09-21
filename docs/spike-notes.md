# Step 1 spike — validation notes

Goal: prove that ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17 + MAVROS actually work together
before writing a single slide against them.

**Result: validated.** `/mavros/state` reports `connected: true`, mode `AUTO.LOITER`, against a
gimballed X500 in Gazebo Harmonic 8.11.0.

## What was confirmed

| Question | Answer |
|---|---|
| Does MAVROS exist for Jazzy? | Yes — `ros-jazzy-mavros` 2.14.0 from apt, no source build |
| Does PX4 1.17 ship a gimbal airframe? | Yes — `4019_gz_x500_gimbal`, stock. No custom model needed |
| Can PX4 build against the ROS *vendored* Gazebo? | **Yes.** `GZGimbal.cpp` compiles and links. One Gazebo in the image, not two |
| Does the camera exist in sim? | Yes — image + `camera_info` under `.../camera_link/sensor/camera/` |
| Is ground truth available for scoring? | Yes — `/world/default/pose/info` and `dynamic_pose/info` |
| Image size | 3.42 GB (vs the ~20 GB originally budgeted) |

Gazebo topics present on the gimballed X500: `command/gimbal_{yaw,pitch,roll}`, camera image and
`camera_info`, IMU, magnetometer, navsat, air pressure, `odometry_with_covariance`.

## Gotchas found, and what they cost

Each of these would have surfaced in front of the class.

**1. numpy.** `pip install ultralytics` resolves numpy 2.x, which pip cannot install over the
dpkg-owned numpy (`RECORD file not found`) — and which would break `cv_bridge` at import if it
could. Pinned with a constraint file plus a build-time import guard.

**2. CUDA bloat.** The default torch wheel pulls `cuda-toolkit`, `cudnn`, `cublas` and `triton`,
roughly 3 GB most student laptops cannot use. CPU index is the default; CUDA is a separate tag.

**3. PX4 must be built as the course user.** SITL writes into its own build tree at run time, so
a root-owned PX4 tree dies with `Error creating symlink .../etc`. Chowning afterwards would
duplicate 2 GB into another layer, so the clone and build happen under `USER user`.

**4. The vendored Gazebo libraries need ROS on `LD_LIBRARY_PATH`.** Without it the px4 binary
fails with `libgz-utils2.so.2: cannot open shared object file`. Sourcing ROS adds 18 vendor
directories.

**5. `.bashrc` is not enough.** `docker exec bash -c` reads no init file, and Ubuntu's stock
`.bashrc` *returns early for non-interactive shells* — so anything appended after that guard
silently never runs. Static values are now `ENV`; the `course` script sources ROS itself.

**6. `set -u` breaks ROS.** `setup.bash` reads unbound variables, so any script with
`set -u` must disable it around the source.

**7. The `gz` CLI needs `GZ_CONFIG_PATH`.** Without it `gz topic -l` returns **nothing at all,
silently** — it looks exactly like a discovery failure but is actually the CLI being unable to
load its own command plugins. Cost the most debugging time of anything here.

**8. Network isolation — the important one.** gz-transport discovers peers by multicast and is
**not** scoped by `ROS_DOMAIN_ID`. With `--network host` and no `GZ_PARTITION`, a PX4 instance
will find *any* Gazebo reachable on the network and spawn its model into that world. This was
not theoretical: during this spike a test container attached to a simulation already running on
the host and spawned `x500_gimbal_0` into it.

In a classroom this means thirty students sharing one simulation. `run.sh` therefore uses bridge
networking, a per-user `GZ_PARTITION`, a per-user `ROS_DOMAIN_ID`, and
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

**9. QoS, exactly as predicted.** `ros2 topic echo /mavros/state` shows nothing until you pass
`--qos-reliability best_effort`. This is the trap flagged as the day-3 lecture's "#1 student
trap", and it is worth demonstrating live rather than describing.

## Not yet done

- TF tree for the gimbal (`base_link → gimbal_yaw → gimbal_pitch → camera_optical`) — needs the
  `ros_gz` bridge plus `robot_state_publisher` from a xacro. This is the next piece of work and
  Lab 4 depends on it.
- Bridging the camera into ROS, and YOLO11n against that stream.
- The driving ground-target model and the scoring node.

---

# Step 2 — camera and TF bringup

`ros2 launch drone_course_sim sim_bringup.launch.py` starts MAVROS, the gz parameter bridge,
the image bridge and the gimbal TF node against a running `course sim`.

## Working

- **Camera in ROS**: `/camera/image_raw` at ~25 Hz (1280x720, 2.0 rad HFOV), plus
  `/camera/camera_info`. Images go through `ros_gz_image`'s dedicated bridge rather than the
  parameter bridge, which avoids a serialise/deserialise round trip per frame.
- **Gimbal feedback path confirmed**: `/mavros/gimbal_control/device/attitude_status` publishes a
  quaternion at `frame_id: base_link_frd`, and `/mavros/gimbal_control/manager/pitchyaw` commands
  the gimbal. This is the same shape of interface the real A8 mini exposes, which is what lets a
  student's node run unchanged on hardware.
- **TF chain publishes**: `base_link -> camera_link -> camera_optical_frame`, built from the
  gimbal's own attitude report rather than from Gazebo ground truth — deliberately, since ground
  truth does not exist on the real aircraft.

## Not yet correct — must be fixed before Lab 4 is written

**The optical frame orientation is wrong.** `tf2_echo base_link camera_optical_frame` currently
gives RPY [90, 0, 0] deg, whose rotation matrix puts optical +z (the view direction) along
base_link -y — i.e. the camera "looks" to the right instead of forward.

The cause is a chain of rotations that cannot safely be derived on paper:

- PX4's `x500_gimbal` merges the gimbal model at `pose 0 0 0.26 0 0 3.14`, so there is a 180 deg
  yaw built into the mount;
- the camera *sensor* inside `gimbal/model.sdf` carries its own `yaw 3.14`;
- MAVROS reports the gimbal attitude in FRD while TF is FLU;
- and at rest the reported quaternion is (0, 0, -0.707, -0.707), i.e. 90 deg about the FRD down
  axis, which is not obviously the identity-looking-forward pose one would assume.

**This must be calibrated empirically, not derived.** The plan: command the gimbal to a known
pitch/yaw, compare the TF-derived camera orientation against Gazebo's true `camera_link` pose,
and solve for the constant offset. Getting this wrong is invisible until geolocation results are
quietly biased, which makes it exactly the kind of thing to nail down before any student sees it.

**Ground truth is not usable yet either.** Bridging `gz.msgs.Pose_V` to `tf2_msgs/TFMessage`
produces messages with empty `frame_id` and `child_frame_id`, so the link names are lost. The
scoring script needs another route — most likely a small node reading `/world/default/pose/info`
directly over gz-transport.

## Next

1. Calibrate the camera optical frame against Gazebo ground truth, with a regression check.
2. A ground-truth node that preserves link names, for scoring.
3. A high-rate gimbal command interface matching the SIYI driver's topic API (MAVROS exposes
   `pitchyaw` as a *service*, which is unsuitable for a 30 Hz tracking loop — the same problem
   the real driver had to solve).
4. YOLO11n node against `/camera/image_raw`.

---

# Step 3 — course gimbal model, and a verified camera frame

**The camera optical frame is now correct and asserted by a test.**

```
gimbal joints: {vertical_arm: 0.0, horizontal_arm: 0.0, camera: 0.0}
translation: (+0.0000, +0.0000, +0.0200)
  ok   optical x (image right)  = (-0.000, -1.000, -0.000)   aircraft right
  ok   optical y (image down)   = (-0.000, +0.000, -1.000)   aircraft down
  ok   optical z (view dir)     = (+1.000, -0.000, -0.000)   aircraft forward
```

## What was actually wrong

**The mount offset was 0.26 m; it should be 0.02 m.** PX4 merges the gimbal at z = 0.26 in the
*model* frame, but `base_link` itself sits at z = 0.24 there — so relative to `base_link` the
gimbal centre is 0.02 m up, not 0.26 m. Reading it off the SDF gives the wrong answer; reading it
out of a running simulation gives the right one. A 24 cm lever-arm error is not catastrophic at
30 m range, but it is exactly the kind of quiet bias that makes a geolocation lab unteachable.

**The rotation composition was never derived, it was verified.** Rather than reason about three
chained 180 degree rotations on paper, the link poses were dumped from a live simulation
(`/world/default/dynamic_pose/info`), which showed every gimbal link origin coinciding at the
rotation centre with a uniform 180 degree yaw. The URDF encodes exactly that, and
`verify_camera_frame.py` asserts the resulting axes.

## Course model

`models/x500_course_gimbal` (see `models/README.md`):

- **FOV 2.0 rad -> 1.414 rad.** The stock lens is a 114.6 degree fisheye; the A8 mini is ~81. The
  whole altitude/look-angle/detection-range discussion is meaningless with the wrong optics.
- **Joint state feedback added**, so TF is built from measured angles and can be checked
  against truth.
- **`3.14` replaced with pi.** The stock model's literal `3.14` is a built-in 0.09 degree error,
  which is indefensible in a course that teaches an angular error budget.
- **Kinematic conventions deliberately unchanged**, because PX4's gimbal control depends on the
  joint signs. The frames are handled explicitly in the URDF instead.

Installed into the PX4 tree at image build time with airframe `4090_gz_x500_course_gimbal`, so
`PX4_SIM_MODEL` selects it with no PX4 source changes.

## More gotchas

**10. `robot_description` must be wrapped.** A `Command(["xacro", ...])` substitution passed
straight to `robot_state_publisher` makes launch try to parse the URDF as YAML and abort the
*entire* launch file. It needs `ParameterValue(..., value_type=str)`.

**11. Editing SDF with string splices is dangerous.** An `rindex("</model>")` splice silently ate
the closing `</sdf>`, and the failure surfaced much later as an opaque Gazebo parse error. The
build now validates both model SDFs as XML, alongside the numpy/cv_bridge guard.

**12. Image size, corrected.** Earlier notes said 3.42 GB. That is the *content* size; **disk
usage is 12.9 GB**. Students need ~13 GB free.

## Open issue: commanding the gimbal from ROS

`/mavros/gimbal_control/manager/pitchyaw` returns `success=False, result=2` (DENIED), with or
without acquiring control via `manager/configure` (which itself succeeds), and with either
`gimbal_device_id` 0 or 1. Manager status reports `gimbal_device_id: 1`, `sysid_primary: 1`,
`compid_primary: 1`.

Publishing straight to the Gazebo joint command topics does not work either: PX4 republishes the
commanded angles continuously and immediately overrides them.

So the gimbal currently cannot be pointed from ROS. This does not affect the TF tree, which is
verified independently, but it **blocks the Lab 4 tracking loop** and is the next thing to fix.
Note also that `pitchyaw` is a *service*: even once it works it is unsuitable for a 30 Hz
tracking loop, so the course needs a high-rate topic interface over it — the same problem the
real SIYI driver had to solve.


---

# Step 4 — the full TF chain

`map -> base_link -> ... -> camera_optical_frame` now resolves, and `course verify` asserts the
whole chain rather than just the gimbal half. Geolocation is unblocked.

**13. MAVROS plugin sub-nodes never see the parameters file.** MAVROS's `local_position` plugin
can publish `map -> base_link` itself via `tf.send`, but its plugins run as sub-nodes
(`/mavros/local_position`) that are constructed without the params file. Measured directly:
`use_sim_time` is True on `/mavros` and False on `/mavros/local_position`, and `tf.send` stays
False whatever the YAML says — it can only be changed with `ros2 param set` at runtime.

Two consequences. `px4.launch` also *hardcodes* its config path, so the config has to be vendored
and `node.launch` driven directly. And even then the plugin parameters do not apply, so the
transform is published by `vehicle_tf_node` instead. `tf.send` is left off deliberately, so
MAVROS cannot publish a competing copy.

**14. A verification that does not retry is a broken verification.** The first full-chain check
failed with "not part of the same tree" while *both halves resolved perfectly* when tested by
hand. The transforms were fine; the check looked up `map -> camera_optical_frame` exactly once,
before `map -> base_link` had arrived. Worth remembering before trusting a negative result: two
of the three "failures" in this step were the test, not the system.


---

# Step 5 — the detector

YOLO11n runs on the camera stream at 22-26 ms/frame on CPU. `/detections` carries
`vision_msgs/Detection2DArray` stamped with the image's own time and frame.

**15. Gazebo stamped the camera with the wrong frame.** The sensor's `gz_frame_id` was
`camera_link`, but ROS convention is that images and `camera_info` carry the *optical* frame, and
projection code reads `header.frame_id` directly. On this gimbal `camera_link`'s +x points
backwards, so every student geolocation would have been quietly rotated -- with nothing failing
and nothing to see. Now set to `camera_optical_frame` in the model.

**16. XML comments cannot contain `--`.** A comment written with an em-dash-style `--` broke the
model SDF, and Gazebo reports it only as a generic parse error at the wrong line. The build-time
XML guard caught it immediately, which is precisely why that guard exists.

**17. PX4 make targets are generated from airframes MULTIPLIED BY worlds.** Moving the sim assets
to install after the PX4 build (so model edits stop costing a five minute rebuild) also moved the
world -- and cmake then globbed no `course_world.sdf`, so the target vanished and `course sim`
died with `ninja: unknown target gz_x500_course_gimbal_course_world`. The airframe **and** the
world must exist before the build; only the models can come after.


---

# Step 6 — the ground target

`course test` runs 8 end-to-end checks against a live sim and all pass, including the one that
matters: the detector really finds the target vehicle (272 hits over 20 s, best confidence 0.60).

**18. A target driving in a straight line drives to infinity.** Tier 1 was "straight at 3 m/s",
which is pedagogically right for the feedforward lesson and practically useless: the truck was at
x = -167 m within a couple of minutes, far outside detection range, and every detection check
failed for reasons that had nothing to do with detection. All routes are now bounded loops. It
also drove *away* from the drone, because it spawns facing it and `linear.x` is body-frame
forward.

**19a. A test that moves the world must put it back.** The scenario check drives the target for
six seconds to prove `cmd_vel` works, which moves it 17 m — out of the frame of an aircraft
sitting on the ground, so the blackout-recovery check that follows found nothing and failed for
reasons that had nothing to do with blackouts. It now drives the target back before continuing.

**19. A test that depends on a route is not a test.** The scenario check now runs with `tier:=-1`
(route idle) and commands the target itself, so "does the target move" is deterministic rather
than a question of when the test happened to start relative to the route.

**20. The self-containment and XML guards keep paying for themselves.** Adding a fifth model cost
nothing to validate: both guards were already wired to fail the build, and the new model was
covered by extending two lists.

---

# Step 7 — making the capstone actually fly

The reference solution was written, flown against ground truth and scored. The first honest score
was **13.3 / 85**; it is now **66–80 / 85** depending on tier. Nothing in between was a typo.
Every one of these was invisible from reading the code and obvious from one measurement, which is
the whole argument for having a scoring script before having an opinion.

**21. PX4 will not arm in headless SITL, and will not say why.** Every arming request came back
`Arming denied: Resolve system health failures first`, which names no cause. Two separate
requirements were failing:

* no RC transmitter — PX4 reports the missing receiver as a health failure. `COM_RC_IN_MODE = 4`
  ("manual control disabled") clears it.
* no ground station — `rcAndDataLinkCheck.cpp` makes a live GCS connection an *arming
  requirement* whenever `NAV_DLL_ACT` is non-zero, and the PX4 SITL default is 2. MAVROS on the
  onboard link does not count as a GCS. `NAV_DLL_ACT = 0` clears it.

Both are now defaults in the course airframe, both are clearly marked simulation-only, and both
are Day 2 teaching material: the reason they had to be set is precisely what the failsafe lecture
is about. Finding this took an hour, most of it spent reading PX4 source to discover that the
message is generic by design.

**22. `imgsz=640` is the wrong default and it fails silently.** The detector's input size, not
the camera resolution, decides whether anything is found. A target 64 px tall in a 1280-wide
frame letterboxes down to ~32 px — exactly YOLO's recall cliff — and the model finds **no
vehicle at all**, confidently reporting an *aeroplane* at 0.73 instead. Measured on a real course
frame:

| imgsz | inference | best vehicle detection |
|---|---|---|
| 640 | 25 ms | nothing |
| 800 | 34 ms | truck 0.28 |
| **960** | **42 ms** | **truck 0.58** |
| 1120 | 59 ms | truck 0.47 |
| 1280 | 66 ms | truck 0.38 |

The build now fails if the detector cannot find a vehicle in a **frame taken from the air at the
course geometry**, not merely in a stock sample image. A model that detects buses on a street and
nothing from a drone passes the old guard and breaks the lab.

**23. Detection confidence collapses toward nadir.** Measured at 960 px, aircraft at 12 m:

| look-down | 30° | 40° | 45° | 50° | 55° | 60° | 70° |
|---|---|---|---|---|---|---|---|
| truck confidence | 0.49 | 0.51 | 0.40 | 0.27 | 0.13 | none | none |

COCO contains almost no vehicles photographed from above. **The detector chooses the flight
geometry**, which is why the capstone stands off 18 m at 12 m altitude (34°) rather than the 45°
the error model alone would prefer. The shallower look costs accuracy — 1/sin θ is 1.80 rather
than 1.41 — and that is the right trade, because an accurate fix you never get is worth nothing.

**24. Guessing a sign convention costs more than measuring it.** `yaw_sign` in the gimbal pointer
was a guess and it was backwards, so the pointer drove the target *out* of frame and FOLLOW
lasted nine seconds. One measurement settled it: command +0.25 rad/s for 2 s, the gimbal moves
+23.5°, and the image content moves **353 px left**. Positive yaw pulls the image left, so a
target right of centre needs positive yaw. Two minutes of experiment, and the answer is now
written down next to the parameter.

**25. A pointer that only reacts to pixels never looks back.** Lose one frame and the gimbal
holds its last angle forever, so a momentary dropout becomes a permanently lost target. The fix
is the best argument for the Kalman filter in the whole course: project the filter's *prediction*
into the optical frame and it yields exactly the quantity the pixels would have —
`atan2(x, z)` and `atan2(y, z)` are the pixel errors divided by the focal length — so it feeds
the same PID with the same signs, and nothing downstream knows which source it came from.

**26. The velocity field of `PositionTarget` is a feedforward, not a control law.** Sending
`v_T + Kp·e` there doubles the loop gain, because PX4 computes its own proportional term from the
position setpoint you are already sending, with `MPC_XY_P` (0.95 by default). Measured: the
aircraft ran 5 m past a stationary target, put it behind the camera, and lost it. The field
carries `v_T` and nothing else. The lag experiment is unaffected and still honest — the
steady-state lag without feedforward is `v_T / MPC_XY_P`.

**27. "Hold overhead" is the wrong fallback for a stopped target.** It throws away the look angle
the entire error model is built on, and it puts the camera at nadir where the detector sees
nothing: measured, detections went to zero the moment the aircraft arrived over a stationary
truck in plain sight. Hold the *current bearing* at the standoff distance instead — which is what
the course document said all along.

**28. The target was running the drone over, and the obvious fix broke something else.** Spawned
15 m ahead facing the aircraft, and driven forward in its own body frame, the truck drove
straight through the spawn point and shoved the drone 13 m across the field before it could take
off. Tiers 2 and 3 then scored as if the follower had failed, when in fact it had been hit by a
truck. **Two bugs that produce the same symptom will be diagnosed as one**, and this one hid
behind the detector problem for three test runs.

Turning it to face *away* fixed the collision and broke the thirty-second ground check: from the
ground the drone then sees only the truck's rear, a dark open bed at a grazing angle, and the
detector finds nothing. `course test` caught it immediately, which is exactly what that check is
for. Broadside (yaw 90°) satisfies both — the target drives across the aircraft's nose rather
than at it, and broadside is the easiest aspect for the detector: confidence went from 0.60 to
**0.81**.

That aspect effect is worth carrying into the course, because it has a consequence nobody
expects: a follower sits *behind* its target, so it spends its whole flight looking at the rear
of a vehicle, which is the aspect the detector is worst at. **Aspect matters as much as angle,
and the guidance law knows about neither.**

**29. EKF2's yaw is 5–6° off truth, and it is not ours.** Chased properly before being accepted:
5.0° with PX4's stock magnetic field, 6.2° with the field rotated to the declination PX4's own
geo lookup returns, and **5.9° with stock PX4, the stock x500 and the stock world** — no course
assets involved. It is the largest term in the geolocation error budget, because a yaw error
rotates the bearing ray about the vertical: about 0.1 × ground range of lateral error, 1.6 m at
the capstone geometry, more than the gimbal, the pixel and the box centre combined. It is wired
into the course rather than hidden: the scoring thresholds allow for it and the Day 3 error
budget names attitude as the dominant term, which is also true on the real aircraft.

**30. A detection box is centred on the vehicle, not on its footprint.** The box centre sits
about half the vehicle's height above the ground it stands on, so a ray aimed at it
systematically over-ranges. At 15 m and 45° that was 0.6 m of a 1.2 m total error, and it does
not average out. Intersect the plane at `z = ground + target_height/2` instead: one line.

**31. Measure the model before trusting it.** The vendored pickup mesh has its length along its
own **+Y** in inches, so dropped in unrotated it sat broadside to the direction
`VelocityControl` drives it — the target drove sideways across the world, heading permanently
90° from its velocity. Since the whole guidance law is built on the target's direction of travel,
that is not cosmetic. The collision box said 5.4 m along X and the mesh said 5.66 m along Y, and
neither complained about the other.

**32. Do not "fix" something you have not reproduced.** The camera appeared to be staring at the
aircraft's own landing leg, so the gimbal was moved forward — which put the lens *inside* the
airframe and made every frame a close-up blur. A proper measurement afterwards (hover, sweep the
pitch from 30° to 70°, count dark pixels) showed **0–1.7% obstruction at every angle**: the
original mount was fine, and the blocked frame had been captured during a transient bank. The
move was reverted. The build-time check that the URDF and SDF mount offsets agree was kept,
because that one is worth having either way.
