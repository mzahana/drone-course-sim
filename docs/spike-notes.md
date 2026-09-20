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
