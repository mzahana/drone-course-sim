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
