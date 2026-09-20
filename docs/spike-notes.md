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
