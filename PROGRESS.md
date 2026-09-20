# Progress & session handoff

**Read this first.** It is written so a new session with no prior context can pick the work up
without re-deriving anything. Newest work at the top of "Done". Every gotcha and its cost is in
[`docs/spike-notes.md`](docs/spike-notes.md) — read that before debugging anything that smells
environmental.

---

## 1. What this project is

A practical drone course for a fellowship cohort of selected senior undergraduates:
**2 days/week for 2 weeks, 3 hours per day — 12 contact hours total.**

The course is taught as three layers: hardware assembly, the PX4 autopilot, and the application
layer on an onboard computer. Heavy on simulation and live coding, with a hardware session on a
real X500 V2 + SIYI A8 mini + Raspberry Pi 5 / Jetson Orin Nano.

**Capstone:** the drone follows a moving *ground* target — detect it with YOLO11n, locate it on
the ground plane, estimate where it is going, and fly a standoff above and behind it. The same
code runs in SITL and on the real aircraft. (An earlier air-to-air leader-follower version was
deliberately dropped: two aircraft under student code is not a risk worth taking, and the
architecture is identical anyway.)

## 2. Where everything lives

| What | Where |
|---|---|
| **Simulation environment (this repo)** | `~/src/drone-course-sim` → `git@github.com:mzahana/drone-course-sim.git` |
| **Course design document** | `~/src/drone_courses/COURSE_PROPOSAL.md` (not in git) |
| Source material (past courses) | `~/src/drone_courses/*.pptx`, `*.docx`, `codes-*.zip` |
| Throwaway PX4 clone used for inspecting airframes/models | `~/src/drone_courses/spike/PX4-Autopilot` (1.8 GB, deletable) |

The course design document holds the session plan, the theory broken into six teachable steps,
the given-vs-written split, the scoring rubric, and the mini-project catalogue. **Slides do not
exist yet** — deliberately, because they are downstream of facts only a working stack can
establish.

Reusable packages that exist elsewhere on this machine and are meant to be pulled in later:
`mav_navigator_ros` (flight state machine + trajectory planner), `mav_controllers_ros`
(geometric controller), `multi_target_kf`, `geo_tuner`, `yolov8_ros`, `siyi_ros2` + `siyi_msgs`.
Most are ROS 2 Humble and need a Jazzy port. They live under
`~/d2dtracker_cuda_shared_volume/ros2_ws/src` and `~/ros2_ws/src`.

## 3. Environment facts worth knowing before you start

- The host is Ubuntu 24.04 with **ROS 2 Jazzy and Gazebo Harmonic 8.11 already installed**, and
  an RTX 5090.
- **There is no passwordless sudo.** You cannot `apt install` on the host. Docker works without
  sudo, so all work happens in the image.
- The host runs other simulations (a `d2dtracker_cuda` container with its own Gazebo).
  **Never run the course container with `--network host`** — see the isolation note below.

## 4. How to run it

```bash
cd ~/src/drone-course-sim
docker build -t drone-course-sim:jazzy -f docker/Dockerfile .   # context is the REPO ROOT
./run.sh                                                        # or see the test recipe below
```

Inside the container:

```bash
course doctor     # verifies everything; start here if anything looks wrong
course sim        # PX4 SITL + Gazebo with the course aircraft
course bringup    # MAVROS + gz bridges + TF tree   (second terminal)
course verify     # asserts the camera optical frame is correct
```

Test recipe used throughout development (headless, isolated, no GUI needed):

```bash
docker run -d --name dc-test -e GZ_PARTITION=course_test -e ROS_DOMAIN_ID=47 \
    drone-course-sim:jazzy sleep infinity
docker exec -d dc-test bash -c 'HEADLESS=1 course sim > /tmp/sitl.log 2>&1 < /dev/null'
# wait ~60 s
docker exec -d dc-test bash -c 'source /opt/ros/jazzy/setup.bash; \
    source /opt/course_ws/install/setup.bash; \
    ros2 launch drone_course_sim sim_bringup.launch.py > /tmp/bringup.log 2>&1'
# wait ~45 s, then
docker exec dc-test bash -c 'source /opt/ros/jazzy/setup.bash; \
    source /opt/course_ws/install/setup.bash; course verify'
```

**Build takes ~10 min** from cold on the PX4 layers. Image is ~3.4 GB to download, **~13 GB on
disk**. Iterating on anything that touches `models/`, `px4/` or the PX4 clone re-triggers the PX4
build (~5 min); touching only `ros2_ws/` or `docker/scripts/` is seconds.

## 5. Status

| Piece | Status |
|---|---|
| ROS 2 Jazzy + Gazebo Harmonic + PX4 v1.17 + MAVROS in one image | **done**, validated |
| Self-contained sim assets (airframe, gimbal, world) | **done**, asserted at build time |
| Camera into ROS (`/camera/image_raw`, `camera_info`) | **done**, ~25 Hz |
| TF `map -> base_link -> camera_optical_frame` | **done**, full chain asserted by `course verify` |
| Gimbal control from ROS topics | **done** — angle, rate, watchdog |
| Ground truth for scoring | **blocked** — bridge drops link names |
| YOLO11n detector on the camera stream | **done**, 22-26 ms/frame on CPU |
| Moving ground target + scenario tiers | not started |
| Scoring script + detector-blackout injector | not started |
| Lab skeletons and solutions | not started |
| Beamer slides | not started |

## 6. Done, with the reasoning that is expensive to re-derive

### Gimbal ROS command interface

| Topic | Type | Use |
|---|---|---|
| `/gimbal/cmd/angle` | `Vector3Stamped` (rad) | absolute — "look there" |
| `/gimbal/cmd/rate` | `Vector3Stamped` (rad/s) | closed-loop tracking |
| `/gimbal/attitude` | `Vector3Stamped` (rad) | measured, from the joints |
| `/gimbal/saturated` | `Vector3Stamped` | per-axis limit flag |

`Vector3` is always `x=roll, y=pitch, z=yaw`. Roll is not commandable, matching the A8 mini.

**Why not MAVROS, and do not "fix" this later.** On the real aircraft the A8 mini hangs off the
*onboard computer* over Ethernet — PX4 never sees the commands — so driving it from ROS is the
faithful arrangement, not a shortcut. Separately, MAVROS exposes gimbal control as *services*,
which block the executor and are unusable in a 30 Hz pixel-error loop.

PX4 does not fight this: `GZGimbal::Run()` publishes joint commands only inside
`if (pollSetpoint())`, i.e. only when a new setpoint arrives. It does **not** stream.

### Self-contained simulation assets

`models/` holds `course_x500_base`, `course_x500`, `course_gimbal`, `x500_course_gimbal`;
`worlds/` holds `course_world.sdf`. Nothing is inherited from whatever PX4 ships. The build
**asserts** it — any `model://` reference that is not a `course_*` asset fails the build.

PX4 spawns models from `PX4_GZ_MODELS` and globs `PX4_GZ_WORLDS` to generate make targets, and
`gz_env.sh` overwrites both at run time — which is why the assets are installed *into* the PX4
tree at image build rather than pointed at from outside. Airframe `4090_gz_x500_course_gimbal`
gives the make target `gz_x500_course_gimbal_course_world`. No PX4 source changes are needed.

### Model fixes (see `models/README.md` for the full rationale)

- FOV 2.0 → 1.414 rad. The stock lens is a 114.6° fisheye; the A8 mini is ~81°.
- Joint controllers can now integrate: `i_max`/`i_min` were pinned to `0`, making `i_gain` dead
  code and leaving a 2.4° standing pointing error — 1.2 m on the ground at 28 m slant range.
- Literal `3.14` replaced with pi (a built-in 0.09° error).
- **Kinematic conventions deliberately unchanged**: PX4's gimbal control depends on the joint
  signs, so flipping the mount silently inverts commanded yaw and roll. Frames are handled
  explicitly in the URDF instead.

### YOLO11n detector

Course-provided infrastructure — students consume `/detections`, they do not write the detector.

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | in |
| `/detections` | `vision_msgs/Detection2DArray` | out |
| `/detector/image_annotated` | `sensor_msgs/Image` | out, optional |
| `/detector/blackout` | `std_msgs/Float64` | in — seconds to go blind |

22-26 ms per frame for YOLO11n at 640 on CPU, throttled to 10 Hz. It **drops** frames rather
than queueing them: a backlog produces detections describing where the target used to be.
Detections carry the image's stamp and frame, never the clock's.

`/detector/blackout` is the robustness-lab tool — publish a duration and the detector silently
stops reporting (still publishing empty messages) so students can watch their tracker coast on
prediction, or fail to.

The image, `camera_info` and detections all carry **`camera_optical_frame`**. This was wrong at
first: Gazebo stamped them `camera_link`, whose +x points *backwards* on this gimbal, and student
projection code reads `header.frame_id` directly — every geolocation would have been quietly
rotated. Fixed in the model's `gz_frame_id`.

The build runs a **detector self-test** against a baked sample image and fails if the model finds
fewer than 3 objects. A weights file that loads but returns nothing is otherwise a silent failure
that surfaces mid-lab.

### `map -> base_link`, and why MAVROS does not publish it

`vehicle_tf_node` publishes it from `/mavros/local_position/pose`. MAVROS *can* publish this
itself via `local_position`'s `tf.send`, and that is the obvious thing to reach for — but its
plugins run as **sub-nodes that never receive the parameters file**. `use_sim_time` included.
Confirmed directly: `use_sim_time` reads True on `/mavros` and False on
`/mavros/local_position`, and `tf.send` stays False no matter what the YAML says (it can only
be set at runtime with `ros2 param set`).

So `tf.send` is deliberately left **off** in `config/mavros_px4.yaml`, and the transform is
published by a course node that runs on the same clock as everything else. This is also the
honest arrangement: on the real aircraft this transform is an EKF2 *estimate* arriving over
MAVLink, and `/mavros/local_position/pose` is exactly what a student's node consumes there too.

### Verified camera frame

`course verify` asserts optical z = forward, x = right, y = down. The two original bugs were a
mount offset of 0.26 m that should have been 0.02 m, and a wrong rotation composition. Both were
fixed by **reading link poses out of a running simulation** rather than deriving three chained
180° rotations on paper. Do the same if it ever breaks again.

### Network isolation — do not undo this

gz-transport discovers peers by multicast and is **not** scoped by `ROS_DOMAIN_ID`. With host
networking and no `GZ_PARTITION`, a PX4 instance finds *any* Gazebo on the network and spawns its
model into that world. This actually happened during development: a test container attached to
the host's running `d2dtracker_cuda` simulation. In a classroom it means thirty students sharing
one simulation. `run.sh` therefore uses bridge networking, a per-user `GZ_PARTITION`, a per-user
`ROS_DOMAIN_ID`, and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

## 7. Next, in order

1. **The scenario**: a driving ground target on a scripted route, with the difficulty tiers from
   the course document (stationary -> straight -> turning -> with a detector blackout). Until a
   target model exists the detector has nothing to find, so this also completes the first
   genuine end-to-end test of detect -> locate.
2. **Ground truth for scoring.** Bridging `gz.msgs.Pose_V` → `tf2_msgs/TFMessage` produces empty
   `frame_id`/`child_frame_id`, so link names are lost. Needs a small node reading
   `/world/course_world/pose/info` over gz-transport directly.
4. **The scenario**: a driving ground target on a scripted route, with the difficulty tiers from
   the course document (stationary → straight → turning → with a detector blackout).
5. **Scoring script** reading ground truth, plus the blackout injector.
6. **Then, and only then, slides.** They depend on numbers this stack produces.

## 8. Open issues

- Ground truth link names lost in the bridge (blocks scoring).
- `/mavros/gimbal_control/manager/pitchyaw` returns DENIED regardless of control acquisition.
  Worked around by not using it; documented in the spike notes, not fixed.
- Gimbal residual pointing error 0.3° single-axis, 1.1–1.4° on a combined move. Acceptable for
  now; the joint gains are where to look if tracking needs tighter pointing.

## 9. Conventions

- Course-provided ROS code is pre-built into the image at `/opt/course_ws`. **Student** code goes
  in the shared volume at `~/shared_volume/ros2_ws/src` (host:
  `~/drone_course_shared_volume/ros2_ws/src`).
- Anything that would break a lab is guarded **at image build time**, so it fails the build
  rather than the class. Existing guards: numpy stays 1.x, `cv_bridge` imports, model SDFs are
  valid XML, models are self-contained.
- Every non-obvious decision gets its reasoning written down next to it, because the cost here is
  rediscovery, not typing.
