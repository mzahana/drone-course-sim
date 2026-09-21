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
the given-vs-written split, the scoring rubric, and the mini-project catalogue. The **slides**
live at `~/src/drone_courses/slides` — a Beamer theme, a shared TikZ figure library and one deck
per day; `make` builds all four. Read `slides/STYLE.md` before editing any of them.

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
course doctor        # verifies everything; start here if anything looks wrong
course sim           # PX4 SITL + Gazebo with the course aircraft
course bringup       # MAVROS + gz bridges + TF tree   (second terminal)
course verify        # asserts the camera optical frame is correct
course test          # end-to-end scenario check
course new lab4      # scaffold a lab skeleton into the shared volume
course solution 4    # reveal the reference solution
course score         # grade a running mission against ground truth
course qgc           # QGroundControl
course desktop       # browser desktop, for machines with no working X11
course rviz          # RViz with the course layout
course log           # copy the newest PX4 log into the shared volume
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

**Build takes ~10 min** from cold on the PX4 layers. Image is ~3.7 GB to download, **~14 GB on
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
| Ground truth for scoring | **done** — `/target/ground_truth`, `/drone/ground_truth` |
| YOLO11n detector on the camera stream | **done**, 42 ms/frame on CPU at 960 px |
| Moving ground target + scenario tiers | **done**, 8/8 end-to-end checks pass |
| Detector blackout injector | **done** |
| Reference solutions for every lab and the capstone | **done**, flown and scored |
| Scoring script | **done**, `course score`, validated at tiers 0-3 |
| Lab skeletons and solutions | **done**, generated from the solutions |
| QGroundControl in the image | **done**, v4.4.4, extracted not run |
| Browser desktop fallback (noVNC) | **done**, `course desktop` |
| Beamer slides | **done**, 4 decks, 279 slides |

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

### Ground target and scenario tiers

`course_target_vehicle` drives a route set by `target_route_node`'s `tier` parameter:

| Tier | Route | What it tests |
|---|---|---|
| -1 | idle, publishes nothing | lets a test or a person own `/target/cmd_vel` |
| 0 | stationary | does the loop close at all |
| 1 | out and back, long straights | steady-state lag -- the feedforward lesson |
| 2 | circuit with corners and stops | gimbal saturation, staying in frame, and the degeneracy when a stopped target has no heading |
| 3 | tier 2 plus scheduled blackouts | prediction and graceful degradation |

**Every route is bounded.** The first version of tier 1 drove in a straight line forever: within
seconds the target was hundreds of metres away, far out of detection range, and the scenario was
neither testable nor teachable. Measured after the fix, the target circulates within about 35 m.

`/target/ground_truth` (`nav_msgs/Odometry`) is the scoring signal. Never for flight.

### `course test` -- the end-to-end check

`course test` asserts, against a live sim, the things a lab actually depends on:

```
  ok   map -> camera_optical_frame resolves
  ok   camera publishing                    [268 frames]
  ok   detector publishing                  [118 messages]
  ok   detector FINDS the target vehicle    [118 hits, best conf 0.81]
  ok   ground truth odometry available
  ok   target responds to cmd_vel           [moved 17.1 m in 6 s at 3 m/s]
  ok   blackout silences detection          [0 hits during blackout]
  ok   detection recovers after blackout    [13 hits after]
```

It has since earned its keep twice over. It caught the target-facing change that made the
detector blind from the ground, and it caught its own bug: driving the target for six seconds to
prove `cmd_vel` works moves it 17 m, out of frame, so the blackout-recovery check that followed
failed for reasons that had nothing to do with blackouts. It now puts the target back first.

Run it with `tier:=-1` so the test owns `/target/cmd_vel`. The detection check is the one that
matters: every other check can pass while the detector quietly sees nothing, and the first person
to notice would be a student in the middle of Lab 4.

### YOLO11n detector

Course-provided infrastructure — students consume `/detections`, they do not write the detector.

| Topic | Type | Direction |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | in |
| `/detections` | `vision_msgs/Detection2DArray` | out |
| `/detector/image_annotated` | `sensor_msgs/Image` | out, optional |
| `/detector/blackout` | `std_msgs/Float64` | in — seconds to go blind |

**42 ms per frame at `imgsz=960`, conf 0.20**, throttled to 10 Hz. Not 640: at 640 a target
64 px tall in a 1280-wide frame letterboxes down to ~32 px, YOLO finds no vehicle at all, and
reports an *aeroplane* at 0.73 instead. 640 costs 25 ms and is worthless from the air. It **drops** frames rather
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

### The capstone actually flies: what it took, and why each thing mattered

The reference solution was written, run against ground truth, and scored. The first honest
score was **13.3 / 85**. Getting to **67.6 / 85** took six separate defects, every one of which
was invisible from reading the code and obvious from one measurement. They are listed because
each is also a lesson the course now teaches.

| What was wrong | How it showed up | Fix |
|---|---|---|
| Detector ran at `imgsz=640` | A target 64 px tall in a 1280-wide frame is 32 px after letterboxing, right on YOLO's recall cliff. It reported an **aeroplane** at 0.73 and no vehicle at all | `imgsz=960`: truck at 0.58 for 42 ms |
| Gimbal `yaw_sign` was a guess, and wrong | The pointer drove the target *out* of frame; FOLLOW lasted 9 s | Measured: +23.5° of yaw moves the image content 353 px left, so the sign is **+1** |
| Pointer only reacted to pixels | One lost frame became a lost target for good, because nothing ever looked back | It now falls back to the filter's prediction, projected into the optical frame, feeding the same PID |
| Guidance sent `Kp·e` as the velocity *feedforward* | PX4 adds its own position P term, so the gain was double: the aircraft ran 5 m past a stationary target | The feedforward carries `v_T` only. The proportional term is PX4's `MPC_XY_P` |
| Stationary-target fallback sat directly overhead | Nadir view; detections went to zero while hovering over a truck in plain sight | Hold the current bearing at the standoff distance, as the course document always said |
| Look angle was 45° and drifting steeper | See the table below | 12 m up, 18 m back: a 34° look-down |

**Detection confidence against look-down angle**, measured on this target at 960 px:

| look-down | 30° | 40° | 45° | 50° | 55° | 60° | 70° |
|---|---|---|---|---|---|---|---|
| truck confidence | 0.49 | 0.51 | 0.40 | 0.27 | 0.13 | none | none |

COCO contains almost no pictures of vehicles taken from above, so a near-nadir pickup is not
something YOLO was ever taught to recognise — past 60° it reports an aeroplane instead. **The
detector, not the geometry, chooses the look angle.** That is the course's "how well you see
determines how you are allowed to fly" lesson, with numbers, and it is why the standoff is 18 m
behind rather than 12.

Scores at the end, 180-second runs, reference solution:

| tier | on station | estimate RMS | kept in frame | automated total |
|---|---|---|---|---|
| 0 stationary | 100% | 1.8 m | 99% | 84.8 / 85 |
| 1 out and back | 45% | 1.9 m | 96% | 67.6 / 85 |
| 2 corners and stops | 71% | 1.5 m | 97% | 75.9 / 85 |
| 3 plus blackouts | 51% | 4.3 m | 85% | 55.7 / 85 |

The gradient is the right shape. Tier 0 is nearly full marks, as "does the loop close at all"
should be. Tier 1 scores *lower than tier 2*, which is not a mistake: its 180-degree U-turns
swing the standoff point through a wider arc, faster, than tier 2's 90-degree corners at half
speed. And tier 3 is the only one where the estimate degrades at all -- RMS 4.3 m against 1.5-1.9
elsewhere -- because a blackout is the one thing a filter cannot see through, only coast through.
Every tier is a pass, none is a walkover, and all four are beatable.

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

Everything on the original list is done. What is left is rehearsal, not construction:

1. **Publish the image to a registry** and set `IMAGE` in `install.sh`. Until then a student
   machine cannot `./install.sh` — it falls back to telling them to build locally, which takes
   half an hour.
2. **Record the demo videos.** Every `\dcvideoplaceholder` in the decks is a recording that has
   not been made. Never run a live demo without a recorded fallback.
3. **Hardware rehearsal** on the real X500 V2 + A8 mini + RPi 5 / Orin, and check the Day 1
   mass and thrust numbers against the actual airframe (see open issues).
4. **Dry run with two test students** on clean laptops, one Linux and one Windows/WSL2.

## 8. Open issues

- **EKF2 yaw sits 5-6° off simulator truth.** Not ours: reproduced with stock PX4, the stock
  x500 and the stock world. It is the largest single term in the geolocation error budget
  (a yaw error rotates the bearing ray about the vertical, so ~0.1 × ground range of lateral
  error). The scoring thresholds and the Day 3 error budget are both set with this in mind.
  Full reasoning is in `worlds/course_world.sdf` next to the magnetic field.
- The Day 1 deck's thrust-to-weight worked example lands at **1.79**, below the 2:1 rule the
  same slide teaches, for the X500 V2 with an A8 mini and an Orin. The deck says so plainly
  rather than hiding it, but the mass budget should be checked against the real airframe at the
  hardware rehearsal.
- `/mavros/gimbal_control/manager/pitchyaw` returns DENIED regardless of control acquisition.
  Worked around by not using it; documented in the spike notes, not fixed.
- Gimbal residual pointing error 0.3° single-axis, 1.1–1.4° on a combined move. Acceptable; the
  joint gains are where to look if tracking ever needs tighter pointing.
- The `Pickup` mesh is vendored from Gazebo Fuel (Nate Koenig / Open Robotics). Attribution is
  in `models/README.md`; its licence is not stated in the model metadata.

## 9. Conventions

- Course-provided ROS code is pre-built into the image at `/opt/course_ws`. **Student** code goes
  in the shared volume at `~/shared_volume/ros2_ws/src` (host:
  `~/drone_course_shared_volume/ros2_ws/src`).
- Anything that would break a lab is guarded **at image build time**, so it fails the build
  rather than the class. Existing guards: numpy stays 1.x, `cv_bridge` imports, model SDFs are
  valid XML, models are self-contained.
- Every non-obvious decision gets its reasoning written down next to it, because the cost here is
  rediscovery, not typing.
