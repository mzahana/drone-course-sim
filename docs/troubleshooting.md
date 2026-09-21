# Troubleshooting — indexed by the message you are looking at

**Audience:** students during a lab, and instructors triaging one.

This is organised by **the error text on your screen**, because that is what you will search for.
Use your browser's or editor's find. If you cannot find the message, jump to
[Nothing is broken but nothing works](#nothing-is-broken-but-nothing-works) — the worst failures
in this stack are silent.

Almost every entry here came from something that actually happened while the environment was
built; the full accounts are in [`spike-notes.md`](spike-notes.md).

**Before anything else:** run `course doctor`. It checks ROS, Gazebo, the PX4 build, the course
airframe and models, the world, the GeographicLib geoids, ultralytics, the YOLO weights, CUDA,
`DISPLAY` and your workspace — in about two seconds. Most of this page is only needed when
`doctor` is clean and something *else* is wrong.

---

## Contents

- [Docker and getting in](#docker-and-getting-in)
- [ROS not sourced](#ros-not-sourced)
- [Nothing is broken but nothing works](#nothing-is-broken-but-nothing-works)
- [QoS mismatch — the subscriber that never fires](#qos-mismatch--the-subscriber-that-never-fires)
- [`use_sim_time` — TF lookups return the wrong transform](#use_sim_time--tf-lookups-return-the-wrong-transform)
- [TF errors](#tf-errors)
- [Gazebo and `gz`](#gazebo-and-gz)
- [Somebody else's simulation](#somebody-elses-simulation)
- [No DISPLAY / no GUI](#no-display--no-gui)
- [MAVROS](#mavros)
- [OFFBOARD and arming](#offboard-and-arming)
- [Detector and perception](#detector-and-perception)
- [Gimbal](#gimbal)
- [Building your own package](#building-your-own-package)

---

## Docker and getting in

### `permission denied while trying to connect to the Docker daemon socket`

**What is wrong:** your user is not in the `docker` group, or you added yourself and never logged
out.

**Fix:**

```bash
sudo usermod -aG docker $USER
```

then log out and back in. On WSL2, run `wsl --shutdown` in PowerShell and reopen the terminal.
`newgrp docker` works for the current shell only and will bite you in the next terminal.

### `install.sh` says `Docker is installed but not usable by this user`

Same cause, same fix. `install.sh` checks `docker info` specifically so this is caught before the
3.4 GB pull, not after it.

### `no space left on device` during the pull

The image is about **3.4 GB to download and about 13 GB on disk**. Check with `df -h ~` and clear
space; `docker system prune -a` reclaims old images if you have used Docker before.

### `./run.sh` drops me into a *new* container and my simulation is gone

`run.sh` re-enters the existing container if one is running or stopped under the name
`drone-course`. If you renamed it, or removed it with `docker rm`, you get a fresh one. Your code
survives regardless — it is in the shared volume on your host. **Anything outside
`~/shared_volume` inside the container does not survive.**

---

## ROS not sourced

### `course doctor` prints `FAIL  ROS not sourced`

### `bash: ros2: command not found`

### `Package 'drone_course_sim' not found`

### `ModuleNotFoundError: No module named 'rclpy'`

**What is wrong:** this shell has no ROS environment. It happens in one specific situation and it
is worth understanding, because it recurs: **`docker exec bash -c '...'` reads no init file at
all**, and Ubuntu's stock `.bashrc` *returns early for non-interactive shells*, so anything
appended after that guard never runs.

**Fix** — source the three layers by hand, in this order:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/course_ws/install/setup.bash          # course-provided packages
source ~/shared_volume/ros2_ws/install/setup.bash # your own, once you have built it
```

Interactive terminals started by `./run.sh` do this for you: the image **prepends** the sourcing
lines to `.bashrc` rather than appending them, precisely so they run before that early return.
The `course` script also sources ROS itself for the same reason.

**If you are writing a script**, note that ROS's `setup.bash` reads unbound variables, so
`set -u` must be switched off around the `source` line:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
```

### `AMENT_PREFIX_PATH` is set but your package still is not found

You built it but did not re-source. After every `colcon build`, source the workspace again in
**every** terminal that needs it. A terminal opened before the build has a stale environment.

---

## Nothing is broken but nothing works

Read this section when there are no error messages at all. In this stack, four failures are
completely silent, and between them they account for most lost lab time:

| Symptom | Section |
|---|---|
| `ros2 topic echo` prints nothing, your callback never runs | [QoS mismatch](#qos-mismatch--the-subscriber-that-never-fires) |
| Geolocation results are biased, nothing errors | [`use_sim_time`](#use_sim_time--tf-lookups-return-the-wrong-transform) |
| `gz topic -l` prints nothing | [Gazebo and `gz`](#gazebo-and-gz) |
| Your drone appears in someone else's world | [Somebody else's simulation](#somebody-elses-simulation) |

---

## QoS mismatch — the subscriber that never fires

### `ros2 topic echo /mavros/state` prints nothing, and `ros2 topic hz` says nothing is publishing

### Your Python subscriber's callback is never called, with no error anywhere

### `ros2 topic info -v` shows one publisher and one subscriber that are not connected

**What is wrong.** This is the single most common trap in the course, and it produces **no error
message by design**. ROS 2 will not connect a **best-effort publisher** to a **reliable
subscriber**: the subscriber is asking for a guarantee the publisher refuses to make, so the
match is silently rejected. `rclpy`'s default subscriber QoS is *reliable*, and most sensor and
autopilot streams publish *best effort*.

In this environment the best-effort publishers you will meet are the MAVROS state and position
topics, `/camera/image_raw`, `/camera/camera_info`, and the ground-truth odometry topics.

**Diagnose:**

```bash
ros2 topic info -v /mavros/state
```

Look at `Reliability` under the publisher, and compare it with your node's.

**Fix at the command line** — add the matching profile:

```bash
ros2 topic echo /mavros/state --qos-reliability best_effort
ros2 topic echo /camera/camera_info --qos-reliability best_effort --qos-durability volatile
```

**Fix in your node** — use the sensor-data profile for anything sensor-like:

```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(State, "/mavros/state", self.on_state,
                         qos_profile_sensor_data)
```

**Rule of thumb for this course:** subscribe to autopilot and sensor streams with
`qos_profile_sensor_data`; publish your own results (estimates, commands, mission state) with the
default reliable QoS and a depth of 10. If a topic you publish yourself is not arriving, the
mismatch is the other way round and the same `ros2 topic info -v` tells you.

**A reliable subscriber on a best-effort topic fails silently. A best-effort subscriber on a
reliable topic works.** When in doubt, be best-effort.

---

## `use_sim_time` — TF lookups return the wrong transform

### `Lookup would require extrapolation into the past` / `into the future`

### `Lookup would require extrapolation at time ..., but the latest data is at time ...` where the two times differ by hours

### Everything runs, nothing errors, but your target position is consistently displaced

**What is wrong.** In SITL, Gazebo publishes `/clock` and every node must use it. A node running
with `use_sim_time` **false** timestamps with wall-clock time, which has nothing to do with
simulation time. Then:

- if your node is on wall time and TF is on sim time, a lookup fails outright with an
  extrapolation error and times that look absurd (one is a Unix epoch, the other is seconds since
  the sim started);
- if *both* are on sim time but you look the transform up at `rclpy.time.Time()` — meaning
  "latest available" — instead of at the **image's own timestamp**, nothing errors and your
  answer is quietly wrong.

The second one is the expensive one, and it is a graded part of Lab 4. Detections carry the
image's stamp and frame, never the clock's. The gimbal reports at 50 Hz and the image arrives
50–150 ms later; during a 30 °/s slew, 100 ms of skew is 3° of pointing error, which at 30 m
slant range is **1.6 m on the ground.**

**Fix, three parts, all required:**

1. Launch your node with sim time on. Every node in `sim_bringup.launch.py` already declares
   `use_sim_time` (default `true`); yours must too:

   ```bash
   ros2 run my_pkg my_node --ros-args -p use_sim_time:=true
   ```

2. Look the transform up **at the stamp of the message you are processing**:

   ```python
   tf = self.buffer.lookup_transform(
       "map", det.header.frame_id, det.header.stamp,
       timeout=Duration(seconds=0.1))
   ```

3. Check it: `ros2 param get /your_node use_sim_time` must say `True`, and
   `ros2 topic hz /clock` must show the clock arriving.

**Known quirk, do not chase it:** MAVROS's plugins run as sub-nodes (`/mavros/local_position`)
that are constructed **without the parameters file**, so `use_sim_time` reads `True` on `/mavros`
and `False` on `/mavros/local_position`. This is why `map -> base_link` is published by the
course's own `vehicle_tf_node` and MAVROS's own `tf.send` is deliberately left off. Do not turn
`tf.send` on — you will get two competing copies of the same transform.

---

## TF errors

### `"map" passed to lookupTransform argument target_frame does not exist`

### `Could not find a connection between 'map' and 'camera_optical_frame' because they are not part of the same tree`

**What is wrong, most likely:** you asked too early. The chain is
`map -> base_link -> gimbal links -> camera_link -> camera_optical_frame`, and `map -> base_link`
only starts once MAVROS has a position estimate from PX4. A check that looks the full chain up
exactly once, at startup, will fail while both halves are perfectly fine — this happened during
development and two of the three "failures" in that step were the test, not the system.

**Fix:** retry with a timeout, and wait for the tree rather than asserting on it:

```python
self.buffer.lookup_transform("map", "camera_optical_frame", stamp,
                             timeout=Duration(seconds=2.0))
```

**Then check the tree is actually there:**

```bash
course verify                              # asserts the full chain and the optical axes
ros2 run tf2_tools view_frames             # writes frames.pdf
ros2 run tf2_ros tf2_echo map camera_optical_frame
```

**If `course bringup` is not running**, there is no TF tree at all. `course sim` alone gives you
PX4 and Gazebo and nothing on the ROS side.

### The geolocation is rotated by 90° or 180°

You are using `camera_link` where you should be using `camera_optical_frame`. They are not the
same frame: optical is **z forward, x right, y down**, and on this gimbal `camera_link`'s +x
points *backwards*. Read `header.frame_id` off the message and use that — the image,
`camera_info` and `/detections` all carry `camera_optical_frame`. `course verify` asserts the
axes, so run it before you go looking in your own maths.

---

## Gazebo and `gz`

### `gz topic -l` returns nothing at all, silently

**What is wrong:** almost certainly **not** a discovery failure, even though it looks exactly
like one. The `gz` CLI needs `GZ_CONFIG_PATH` to load its own command plugins; without it, it
loads no subcommands and prints nothing rather than complaining. This cost more debugging time
than anything else in the build.

**Fix:** source ROS in that shell (the vendored Gazebo's environment hooks set the variable):

```bash
source /opt/ros/jazzy/setup.bash
gz topic -l | head
```

If it is still empty *after* sourcing, then check the simulation is really up
(`ps aux | grep px4`) and that `GZ_PARTITION` matches — see the next section.

### `ninja: unknown target gz_x500_course_gimbal_course_world`

**What is wrong:** PX4 generates its make targets as **airframes × worlds**, at cmake time. If
the world file is missing when PX4 is configured, the target simply does not exist. This is a
build-order fault in the image, not something you can fix from inside a running container.

**Fix:** re-pull the image. If you built it yourself, the airframe **and** the world must be
installed into the PX4 tree *before* the PX4 build; only the models may be installed after.

### Gazebo starts but the world is empty / the aircraft never appears

Check `course sim`'s output for the PX4 spawn lines. If PX4 reported a successful spawn and you
see nothing, you are very likely looking at a different Gazebo — read the next section.

### `libgz-utils2.so.2: cannot open shared object file`

The `px4` binary cannot find the ROS-vendored Gazebo libraries because ROS is not on
`LD_LIBRARY_PATH` in this shell. Source ROS (`source /opt/ros/jazzy/setup.bash`) — it adds the 18
vendor directories the binary needs.

---

## Somebody else's simulation

### My aircraft spawned into another student's world

### There are two drones in my Gazebo and I only started one

### `ros2 topic list` shows topics from a simulation I did not start

**What is wrong:** gz-transport discovers peers by **multicast** and is **not** scoped by
`ROS_DOMAIN_ID`. With host networking and no partition, a PX4 instance will find *any* Gazebo
reachable on the network and spawn its model into that world. In a classroom that means thirty
students sharing one simulation — and it is not hypothetical: during development a test container
attached itself to a simulation already running on the host.

**Fix:** use `./run.sh` unmodified. It sets, per user:

```
--network bridge
GZ_PARTITION=course_<yourname>
ROS_DOMAIN_ID=<derived from your uid>
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

**Never set `USE_HOST_NETWORK=1`**, and never run the container with `--network host`. If someone
tells you host networking will fix your networking problem, it will not — it will create this one.

Check what you are on:

```bash
echo "$GZ_PARTITION  $ROS_DOMAIN_ID  $ROS_AUTOMATIC_DISCOVERY_RANGE"
```

All three terminals you use must show the same values. They will, if all three came from
`./run.sh`.

---

## No DISPLAY / no GUI

### `course doctor` prints `warn  no DISPLAY — GUI tools will not open`

### `qt.qpa.xcb: could not connect to display`

### `Authorization required, but no authorization protocol specified`

### `cannot open display: :0`

### `course qgc` or `course rviz` says `No DISPLAY. Run 'course desktop' first.`

**What is wrong:** the container has no X11 connection to a desktop. `run.sh` passes `DISPLAY`,
mounts `/tmp/.X11-unix` and runs `xhost +local:docker`, but that only works if there is a display
to pass in the first place.

**Fix — the one that always works.** Start the browser desktop inside the container:

```bash
course desktop
export DISPLAY=:99          # in THIS terminal; later terminals pick it up on their own
```

Then open **<http://localhost:6080/vnc.html>**. That is a full Linux desktop in a browser tab,
and Gazebo, RViz and QGroundControl all run on it. `run.sh` publishes the port on `127.0.0.1`
only, so nobody else on the network can reach it. Set `VNC_PORT` before `run.sh` if something
else already uses 6080 on your machine — note that changing it means recreating the container.

This is the supported path on Windows and macOS. It is not a degraded fallback; it is how the
GUI is expected to work on anything that is not a Linux desktop.

**Fix — native X11, if you are on Linux and would rather have real windows:**

1. Run `./run.sh` **from a graphical desktop session**, not from an SSH shell. Check on the host:
   `echo $DISPLAY` — if it is empty there, it will be empty in the container.
2. On the host, `xhost +local:docker`, then restart the container.
3. Inside the container, `echo $DISPLAY` should be non-empty.
4. **Over SSH:** `ssh -X` *and* re-run `./run.sh` in that session. Gazebo over X forwarding is
   slow; `course desktop` over the same SSH tunnel is usually better.

**And you can still work with no display at all.** Everything except the GUI applications runs
headless:

```bash
HEADLESS=1 course sim
```

`course bringup`, `course verify`, `course test`, `course score`, the detector and all student
nodes are headless already. This is exactly how the environment was validated. What you lose is
the ability to *watch*, which matters most in Lab 4 — so get the desktop working before Day 3.

### QGroundControl does not connect

`course qgc` starts QGroundControl v4.4.4, which is extracted into the image rather than run as
an AppImage (AppImages need FUSE, which containers do not have). It finds SITL by itself on
**UDP 14550** and needs no configuration. If it shows no vehicle:

1. Is `course sim` actually running, in another terminal?
2. Give it a few seconds — it polls.
3. Is it the same container? Both terminals must have come from `./run.sh`.

---

## MAVROS

### `Can't open geoid model ... egm96-5` / MAVROS exits immediately on start

### `terminate called after throwing an instance of 'GeographicLib::GeographicErr'`

**What is wrong:** MAVROS refuses to start without the **GeographicLib geoid dataset**. It is the
classic first-run failure on a hand-built system, which is why the dataset is installed at image
**build** time here.

**Fix:** if you see this, your image is broken or incomplete — `course doctor` checks for
`/usr/share/GeographicLib/geoids` explicitly and will already have told you. Re-pull the image.
Do not try to install the dataset by hand inside the container; that is a symptom of running the
wrong image.

### `FCU: DeviceError` / `connected: false` / MAVROS starts but sees no autopilot

**What is wrong:** nothing is listening on the other end of the UDP link.

**Fix, in order:**

1. Is PX4 running? `course sim` must be up **before** `course bringup`, in another terminal.
2. Is the URL right? The default is `udp://:14540@127.0.0.1:14557`, and PX4 SITL offers 14540.
3. Check:

   ```bash
   ros2 topic echo /mavros/state --qos-reliability best_effort
   ```

   A healthy link reports `connected: true` and a mode such as `AUTO.LOITER`.
4. If PX4 is up and MAVROS is not connecting, you are probably talking to a **different**
   container's PX4 — check `GZ_PARTITION` and `ROS_DOMAIN_ID` in both terminals.

### `/mavros/state` echoes nothing

Not a MAVROS fault — see [QoS mismatch](#qos-mismatch--the-subscriber-that-never-fires).

### A parameter I set in `mavros_px4.yaml` has no effect

Expected, for **plugin** parameters. MAVROS's plugins run as sub-nodes that never receive the
parameters file; `tf.send` and `use_sim_time` on `/mavros/local_position` are the known cases.
Plugin parameters can only be changed at runtime with `ros2 param set`. Do not spend a lab on
this — the course works around it with `vehicle_tf_node`.

---

## OFFBOARD and arming

### `Failure: no Offboard Setpoints` in the PX4 console

### `Rejecting Offboard mode` / the mode switch service returns `success: false`

### The vehicle enters OFFBOARD and immediately drops back to `AUTO.LOITER` or `POSCTL`

**What is wrong:** PX4 will not *enter* OFFBOARD until a setpoint stream is already arriving, and
it will *leave* OFFBOARD if that stream stops for more than about 0.5 s. The setpoint stream comes
first; the mode change comes second. This is not negotiable and it is not a bug — it is what stops
an aircraft flying to whatever stale command it last heard.

**Fix — the correct order, every time:**

1. Start publishing setpoints at **≥ 10 Hz** (20 Hz is a comfortable margin — the course uses
   20 Hz). Publish for at least a second *before* you ask for anything.
2. Call `/mavros/set_mode` with `custom_mode: "OFFBOARD"`.
3. Call `/mavros/cmd/arming` with `value: true`.
4. **Keep publishing** — in the same loop, unconditionally, including while you wait for service
   responses. A publisher that only runs inside a callback that itself depends on the mode is the
   classic deadlock here.

```python
# Prime the stream before requesting anything.
for _ in range(40):                 # 2 s at 20 Hz
    self.pub_sp.publish(sp)
    rate.sleep()
```

### `Arming denied: Resolve system health failures first`

**What is wrong:** the message names no cause, which is what makes it expensive. In headless SITL
there is **neither an RC transmitter nor a ground station**, and PX4 treats both as arming
requirements: the missing RC receiver is a system health failure, and `NAV_DLL_ACT` being
non-zero (the SITL default is 2, Return) makes a live GCS connection an arming requirement too.
MAVROS on the onboard link does not count as a ground station.

**Fix:** you are almost certainly running a stock airframe rather than the course one. The course
airframe `4090_gz_x500_course_gimbal` already sets:

```
param set-default COM_RC_IN_MODE 4     # stick input disabled: stop expecting a transmitter
param set-default NAV_DLL_ACT 0        # no data-link-loss action, so no GCS arming requirement
```

Check what you launched — `course sim` selects the course airframe. If you are experimenting with
a different one, set those two parameters in QGroundControl.

**Both are simulation-only defaults and must never be copied to the real aircraft.** On real
hardware the RC link and the data link are safety devices, and their loss actions are Day 2
material and part of the Day 4 field procedure. An aircraft with `COM_RC_IN_MODE 4` cannot be
taken over by the pilot.

### `Arming denied` / `Arm: DENIED` with a reason in the PX4 console

PX4 tells you which pre-arm check failed; read the console line. In SITL the usual ones are:

| Console text | Cause | Fix |
|---|---|---|
| `Preflight Fail: ekf2 missing data` / `High Accelerometer Bias` | EKF2 has not converged yet | Wait. After `course sim` starts, give it 20–30 s until `Ready for takeoff!` |
| `Preflight Fail: Position estimate not valid` | No valid local position, so no position-requiring mode | Wait for GPS lock in sim; check `/mavros/local_position/pose` is publishing |
| `Arming denied: Geofence violation` | Outside the fence, or the fence has no home | Land/reset, or check home is set |
| `Not arming: Safety switch not disengaged` | Real hardware only | See `hardware-checklist.md` |

### The aircraft overshoots the target, then oscillates or loses it

**What is wrong:** you put `Kp * error` in the **velocity** field of `PositionTarget` while also
sending a position setpoint. PX4 already computes its own proportional term from the position
setpoint, with gain `MPC_XY_P` — so the loop gain is roughly doubled and the aircraft sails past
the target.

**The velocity field is a feedforward, not a second controller.** It carries the *target's*
velocity and nothing else:

```python
# right
sp.position.x, sp.position.y = p_des_x, p_des_y
sp.velocity.x, sp.velocity.y = v_target_x, v_target_y      # feedforward only

# wrong -- PX4 is already doing this from the position setpoint
sp.velocity.x = kp * (p_des_x - p_drone_x)
```

This is the same result from the other direction as the feedforward lesson: without `v_T` you lag
by `v_T / K_p`; with `v_T` *and* your own proportional term on top of PX4's, you overshoot. Send
the position, send the target's velocity, and let PX4 own the gain.

### The vehicle arms but does not climb

You commanded a setpoint at the current altitude, or your setpoint frame is wrong. Note the frame
convention: MAVROS local topics are **ENU** (x east, y north, z **up**); PX4 internally is NED.
`z = +5` is five metres up in MAVROS, five metres *down* in raw PX4 messages. If your drone tries
to dig, this is why.

---

## Detector and perception

### `/detections` publishes, but always with an empty `detections` array

Three possibilities, in order of likelihood:

1. **A blackout is running.** `/detector/blackout` is a `std_msgs/Float64` in seconds, and while
   it is active the detector keeps publishing *empty* messages rather than going quiet — that is
   the point, it is the robustness-lab tool. Tier 3 schedules blackouts automatically. Check:
   `ros2 topic echo /detector/blackout`. Publish `0.0` to end one early.
2. **Nothing is in view.** The gimbal is pointing somewhere else, or the aircraft is too high.
   Look at `/detector/image_annotated` in RViz or `rqt_image_view`.
3. **The target is out of range.** Every route is a bounded loop and the target circulates within
   about 35 m of its start, so if you are far from the world origin you will see nothing.

The detector reports the classes `person`, `car`, `truck`, `bus`, `motorcycle` at confidence
≥ 0.35 by default. Check `detection.results[0].hypothesis.class_id` before trusting a box.

### The detector finds nothing from the air, or reports an `airplane`

**What is wrong:** the target is too few pixels tall for YOLO at the inference size you are using.
At the usual `imgsz=640`, a target that is 64 px tall in the 1280-wide frame letterboxes down to
about **32 px — exactly where YOLO's recall falls off a cliff.** Measured on the course scenario,
with the aircraft at 15 m looking down 45° at a target 21 m away:

| `imgsz` | Inference | Best vehicle detection |
|---|---|---|
| 640 | 25 ms | **nothing** — it reports an `airplane` at 0.73 |
| 800 | 34 ms | truck 0.28 |
| **960** | **42 ms** | **truck 0.58** — the knee |
| 1120 | 59 ms | truck 0.47 |
| 1280 | 66 ms | truck 0.38 |

**Fix:** the course defaults are **`imgsz=960`, `confidence=0.20`**, and they are already set. If
you changed them, change them back. The stray `airplane` is the model's honest answer to a blob
it cannot resolve; it is a symptom of resolution, not of a wrong class list.

### The detector goes quiet as the aircraft climbs or looks straight down

**What is wrong:** confidence collapses as the view approaches nadir, because COCO — what YOLO11n
was trained on — contains almost no vehicles photographed from above. Measured on this scenario:

| Look-down angle | Best detection confidence |
|---|---|
| 40° | 0.51 |
| 50° | 0.27 |
| 60° and beyond | nothing |

**Fix:** fly a shallower look angle. **When a detector goes quiet, ask what look angle it is
flying before you look at anything else** — it is the first question, not the last.

This is not a defect to work around, it is a design constraint and it is examinable: step 4 of
the theory says position error is `R_slant · σ_angle`, so a shallow look angle costs you ranging
accuracy while a steep one costs you detections. **How well you see determines how you are
allowed to fly.**

### The target's estimated position is consistently off to one side

**What is wrong, and it is probably not your bug:** EKF2's yaw in this simulator settles **5–6°
away from truth**. This is reproducible with stock PX4 in the stock world, so it is not something
the course can fix and not something you can tune out. A yaw error rotates the whole bearing ray
about the vertical, which puts the ground intersection sideways by roughly

```
lateral error ≈ 0.1 × ground range
```

At 20 m ground range that is about 2 m, always to the same side. **It is the largest single term
in the error budget** — larger than the pixel error and larger than the gimbal's own residual
pointing error.

**What to do:**

- Recognise the signature: a *consistent* lateral offset that does not change with range in
  proportion, and does not average out over a run. A bug in your frame chain usually produces a
  rotation, a mirror, or an offset that scales differently.
- The scoring script already accounts for it: full marks are awarded at 2 m RMS, not at the 0.5 m
  the pinhole error model alone would suggest, and the on-station tolerance is 8 m rather than 5 m
  for the same reason. A tighter tolerance would measure the simulator rather than you.
- Put it in your report's error budget. Quantifying an error you cannot remove is worth marks;
  pretending it is not there is not.

### Detections describe where the target *used* to be

The detector **drops** frames rather than queueing them, precisely to avoid this — so if you are
seeing lag, it is in your own pipeline. Check you are not queueing images with a large subscriber
depth, and that you are looking TF up at the **detection's** stamp.

### `ImportError: numpy.core.multiarray failed to import` / `cv_bridge` fails on import

**What is wrong:** numpy 2.x got installed over the 1.x that ROS 2 Jazzy is built against. This
breaks `cv_bridge` at import, i.e. every perception lab.

**Fix:** do not `pip install` anything that pulls numpy without a constraint. The image pins
`numpy<2` and guards it at build time. If you have broken your container, the fastest fix is to
delete it and start a fresh one — your code is in the shared volume and is not affected:

```bash
docker rm -f drone-course && ./run.sh
```

### Inference is slow / `no CUDA — inference will run on CPU`

Expected and fine. YOLO11n is **22–26 ms per frame at 640 px on CPU**, and the detector is
throttled to 10 Hz. The default image ships the CPU PyTorch build deliberately.

---

## Gimbal

### The gimbal ignores `/gimbal/cmd/rate` after a moment and drifts to a stop

By design: the rate command has a **0.3 s watchdog**. Stop publishing and it stops moving. A
30 Hz tracking loop must publish every cycle, including the cycles where the pixel error is zero.

### The gimbal stops short of where I commanded

You hit a travel limit. The interface clamps to **pitch −90° … +25°**, **yaw ±135°**, rate limit
**90 °/s** — and reports it: `/gimbal/saturated` raises a per-axis flag. That flag is not a fault,
it is the hand-off signal. When yaw saturates, the *aircraft* must yaw; that is the Lab 4 lesson.

### `Vector3` axis confusion

`Vector3` is always `x = roll, y = pitch, z = yaw`, in radians (or rad/s). **Roll is not
commandable** — it is stabilisation only, matching the real A8 mini.

### `/mavros/gimbal_control/manager/pitchyaw` returns `success: false, result: 2`

Known and **not** to be fixed: it returns DENIED regardless of control acquisition. Do not use it.
Command the gimbal on `/gimbal/cmd/angle` and `/gimbal/cmd/rate`. This is also the faithful
arrangement: on the real aircraft the A8 mini hangs off the **onboard computer** over Ethernet and
PX4 never sees the commands. It is additionally a *service*, which would block the executor and is
unusable in a 30 Hz loop.

---

## Building your own package

### `colcon build` fails with `Could not find a package configuration file provided by ...`

Source ROS and the course workspace before building, and build from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/course_ws/install/setup.bash
cd ~/shared_volume/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### `No executable found`

Your Python script is not installed, or is not executable. It needs to be listed in
`install(PROGRAMS ...)` in `CMakeLists.txt` (or in `setup.py`'s `console_scripts`), have a
`#!/usr/bin/env python3` shebang, and be `chmod +x`. Rebuild afterwards.

### My changes have no effect

You are running the installed copy, not your edit. Build with `--symlink-install`, and re-source
the workspace in every terminal after a build.

### I edited the file on my host and the container does not see it

Check you edited inside `~/drone_course_shared_volume/ros2_ws/src` on the host — that is the only
folder shared with the container, where it appears as `~/shared_volume/ros2_ws/src`.

---

## When you are properly stuck

In this order:

1. `course doctor` — environment.
2. `course verify` — the TF chain and the camera axes.
3. `course test` — camera, detector, target motion, blackout, end to end. Run it with `tier:=-1`
   so the test owns `/target/cmd_vel` and the result does not depend on where the route happens to
   be.
4. `ros2 topic info -v <topic>` — QoS, publisher count, subscriber count. This one answers more
   questions than it looks like it should.
5. `ros2 param get /<node> use_sim_time` — must be `True` for every node in a SITL session.
6. `course log` — copies the newest PX4 flight log into `~/shared_volume/logs/`, where you can
   reach it from your host and upload it to <https://review.px4.io>. PX4 writes its logs inside
   its own build tree, which is not in the shared volume, so this is the only way to get one out.
7. [`../exercises/INTERFACES.md`](../exercises/INTERFACES.md) — the frozen interface contract.
   Every topic name, type, frame, QoS setting and gimbal limit the labs and the scoring script
   agree on. If your node is not talking to anything, check your strings against it first.
8. [`spike-notes.md`](spike-notes.md) — twenty numbered gotchas, with what each one cost.

Then call the instructor, with the output of those six commands rather than a description of them.
