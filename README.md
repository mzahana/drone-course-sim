# Drone Course — Simulation Environment

The container students use for the **Autonomous Aerial Systems** course, and the reference
environment the course material is written against.

| Component | Version |
|---|---|
| Ubuntu | 24.04 (noble) |
| ROS 2 | Jazzy, `desktop-full` |
| Gazebo | Harmonic (Sim 8.x) |
| PX4 | v1.17.0 |
| Autopilot bridge | MAVROS 2.14 |
| Perception | Ultralytics YOLO11n (weights baked in) |

The course project is **following a moving ground target from the air**: detect it, work out
where it is on the ground, estimate where it is going, and fly a standoff above and behind it.
The same code runs in simulation and on the real X500 V2 with a SIYI A8 mini.

---

## Quick start (students)

```bash
git clone <this repo> && cd drone-course-sim
./install.sh          # checks Docker, pulls the image, creates the shared volume
./run.sh              # drops you into the container
```

Inside the container:

```bash
course doctor         # verifies PX4, Gazebo, MAVROS, TF and the GPU
course sim            # brings the simulation up
```

Your code lives in `~/shared_volume/ros2_ws/src` **inside** the container, which is
`~/drone_course_shared_volume/ros2_ws/src` **on your own machine**. Edit it with your normal
editor on the host; build and run it inside the container. Nothing you write is lost when the
container is removed.

---

## Design decisions worth knowing

These are the choices that cost time to rediscover, so they are written down rather than
buried in the Dockerfile.

**One Gazebo, not two.** ROS 2 Jazzy's `ros_gz` ships Gazebo Harmonic through the
`gz_*_vendor` packages. Installing Gazebo *again* from the OSRF repository — which is what
PX4's own setup script does — leaves two installations in the image and a bridge that may be
ABI-skewed against the simulator it is talking to. Instead PX4 is built with
`CMAKE_PREFIX_PATH` pointed at the vendored Gazebo, so the bridge and the simulator are by
construction the same build.

**PX4's `Tools/setup/ubuntu.sh` is not used.** It installs the NuttX cross-toolchain and that
second Gazebo. SITL needs neither. `docker/scripts/px4_deps.sh` installs only what the host
build actually requires.

**The GeographicLib geoid dataset is installed at build time.** Without it MAVROS refuses to
start, and it would otherwise be the first error message every student sees.

**YOLO weights are baked into the image.** A lab must never begin with thirty people
downloading the same file over conference wifi.

**numpy stays on 1.x, deliberately.** ROS 2 Jazzy is built against the Debian numpy 1.26.
Installing `ultralytics` without a constraint makes pip resolve numpy 2.x, which does two bad
things: it cannot uninstall the dpkg-owned numpy (`RECORD file not found`), and if it could, it
would break `cv_bridge` at import — i.e. every perception lab. A constraint file pins it, and a
build-time import guard fails the build rather than the class.

**torch comes from the CPU index.** The default PyPI wheel pulls `cuda-toolkit`, `cudnn`,
`cublas` and `triton` — roughly 3 GB that most student laptops cannot use. YOLO11n on CPU is
fast enough for every lab. A CUDA variant is a separate image tag, not the default.

**`pip` uses `--break-system-packages`.** Ubuntu 24.04 enforces PEP 668. Inside a container
that is the right trade: the alternative is a virtualenv that every student then has to
remember to activate in every terminal.

---

## Repository layout

```
docker/
  Dockerfile          the image
  Makefile            build helpers
  scripts/
    px4_deps.sh       PX4 SITL build dependencies
    entrypoint.sh     sources ROS, PX4 paths and the student workspace
install.sh            host-side setup
run.sh                start / re-enter the container
docs/                 validation notes
```

## Building it yourself

```bash
cd docker && make build
```

Students should **pull** rather than build — the build takes a while.

**Disk**: about 3.4 GB to download, but roughly **13 GB on disk** once unpacked. Tell students
to have the space free before the first session.
