# drone-course-sim

Simulation environment for a practical drone course (ROS 2 Jazzy · Gazebo Harmonic · PX4 v1.17 ·
MAVROS · YOLO11n).

## Start here

**Read [`PROGRESS.md`](PROGRESS.md) before doing anything.** It is the session handoff: current
status, how to build and run, what is done and why, what is next, and the open issues. It is
written for a session with no prior context.

Then, before debugging anything that smells environmental, read
[`docs/spike-notes.md`](docs/spike-notes.md) — a dozen gotchas that each cost real time
(numpy vs cv_bridge, non-interactive `.bashrc`, `gz` needing `GZ_CONFIG_PATH`, gz-transport
multicast escaping onto the network, and so on).

The course design itself — session plan, theory, capstone, rubric — lives outside this repo in
`~/src/drone_courses/COURSE_PROPOSAL.md`.

## Rules for working here

- **Docker build context is the repo root**: `docker build -f docker/Dockerfile .`
- **Never run the container with `--network host`.** gz-transport discovery is not scoped by
  `ROS_DOMAIN_ID`; a PX4 instance will attach to any Gazebo it can reach on the network. This
  has already happened once. Use `run.sh`, which sets `GZ_PARTITION`, `ROS_DOMAIN_ID` and
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.
- **No passwordless sudo on this host.** Do not try to `apt install` outside the image.
- Guard anything that would break a lab **at image build time**, so it fails the build rather
  than the class.
- Simulation assets are vendored on purpose. Do not reintroduce a dependency on PX4's own models
  or worlds — the build asserts against it.
- Verify frames by reading poses out of a **running** simulation, not by deriving rotations on
  paper. That is how the camera frame bug was found and fixed.
- Write the reasoning down next to non-obvious decisions. The cost here is rediscovery.
