# Autonomous Aerial Systems — a practical course

**Instructor:** Dr. Mohamed Abdelkader
**Audience:** selected senior undergraduates (fellowship cohort)
**Format:** 2 days per week for 2 weeks · 3 hours per day · **4 sessions, 12 contact hours**
**Delivery:** in person. Every session mixes a short lecture with hands-on work; students write
and run code in all four sessions.

> **TODO:** course dates, room, and cohort size.

---

## 1. What the course is

Students learn how an autonomous drone works, from the physical aircraft up to the software that
decides where it goes — and then they build the top layer of that software themselves.

By the end, each team has a drone that can **find a moving vehicle on the ground, keep a camera
pointed at it, work out where it is on the map, predict where it is going, and follow it from the
air.** The same code runs in a realistic simulator and on a real aircraft; demonstrating that it
transfers from one to the other is a deliberate part of the course.

The course is taught as **three layers**, and every session says which layer it is about:

| Layer | What it is | Session |
|---|---|---|
| **1 — Hardware** | The aircraft itself: frame, motors, propellers, battery, radio, camera | Day 1 |
| **2 — Autopilot** | The flight computer that keeps the aircraft stable and in the right place | Day 2 |
| **3 — Application** | A small onboard computer that sees, decides and commands the autopilot | Days 3 and 4 |

This framing doubles as a diagnostic habit: when something misbehaves, the first question is
always *which layer is this?*

---

## 2. Time commitment

| Block | Time | When |
|---|---|---|
| **Pre-work** — install the software environment on their own laptop and verify it | ~2 h | Before Day 1 |
| **Contact hours** — four 3-hour sessions | 12 h | The two course weeks |
| **Project work** — the capstone, in simulation | ~10 h | After Day 4 |
| **Project demonstration** | 1 h | **TODO:** a short fifth meeting, or an asynchronous submission |

**The pre-work is mandatory and it is checked.** The whole software environment is distributed as
a single pre-built download (about 3.4 GB to download, about 13 GB installed) so that **nothing is
installed during class**. Class time is too short to spend on installation, and a student who
arrives without it loses roughly a third of Day 1. The instructions are in `prework.md` and end
with a checklist and a one-command self-test.

---

## 3. Prerequisites

Students should be comfortable with:

- **Programming in Python** — writing and debugging their own code, not just reading it.
- **The Linux command line** — moving around a filesystem, running programs, reading output.
- **Undergraduate mathematics** — vectors, matrices, basic calculus, and the idea of a coordinate
  frame.

Helpful but **not** required, and taught where needed:

- ROS 2 (the robotics software framework used throughout). A refresher is suggested in the
  pre-work; the essentials are taught at the start of Day 3.
- Any prior flying or drone-building experience.

**Hardware:** a laptop with at least 4 cores, 8 GB of memory and 20 GB of free disk, running
Ubuntu Linux or Windows 10/11 with WSL2. A graphics card is **not** needed — the vision software
runs fast enough on an ordinary processor (42 milliseconds per image at the resolution the course
uses). Graphics drivers are not a barrier either: the environment carries its own desktop that
students open in a web browser. Students on Apple Silicon Macs should contact the instructor
before Day 1 — the software runs, but under processor emulation that makes the simulator slow.

---

## 4. Learning outcomes

On completing the course, a student can:

1. **Identify every component of a multirotor aircraft** and explain what each one does and how
   its choice constrains the others.
2. **Size an aircraft from its numbers** — compute thrust-to-weight ratio, estimate endurance, and
   build a power budget for a given camera and onboard computer — and defend those choices.
3. **Explain how an autopilot keeps an aircraft in the air**: the chain from a desired position
   down to individual motor commands, and which sensors each step depends on.
4. **Configure, calibrate and fly** an autopilot-controlled aircraft in simulation: parameters,
   sensor calibration, flight modes, a planned mission, and the safety behaviours.
5. **Diagnose a fault from a flight log** — deliberately break the aircraft, then find the fault
   in the recorded data rather than by guessing.
6. **Write software that commands an aircraft**: take off, fly a path, and switch between
   commanding positions and commanding velocities, all under the safety limits of the autopilot.
7. **Convert a camera detection into a map position.** Understand why a pixel gives a direction
   and not a place, use the ground surface to supply the missing distance, chain the coordinate
   frames correctly, and **state how large the resulting error is and what drives it.**
8. **Keep a camera pointed at a moving object** with a gimbal control loop, and handle the case
   where the gimbal runs out of travel and the aircraft itself must turn.
9. **Estimate where a target is going** with a simple filter, and keep tracking it through gaps
   when the camera loses it.
10. **Design a following behaviour that stays safe**: hold a standoff distance, use the target's
    velocity to remove the lag, and enforce altitude, speed and field-of-view limits.
11. **Run a bench test and a supervised flight safely** on real hardware, following a written
    checklist and a briefed abort procedure.
12. **Transfer software from simulation to a real aircraft** and explain what did and did not
    change, and why.
13. **Diagnose the common silent failures** of this kind of system — the ones that produce wrong
    answers rather than error messages.

---

## 5. Session outlines

Each session is 3 hours: lecture, a short quiz, and hands-on labs. Every lab produces a piece of
the final project, so the project is **assembled** from work already done rather than started from
nothing.

### Day 1 — The machine, and the environment that simulates it

- The three-layer model, and the final project shown up front so everyone knows where this ends.
- Aircraft anatomy: frame, motors, propellers, speed controllers, batteries, flight controller,
  radio link, camera gimbal, onboard computer.
- **Sizing by numbers**: thrust-to-weight ratio, endurance, and a power budget, worked through on
  the actual aircraft the course uses.
- **Hardware bench**: the real aircraft on the table. Every wire traced, the camera gimbal mount
  and cabling examined, and the onboard computer's connection to the flight controller followed
  end to end.
- **Lab**: bring the simulation up and fly a simulated aircraft. Everyone leaves Day 1 with a
  flying drone on their own laptop.

### Day 2 — The autopilot

- Why simply setting motor speeds does not work, and how a real autopilot layers its control:
  position, then velocity, then attitude, then rotation rate, then motors.
- Where the aircraft thinks it is: what the navigation filter combines from the inertial sensors,
  GPS, barometer and compass, and which flight modes therefore need what.
- Flight modes, the arming sequence, geofences and failsafes — and the safety briefing.
- **Labs**: configure and calibrate the aircraft, plan and fly a mission in simulation, then
  **deliberately break it** — remove GPS, mistune a control gain — and recover. Then download the
  flight log and **find the fault in the data.**

### Day 3 — Seeing the target

- The essentials of the robotics software framework used by the onboard computer, including the
  one configuration mistake that silently stops messages arriving.
- **Coordinate frames** — the highest-value theory in the course, and the source of most real
  bugs. How to convert between the several conventions in play, and the full chain from the camera
  through the gimbal to the map.
- **From a pixel to a place**: a detection gives a direction; the ground surface supplies the
  distance. And then the honest question — **how wrong is this answer?** The error grows with
  distance and with the angle you look at, which makes flight altitude and camera angle
  engineering choices rather than free parameters.
- **Labs**: write a program that takes off and flies a path; then a program that takes the camera
  detections, keeps the gimbal on the target, and publishes the target's map position **with its
  error ellipse drawn on screen.**

### Day 4 — Following, safety, and real hardware

- **Prediction**: a simple filter that tracks the target's position *and velocity*, and keeps
  coasting through the moments the camera loses it.
- **Guidance**: how to sit a fixed distance behind a moving target without lagging behind it, and
  the limits — altitude, speed, field of view — that turn "follow it" into something that can be
  guaranteed rather than hoped for.
- Where commands go, and why the component that owns safety must be the last one in the chain and
  able to refuse the student's commands.
- **Hardware session**: the same software on the real aircraft with the real camera gimbal. A
  bench test with the propellers removed, then a **supervised flight** — detect, track, locate and
  follow a ground target at a fixed altitude — under a written checklist and a briefed abort
  procedure.
- **Project brief**, team formation, and a guided start.

---

## 6. The capstone project

**Follow a moving ground target.**

A vehicle drives an unknown route. The drone must take off, find it, and follow it from the air at
a fixed distance and altitude until it stops — keeping it in the camera's view the whole time, and
breaking off safely if it loses it.

Students are **given** the simulated world, the vehicle, the aircraft with its camera gimbal, the
object detector, a tracking filter, the flight-safety layer, and an automatic scoring script.
Students **write** four small programs, each a hardened version of something they already built in
a lab:

| Program | What it does |
|---|---|
| Target locator | Turns each camera detection into a map position, with an honest error estimate |
| Gimbal pointer | Keeps the camera on the target; hands over to the aircraft when the gimbal runs out of travel |
| Follow guidance | Works out where the drone should be, and how fast, within the safety limits |
| Mission manager | Take off → search → follow → lost → return home |

**The capstone is integration and tuning, not a blank page.**

Runs are graded at four difficulty levels, so a team that only succeeds against a slow,
straight-line target still scores meaningfully:

Runs are graded against fixed numbers — the drone should sit **18 m behind the target at 12 m
altitude**, and counts as on station when it is within **8 m** of that — so every team is measured
the same way and can check its own score while it works.

| Level | Target behaviour | What it exposes |
|---|---|---|
| 0 | Stationary | Does the whole loop close at all |
| 1 | Straight, 3 m/s | Lag — and the fix for it |
| 2 | Turns and stops | Camera limits, and what to do when a stopped target has no direction |
| 3 | Adds a 3-second blind period | Prediction and graceful degradation |

**Bonus:** the same software, unchanged, flying the real aircraft against a real vehicle.

---

## 7. Assessment

| Criterion | Weight | How it is measured |
|---|---|---|
| **Time on station** — the fraction of the run spent at the correct distance behind the target | **30 %** | Automatically, against the simulator's true positions |
| **Estimation quality** — how close the team's estimate of the target's position is to the truth | **20 %** | Automatically (RMS error in metres) |
| **Target kept in view** — the fraction of camera frames containing the target | **20 %** | Automatically |
| **Robustness** — survives the blind period, and breaks off cleanly when the target is genuinely lost | **15 %** | Automatically, plus review |
| **Code quality, reproducibility, report and demonstration** | **15 %** | Instructor review — the run must reproduce from a single command |

Four short in-class quizzes (one per session, 4 questions each) are used for immediate feedback
and are not part of the grade.

Three quarters of the grade is measured by a script against the simulator's ground truth, so it is
objective, repeatable, and visible to the students while they work.

---

## 8. Equipment and software

**Aircraft (provided; students do not build it)**

- X500 V2 quadrotor airframe with a Pixhawk flight controller running PX4 v1.17
- SIYI A8 mini 3-axis camera gimbal
- Raspberry Pi 5 or NVIDIA Jetson Orin Nano as the onboard computer
- Radio control link, telemetry link, flight batteries and safety equipment

**Software environment (provided as one pre-built download)**

- Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic simulator · PX4 v1.17 · MAVROS
- QGroundControl v4.4.4, the standard ground-station software, already installed
- Ultralytics YOLO11n object detector, with its trained weights already included
- A course command-line tool that checks the environment, starts everything, self-tests, hands
  out the lab exercises and their reference solutions, and grades a run
- A **browser-based desktop**, so a student whose laptop graphics do not cooperate opens a web
  page instead and loses nothing. Nobody is blocked on Day 1 by their own machine.
- A simulated world containing the aircraft, its gimballed camera and a driving ground vehicle,
  with a scoring script that grades a run automatically against the simulator's true positions

**Students provide** a laptop meeting the specification in §3. A graphics card is not required.

The four in-class lab exercises are handed out as skeleton programs with the structure in place
and the interesting parts left blank, so students spend their lab time on the idea rather than on
boilerplate. Reference solutions are released after each lab and install **alongside** a student's
own version rather than replacing it, so the two can be run and compared.

---

## 9. Notes for the programme office

- **Cohort size is limited by the hardware session**, not by the simulation. One aircraft and a
  supervised flight in the Day 4 slot sets the practical limit.
  **TODO:** confirm the cohort size and whether a second aircraft is available.
- **The Day 4 flight requires a site, permission and acceptable weather.** If any of those is
  unavailable the session becomes a bench session plus a recorded flight, which still meets the
  learning outcomes but must be planned in advance rather than decided on the day.
  **TODO:** confirm the site and the permissions.
- **The pre-work download must be possible on the students' own connections.** If any student
  cannot download 3.4 GB before Day 1, arrange a USB copy in advance.
- All safety procedures are written down and are followed on the day: `hardware-checklist.md` and
  `field-procedure.md`.
