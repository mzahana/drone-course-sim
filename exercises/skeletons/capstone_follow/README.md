# Capstone — follow a moving ground target

**Where this goes:** `~/shared_volume/ros2_ws/src/capstone_follow/`.
`course new capstone` put it there.

**What you write:** `scripts/mission_manager.py` — three `TODO(student)` blocks, the state
machine transitions. Everything else you already built in Labs 4 and 5.

```
IDLE → TAKEOFF → SEARCH → FOLLOW → LOST → RTL
                    ↑________|        |
                             |________|   reacquired
```

## What it must do

The aircraft takes off, finds the target, follows it at a fixed standoff and altitude while
keeping it in the camera frame, and breaks off safely when it loses it.

`mission_manager` is the **only** node that arms the aircraft, the only one that asks for a
mode, and the only one that decides whether guidance is allowed to drive. While `FOLLOW` is
active, `follow_guidance` owns `/mavros/setpoint_raw/local` and the mission manager publishes
nothing to it. Two writers to one setpoint topic is a fight, and the later publisher wins,
which is not a design.

It publishes its state on `/mission/state` (`std_msgs/String`, one of `TAKEOFF`, `SEARCH`,
`FOLLOW`, `LOST`, `RTL`) and gates guidance with `/guidance/enable` (`std_msgs/Bool`).

## Run it

```bash
course sim                                   # 1
course bringup tier:=2                       # 2
                                             # 3
cd ~/shared_volume/ros2_ws
colcon build --packages-select lab4_perception lab5_follow capstone_follow
source install/setup.bash
ros2 launch capstone_follow capstone.launch.py score:=true duration:=180.0 tier:=2
```

## How it is scored

`course score` (or `score:=true` above) runs the scoring node against Gazebo ground truth and
writes `/tmp/score.json`.

| Criterion | Weight | What it measures |
|---|---|---|
| Time on station | 30 | Fraction of the flight within tolerance of the standoff point, recomputed from ground truth |
| Estimation quality | 20 | RMS error of your estimate against truth, scaled by how much of the run you actually had an estimate |
| Kept in the camera frame | 20 | Fraction of samples where truth projects inside the image |
| Robustness | 15 | No altitude-floor or geofence violation; the estimate survives the blackout |
| Code, reproducibility, report, demo | 15 | Marked by hand |

Graded at tiers 1 to 3, so a team that only closes the loop on a straight-line target still
scores meaningfully. Run all three and report all three.

The "on station" tolerance is 8 m, deliberately wider than the roughly 3.5 m of estimation error
no team can remove (most of it the simulator's own 5° yaw bias). A tighter tolerance would grade
you on the simulator's compass rather than on your work.

**What the reference solution scores**, over 180-second runs, so you know what good looks like:

| tier | on station | estimate RMS | kept in frame | automated total |
|---|---|---|---|---|
| 0 stationary | 99% | 3.3 m | 99% | 80 / 85 |
| 1 out and back | 45% | 1.3 m | 97% | 68 / 85 |
| 2 corners and stops | 66% | 2.6 m | 83% | 67 / 85 |

Tier 1 scores lowest on station because the U-turns swing the standoff point through a wide arc
faster than the aircraft can follow it. Beating these numbers is possible and is worth saying so
in your report.

## Rules

**Nothing that flies may read `/target/ground_truth` or `/drone/ground_truth`.** They do not
exist on the real aircraft. A mission that subscribes to one works perfectly in simulation and
cannot be transferred — which is exactly the failure this course is built to make visible. The
scoring script reads them; your code does not.

**One command must reproduce your result.** A launch file, and a README saying which tier and
how long. "It worked on my machine last night" is not a submission.

## Going further

The same launch file, with the same four nodes, runs against the real aircraft on Day 4 — only
the camera source, the gimbal driver and the autopilot link change, and all three are behind
topic names you never touch.
