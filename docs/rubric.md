# Capstone rubric — how your run is marked

**Audience:** students, while you are still building. **Read it before you tune anything.**

100 points. **85 of them are produced by a script** — `course score`, which is
`ros2 run drone_course_sim scoring_node.py` — and you can run it yourself as many times as you
like before you submit. The remaining **15 are marked by a person** reading your repository, your
report and your demo.

Nothing here is a surprise on the day. The scorer is
`ros2_ws/src/drone_course_sim/scripts/scoring_node.py` and it is in the image you already have.
If this page and that file ever disagree, **the file is what marks you** — tell the instructor.

---

## 1. What the scorer is, and why you cannot use it

The scorer subscribes to `/target/ground_truth` and `/drone/ground_truth` — two Gazebo
`OdometryPublisher` plugins reporting exactly where the target and the aircraft really are. It
compares your flight against that truth at **10 Hz** for the configured duration and writes
`/tmp/score.json` as well as printing a breakdown.

**Your flight code may not subscribe to either topic.** They do not exist on the real aircraft.
A mission that reads ground truth scores well in simulation and cannot be transferred, which is
the single failure this whole course is built to make visible. A submission that reads ground
truth in any node that flies scores **zero** on the automated 85, regardless of what the script
printed.

That asymmetry is deliberate, and it is the reason the numbers below are worth anything: the
scorer knows something you are not allowed to know, so you cannot optimise directly against it.
You get to see the mark; you do not get to see the measurement.

Two more consequences of how it works, both of which change how you should read your own score:

- **The standoff reference is recomputed from truth at every sample**, not read from your
  `follow_guidance` node. You are scored on where you were *asked* to be, not on where you
  decided to aim.
- **"Kept in frame" is measured by projecting truth into the camera**, not by counting your
  detections. A flaky detector with a correctly pointed gimbal is not punished twice, and a lucky
  false positive does not rescue a target that has left the frame.

---

## 2. The automated 85

Run geometry, fixed for everybody and set as scorer parameters: **standoff `d` = 18 m behind the
target, altitude `h` = 12 m above it** — a 34° look-down at 21.6 m slant range.

### Time on station — 30 points

**Measured:** at each sample the scorer builds the reference point
`p = target − 18 m · (target heading) + 12 m · ẑ` from ground truth, and measures your aircraft's
3-D distance to it.

**Threshold:** you are *on station* when that distance is **≤ 8 m** (`station_tol`).

**Score:** `30 × (samples on station ÷ samples scored)`. It is a straight fraction — there is no
partial credit for being 8.1 m off, and no bonus for being 0.5 m off. The mean error is printed
and stored, but it does not earn points.

**Why 8 m and not 5 m.** Two things eat the margin before your guidance law does. About 1.7 m of
target-position error is not removable by anybody in this cohort — most of it is EKF2's yaw in
this simulator sitting 5–6° off truth with stock PX4 and the stock world, and a yaw error rotates
the bearing ray about the vertical. On top of that the reference point itself *sweeps* while the
target turns: at 18 m standoff, a 180° turn moves the point you are being scored against by 36 m,
faster than an 8 m/s aircraft can follow it. A tighter tolerance would grade you on the
simulator's compass and on the target's steering, instead of on your work.

**The heading detail that will bite you:** the scorer latches the target's heading only while the
target is moving faster than **0.5 m/s**, and it keeps the last latched heading after the target
stops. Before the target has ever moved there is no heading at all, so the reference is placed
18 m from the target *along whatever bearing you are currently holding* — which is exactly what
the guidance law is asked to do in that case. This is why tier 0 is winnable and why a stopped
target does not silently destroy your score.

### Estimation quality — 20 points

**Measured:** horizontal RMS error of your published target position against truth, over the
scored part of the run. The scorer reads `/target/track` (`nav_msgs/Odometry`, filtered) if you
publish it, otherwise `/target/estimate` (`geometry_msgs/PoseWithCovarianceStamped`, raw).
Publishing both is normal and correct. An estimate older than **1 s** (`estimate_timeout`) counts
as no estimate at all.

**Thresholds:** full quality at **RMS ≤ 2.0 m**, zero at **RMS ≥ 8.0 m**, linear in between:
`quality = (8.0 − RMS) ÷ 6.0`, clamped to [0, 1].

**Score:** `20 × quality × coverage`, where `coverage` is the fraction of scored samples on which
a fresh estimate existed. **Accuracy is only worth what it covers.** An estimate that is perfect
for two seconds out of sixty is not a working locator, and the coverage factor is what says so.
If you publish nothing, this category is 0 and the scorer prints a yellow line telling you which
two topics it looked at.

**Why full marks at 2 m and not at the 0.5 m the pinhole error model suggests.** The yaw error
above rotates the bearing ray about the vertical, and at this geometry that alone is worth more
than a metre on the ground. Setting the threshold below the achievable floor grades everyone on
the same simulator bug.

### Target kept in the camera frame — 20 points

**Measured:** the scorer transforms the true target position into `camera_optical_frame` using
your live TF, projects it with the real `k` from `/camera/camera_info`, and checks whether it
lands inside the image: `z > 0.1 m` in front of the camera and `0 ≤ u < width`, `0 ≤ v < height`.

**Score:** `20 × (samples inside the image ÷ samples checked)`. If no `CameraInfo` has arrived,
nothing is checked and this is 0.

This is the category your `gimbal_pointer` is graded on, and it is scored on pointing, not on
detecting.

### Robustness — 15 points, as three separate 5s

| Part | Points | Measured | Threshold |
|---|---|---|---|
| Altitude floor | 5 | Lowest altitude reached while airborne and not exempt | **All 5 or nothing.** One sample below **5 m** loses the whole 5 |
| Geofence | 5 | Largest horizontal radius from the origin while airborne | **All 5 or nothing.** One sample beyond **120 m** loses the whole 5 |
| Coasting through the blackout | 5 | While `/detector/blackout` is active, is your estimate still within **8 m** of truth? | `5 × (good blackout samples ÷ blackout samples)` |

Both clamps are pass/fail, not proportional — a single 10 Hz sample is enough. Do not fly with
2 m of margin against a 5 m floor.

**If no blackout occurs at all** — every tier below 3 — the coasting part is awarded the full
5 and the report says `no blackout occurred (tier < 3)`. Tiers 0 to 2 therefore have 10 of their
15 robustness points riding on two clamps that cost nothing to respect, which is the point:
safety is not supposed to be expensive.

**Publishing no estimate during a blackout is counted, and counted against you.** Those samples
go into the blackout denominator with nothing in the numerator. Coasting on the filter is the
entire skill tier 3 exists to test; a filter that stops predicting when measurements stop is not
a filter.

---

## 2b. Real time, and why your machine's speed is in the mark

The scorer prints the **real-time factor** it observed: simulated seconds divided by wall-clock
seconds over the scored run. On a machine that keeps up it reads `1.00`. On one that does not it
reads lower, and the line goes yellow.

This is not cosmetic. Everything that makes following hard is measured in *simulated* seconds —
how long the detector takes, how often your guidance node runs, how far the target moves between
two frames. A simulation running at 0.4× is a system with a fraction of the real perception
latency, and it will score **better** than the same code on a faster machine. That is backwards,
and it is exactly why the number is printed.

If yours is below about 0.9, the usual cause is Gazebo falling back to software rendering. Check
that the field is textured rather than flat grey, and see
[`troubleshooting.md`](troubleshooting.md). Compare your score only against runs at a similar
factor, and say which one yours was taken at when you report it.

---

## 3. What is exempt, and why

The scorer ignores every sample where either of these is true:

1. The aircraft is **not airborne** — altitude at or below **2 m** (`airborne_altitude`).
2. `/mission/state` currently reads **`RTL`, `LAND`, `IDLE` or `TAKEOFF`**.

In those samples nothing is accumulated: not time on station, not estimation, not frame
retention, and no altitude-floor violation is recorded.

**Why.** Climbing out and coming home are the mission working, not the mission failing. Scoring
the deliberate landing as a floor breach punishes a solution for ending correctly, and scoring
the climb punishes it for starting. Averaging the climb and the trip home into "time on station"
also makes the metric mostly a function of how long your run happened to be, which is not a
property of your code.

Three things follow, and all three are worth acting on:

- **`SEARCH` and `LOST` are *not* exempt.** Time spent hunting for a target you have lost is
  scored as time not on station, with no estimate and nothing in frame. That is intentional:
  losing the target is the failure mode this capstone is about.
- **The geofence check is not exempt either.** Re-read the table above: the state exemption
  applies to the altitude floor only. You can breach the fence during RTL and lose 5 points.
- **If you never publish `/mission/state`, nothing is exempt.** The scorer's state is `None`,
  which is not in the exempt list, so your climb-out and your landing are both scored in full and
  your descent to the ground is counted as floor violations. Publishing `/mission/state` is in
  the interface contract, it costs four lines, and leaving it out is worth around 10 points.

---

## 4. The four tiers

The tier changes only what the target does. Your code does not change between them, and neither
does anything on this page.

| Tier | Target | What it adds |
|---|---|---|
| 0 | Stationary | Does the loop close at all. No heading exists, so the reference sits on your own bearing. The gentlest case |
| 1 | Out and back, long straights at 3 m/s | Steady-state lag. Without velocity feedforward you sit a fixed distance behind and the fixed distance is the whole grade |
| 2 | Circuit with corners and stops | Gimbal saturation, keeping the target in frame through a turn, and the stopped-target degeneracy where `v̂_T` is undefined |
| 3 | Everything in 2, plus a 3-second detector blackout | Prediction. It is the only tier where the coasting 5 is live, and the only tier where the estimate itself degrades |

There is also a **tier −1**: the target publishes nothing, so a test or a person can own
`/target/cmd_vel`. It is a development aid, not a graded tier.

Every route is bounded, and the target does not set off until **your aircraft is above 5 m**, plus 5 s. It waits for you, not for a stopwatch — so a run is the same whether you launch the mission straight after `course bringup` or four minutes later.

**Grade at tiers 1, 2 and 3, and report all three.** A team that only closes the loop on a
straight-line target still scores meaningfully, which is what the tiers are for.

> **TODO:** confirm how the three tier scores combine into the single automated 85 on the
> transcript — best of three, mean, or each tier reported separately.

**What the reference solution scores**, over 180-second runs. This is the yardstick for "good",
not a ceiling; all four are beatable and saying by how much belongs in your report.

| tier | on station | estimate RMS | kept in frame | automated total |
|---|---|---|---|---|
| 0 stationary | 100% | 1.8 m | 99% | 84.8 / 85 |
| 1 out and back | 45% | 1.9 m | 96% | 67.6 / 85 |
| 2 corners and stops | 71% | 1.5 m | 97% | 75.9 / 85 |
| 3 plus blackouts | 51% | 4.3 m | 85% | 55.7 / 85 |

Tier 1 scoring *lower* than tier 2 is not a mistake: its 180° U-turns swing the standoff point
through a wider arc, and faster, than tier 2's 90° corners at half speed. Tier 3 is the only tier
where the estimate itself degrades, because a blackout is the one thing a filter cannot see
through — only coast through.

---

## 5. The manual 15

Marked by a person, from your repository and your demonstration. These are the criteria that will
actually be applied.

### Code quality — 4 points

- The five nodes stay five nodes, with the responsibilities the interface contract gives them.
  A `mission_manager` that also runs a PID loop loses points here even if it scores well.
- **One writer per topic.** While `FOLLOW` is active, `follow_guidance` owns
  `/mavros/setpoint_raw/local` and nothing else publishes to it. Two writers to one setpoint
  topic is a fight in which the later publisher wins, and that is not a design.
- Tuned numbers are named constants or ROS parameters, not literals buried three levels down.
- TF lookups use the **message's** timestamp, not `rclpy.time.Time()`. A "now" lookup during a
  30°/s slew is 3° off and nothing warns you.
- Subscribers to MAVROS telemetry declare `BEST_EFFORT` QoS. A silent non-match is a code defect,
  not bad luck.
- No dead code, no commented-out experiments, no `print` debugging left in the flight path.

### Reproducibility — 4 points

- **One command reproduces your result.** A launch file, plus a README stating the tier and the
  duration. "It worked on my machine last night" is not a submission.
- `colcon build` from a clean workspace succeeds with no manual step and no edit to a path.
- The tier and the duration are launch arguments, not edited constants.
- Nothing that flies reads `/target/ground_truth` or `/drone/ground_truth` — checked by grep, and
  a hit here also zeroes the automated 85.
- Full marks means the marker ran your command, got a score, and it was within a few points of
  the one you reported.

### Report — 4 points

Marked against `report-template.md`. What earns the points is the reasoning, not the length: a
tuning choice defended with a measurement, an error budget whose terms add up to the RMS you
actually observed, and a failure described precisely enough that someone else could have
diagnosed it from your account. A report that narrates what you did in chronological order
without explaining why gets roughly half.

### Demonstration — 3 points

- The run starts from the one command, in front of the marker, and reaches `FOLLOW`.
- You can answer "why is this number this number" about at least your controller gain, your
  process noise, and your standoff.
- You can point at the weakest category in your own score and say what you would change, and the
  answer is specific.
- A recorded run is acceptable if it is the same command and the same commit.

> **TODO:** submission deadline, and whether the demonstration is a short fifth meeting or an
> asynchronous recording.

---

## 5b. Recording the demo

The image can record its own screen, so the demo does not need anything on your host:

```bash
course desktop                                   # if you are not using X11 forwarding
bash $COURSE_TOOLS/media/capture.sh rec my-demo 120
```

That writes `~/media/my-demo.mp4` — 1920x1080, H.264, no mouse pointer. Record the run you are
submitting, not a better one you did earlier, and say in the report which tier it is and what
real-time factor the scorer reported for it.

---

## 6. Running the scorer yourself

```bash
course sim                                     # terminal 1
course bringup tier:=2                         # terminal 2
cd ~/shared_volume/ros2_ws                     # terminal 3
colcon build --packages-select lab4_perception lab5_follow capstone_follow
source install/setup.bash
ros2 launch capstone_follow capstone.launch.py score:=true duration:=180.0 tier:=2
```

`score:=true` starts the scoring node inside the same launch. To score a mission that is already
running, use `course score` in another terminal — it takes the same parameters:

```bash
course score -p duration:=180.0 -p tier:=2
```

The launch file defaults to `duration:=120.0`; the reference numbers in the table above were
measured over **180 s**, so use 180 when you want to compare against them. The result is printed
and written to `/tmp/score.json` — copy that file out to your shared volume before the container
stops, because `/tmp` does not survive it.

Read `/tmp/score.json` rather than only the coloured summary. It carries `coverage`,
`worst_error_m`, `min_altitude_m`, `max_radius_m` and `mission_states_seen`, and those five
fields are usually what tells you *why* a category is low.
