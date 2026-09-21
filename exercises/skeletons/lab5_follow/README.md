# Lab 5 — Track and follow

**Where this goes:** `~/shared_volume/ros2_ws/src/lab5_follow/`.
`course new lab5` put it there.

**What you write:**
* `scripts/target_tracker.py` — a 4-state constant-velocity Kalman filter on the ground plane.
* `scripts/follow_guidance.py` — the standoff reference, the velocity feedforward, the clamps.

Four `TODO(student)` blocks.

## What it must do

`target_tracker` subscribes to `/target/estimate` from Lab 4 and publishes `/target/track`
(`nav_msgs/Odometry`, frame `map`) with position **and velocity**. It must keep publishing
while it coasts through a dropout, and it must stop when coasting has gone on too long.

`follow_guidance` subscribes to `/target/track` and `/mavros/local_position/pose`, and
publishes `/mavros/setpoint_raw/local` (`mavros_msgs/PositionTarget`) with position **plus a
velocity feedforward**.

```
p_des = p_T − d · v̂_T + h · ẑ
v_cmd = v_T + K_p (p_des − p_drone)
```

**Half of that second line is computed inside the autopilot.** You send `p_des` as the position
setpoint and `v_T` as the velocity *feedforward*; PX4 supplies the `K_p (p_des − p_drone)` term
itself, with its own gain `MPC_XY_P` (0.95 by default — read it with
`ros2 param get /mavros/param MPC_XY_P`). Send your own proportional term in the velocity field
as well and the loop gain roughly doubles: measured, the aircraft ran 5 m past a stationary
target, put it behind the camera, and lost it.

## Run it

```bash
course sim                                   # 1
course bringup tier:=1                       # 2
                                             # 3
cd ~/shared_volume/ros2_ws && colcon build --packages-select lab4_perception lab5_follow
source install/setup.bash
ros2 launch lab4_perception lab4.launch.py &
ros2 launch lab5_follow lab5.launch.py
```

## Acceptance test — the feedforward experiment

This is the measurement the lab exists for. Fly the same tier-1 run twice:

```bash
ros2 launch lab5_follow lab5.launch.py feedforward:=false
ros2 launch lab5_follow lab5.launch.py feedforward:=true
```

and score each:

```bash
ros2 run drone_course_sim scoring_node.py --ros-args -p duration:=90.0 -p tier:=1
```

Without the feedforward you should measure a steady-state lag of about `v_T / MPC_XY_P` — about
3.2 m behind a 3 m/s target. With it, the lag should go to roughly zero. Report both numbers;
"it looked better" is not a result.

The default geometry is 12 m altitude and 18 m standoff — a 34° look-down. That angle was chosen
by the **detector**, not by the geometry: measured confidence is 0.51 at 40° down, 0.27 at 50°,
and nothing at all past 60°, because COCO contains almost no vehicles photographed from above.
Flying shallower costs accuracy (the along-range error is amplified by `1/sin θ`) and buys
detections, which is the right trade — an accurate fix you never get is worth nothing.

Then try tier 3, which blacks the detector out on a schedule:

```bash
course bringup tier:=3
```

Your filter should coast through the gap and the aircraft should keep flying a sensible
reference. If the track dies the instant detections stop, your predict step is wrong.

## The three things that will bite you

**A stopped target has no heading.** `v̂_T` is `v_T/|v_T|` and `|v_T|` goes to zero. Latch the
last valid heading; a latched bearing is a decision, dividing by zero is a crash. This happens
on every tier-2 run, and it happens in front of the class.

And when you have never had a heading at all, hold the bearing you are already on at distance
`d` — do **not** sit directly overhead. Overhead throws away the look angle the whole error
model is built on, and a nadir view of a vehicle is one the detector was never trained on:
measured, detections went to zero while the aircraft hovered over a truck in plain sight.

**R must come from the measurement, not from a constant.** Lab 4 publishes a real covariance;
use it. A filter told the measurement is better than it is will chase noise, and told it is
worse will ignore the measurement and drift. Both look like bad tuning and neither is.

**Safety goes last.** Apply the clamps *after* the guidance law and let them overrule it.
Scaling `vx` and `vy` together matters: clamping them independently changes the direction of
travel as well as its magnitude.

## Going further

Turn the Mahalanobis gate off and publish a false detection somewhere across the field. Watch
what one bad measurement does to a filter with no gate.
