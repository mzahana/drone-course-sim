# Lab 4 — See the target

**Where this goes:** `~/shared_volume/ros2_ws/src/lab4_perception/`.
`course new lab4` put it there.

**What you write:**
* `scripts/target_locator.py` — a detection becomes a position in `map`, with a covariance.
* `scripts/gimbal_pointer.py` — a pixel error becomes a gimbal rate.

Five `TODO(student)` blocks between them. Everything else is scaffolding.

## What it must do

`target_locator` subscribes to `/detections` and `/camera/camera_info`, turns the best
detection's centre pixel into a bearing ray, intersects that ray with the ground plane, and
publishes the result on `/target/estimate` (`geometry_msgs/PoseWithCovarianceStamped`, frame
`map`) with a real covariance — plus an ellipse on `/target/estimate_ellipse` for RViz.

`gimbal_pointer` subscribes to `/detections`, converts the pixel error to an angle error, runs
a PID, and publishes a rate on `/gimbal/cmd/rate` (`geometry_msgs/Vector3Stamped`, rad/s,
`x`=roll `y`=pitch `z`=yaw). Roll is not commandable — the A8 mini stabilises it and does not
take it as an input.

## Run it

```bash
course sim                                   # 1
course bringup tier:=1                       # 2
                                             # 3
cd ~/shared_volume/ros2_ws && colcon build --packages-select lab4_perception
source install/setup.bash
ros2 launch lab4_perception lab4.launch.py
```

You need the aircraft in the air to see anything useful. Run your Lab 3 node, or:

```bash
course solution 3 && colcon build --packages-select lab3_offboard_solution
ros2 launch lab3_offboard_solution offboard_square.launch.py
```

## Acceptance test

```bash
ros2 topic hz /target/estimate                       # a few Hz once the target is in frame
ros2 run drone_course_sim scoring_node.py --ros-args -p duration:=60.0
```

The scoring script compares your estimate against Gazebo ground truth. **Estimation quality**
should be worth most of its 20 points; the rest of the score needs Lab 5.

In RViz — `course rviz` opens the course layout with all of this already configured — the
ellipse should sit on the target, and it should visibly stretch *along* the line of sight when
you look down at a shallow angle. If it stays circular, you have not done step 4.

For reference, the solution achieves **1.2–1.3 m RMS** against ground truth at the capstone
geometry. Most of what is left is not yours to fix: see the yaw note below.

## The four things that will bite you

**Use the optical frame, not the camera link frame.** `K^-1 [u, v, 1]^T` produces a ray in
`camera_optical_frame` (z forward, x right, y down). On this gimbal, `camera_link`'s +x points
*backwards*. Read `msg.header.frame_id` and pass it to the TF lookup; never hard-code a frame
name you did not read.

**Look the transform up at the image's timestamp.** `msg.header.stamp`, not
`rclpy.time.Time()`. The gimbal reports at 50 Hz and the image lands 50–150 ms later; during a
30 °/s slew that is 3° of pointing error, which is 1.6 m on the ground at 30 m slant range —
with nothing at all to indicate it went wrong. Try it the naive way once and measure the
difference against ground truth. It is the single most convincing five minutes of the lab.

**λ is the slant range, for free.** Because the bearing ray is a unit vector, the scale factor
that puts it on the ground *is* `R_slant`. The error model `σ_pos ≈ R_slant · σ_angle` then
needs no extra geometry.

**The error is not circular.** An angular error along the line of sight lands on the ground
amplified by `1/sin θ`; the same error across it is not amplified at all. At a 20° look-down
angle that is a factor of three between the two axes. A filter told the error is circular will
trust the along-range direction about three times more than it should.

## Three more things that are true, and measured

**The box centre is not the footprint.** A detection box is centred on the vehicle's body, about
half its height above the ground it stands on, so a ray aimed at it systematically over-ranges.
At 15 m and 45° that was 0.6 m of a 1.2 m total error, and it does not average out. The skeleton
intersects the plane at `ground_z + target_height/2` for you; the parameter is there so you can
turn it off and measure the difference.

**Most of your remaining error is the vehicle's yaw, and you cannot fix it.** EKF2's yaw in this
simulator settles 5–6° away from truth — reproducible with stock PX4 and the stock world, so it
is not your bug and not the course's. A yaw error rotates the bearing ray about the vertical, so
it lands about `0.1 × ground range` to one side: 1.6 m at the capstone geometry, more than the
gimbal, the pixel and the box centre combined. Knowing which term dominates is the point of
step 4. The same is true on the real aircraft, where the compass is the weakest sensor on board.

**Measure the gimbal's sign convention, do not guess it.** Command a rate, watch which way the
box moves, write the answer down. Guessing costs an hour and looks like a broken control loop:
on this gimbal, +0.25 rad/s for 2 s moves it +23.5° and moves the image content **353 px left**,
so positive yaw pulls the image left and a target right of centre needs *positive* yaw. The
skeleton has `yaw_sign` and `pitch_sign` as parameters for exactly this reason.

## Going further

Mini-project 1 in the course notes: plot the position error against altitude and look angle,
and check it really does follow `R_slant · σ_angle`. Then do the same for detection *confidence*
against look angle, and notice that the two curves disagree about where you should fly.
