# Lab 3 — Offboard flight

**Where this goes:** `~/shared_volume/ros2_ws/src/lab3_offboard/`
(on your host machine: `~/drone_course_shared_volume/ros2_ws/src/lab3_offboard/`).
`course new lab3` put it there.

**What you write:** `scripts/offboard_square.py` — four `TODO(student)` blocks.

## What it must do

Arm the aircraft, put it in OFFBOARD, take off to 5 m, fly a 10 m square, then fly a circle
on **velocity** setpoints, then come back and hold.

## Run it

Three terminals inside the container.

```bash
course sim                                   # 1: PX4 SITL + Gazebo
course bringup tier:=0                       # 2: MAVROS, bridges, TF
                                             # 3:
cd ~/shared_volume/ros2_ws && colcon build --packages-select lab3_offboard
source install/setup.bash
ros2 launch lab3_offboard offboard_square.launch.py
```

## Acceptance test

```bash
ros2 topic echo /mavros/state --once          # mode: OFFBOARD, armed: true
ros2 topic hz /mavros/setpoint_raw/local      # >= 20 Hz, never drops below 2
```

You are done when the aircraft flies all four corners and the circle without PX4 kicking it
out of OFFBOARD.

## The two things that will bite you

**OFFBOARD is refused until setpoints are already flowing.** PX4 wants them arriving faster
than 2 Hz *before* the mode change, and it will drop out of OFFBOARD if they ever stop for
more than about half a second. The skeleton already streams from a timer; ask for the mode
from inside that same timer and the ordering cannot go wrong.

**`/mavros/local_position/pose` is published BEST_EFFORT.** A subscriber created with
`create_subscription(..., 10)` asks for RELIABLE, which is stricter than the publisher
offers, so the match is refused — silently, with no error. The skeleton already uses
`SENSOR_QOS`; `ros2 topic info -v /mavros/local_position/pose` is how you would have found it.

## Going further

Fly the square on velocity setpoints alone, with no position term. Watch where it ends up
after four corners, and say why.
