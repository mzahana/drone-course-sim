# Interface contract

**Frozen.** Every lab, the capstone, the scoring script and the reference solutions agree on
these strings. Changing one is a full-stack rework, so it is written down once, here.

The reason this document exists before any lab code does: the whole sim-to-real claim of the
course rests on the student's nodes talking only to topic names whose *meaning* is the same in
simulation and on the real aircraft. Everything that differs between the two lives behind one
of these names.

## Given to you — already running, do not reimplement

| Topic | Type | Frame | Notes |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | `camera_optical_frame` | ~25 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | `camera_optical_frame` | `k` is the 3×3 intrinsics, row-major |
| `/detections` | `vision_msgs/Detection2DArray` | `camera_optical_frame` | YOLO11n, throttled to 10 Hz |
| `/detector/image_annotated` | `sensor_msgs/Image` | `camera_optical_frame` | for looking at, not for code |
| `/detector/blackout` | `std_msgs/Float64` | — | publish N to blind the detector for N seconds |
| `/gimbal/attitude` | `geometry_msgs/Vector3Stamped` | — | measured, rad, `x`=roll `y`=pitch `z`=yaw |
| `/gimbal/saturated` | `geometry_msgs/Vector3Stamped` | — | per-axis: non-zero means that axis is against a limit |
| `/mavros/state` | `mavros_msgs/State` | — | RELIABLE QoS |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | `map` | **BEST_EFFORT QoS** |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | `map` | **BEST_EFFORT QoS** |
| `/tf`, `/tf_static` | | | `map → base_link → … → camera_optical_frame` |

## Written by you

| Topic | Type | Frame | Written by | Lab |
|---|---|---|---|---|
| `/gimbal/cmd/rate` | `geometry_msgs/Vector3Stamped` | — | `gimbal_pointer` | 4 |
| `/gimbal/cmd/angle` | `geometry_msgs/Vector3Stamped` | — | `gimbal_pointer` | 4 |
| `/target/estimate` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | `target_locator` | 4 |
| `/target/track` | `nav_msgs/Odometry` | `map` | `target_tracker` | 5 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | `map` (`FRAME_LOCAL_NED`) | `follow_guidance` | 5 |
| `/mission/state` | `std_msgs/String` | — | `mission_manager` | capstone |
| `/guidance/enable` | `std_msgs/Bool` | — | `mission_manager` → `follow_guidance` | capstone |
| `/guidance/yaw_rate` | `std_msgs/Float64` | — | `gimbal_pointer`, when the gimbal runs out of yaw travel | 4 |
| `/target/estimate_ellipse` | `visualization_msgs/Marker` | `map` | `target_locator`, for RViz | 4 |
| `/guidance/reference` | `geometry_msgs/PoseStamped` | `map` | `follow_guidance`, for RViz | 5 |

### Rules the scoring script assumes

* `/target/estimate` carries the **raw per-detection** ground-plane fix, with a real covariance
  in `pose.covariance` (indices 0, 1, 6, 7 are the xx, xy, yx, yy block). A guessed covariance
  is scored the same as a guessed position, because downstream it *is* one.
* `/target/track` carries the **filtered** estimate, with velocity in `twist.twist.linear`.
  This is the only place the target's velocity exists; the guidance law needs it.
* `/mission/state` is one of `TAKEOFF`, `SEARCH`, `FOLLOW`, `LOST`, `RTL`.
* Scoring reads `/target/track` if it is published, otherwise `/target/estimate`. Publishing
  both is normal and correct.

## Gimbal limits — the same in simulation and on the real A8 mini

| Axis | Range | Commandable |
|---|---|---|
| Roll | stabilised only | **no** |
| Pitch | −90° to +25° | yes |
| Yaw | ±135° | yes |

`Vector3` is always `x`=roll, `y`=pitch, `z`=yaw, in **radians**. A rate command that stops
arriving is treated as zero after 0.3 s — the watchdog is deliberate, so a crashed node parks
the gimbal instead of leaving it slewing.

## Ground truth — scoring only

| Topic | Type |
|---|---|
| `/target/ground_truth` | `nav_msgs/Odometry` |
| `/drone/ground_truth` | `nav_msgs/Odometry` |

**Nothing that flies may read these.** They do not exist on the real aircraft. A mission that
subscribes to one works perfectly in simulation and cannot be transferred, which is precisely
the failure this course is built to make visible. The scoring script reads them; your code
does not.

## QoS

MAVROS publishes vehicle telemetry **best-effort**. A subscriber created with
`create_subscription(..., 10)` asks for RELIABLE, which is stricter than the publisher offers,
so the match is refused — silently, with no error and no warning. Use:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
SENSOR_QOS = QoSProfile(depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
```

`ros2 topic info -v /mavros/local_position/pose` shows the mismatch in seconds.

## Setpoints

`/mavros/setpoint_raw/local` takes position **and** velocity, selected by `type_mask`. The
velocity field is a **feedforward** and carries the target's velocity only. PX4 computes the
proportional term from the position setpoint itself, with `MPC_XY_P` (0.95 by default). Sending
your own `Kp·e` there as well roughly doubles the loop gain and the aircraft overshoots.

PX4 also refuses OFFBOARD unless setpoints are **already** arriving faster than 2 Hz, and drops
out of it if they ever stop for more than about half a second. Stream first, then ask for the
mode.

## Flight geometry

The capstone flies **12 m altitude, 18 m standoff** — a 34° look-down at 21.6 m slant range.
That angle is set by the detector, not by the geometry: measured confidence against look-down
angle is 0.51 at 40°, 0.40 at 45°, 0.27 at 50°, and nothing past 60°, because COCO contains
almost no vehicles photographed from above. Fly steeper and you stop seeing the target; fly
shallower and the along-range error grows as `1/sin θ`.

## Time

Every node runs with `use_sim_time:=true`. Look transforms up at the **message's** timestamp,
never at `rclpy.time.Time()` ("now"): the gimbal reports at 50 Hz and the image arrives
50–150 ms later, so during a 30 °/s slew a "now" lookup is 3° off — 1.6 m on the ground at
30 m slant range, with nothing to indicate anything went wrong.
