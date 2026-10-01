# Gimbal adapters — real aircraft only

Student code talks to the gimbal only through the course topics in
[`exercises/INTERFACES.md`](../../exercises/INTERFACES.md):

| Topic | Type | Direction |
|---|---|---|
| `/gimbal/cmd/rate` | `geometry_msgs/Vector3Stamped`, rad/s | command |
| `/gimbal/cmd/angle` | `geometry_msgs/Vector3Stamped`, rad | command |
| `/gimbal/attitude` | `geometry_msgs/Vector3Stamped`, rad | feedback |
| `/gimbal/saturated` | `geometry_msgs/Vector3Stamped`, 1.0 = at a limit | feedback |

`Vector3` is always `x` = roll, `y` = pitch, `z` = yaw. Pitch negative = looking down.
Yaw positive = turning right, seen from above.

In simulation, `gimbal_interface` (in `ros2_ws/src/drone_course_sim`) serves these topics. On the
aircraft, an **adapter** serves them and passes everything on to the gimbal's own ROS driver.
The adapter is the only piece that knows which gimbal is fitted, so **changing the gimbal means
writing a new adapter, and nothing else changes**: student code, the sim, the scoring and the
slides all stay the same.

| Folder | Gimbal | Driver it talks to |
|---|---|---|
| [`siyi_gimbal_adapter/`](siyi_gimbal_adapter/) | SIYI A8 mini | `siyi_ros2` (`/siyi/...`, degrees) |

These packages are **not** built into the simulation image: they depend on the gimbal driver's
message package, which the sim does not need. Build them on the onboard computer, in the same
workspace as the driver.

## Writing an adapter for another gimbal

Copy `siyi_gimbal_adapter/`, then change only what the new driver needs. An adapter must:

1. **Commands:** subscribe to `/gimbal/cmd/rate` and `/gimbal/cmd/angle` and send the driver's
   own rate and angle commands. Ignore `x` (roll is a stabilisation axis, not commanded).
2. **Attitude:** publish `/gimbal/attitude` in radians, with `frame_id` `base_link`, using angles
   measured **relative to the airframe** (joint angles), as the simulation does.
3. **TF:** publish `/joint_states` with the three joint names of the course URDF
   (`cgo3_vertical_arm_joint` = yaw, `cgo3_horizontal_arm_joint` = roll,
   `cgo3_camera_joint` = pitch), so `robot_state_publisher` builds the same TF tree down to
   `camera_optical_frame`.
4. **Limits:** publish `/gimbal/saturated`, 1.0 on an axis that is at its travel limit.
5. **Stopping:** make sure the gimbal stops if rate commands stop arriving (0.3 s in the course).
   If the driver already has a watchdog, rely on it; otherwise add one.
6. **Signs:** have a sign parameter per axis, and check it on the bench (see the SIYI README).

If the new gimbal's travel is not pitch −90° to +25°, yaw ±135°, the course limits in
`INTERFACES.md` and in `gimbal_interface` must change too, or students tune against a gimbal
they will not fly.
