# siyi_gimbal_adapter

Connects the course gimbal topics to the `siyi_ros2` driver on the real aircraft, so
`gimbal_pointer` and `target_locator` run unchanged on the SIYI A8 mini.

```
gimbal_pointer ──/gimbal/cmd/rate (rad/s)──▶ adapter ──/siyi/cmd/rate (deg/s)──────▶ siyi_ros2 ──UDP──▶ A8 mini
               ──/gimbal/cmd/angle (rad)───▶         ──/siyi/cmd/attitude (deg)──▶
               ◀─/gimbal/attitude (rad)────          ◀─/siyi/attitude (deg)───────
               ◀─/gimbal/saturated─────────          ◀─/siyi/saturation───────────
robot_state_publisher ◀─/joint_states (rad)─
```

| Course side | SIYI side | Conversion |
|---|---|---|
| `/gimbal/cmd/rate`, `Vector3Stamped` y, z | `/siyi/cmd/rate`, `GimbalRateCmd` | rad/s → deg/s, × sign |
| `/gimbal/cmd/angle`, `Vector3Stamped` y, z | `/siyi/cmd/attitude`, `GimbalAttitudeCmd` | rad → deg, × sign |
| `/gimbal/attitude`, `Vector3Stamped` x, y, z | `/siyi/attitude`, `GimbalAttitude` | deg → rad, × sign |
| `/gimbal/saturated` | `/siyi/attitude` + `/siyi/saturation` | 1.0 when within 1° of a limit, or the driver blocked a command in the last 0.3 s |
| `/joint_states` (yaw, roll, pitch joints) | `/siyi/attitude` | deg → rad, × sign |

It has no watchdog of its own: if `gimbal_pointer` stops, nothing is forwarded, and the
`siyi_ros2` watchdog sets the rate to zero after 200 ms.

## Build (onboard computer, same workspace as siyi_ros2)

```bash
cd ~/ros2_ws/src
ln -s ~/drone-course-sim/hardware/gimbal_adapters/siyi_gimbal_adapter .   # or copy it
cd ~/ros2_ws && colcon build --packages-select siyi_msgs siyi_gimbal_adapter
source install/setup.bash
```

## Run

```bash
ros2 launch siyi_ros2 siyi.launch.py                      # the driver
ros2 launch siyi_gimbal_adapter siyi_adapter.launch.py    # this adapter
```

`robot_state_publisher` with the course URDF (`drone_course_sim/urdf/x500_course_gimbal.urdf.xacro`)
turns `/joint_states` into the TF tree. The adapter prints a warning every time `/siyi/attitude`
stops arriving for 1 s.

## Bench check before the first flight — props off

The default signs are all +1, because the course and `siyi_ros2` use the same conventions
(pitch negative = down, yaw positive = right). That is read from the code on both sides; it
must be **confirmed on the real gimbal** once, and again after any firmware update or remount.

1. Start the driver and the adapter. Watch the attitude:
   `ros2 topic echo /gimbal/attitude`
2. **Pitch:** command a small upward rate for about 1 s:
   ```bash
   ros2 topic pub -r 20 -t 20 /gimbal/cmd/rate geometry_msgs/msg/Vector3Stamped "{vector: {y: 0.2}}"
   ```
   The camera must tilt **up**, and `vector.y` must **increase**. If it tilts down, set
   `pitch_sign: -1.0` in `config/siyi_adapter.yaml`.
3. **Yaw:** the same with `{vector: {z: 0.2}}`. The camera must turn **right** (seen from
   above, nose away from you), and `vector.z` must **increase**. If not, set `yaw_sign: -1.0`.
4. **Stop:** press Ctrl-C during a command. The gimbal must stop within about 0.2 s.
5. **Straight down:** `ros2 topic pub --once /gimbal/cmd/angle geometry_msgs/msg/Vector3Stamped "{vector: {y: -1.5708}}"`.
   The camera must look straight down and `vector.y` must read close to −1.571.
6. **TF:** `ros2 run tf2_ros tf2_echo base_link camera_optical_frame`. With the camera
   straight down, the optical `z` axis (the viewing direction) must point along `base_link` −z.

Write the result in the hardware checklist.

## Known limitation: which angles SIYI reports

The course TF tree needs **joint angles**, meaning angles relative to the airframe. That is what
the simulation publishes. SIYI's attitude message may instead report pitch and roll relative to
the horizon, because the gimbal is stabilised. If it does, the TF tree on the aircraft is off by
the aircraft's own tilt, which is a few degrees in forward flight, and ground positions
from `target_locator` move by roughly the ground range × that tilt.

**Check it:** with the gimbal at pitch −45°, tilt the whole aircraft nose-down by about 10° by
hand. If `/siyi/attitude` pitch stays near −45°, the angles are horizon-referenced. If it
moves to about −35°, they are joint angles and nothing needs to change.

**If they are horizon-referenced**, the fix belongs in the driver. The SIYI protocol has a
magnetic-encoder command (0x26) that reports true joint angles, and `siyi_sdk` already supports
it (`client.get_magnetic_encoder()`), but `siyi_ros2` does not publish it yet. Publishing it as
a topic and pointing this adapter's `/joint_states` at it is a small change.
