#!/usr/bin/env python3
"""Lab 4 reference solution, part 2 -- keep the target in the centre of the
image by commanding gimbal rates.

Your tasks:
    TODO 1  Turn the pixel offsets into angle errors
    TODO 2  Run a PID on each axis
Search this file for 'TODO' -- you only need to edit between the
ADD YOUR CODE BELOW / END OF YOUR CODE lines.

How it works:
  * Each detection gives the target's pixel offset (du, dv) from the image
    centre. Dividing by the focal length turns it into an angle error in
    radians (TODO 1).
  * A 30 Hz timer runs one PID per axis (yaw and pitch) on that angle error
    and publishes gimbal rates on /gimbal/cmd/rate (TODO 2).
  * If there is no recent detection, the angle error is computed from the
    filtered target track (/target/track) instead, so the gimbal keeps
    pointing through short dropouts.

Why angles and rates:
  * Working in radians gives the gains a physical meaning: kp = 2.0 means
    2 rad/s of gimbal rate per radian of error (a 0.5 s time constant).
  * A rate command follows directly from the error. An angle command would
    also need the current gimbal angle, for no benefit.
  * On the real X500 the A8 mini gimbal is connected to the onboard computer
    over Ethernet, not to the autopilot. So commanding it from ROS is how the
    real system works, and this node runs unchanged on the real gimbal.

When the gimbal reaches its yaw limit it cannot turn further, so the node
asks the aircraft to turn instead, on /guidance/yaw_rate. Whether anything
acts on that request is up to the other nodes.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray


def quat_to_rot(q):
    """Return the 3x3 rotation matrix (as nested tuples) for quaternion q.

    For a transform from TF, R rotates a vector from the source frame into
    the target frame: v_target = R * v_source. The quaternion is normalised
    first, so a slightly non-unit quaternion still gives a valid rotation.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0   # "or 1.0": avoid / 0
    x, y, z, w = x / n, y / n, z / n, w / n
    # The standard quaternion-to-rotation-matrix formula.
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
            (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)))


class GimbalPointer(Node):
    def __init__(self):
        super().__init__("gimbal_pointer")

        self.declare_parameter("target_classes", ["car", "truck", "bus"])
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("kp", 2.0)                # PID gains, per axis
        self.declare_parameter("ki", 0.15)
        self.declare_parameter("kd", 0.05)
        self.declare_parameter("max_rate", 1.2)          # rad/s, per axis
        self.declare_parameter("i_limit", 0.25)          # limit on the integral (anti-windup)
        self.declare_parameter("deadband_px", 6.0)       # smaller pixel errors count as zero
        self.declare_parameter("lost_timeout", 0.4)      # s without a detection before we use the track
        self.declare_parameter("track_timeout", 3.0)     # s without a track before we stop
        self.declare_parameter("rate_hz", 30.0)
        # yaw_sign and pitch_sign match the gimbal's joint directions to the
        # image directions. They depend on how the gimbal is mounted, so they
        # are parameters: the real A8 mini can be matched without code changes.
        #
        # Measured on this gimbal (target in view, 0.25 rad/s for 2 s):
        #   yaw   +23.5 deg  ->  target moved -353 px in u (image moves LEFT)
        #   pitch +18.4 deg  ->  target moved +326 px in v (image moves DOWN)
        #
        # So a target right of centre (u > cx, positive error) needs POSITIVE
        # yaw to come back: yaw_sign = +1. A target below centre (v > cy)
        # needs NEGATIVE pitch: pitch_sign = -1.
        #
        # To check a new gimbal: command a small rate, see which way the box
        # moves, and write it down.
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pitch_sign", -1.0)

        g = lambda n: self.get_parameter(n).value    # short name for reading a parameter
        self.classes = set(g("target_classes"))
        self.min_conf = float(g("min_confidence"))
        self.kp, self.ki, self.kd = float(g("kp")), float(g("ki")), float(g("kd"))
        self.max_rate = float(g("max_rate"))
        self.i_limit = float(g("i_limit"))
        self.deadband = float(g("deadband_px"))
        self.lost_timeout = float(g("lost_timeout"))
        self.track_timeout = float(g("track_timeout"))
        self.yaw_sign = float(g("yaw_sign"))
        self.pitch_sign = float(g("pitch_sign"))
        period = 1.0 / float(g("rate_hz"))

        self.K = None              # camera matrix from /camera/camera_info
        self.err = None            # (e_yaw, e_pitch) in radians
        self.t_err = None          # time self.err was last set (s)
        self.prev = (0.0, 0.0)     # previous error, for the derivative term
        self.integ = [0.0, 0.0]    # integral of the error, per axis
        self.saturated = (0.0, 0.0, 0.0)   # (roll, pitch, yaw): non-zero = at a limit
        self.track = None          # (t, x, y, z) of the filtered target, in map
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub_rate = self.create_publisher(Vector3Stamped, "/gimbal/cmd/rate", 10)
        self.pub_yaw = self.create_publisher(Float64, "/guidance/yaw_rate", 10)
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "/detections", self._on_det, 10)
        self.create_subscription(Vector3Stamped, "/gimbal/saturated",
                                 self._on_sat, 10)
        self.create_subscription(Odometry, "/target/track", self._on_track, 10)
        self.create_timer(period, self._tick)

    def _now(self):
        """Current ROS time in seconds (sim time in simulation)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, msg):
        # K is the 3x3 camera matrix, stored row by row as 9 numbers.
        self.K = list(msg.k)

    def _on_sat(self, msg):
        self.saturated = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _on_track(self, msg):
        p = msg.pose.pose.position
        self.track = (self._now(), p.x, p.y, p.z)

    def _error_from_track(self):
        """Compute the angle error from the filtered target track instead of
        from a detection. Returns (yaw_error, pitch_error) in radians, or None.

        This keeps the gimbal pointing when detections stop. Without it, the
        gimbal holds its last angle, the target leaves the image, and it is
        not found again. (This happened in testing: the aircraft flew past a
        stationary truck and the mission went to LOST and never recovered.)

        The track position is transformed into the camera optical frame,
        giving p = (x, y, z) with z forward. atan2(x, z) and atan2(y, z) are
        the same angles that TODO 1 computes from pixels, so the same PID and
        the same sign conventions apply.
        """
        if self.track is None or (self._now() - self.track[0]) > self.track_timeout:
            return None
        # rclpy.time.Time() means "the latest transform available".
        try:
            tr = self.tf_buffer.lookup_transform(
                "camera_optical_frame", "map", rclpy.time.Time())
        except Exception:
            return None
        _, tx, ty, tz = self.track
        t = tr.transform.translation
        R = quat_to_rot(tr.transform.rotation)
        # Target position in the optical frame: p = R * p_map + t.
        p = (R[0][0] * tx + R[0][1] * ty + R[0][2] * tz + t.x,
             R[1][0] * tx + R[1][1] * ty + R[1][2] * tz + t.y,
             R[2][0] * tx + R[2][1] * ty + R[2][2] * tz + t.z)
        if p[2] <= 0.1:
            # The target is behind the camera. atan2 would give a misleading
            # angle and the gimbal would turn the wrong way. Instead, ask for
            # a large yaw error (90 deg) towards the side the target is on.
            return (math.copysign(math.pi / 2, p[0] if abs(p[0]) > 1e-6 else 1.0), 0.0)
        return (math.atan2(p[0], p[2]), math.atan2(p[1], p[2]))

    def _on_det(self, msg):
        if self.K is None:
            return
        # Pick the highest-scoring detection of a wanted class.
        best, best_score = None, self.min_conf
        for d in msg.detections:
            for h in d.results:
                if self.classes and h.hypothesis.class_id not in self.classes:
                    continue
                if h.hypothesis.score > best_score:
                    best, best_score = d, h.hypothesis.score
        if best is None:
            return

        # fx, fy = focal lengths in pixels; cx, cy = image centre in pixels.
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        # du, dv = target offset from the image centre, in pixels
        # (du > 0: right of centre, dv > 0: below centre).
        du = best.bbox.center.position.x - cx
        dv = best.bbox.center.position.y - cy
        # Ignore very small errors so the gimbal does not jitter.
        if abs(du) < self.deadband:
            du = 0.0
        if abs(dv) < self.deadband:
            dv = 0.0
        # ------------------------------------------------------------------
        # TODO 1: Turn the pixel offsets into angle errors
        #   - du, dv are the pixel offsets computed above; fx, fy are the
        #     focal lengths in pixels.
        #   - For an offset du, the angle satisfies tan(angle) = du / fx.
        #     Use math.atan2 to get the angle in radians. Same for dv and fy.
        #   - Store the result as self.err = (yaw_error, pitch_error).
        #   - Record the time: self.t_err = self._now().
        # ======================= ADD YOUR CODE BELOW =======================
        # Dividing a pixel error by the focal length gives an angle error.
        self.err = (math.atan2(du, fx), math.atan2(dv, fy))
        self.t_err = self._now()
        # ======================= END OF YOUR CODE ==========================

    def _tick(self):
        now = self._now()
        # fresh = we got a detection within the last lost_timeout seconds.
        fresh = (self.err is not None and self.t_err is not None
                 and (now - self.t_err) <= self.lost_timeout)
        if not fresh:
            # No recent detection: use the filter's predicted target instead.
            self.err = self._error_from_track()
            if self.err is not None:
                self.t_err = now
        if self.err is None:
            # No target at all. Stop the gimbal and reset the PID memory,
            # so the gimbal does not drift away on an old integral.
            self._publish(0.0, 0.0)
            self.integ = [0.0, 0.0]
            self.prev = (0.0, 0.0)
            return

        # ------------------------------------------------------------------
        # TODO 2: Run a PID on each axis
        #   - self.err = (yaw_error, pitch_error) in radians. Loop over both.
        #   - Use dt = 1.0 / 30.0 (the timer runs at 30 Hz).
        #   - Integral: add e * dt to self.integ[i], then limit it to
        #     +/- self.i_limit.
        #   - Derivative: (e - self.prev[i]) / dt.
        #   - Output: u = kp * e + ki * integral + kd * derivative
        #     (gains are self.kp, self.ki, self.kd).
        #   - If |u| > self.max_rate, set u to +/- self.max_rate AND undo this
        #     step's integration (subtract e * dt again). Otherwise the
        #     integral keeps growing while the output is limited, and the
        #     gimbal overshoots when it comes off the limit (this is called
        #     anti-windup).
        #   - Put the results in a list: out = [yaw_rate, pitch_rate].
        #   - At the end, save self.prev = self.err for the next derivative.
        # ======================= ADD YOUR CODE BELOW =======================
        dt = 1.0 / 30.0
        out = []
        for i, e in enumerate(self.err):
            # Integral, limited to +/- i_limit.
            self.integ[i] = max(-self.i_limit,
                                min(self.i_limit, self.integ[i] + e * dt))
            d = (e - self.prev[i]) / dt
            u = self.kp * e + self.ki * self.integ[i] + self.kd * d
            # Limit the output. While it is limited, undo this step's
            # integration so the integral does not keep growing.
            if abs(u) > self.max_rate:
                u = math.copysign(self.max_rate, u)
                self.integ[i] -= e * dt
            out.append(u)
        self.prev = self.err
        # ======================= END OF YOUR CODE ==========================

        # Apply the measured sign conventions (see the parameters above).
        yaw_rate = self.yaw_sign * out[0]
        pitch_rate = self.pitch_sign * out[1]
        self._publish(pitch_rate, yaw_rate)

        # When the gimbal's yaw is at its limit, ask the aircraft to turn at
        # the same rate. Otherwise send 0, so this node does not interfere
        # with the guidance law.
        if abs(self.saturated[2]) > 0.5:
            self.pub_yaw.publish(Float64(data=yaw_rate))
        else:
            self.pub_yaw.publish(Float64(data=0.0))

    def _publish(self, pitch_rate, yaw_rate):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.vector.x = 0.0                 # roll: the gimbal stabilises it; not commandable
        m.vector.y = pitch_rate          # rad/s
        m.vector.z = yaw_rate            # rad/s
        self.pub_rate.publish(m)


def main():
    rclpy.init()
    node = GimbalPointer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
