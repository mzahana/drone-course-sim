#!/usr/bin/env python3
"""Lab 4 skeleton, part 2 -- keep the target in the middle of the frame.

A pixel error is an *angle* error once you divide by the focal length, so the
loop is closed in radians and the gains mean something physical: a P gain of
2.0 means "command 2 rad/s of gimbal rate per radian of pointing error", i.e. a
half-second time constant.

Rate control, not angle control, and not through the autopilot. Two reasons,
both of which survive contact with the real aircraft:

  * A rate command is what a visual servo produces naturally. Converting the
    pixel error to an absolute angle needs the current gimbal angle, so an
    angle loop closes a loop around a loop for no benefit.
  * On the real X500 the A8 mini hangs off the onboard computer over Ethernet.
    The autopilot never sees these commands. Driving it from ROS here is the
    faithful arrangement, not a simulation shortcut -- and it is why the same
    node works on Day 4 with the real gimbal, unchanged.

When an axis runs out of travel the gimbal cannot help any more and the
*aircraft* has to turn. That request goes out on /guidance/yaw_rate; whether
anything acts on it is the mission manager's business, not this node's.
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
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
            (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)))


class GimbalPointer(Node):
    def __init__(self):
        super().__init__("gimbal_pointer")

        self.declare_parameter("target_classes", ["car", "truck", "bus"])
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("ki", 0.15)
        self.declare_parameter("kd", 0.05)
        self.declare_parameter("max_rate", 1.2)          # rad/s, per axis
        self.declare_parameter("i_limit", 0.25)          # rad, anti-windup
        self.declare_parameter("deadband_px", 6.0)
        self.declare_parameter("lost_timeout", 0.4)      # s before we coast on the track
        self.declare_parameter("track_timeout", 3.0)     # s before we give up entirely
        self.declare_parameter("rate_hz", 30.0)
        # Sign conventions between the optical frame and the gimbal joints are
        # a property of the mount, not of the maths. They are parameters so
        # that the real A8 mini can be matched without touching the code.
        #
        # MEASURED on this gimbal, target held in view, 0.25 rad/s for 2 s:
        #   yaw   command +23.5 deg  ->  u moved -353 px   (positive yaw pulls
        #                                the image content LEFT)
        #   pitch command +18.4 deg  ->  v moved +326 px
        #
        # So to drive a target that is right of centre (u > cx, positive error)
        # back to the centre, yaw must be POSITIVE: yaw_sign = +1. To drive one
        # that is below centre (v > cy) back up, pitch must be negative.
        #
        # Guessing these is not worth the hour it costs. Command a rate, watch
        # which way the box moves, write the answer down.
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("pitch_sign", -1.0)

        g = lambda n: self.get_parameter(n).value
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

        self.K = None
        self.err = None            # (e_yaw, e_pitch) in radians
        self.t_err = None
        self.prev = (0.0, 0.0)
        self.integ = [0.0, 0.0]
        self.saturated = (0.0, 0.0, 0.0)
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
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_info(self, msg):
        self.K = list(msg.k)

    def _on_sat(self, msg):
        self.saturated = (msg.vector.x, msg.vector.y, msg.vector.z)

    def _on_track(self, msg):
        p = msg.pose.pose.position
        self.track = (self._now(), p.x, p.y, p.z)

    def _error_from_track(self):
        """The same error the pixels would have given, computed from the filter.

        This is the whole reason the filter exists. A pointer that only reacts
        to detections stops pointing the moment it stops seeing -- and since it
        then never looks back, one lost frame becomes a lost target for good.
        Measured before this existed: the aircraft flew past a stationary truck,
        the gimbal held its last angle, and the mission went to LOST and never
        recovered.

        Projecting the predicted target into the optical frame gives exactly the
        quantity the pixel loop produces -- atan2(x, z) and atan2(y, z) are the
        pixel errors divided by the focal length -- so it feeds the same PID,
        with the same sign conventions, and nothing downstream has to know
        which source it came from.
        """
        if self.track is None or (self._now() - self.track[0]) > self.track_timeout:
            return None
        try:
            tr = self.tf_buffer.lookup_transform(
                "camera_optical_frame", "map", rclpy.time.Time())
        except Exception:
            return None
        _, tx, ty, tz = self.track
        t = tr.transform.translation
        R = quat_to_rot(tr.transform.rotation)
        p = (R[0][0] * tx + R[0][1] * ty + R[0][2] * tz + t.x,
             R[1][0] * tx + R[1][1] * ty + R[1][2] * tz + t.y,
             R[2][0] * tx + R[2][1] * ty + R[2][2] * tz + t.z)
        if p[2] <= 0.1:
            # Behind the camera: atan2 would fold the error back on itself and
            # the gimbal would confidently slew the wrong way. Command a hard
            # turn toward whichever side it went out on instead.
            return (math.copysign(math.pi / 2, p[0] if abs(p[0]) > 1e-6 else 1.0), 0.0)
        return (math.atan2(p[0], p[2]), math.atan2(p[1], p[2]))

    def _on_det(self, msg):
        if self.K is None:
            return
        best, best_score = None, self.min_conf
        for d in msg.detections:
            for h in d.results:
                if self.classes and h.hypothesis.class_id not in self.classes:
                    continue
                if h.hypothesis.score > best_score:
                    best, best_score = d, h.hypothesis.score
        if best is None:
            return

        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        du = best.bbox.center.position.x - cx
        dv = best.bbox.center.position.y - cy
        if abs(du) < self.deadband:
            du = 0.0
        if abs(dv) < self.deadband:
            dv = 0.0
        # TODO(student): turn the pixel offsets du, dv into ANGLE errors in radians.
        # Dividing by the focal length is what makes the gains
        # physical: a P gain of 2.0 then means 2 rad/s of gimbal
        # rate per radian of pointing error.
        # Store them in self.err as (yaw_error, pitch_error) and
        # record self.t_err = self._now().
        raise NotImplementedError

    def _tick(self):
        now = self._now()
        fresh = (self.err is not None and self.t_err is not None
                 and (now - self.t_err) <= self.lost_timeout)
        if not fresh:
            # No detection this instant: fall back to the filter's prediction.
            self.err = self._error_from_track()
            if self.err is not None:
                self.t_err = now
        if self.err is None:
            # Nothing at all. Stop slewing and drop the integrator, or the
            # gimbal walks away while nobody is looking at it.
            self._publish(0.0, 0.0)
            self.integ = [0.0, 0.0]
            self.prev = (0.0, 0.0)
            return

        # TODO(student): a PID per axis, producing out = [yaw_rate, pitch_rate].
        # self.kp, self.ki, self.kd, self.max_rate, self.i_limit.
        # Two things that are not optional:
        #   * clamp the integrator to +/- self.i_limit, and
        #   * when the output clamps at max_rate, do not keep
        #     integrating into the clamp (undo that step's
        #     contribution). Without this the gimbal overshoots
        #     every time it comes off a limit.
        # Remember self.prev for the derivative term.
        raise NotImplementedError

        yaw_rate = self.yaw_sign * out[0]
        pitch_rate = self.pitch_sign * out[1]
        self._publish(pitch_rate, yaw_rate)

        # Out of yaw travel: the gimbal has done all it can, so ask the
        # aircraft to turn. Publishing the request unconditionally would fight
        # the guidance law, which is why it is gated on saturation.
        if abs(self.saturated[2]) > 0.5:
            self.pub_yaw.publish(Float64(data=yaw_rate))
        else:
            self.pub_yaw.publish(Float64(data=0.0))

    def _publish(self, pitch_rate, yaw_rate):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.vector.x = 0.0                 # roll is stabilisation, not an input
        m.vector.y = pitch_rate
        m.vector.z = yaw_rate
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
