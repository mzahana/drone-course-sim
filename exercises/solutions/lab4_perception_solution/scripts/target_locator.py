#!/usr/bin/env python3
"""Lab 4 reference solution, part 1 -- a detection becomes a place.

    pixel  ->  bearing ray  ->  ground-plane intersection  ->  position in map

and, just as importantly, a covariance that is derived rather than guessed.

Three things here are the actual lesson:

1. **The ray comes out of the OPTICAL frame** (z forward, x right, y down).
   The camera *link* frame on this gimbal has +x pointing backwards. Reading
   `header.frame_id` instead of assuming it is the difference between a correct
   answer and a plausible wrong one that never raises an error.

2. **The transform is looked up at the image's timestamp**, not at "now". The
   gimbal reports at 50 Hz and the image lands 50-150 ms later; during a 30 deg/s
   slew, "now" is 3 deg off, which is 1.6 m on the ground at 30 m slant range.

3. **The slant range falls out of the intersection for free.** Because the ray
   is a unit vector, the scale factor lambda that puts it on the ground *is* the
   slant range -- so the error model sigma_pos ~ R_slant * sigma_angle needs no
   extra geometry.

The covariance is deliberately anisotropic. An angular error along the
line-of-sight direction lands on the ground amplified by 1/sin(theta); the same
angular error across it does not. Looking down at 20 deg, that is a factor of
three between the two axes, and a filter told the error is circular will trust
the along-range direction about three times more than it should.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
            (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)))


class TargetLocator(Node):
    def __init__(self):
        super().__init__("target_locator")

        self.declare_parameter("target_classes", ["car", "truck", "bus"])
        self.declare_parameter("ground_z", 0.0)
        # A detection box is centred on the vehicle's BODY, roughly half its
        # height above the ground it stands on -- so intersecting the ground
        # plane with a ray aimed at the box centre systematically over-ranges.
        # Measured at 15 m altitude and 45 deg look-down: 0.6 m of the 1.2 m
        # total error was this one effect, and it does not average out.
        # Intersecting the plane through the middle of the vehicle instead
        # costs one line and removes it.
        self.declare_parameter("target_height", 1.86)
        self.declare_parameter("sigma_angle_deg", 1.0)   # gimbal + attitude + pixel, total
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("max_range", 200.0)
        self.declare_parameter("tf_timeout", 0.15)
        self.declare_parameter("publish_marker", True)

        self.classes = set(self.get_parameter("target_classes").value)
        self.ground_z = float(self.get_parameter("ground_z").value)
        self.plane_z = self.ground_z + 0.5 * float(self.get_parameter("target_height").value)
        self.sigma_angle = math.radians(float(self.get_parameter("sigma_angle_deg").value))
        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout").value))
        self.want_marker = bool(self.get_parameter("publish_marker").value)

        self.K = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/target/estimate", 10)
        self.pub_marker = self.create_publisher(Marker, "/target/estimate_ellipse", 1)
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "/detections", self._on_detections, 10)

        self.n_out = 0
        self.n_dropped_tf = 0
        self.create_timer(10.0, self._report)

    def _on_info(self, msg):
        self.K = list(msg.k)

    def _report(self):
        if self.n_out or self.n_dropped_tf:
            self.get_logger().info(
                f"{self.n_out} fixes published, {self.n_dropped_tf} dropped for want of a transform")
            self.n_out = self.n_dropped_tf = 0

    def _best(self, msg):
        best, best_score = None, self.min_conf
        for d in msg.detections:
            for h in d.results:
                if self.classes and h.hypothesis.class_id not in self.classes:
                    continue
                if h.hypothesis.score > best_score:
                    best, best_score = d, h.hypothesis.score
        return best, best_score

    def _on_detections(self, msg):
        if self.K is None or not msg.detections:
            return
        det, score = self._best(msg)
        if det is None:
            return

        # The transform at the IMAGE's stamp, in the IMAGE's frame.
        try:
            tr = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, msg.header.stamp, self.tf_timeout)
        except tf2_ros.TransformException:
            self.n_dropped_tf += 1
            return

        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        u, v = det.bbox.center.position.x, det.bbox.center.position.y

        # Step 1: a pixel is a direction. In the optical frame, z is forward.
        rx, ry, rz = (u - cx) / fx, (v - cy) / fy, 1.0
        n = math.sqrt(rx * rx + ry * ry + rz * rz)
        r_opt = (rx / n, ry / n, rz / n)

        R = quat_to_rot(tr.transform.rotation)
        r = tuple(sum(R[i][k] * r_opt[k] for k in range(3)) for i in range(3))
        t = tr.transform.translation
        p_cam = (t.x, t.y, t.z)

        # Step 2: the ground supplies the missing dimension.
        if r[2] > -1e-3:
            return                      # not looking down; there is no intersection
        lam = (self.plane_z - p_cam[2]) / r[2]
        if lam <= 0.0 or lam > self.max_range:
            return
        px = p_cam[0] + lam * r[0]
        py = p_cam[1] + lam * r[1]

        # Step 4: how wrong is this? lambda is the slant range, because r is a
        # unit vector -- so the error model needs nothing else.
        sin_theta = max(-r[2], 1e-3)               # depression angle of the ray
        sigma_along = lam * self.sigma_angle / sin_theta
        sigma_cross = lam * self.sigma_angle
        az = math.atan2(r[1], r[0])                # ground bearing of the ray
        ca, sa = math.cos(az), math.sin(az)
        a2, c2 = sigma_along ** 2, sigma_cross ** 2
        cxx = a2 * ca * ca + c2 * sa * sa
        cyy = a2 * sa * sa + c2 * ca * ca
        cxy = (a2 - c2) * ca * sa

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp        # the image's time, not now
        out.header.frame_id = "map"
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = self.ground_z
        out.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[0], cov[1] = cxx, cxy
        cov[6], cov[7] = cxy, cyy
        cov[14] = 0.01                             # z is known: it is the ground
        cov[21] = cov[28] = cov[35] = 1e6          # orientation is not observed
        out.pose.covariance = cov
        self.pub.publish(out)
        self.n_out += 1

        if self.want_marker:
            self._marker(out, sigma_along, sigma_cross, az, score)

    def _marker(self, pose, sa, sc, az, score):
        m = Marker()
        m.header = pose.header
        m.ns = "target_estimate"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose = pose.pose.pose
        m.pose.position.z = self.ground_z + 0.05
        m.pose.orientation.z = math.sin(az / 2.0)
        m.pose.orientation.w = math.cos(az / 2.0)
        # 2 sigma, so the ellipse drawn is the 95% region a reader expects.
        m.scale.x = max(4.0 * sa, 0.2)
        m.scale.y = max(4.0 * sc, 0.2)
        m.scale.z = 0.05
        m.color.r, m.color.g, m.color.b, m.color.a = 0.9, 0.5, 0.05, 0.45
        m.lifetime.sec = 1
        self.pub_marker.publish(m)


def main():
    rclpy.init()
    node = TargetLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
