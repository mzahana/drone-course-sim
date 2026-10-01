#!/usr/bin/env python3
"""Lab 4 reference solution, part 1 -- turn a detection into a position on
the ground, with a covariance.

Your tasks:
    TODO 1  Turn the detection's centre pixel into a unit ray
    TODO 2  Intersect the ray with the ground plane
    TODO 3  Build the covariance of the ground position
Search this file for 'TODO' -- you only need to edit between the
ADD YOUR CODE BELOW / END OF YOUR CODE lines.

What happens to each detection:

    pixel (u, v)
      -> unit ray in the camera optical frame        TODO 1  (slides: step 1)
      -> the same ray in map                         given: TF lookup
      -> point where the ray meets the ground plane  TODO 2  (slides: step 2)
      -> covariance of that point                    TODO 3  (slides: step 4)
      -> published on /target/estimate

Four facts the code relies on:

1. The ray is built in the camera OPTICAL frame: z forward, x right, y down.
   On this gimbal the camera *link* frame has +x pointing backwards, so the
   two frames are different. The code reads the frame name from
   msg.header.frame_id instead of assuming it.

2. The transform is looked up at the IMAGE's timestamp, not at "now".
   The gimbal reports its pose at 50 Hz and the image arrives 50-150 ms
   later. During a 30 deg/s gimbal turn, "now" is about 3 deg off, which is
   1.6 m on the ground at 30 m slant range.

3. The ray r has length 1. So lam, the distance along the ray to the ground,
   is also the slant range (the camera-to-target distance). The position
   error is then about slant_range * sigma_angle, with no extra geometry.

4. The error on the ground is not circular. An angle error along the line
   of sight is stretched by 1/sin(theta), where theta is the look-down
   angle. An angle error across the line of sight is not stretched. At
   20 deg look-down, 1/sin(20 deg) is about 3. If you report a circular
   error, the filter will trust the along-range direction about 3 times
   too much.
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


class TargetLocator(Node):
    def __init__(self):
        super().__init__("target_locator")

        self.declare_parameter("target_classes", ["car", "truck", "bus"])
        self.declare_parameter("ground_z", 0.0)          # m, height of the ground in map
        # A detection box is centred on the vehicle's body, about half the
        # vehicle's height above the ground. A ray through the box centre
        # therefore hits the ground too far away. Measured at 15 m altitude
        # and 45 deg look-down, this caused 0.6 m of the 1.2 m total error,
        # and it does not average out. So we intersect the ray with a plane
        # at half the vehicle's height (self.plane_z) instead of the ground.
        self.declare_parameter("target_height", 1.86)    # m
        self.declare_parameter("sigma_angle_deg", 1.0)   # total angle error: gimbal + attitude + pixel
        self.declare_parameter("min_confidence", 0.35)   # ignore weaker detections
        self.declare_parameter("max_range", 200.0)       # m, ignore fixes farther than this
        self.declare_parameter("tf_timeout", 0.15)       # s, how long to wait for a transform
        self.declare_parameter("publish_marker", True)   # draw the error ellipse in RViz

        self.classes = set(self.get_parameter("target_classes").value)
        self.ground_z = float(self.get_parameter("ground_z").value)
        self.plane_z = self.ground_z + 0.5 * float(self.get_parameter("target_height").value)
        self.sigma_angle = math.radians(float(self.get_parameter("sigma_angle_deg").value))
        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout").value))
        self.want_marker = bool(self.get_parameter("publish_marker").value)

        self.K = None              # camera matrix from /camera/camera_info
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/target/estimate", 10)
        self.pub_marker = self.create_publisher(Marker, "/target/estimate_ellipse", 1)
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "/detections", self._on_detections, 10)

        # Counters for the log message printed every 10 s.
        self.n_out = 0
        self.n_dropped_tf = 0
        self.create_timer(10.0, self._report)

    def _on_info(self, msg):
        # K is the 3x3 camera matrix, stored row by row as 9 numbers.
        self.K = list(msg.k)

    def _report(self):
        if self.n_out or self.n_dropped_tf:
            self.get_logger().info(
                f"{self.n_out} fixes published, {self.n_dropped_tf} dropped for want of a transform")
            self.n_out = self.n_dropped_tf = 0

    def _best(self, msg):
        """Return (detection, score) for the highest-scoring detection of a
        wanted class. Returns (None, min_confidence) if none is good enough."""
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

        # Camera pose in map, at the time the IMAGE was taken, for the frame
        # the image names (the optical frame).
        try:
            tr = self.tf_buffer.lookup_transform(
                "map", msg.header.frame_id, msg.header.stamp, self.tf_timeout)
        except tf2_ros.TransformException:
            self.n_dropped_tf += 1
            return

        # ------------------------------------------------------------------
        # TODO 1: Turn the detection's centre pixel into a unit ray
        #   - self.K is the 3x3 camera matrix stored row by row:
        #     fx = K[0], fy = K[4] (focal lengths in pixels),
        #     cx = K[2], cy = K[5] (image centre in pixels).
        #   - The pixel is u = det.bbox.center.position.x,
        #     v = det.bbox.center.position.y.
        #   - The ray in the optical frame (z forward, x right, y down) is
        #     K^-1 [u, v, 1]^T = ((u - cx) / fx, (v - cy) / fy, 1).
        #   - Divide it by its length so it has length 1. TODO 2 and TODO 3
        #     depend on this.
        #   - Call the result r_opt (a tuple of 3 numbers).
        # ======================= ADD YOUR CODE BELOW =======================
        fx, fy, cx, cy = self.K[0], self.K[4], self.K[2], self.K[5]
        u, v = det.bbox.center.position.x, det.bbox.center.position.y

        # Pixel offset from the centre, divided by the focal length, gives
        # the ray direction. z = 1 means "straight ahead of the camera".
        rx, ry, rz = (u - cx) / fx, (v - cy) / fy, 1.0
        n = math.sqrt(rx * rx + ry * ry + rz * rz)
        r_opt = (rx / n, ry / n, rz / n)
        # ======================= END OF YOUR CODE ==========================

        # Rotate the ray into map: r = R * r_opt. A rotation keeps the
        # length, so r is still a unit vector.
        R = quat_to_rot(tr.transform.rotation)
        r = tuple(sum(R[i][k] * r_opt[k] for k in range(3)) for i in range(3))
        # p_cam = camera position in map.
        t = tr.transform.translation
        p_cam = (t.x, t.y, t.z)

        # z is up in map, so a ray that meets the ground must have r[2] < 0.
        if r[2] > -1e-3:
            return                      # not looking down: no intersection
        # ------------------------------------------------------------------
        # TODO 2: Intersect the ray with the ground plane
        #   - Points on the ray are p_cam + lam * r, where lam is the
        #     distance along the ray.
        #   - Find lam so that the z part equals self.plane_z:
        #     p_cam[2] + lam * r[2] = self.plane_z.
        #   - Because r has length 1, lam is also the slant range.
        #   - Drop the detection (return) if lam <= 0 (the plane is behind the
        #     camera) or lam > self.max_range (the ray is almost horizontal
        #     and the result is not useful).
        #   - Call the ground position px, py (in map).
        # ======================= ADD YOUR CODE BELOW =======================
        lam = (self.plane_z - p_cam[2]) / r[2]
        if lam <= 0.0 or lam > self.max_range:
            return
        px = p_cam[0] + lam * r[0]
        py = p_cam[1] + lam * r[1]
        # ======================= END OF YOUR CODE ==========================

        # ------------------------------------------------------------------
        # TODO 3: Build the covariance of the ground position
        #   - lam is the slant range, so the error across the line of sight
        #     is sigma_cross = lam * self.sigma_angle (metres).
        #   - Along the line of sight the error is larger:
        #     sigma_along = sigma_cross / sin(theta), where theta is the
        #     look-down angle and sin(theta) = -r[2]. Keep sin(theta) >= 1e-3.
        #   - That gives the matrix diag(sigma_along^2, sigma_cross^2) in axes
        #     that point along and across the ray. Rotate it by the ray's
        #     ground bearing az = atan2(r[1], r[0]) to get the map entries
        #     cxx, cyy and cxy.
        #   - The code after this block also uses sigma_along, sigma_cross and
        #     az (for the RViz ellipse), so use those names.
        #   - Tip: start with a circular error, look at the ellipse in RViz at
        #     a shallow look angle, then add the stretch.
        # ======================= ADD YOUR CODE BELOW =======================
        # theta = angle of the ray below the horizontal. r has length 1 and
        # z points up, so sin(theta) = -r[2].
        sin_theta = max(-r[2], 1e-3)               # at least 1e-3: avoid / 0
        sigma_along = lam * self.sigma_angle / sin_theta   # along the line of sight
        sigma_cross = lam * self.sigma_angle               # across it
        az = math.atan2(r[1], r[0])                # ground bearing of the ray
        ca, sa = math.cos(az), math.sin(az)
        a2, c2 = sigma_along ** 2, sigma_cross ** 2
        # C = Rot(az) * diag(a2, c2) * Rot(az)^T, with
        # Rot(az) = [[ca, -sa], [sa, ca]]. Written out element by element:
        cxx = a2 * ca * ca + c2 * sa * sa
        cyy = a2 * sa * sa + c2 * ca * ca
        cxy = (a2 - c2) * ca * sa
        # ======================= END OF YOUR CODE ==========================

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp        # the image's time, not now
        out.header.frame_id = "map"
        out.pose.pose.position.x = px
        out.pose.pose.position.y = py
        out.pose.pose.position.z = self.ground_z
        out.pose.pose.orientation.w = 1.0
        # The ROS covariance is a 6x6 matrix stored row by row, for
        # (x, y, z, roll, pitch, yaw). Index 0 = xx, 1 = xy, 6 = yx, 7 = yy,
        # 14 = zz, and 21, 28, 35 are the three rotation variances.
        cov = [0.0] * 36
        cov[0], cov[1] = cxx, cxy
        cov[6], cov[7] = cxy, cyy
        cov[14] = 0.01                             # z is known: it is the ground
        cov[21] = cov[28] = cov[35] = 1e6          # orientation is not measured
        out.pose.covariance = cov
        self.pub.publish(out)
        self.n_out += 1

        if self.want_marker:
            self._marker(out, sigma_along, sigma_cross, az, score)

    def _marker(self, pose, sa, sc, az, score):
        """Draw the position error as a flat ellipse (a thin cylinder) in RViz.

        sa, sc = sigma along and across the line of sight; az = ground bearing
        of the ray. The ellipse's x axis points along the line of sight.
        """
        m = Marker()
        m.header = pose.header
        m.ns = "target_estimate"
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose = pose.pose.pose
        m.pose.position.z = self.ground_z + 0.05
        # Yaw-only quaternion: rotate the ellipse by az about the z axis.
        m.pose.orientation.z = math.sin(az / 2.0)
        m.pose.orientation.w = math.cos(az / 2.0)
        # scale is the full width, so 4 sigma = 2 sigma on each side.
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
