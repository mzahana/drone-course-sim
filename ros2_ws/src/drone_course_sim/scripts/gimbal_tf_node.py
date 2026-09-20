#!/usr/bin/env python3
"""Publish the gimbal part of the TF tree from the autopilot's gimbal feedback.

Why this node exists, and why it does not read Gazebo directly:

The geolocation chain needs T_base_link -> camera_optical. In simulation we
*could* take that straight from Gazebo's ground truth, and it would be exact.
We deliberately do not. On the real aircraft there is no ground truth -- the
only thing available is the gimbal's own attitude report, at 50 Hz, with
latency. Building the TF tree from the same signal in both places is what makes
a student's node run unchanged on the real drone.

Frames, and the two conversions that trip everyone up:

  * MAVROS reports the gimbal quaternion in base_link_frd (Forward-Right-Down),
    while the ROS TF tree is FLU (Forward-Left-Up). That is a 180 deg roll.
  * The camera *link* frame (x forward) is not the camera *optical* frame
    (z forward, x right, y down), which is what image geometry expects.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped, Quaternion
from mavros_msgs.msg import GimbalDeviceAttitudeStatus
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def quat_mul(a, b):
    """Hamilton product, (x, y, z, w) ordering."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


# 180 deg about x: converts a rotation expressed in FRD into FLU.
Q_FRD_TO_FLU = (1.0, 0.0, 0.0, 0.0)

# camera_link (x forward, y left, z up) -> camera_optical (z forward, x right,
# y down). The standard REP-103 / REP-105 optical rotation: -90 about z then
# -90 about x.
_h = math.sqrt(2.0) / 2.0
Q_LINK_TO_OPTICAL = (-0.5, 0.5, -0.5, 0.5)


class GimbalTfNode(Node):
    def __init__(self):
        super().__init__("gimbal_tf_node")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("optical_frame", "camera_optical_frame")
        # Where the gimbal rotation centre sits on the airframe. From PX4's
        # x500_gimbal model: the gimbal is merged at z = 0.26 m.
        self.declare_parameter("mount_xyz", [0.0, 0.0, 0.26])

        self.base = self.get_parameter("base_frame").value
        self.cam = self.get_parameter("camera_frame").value
        self.opt = self.get_parameter("optical_frame").value
        self.mount = list(self.get_parameter("mount_xyz").value)

        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_optical_frame()

        # MAVROS publishes telemetry BEST_EFFORT. Subscribing RELIABLE here is
        # the single most common reason a student sees no data at all.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            GimbalDeviceAttitudeStatus,
            "/mavros/gimbal_control/device/attitude_status",
            self.on_attitude,
            qos,
        )
        self.get_logger().info(
            f"publishing {self.base} -> {self.cam} -> {self.opt} from gimbal feedback"
        )

    def _publish_optical_frame(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.cam
        t.child_frame_id = self.opt
        t.transform.rotation = Quaternion(
            x=Q_LINK_TO_OPTICAL[0], y=Q_LINK_TO_OPTICAL[1],
            z=Q_LINK_TO_OPTICAL[2], w=Q_LINK_TO_OPTICAL[3],
        )
        self.static_tf.sendTransform(t)

    def on_attitude(self, msg: GimbalDeviceAttitudeStatus):
        q_frd = (msg.q.x, msg.q.y, msg.q.z, msg.q.w)
        q_flu = quat_mul(Q_FRD_TO_FLU, q_frd)

        t = TransformStamped()
        # Stamp with the message time, not "now". Everything downstream looks
        # transforms up by image timestamp; using wall time here reintroduces
        # exactly the skew the lab is trying to teach away.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.base
        t.child_frame_id = self.cam
        t.transform.translation.x = self.mount[0]
        t.transform.translation.y = self.mount[1]
        t.transform.translation.z = self.mount[2]
        t.transform.rotation = Quaternion(
            x=q_flu[0], y=q_flu[1], z=q_flu[2], w=q_flu[3]
        )
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = GimbalTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
