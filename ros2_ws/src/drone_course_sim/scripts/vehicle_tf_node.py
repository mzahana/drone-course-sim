#!/usr/bin/env python3
"""Publish map -> base_link from the autopilot's local position estimate.

Why this is not left to MAVROS. MAVROS's local_position plugin can publish this
transform itself (`tf.send`), but its plugins run as sub-nodes that do NOT
receive the parameters file -- `use_sim_time` included. The transform is then
stamped in wall time while every other transform in the tree is on simulator
time, so the tree spans two clocks and every lookup fails with a frame that
"does not exist". Publishing it here puts it on the same clock as the rest.

It is also the honest arrangement. On the real aircraft this transform is an
*estimate* from EKF2 arriving over MAVLink, not ground truth, and the pose topic
is exactly what a student's node would consume there too.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class VehicleTf(Node):
    def __init__(self):
        super().__init__("vehicle_tf")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.tf = TransformBroadcaster(self)
        self.count = 0

        # MAVROS publishes telemetry BEST_EFFORT. Subscribing RELIABLE is the
        # single most common reason a student sees an empty topic.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.on_pose, qos)
        self.create_timer(5.0, self._nag)
        self.get_logger().info(
            f"publishing {self.map_frame} -> {self.base_frame} from local_position/pose")

    def _nag(self):
        if self.count == 0:
            self.get_logger().warn(
                "no /mavros/local_position/pose yet - is MAVROS connected and "
                "does the vehicle have a valid position estimate?")

    def on_pose(self, msg: PoseStamped):
        t = TransformStamped()
        # Carry the pose's own stamp. Everything downstream looks transforms up
        # by image timestamp; restamping with "now" reintroduces the skew the
        # course spends a lecture teaching away.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf.sendTransform(t)
        self.count += 1


def main():
    rclpy.init()
    node = VehicleTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
