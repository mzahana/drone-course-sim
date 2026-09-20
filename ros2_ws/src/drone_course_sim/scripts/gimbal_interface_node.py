#!/usr/bin/env python3
"""ROS topic interface to the gimbal.

Why topics and not the MAVLink gimbal manager:

On the real aircraft the SIYI A8 mini hangs off the ONBOARD COMPUTER over
Ethernet, not off the autopilot. The onboard computer points it; PX4 never sees
the commands. This node mirrors that, so a tracking loop written against these
topics runs unchanged on hardware with the SIYI driver underneath.

It is also the only interface that works for tracking. MAVROS exposes gimbal
control as *services*, which block the executor on every call -- unusable in a
30 Hz pixel-error loop.

Interface (angles in radians, Vector3 is x=roll, y=pitch, z=yaw):

  /gimbal/cmd/angle   Vector3Stamped   absolute setpoint; use for "look there"
  /gimbal/cmd/rate    Vector3Stamped   rad/s; use for closed-loop tracking
  /gimbal/attitude    Vector3Stamped   measured, from the gimbal's own joints
  /gimbal/saturated   Vector3Stamped   1.0 on any axis currently against a limit

Roll is not commandable, matching the A8 mini: it is a stabilisation axis.

The rate input is watchdogged. If commands stop arriving the gimbal stops
slewing rather than continuing until it hits a limit -- the exact failure a
crashed tracker would otherwise cause.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Joint names in the model, and which axis each one is.
JOINT_YAW = "cgo3_vertical_arm_joint"
JOINT_ROLL = "cgo3_horizontal_arm_joint"
JOINT_PITCH = "cgo3_camera_joint"


class GimbalInterface(Node):
    def __init__(self):
        super().__init__("gimbal_interface")

        # Travel limits, defaulted to the SIYI A8 mini rather than to what the
        # simulated joints could physically do. Code tuned against limits the
        # real gimbal does not have would not transfer.
        self.declare_parameter("pitch_min", math.radians(-90.0))
        self.declare_parameter("pitch_max", math.radians(25.0))
        self.declare_parameter("yaw_min", math.radians(-135.0))
        self.declare_parameter("yaw_max", math.radians(135.0))
        self.declare_parameter("max_rate", math.radians(90.0))   # rad/s
        self.declare_parameter("rate_timeout", 0.3)              # s
        self.declare_parameter("update_rate", 50.0)              # Hz

        self.pitch_min = self.get_parameter("pitch_min").value
        self.pitch_max = self.get_parameter("pitch_max").value
        self.yaw_min = self.get_parameter("yaw_min").value
        self.yaw_max = self.get_parameter("yaw_max").value
        self.max_rate = self.get_parameter("max_rate").value
        self.rate_timeout = self.get_parameter("rate_timeout").value
        hz = self.get_parameter("update_rate").value

        self.pitch_sp = 0.0
        self.yaw_sp = 0.0
        self.rate_pitch = 0.0
        self.rate_yaw = 0.0
        self.last_rate_msg = None
        self.measured = None
        self.sat = [0.0, 0.0, 0.0]

        self.pub_pitch = self.create_publisher(Float64, "/gimbal/_gz/pitch", 10)
        self.pub_yaw = self.create_publisher(Float64, "/gimbal/_gz/yaw", 10)
        self.pub_roll = self.create_publisher(Float64, "/gimbal/_gz/roll", 10)
        self.pub_att = self.create_publisher(Vector3Stamped, "/gimbal/attitude", 10)
        self.pub_sat = self.create_publisher(Vector3Stamped, "/gimbal/saturated", 10)

        self.create_subscription(Vector3Stamped, "/gimbal/cmd/angle", self.on_angle, 10)
        self.create_subscription(Vector3Stamped, "/gimbal/cmd/rate", self.on_rate, 10)
        # Joint states arrive over the gz bridge with sensor QoS.
        self.create_subscription(
            JointState, "/joint_states", self.on_joints,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10))

        self.dt = 1.0 / hz
        self.create_timer(self.dt, self.update)
        self.get_logger().info(
            "gimbal ready: /gimbal/cmd/angle (absolute), /gimbal/cmd/rate (tracking)")

    # -- inputs ------------------------------------------------------------
    def on_angle(self, msg: Vector3Stamped):
        self.pitch_sp = msg.vector.y
        self.yaw_sp = msg.vector.z
        self.rate_pitch = self.rate_yaw = 0.0   # an absolute command cancels slewing
        self.last_rate_msg = None

    def on_rate(self, msg: Vector3Stamped):
        self.rate_pitch = self._clamp(msg.vector.y, -self.max_rate, self.max_rate)
        self.rate_yaw = self._clamp(msg.vector.z, -self.max_rate, self.max_rate)
        self.last_rate_msg = self.get_clock().now()

    def on_joints(self, msg: JointState):
        j = dict(zip(msg.name, msg.position))
        if JOINT_PITCH in j:
            self.measured = j

    # -- loop --------------------------------------------------------------
    def update(self):
        # Watchdog: a tracker that dies must not leave the gimbal slewing.
        if self.last_rate_msg is not None:
            age = (self.get_clock().now() - self.last_rate_msg).nanoseconds / 1e9
            if age > self.rate_timeout:
                if self.rate_pitch or self.rate_yaw:
                    self.get_logger().warn(
                        f"no rate command for {age:.2f}s - holding position")
                self.rate_pitch = self.rate_yaw = 0.0
                self.last_rate_msg = None

        if self.rate_pitch or self.rate_yaw:
            self.pitch_sp += self.rate_pitch * self.dt
            self.yaw_sp += self.rate_yaw * self.dt

        pitch, sat_p = self._clamp_sat(self.pitch_sp, self.pitch_min, self.pitch_max)
        yaw, sat_y = self._clamp_sat(self.yaw_sp, self.yaw_min, self.yaw_max)
        self.pitch_sp, self.yaw_sp = pitch, yaw
        self.sat = [0.0, sat_p, sat_y]

        self.pub_pitch.publish(Float64(data=float(pitch)))
        self.pub_yaw.publish(Float64(data=float(yaw)))
        self.pub_roll.publish(Float64(data=0.0))

        now = self.get_clock().now().to_msg()
        if self.measured:
            att = Vector3Stamped()
            att.header.stamp = now
            att.header.frame_id = "base_link"
            att.vector.x = float(self.measured.get(JOINT_ROLL, 0.0))
            att.vector.y = float(self.measured.get(JOINT_PITCH, 0.0))
            att.vector.z = float(self.measured.get(JOINT_YAW, 0.0))
            self.pub_att.publish(att)

        s = Vector3Stamped()
        s.header.stamp = now
        s.vector.x, s.vector.y, s.vector.z = self.sat
        self.pub_sat.publish(s)

    # -- helpers -----------------------------------------------------------
    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    @staticmethod
    def _clamp_sat(v, lo, hi):
        if v < lo:
            return lo, 1.0
        if v > hi:
            return hi, 1.0
        return v, 0.0


def main():
    rclpy.init()
    node = GimbalInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
