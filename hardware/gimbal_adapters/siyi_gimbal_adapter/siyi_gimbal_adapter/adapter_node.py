#!/usr/bin/env python3
"""Course gimbal topics <-> siyi_ros2 driver, for the real aircraft.

Student code talks to the gimbal only through the course topics (radians,
Vector3 x=roll, y=pitch, z=yaw). In simulation, gimbal_interface serves them.
On the aircraft, this node serves them and passes everything on to the
siyi_ros2 driver, which talks to the A8 mini over Ethernet. Changing the
gimbal means writing a new adapter like this one; student code stays the same.

  course side (radians)                    siyi_ros2 side (degrees)
  /gimbal/cmd/rate   Vector3Stamped   ->   /siyi/cmd/rate      GimbalRateCmd
  /gimbal/cmd/angle  Vector3Stamped   ->   /siyi/cmd/attitude  GimbalAttitudeCmd
  /gimbal/attitude   Vector3Stamped   <-   /siyi/attitude      GimbalAttitude
  /gimbal/saturated  Vector3Stamped   <-   /siyi/attitude + /siyi/saturation
  /joint_states      JointState       <-   /siyi/attitude  (for robot_state_publisher)

/joint_states carries the same three joint names as the simulated gimbal, so
robot_state_publisher with the course URDF builds the same TF tree
(base_link -> ... -> camera_optical_frame) on the aircraft as in simulation.

Roll is not commandable (it is a stabilisation axis): the x of a command is
ignored, as in simulation.

No watchdog here on purpose: if student commands stop, this node stops
forwarding them, and the siyi_ros2 watchdog zeroes the rate (200 ms).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from siyi_msgs.msg import (GimbalAttitude, GimbalAttitudeCmd, GimbalRateCmd,
                           GimbalSaturation)

from siyi_gimbal_adapter.convert import at_limit, attitude_to_course, cmd_to_siyi


class SiyiGimbalAdapter(Node):
    def __init__(self):
        super().__init__("siyi_gimbal_adapter")

        # Sign of each axis, SIYI relative to the course. +1 means "same
        # direction". Check on the bench (README.md) before the first flight.
        self.declare_parameter("roll_sign", 1.0)
        self.declare_parameter("pitch_sign", 1.0)
        self.declare_parameter("yaw_sign", 1.0)
        # Travel limits, used for /gimbal/saturated. Same values as the
        # simulated gimbal and the siyi_ros2 defaults.
        self.declare_parameter("pitch_min_deg", -90.0)
        self.declare_parameter("pitch_max_deg", 25.0)
        self.declare_parameter("yaw_min_deg", -135.0)
        self.declare_parameter("yaw_max_deg", 135.0)
        self.declare_parameter("limit_margin_deg", 1.0)  # "at the limit" if this close
        self.declare_parameter("saturation_hold", 0.3)   # s to keep a driver saturation flag
        self.declare_parameter("attitude_timeout", 1.0)  # s without attitude before we warn
        # Joint names of the course URDF (yaw, roll, pitch joints).
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_yaw", "cgo3_vertical_arm_joint")
        self.declare_parameter("joint_roll", "cgo3_horizontal_arm_joint")
        self.declare_parameter("joint_pitch", "cgo3_camera_joint")

        g = lambda n: self.get_parameter(n).value    # short name for reading a parameter
        self.signs = dict(roll_sign=float(g("roll_sign")),
                          pitch_sign=float(g("pitch_sign")),
                          yaw_sign=float(g("yaw_sign")))
        self.pitch_lim = (float(g("pitch_min_deg")), float(g("pitch_max_deg")))
        self.yaw_lim = (float(g("yaw_min_deg")), float(g("yaw_max_deg")))
        self.margin = float(g("limit_margin_deg"))
        self.sat_hold = float(g("saturation_hold"))
        self.att_timeout = float(g("attitude_timeout"))
        self.publish_joints = bool(g("publish_joint_states"))
        self.joint_names = [g("joint_yaw"), g("joint_roll"), g("joint_pitch")]

        self.t_att = None           # time of the last /siyi/attitude (s)
        self.t_drv_sat = -1e9       # time of the last /siyi/saturation (s)
        self.drv_sat = (False, False)   # (pitch, yaw) from the driver
        self.warned = False

        # To the driver. siyi_ros2 subscribes with sensor-data QoS.
        self.pub_rate = self.create_publisher(GimbalRateCmd, "/siyi/cmd/rate",
                                              qos_profile_sensor_data)
        self.pub_angle = self.create_publisher(GimbalAttitudeCmd, "/siyi/cmd/attitude",
                                               qos_profile_sensor_data)
        # To student code and robot_state_publisher: same QoS as in simulation.
        self.pub_att = self.create_publisher(Vector3Stamped, "/gimbal/attitude", 10)
        self.pub_sat = self.create_publisher(Vector3Stamped, "/gimbal/saturated", 10)
        self.pub_joints = self.create_publisher(JointState, "/joint_states", 10)

        self.create_subscription(Vector3Stamped, "/gimbal/cmd/rate", self.on_rate, 10)
        self.create_subscription(Vector3Stamped, "/gimbal/cmd/angle", self.on_angle, 10)
        # siyi_ros2 publishes attitude best-effort (sensor-data QoS).
        self.create_subscription(GimbalAttitude, "/siyi/attitude", self.on_attitude,
                                 qos_profile_sensor_data)
        self.create_subscription(GimbalSaturation, "/siyi/saturation", self.on_driver_sat, 10)

        self.create_timer(0.5, self.check_attitude)
        self.get_logger().info(
            f"adapter ready: /gimbal/... <-> /siyi/..., signs {self.signs}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # -- commands: course -> driver ---------------------------------------
    def on_rate(self, msg: Vector3Stamped):
        yaw, pitch = cmd_to_siyi(msg.vector.y, msg.vector.z,
                                 self.signs["pitch_sign"], self.signs["yaw_sign"])
        out = GimbalRateCmd()
        out.header = msg.header
        out.yaw_rate_dps = float(yaw)
        out.pitch_rate_dps = float(pitch)
        self.pub_rate.publish(out)

    def on_angle(self, msg: Vector3Stamped):
        yaw, pitch = cmd_to_siyi(msg.vector.y, msg.vector.z,
                                 self.signs["pitch_sign"], self.signs["yaw_sign"])
        out = GimbalAttitudeCmd()
        out.header = msg.header
        out.yaw_deg = float(yaw)
        out.pitch_deg = float(pitch)
        self.pub_angle.publish(out)

    # -- feedback: driver -> course ---------------------------------------
    def on_driver_sat(self, msg: GimbalSaturation):
        # The driver publishes this only when it blocks a command at a limit.
        self.drv_sat = (msg.pitch_saturated, msg.yaw_saturated)
        self.t_drv_sat = self._now()

    def on_attitude(self, msg: GimbalAttitude):
        self.t_att = self._now()
        roll, pitch, yaw = attitude_to_course(msg.roll_deg, msg.pitch_deg, msg.yaw_deg,
                                              **self.signs)
        stamp = msg.header.stamp

        att = Vector3Stamped()
        att.header.stamp = stamp
        att.header.frame_id = "base_link"    # same as in simulation
        att.vector.x, att.vector.y, att.vector.z = roll, pitch, yaw
        self.pub_att.publish(att)

        # Saturated: the measured angle is at a limit, or the driver said so
        # in the last saturation_hold seconds.
        recent = (self._now() - self.t_drv_sat) < self.sat_hold
        sat_pitch = at_limit(msg.pitch_deg, *self.pitch_lim, self.margin) or \
            (recent and self.drv_sat[0])
        sat_yaw = at_limit(msg.yaw_deg, *self.yaw_lim, self.margin) or \
            (recent and self.drv_sat[1])
        sat = Vector3Stamped()
        sat.header.stamp = stamp
        sat.vector.x = 0.0
        sat.vector.y = 1.0 if sat_pitch else 0.0
        sat.vector.z = 1.0 if sat_yaw else 0.0
        self.pub_sat.publish(sat)

        if self.publish_joints:
            js = JointState()
            js.header.stamp = stamp
            js.name = list(self.joint_names)
            js.position = [yaw, roll, pitch]      # same order as joint_names
            self.pub_joints.publish(js)

    def check_attitude(self):
        # A missing driver is the most likely field fault: say so clearly.
        stale = self.t_att is None or (self._now() - self.t_att) > self.att_timeout
        if stale and not self.warned:
            self.get_logger().warn(
                "no /siyi/attitude - is siyi_node running and the A8 mini reachable?")
        elif not stale and self.warned:
            self.get_logger().info("/siyi/attitude is arriving")
        self.warned = stale


def main():
    rclpy.init()
    node = SiyiGimbalAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
