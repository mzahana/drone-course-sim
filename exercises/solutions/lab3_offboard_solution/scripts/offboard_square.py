#!/usr/bin/env python3
"""Lab 3 reference solution -- arm, OFFBOARD, take off, fly a square, then
switch to velocity setpoints.

The one thing that trips everybody: **PX4 refuses OFFBOARD unless a setpoint
stream is already flowing.** It wants setpoints arriving at better than 2 Hz
before the mode change, and it will drop out of OFFBOARD if they ever stop for
more than half a second. So this node starts streaming immediately and only
then asks for the mode -- the request is made from inside the same timer that
does the streaming, which is the simplest way to guarantee the ordering.

Everything on a /mavros/... topic is ENU. PositionTarget.FRAME_LOCAL_NED is
named after the MAVLink frame, not the frame you are writing in: MAVROS has
already converted, and x is East, y is North, z is Up.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

# MAVROS telemetry is published best-effort. A subscriber that asks for
# RELIABLE is stricter than the publisher offers, so it silently never fires.
SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

# type_mask says which fields of PositionTarget to believe.
POSITION_ONLY = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                 PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                 PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                 PositionTarget.IGNORE_YAW_RATE)
VELOCITY_ONLY = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                 PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                 PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                 PositionTarget.IGNORE_YAW_RATE)


class OffboardSquare(Node):
    def __init__(self):
        super().__init__("offboard_square")

        self.declare_parameter("altitude", 5.0)
        self.declare_parameter("side", 10.0)
        self.declare_parameter("accept_radius", 0.6)
        self.declare_parameter("cruise_speed", 2.0)
        self.alt = float(self.get_parameter("altitude").value)
        self.side = float(self.get_parameter("side").value)
        self.accept = float(self.get_parameter("accept_radius").value)
        self.cruise = float(self.get_parameter("cruise_speed").value)

        self.state = State()
        self.pose = None

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, SENSOR_QOS)
        self.sp = self.create_publisher(PositionTarget,
                                        "/mavros/setpoint_raw/local", 10)

        self.arming = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")

        self.phase = "WAIT_FCU"
        self.corner = 0
        self.home = None
        self.t_phase = self.get_clock().now()
        self.last_request = self.get_clock().now()

        # 20 Hz: comfortably above PX4's 2 Hz floor, and the same loop both
        # streams setpoints and drives the sequence, so they cannot get out of
        # order.
        self.create_timer(0.05, self._tick)
        self.get_logger().info("waiting for the autopilot")

    # ----------------------------------------------------------- callbacks
    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg.pose.position

    # -------------------------------------------------------------- helpers
    def _send_position(self, x, y, z, yaw=0.0):
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = POSITION_ONLY
        m.position.x, m.position.y, m.position.z = x, y, z
        m.yaw = yaw
        self.sp.publish(m)

    def _send_velocity(self, vx, vy, vz, yaw=0.0):
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = VELOCITY_ONLY
        m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
        m.yaw = yaw
        self.sp.publish(m)

    def _corner_xy(self, i):
        hx, hy = self.home
        return [(hx, hy), (hx + self.side, hy),
                (hx + self.side, hy + self.side), (hx, hy + self.side)][i % 4]

    def _elapsed(self):
        return (self.get_clock().now() - self.t_phase).nanoseconds * 1e-9

    def _enter(self, phase):
        self.phase = phase
        self.t_phase = self.get_clock().now()
        self.get_logger().info(f"-> {phase}")

    def _request_every_second(self, fn):
        """Services are asked at most once a second. Spamming set_mode while
        the autopilot is still thinking gets you nothing but a full queue."""
        now = self.get_clock().now()
        if (now - self.last_request).nanoseconds * 1e-9 < 1.0:
            return
        self.last_request = now
        fn()

    # ----------------------------------------------------------------- loop
    def _tick(self):
        if self.pose is None or not self.state.connected:
            return

        if self.phase == "WAIT_FCU":
            self.home = (self.pose.x, self.pose.y)
            self._enter("STREAM")

        # Keep a setpoint flowing in every phase. This is the whole trick.
        if self.phase == "STREAM":
            self._send_position(self.home[0], self.home[1], self.alt)
            if self._elapsed() > 1.0:
                self._enter("MODE")

        elif self.phase == "MODE":
            self._send_position(self.home[0], self.home[1], self.alt)
            if self.state.mode != "OFFBOARD":
                self._request_every_second(
                    lambda: self.set_mode.call_async(
                        SetMode.Request(custom_mode="OFFBOARD")))
            elif not self.state.armed:
                self._request_every_second(
                    lambda: self.arming.call_async(
                        CommandBool.Request(value=True)))
            else:
                self._enter("TAKEOFF")

        elif self.phase == "TAKEOFF":
            self._send_position(self.home[0], self.home[1], self.alt)
            if abs(self.pose.z - self.alt) < self.accept:
                self._enter("SQUARE")

        elif self.phase == "SQUARE":
            x, y = self._corner_xy(self.corner)
            self._send_position(x, y, self.alt)
            if math.dist((self.pose.x, self.pose.y), (x, y)) < self.accept:
                self.corner += 1
                self.get_logger().info(f"corner {self.corner}/4 reached")
                if self.corner >= 4:
                    self._enter("VELOCITY")

        elif self.phase == "VELOCITY":
            # A circle flown open-loop on velocity alone. It drifts, and that
            # is the point: a velocity setpoint has no position memory.
            w = 0.25
            t = self._elapsed()
            self._send_velocity(self.cruise * math.cos(w * t),
                                self.cruise * math.sin(w * t),
                                0.6 * (self.alt - self.pose.z))
            if t > 2 * math.pi / w:
                self._enter("HOLD")

        elif self.phase == "HOLD":
            self._send_position(self.home[0], self.home[1], self.alt)


def main():
    rclpy.init()
    node = OffboardSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
