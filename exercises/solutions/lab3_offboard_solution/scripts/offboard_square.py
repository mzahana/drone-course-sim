#!/usr/bin/env python3
"""Lab 3 reference solution -- arm, switch to OFFBOARD, take off, fly a
square, then fly a circle on velocity setpoints.

Your tasks:
    TODO 1  Build a position setpoint and publish it
    TODO 2  Build a velocity setpoint and publish it
    TODO 3  Return corner i of the square
    TODO 4  Switch to OFFBOARD, then arm, then go to TAKEOFF
Search this file for 'TODO' -- you only need to edit between the
ADD YOUR CODE BELOW / END OF YOUR CODE lines.

How it works:
    A 20 Hz timer (_tick) does two jobs. It publishes a setpoint on every
    tick, and it steps through the phases
        WAIT_FCU -> STREAM -> MODE -> TAKEOFF -> SQUARE -> VELOCITY -> HOLD

Two rules shape this node:
  * PX4 only accepts OFFBOARD mode if setpoints are already arriving faster
    than 2 Hz. If they stop for more than about 0.5 s, PX4 leaves OFFBOARD.
    So the node starts streaming first, and asks for the mode from the same
    timer that streams. That keeps the order right.
  * All /mavros/... topics use the ENU frame: x = East, y = North, z = Up.
    PositionTarget.FRAME_LOCAL_NED is only the name of the MAVLink frame.
    MAVROS converts ENU to NED for you, so you always write ENU values.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

# MAVROS publishes sensor topics (such as the pose) as BEST_EFFORT.
# A subscriber that asks for RELIABLE never receives from such a publisher,
# and ROS shows no error. So we subscribe with a BEST_EFFORT profile.
SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

# type_mask tells PX4 which fields of a PositionTarget to IGNORE.
# Each IGNORE_* constant is one bit; OR-ing them together ignores all of them.
# POSITION_ONLY: use position and yaw; ignore velocity, acceleration, yaw rate.
POSITION_ONLY = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                 PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                 PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                 PositionTarget.IGNORE_YAW_RATE)
# VELOCITY_ONLY: use velocity and yaw; ignore position, acceleration, yaw rate.
VELOCITY_ONLY = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
                 PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX |
                 PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                 PositionTarget.IGNORE_YAW_RATE)


class OffboardSquare(Node):
    def __init__(self):
        super().__init__("offboard_square")

        self.declare_parameter("altitude", 5.0)         # m, flight altitude
        self.declare_parameter("side", 10.0)            # m, side of the square
        self.declare_parameter("accept_radius", 0.6)    # m, "close enough" to a point
        self.declare_parameter("cruise_speed", 2.0)     # m/s, speed on the circle
        self.alt = float(self.get_parameter("altitude").value)
        self.side = float(self.get_parameter("side").value)
        self.accept = float(self.get_parameter("accept_radius").value)
        self.cruise = float(self.get_parameter("cruise_speed").value)

        self.state = State()       # latest autopilot state (mode, armed, connected)
        self.pose = None           # latest aircraft position (ENU, m)

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, SENSOR_QOS)
        self.sp = self.create_publisher(PositionTarget,
                                        "/mavros/setpoint_raw/local", 10)

        self.arming = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")

        self.phase = "WAIT_FCU"    # current phase of the flight
        self.corner = 0            # number of square corners reached so far
        self.home = None           # (x, y) where the flight started
        self.t_phase = self.get_clock().now()        # when the phase started
        self.last_request = self.get_clock().now()   # last service request

        # 20 Hz is well above PX4's 2 Hz minimum. The same timer streams the
        # setpoints and runs the phases, so they cannot get out of order.
        self.create_timer(0.05, self._tick)
        self.get_logger().info("waiting for the autopilot")

    # ----------------------------------------------------------- callbacks
    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg.pose.position

    # -------------------------------------------------------------- helpers
    def _send_position(self, x, y, z, yaw=0.0):
        # ------------------------------------------------------------------
        # TODO 1: Build a position setpoint and publish it
        #   - Create a PositionTarget message m.
        #   - Stamp it with self.get_clock().now().to_msg() (the sim clock,
        #     not time.time()).
        #   - Set m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED.
        #     The name says NED, but MAVROS converts for you: x = East,
        #     y = North, z = Up.
        #   - Set m.type_mask = POSITION_ONLY.
        #   - Put x, y, z in m.position and yaw in m.yaw.
        #   - Publish it with self.sp.publish(m).
        # ======================= ADD YOUR CODE BELOW =======================
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = POSITION_ONLY
        m.position.x, m.position.y, m.position.z = x, y, z
        m.yaw = yaw
        self.sp.publish(m)
        # ======================= END OF YOUR CODE ==========================

    def _send_velocity(self, vx, vy, vz, yaw=0.0):
        # ------------------------------------------------------------------
        # TODO 2: Build a velocity setpoint and publish it
        #   - The same as TODO 1, with two changes:
        #     m.type_mask = VELOCITY_ONLY, and put vx, vy, vz in m.velocity
        #     (instead of m.position).
        #   - Still set m.yaw = yaw.
        # ======================= ADD YOUR CODE BELOW =======================
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = VELOCITY_ONLY
        m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
        m.yaw = yaw
        self.sp.publish(m)
        # ======================= END OF YOUR CODE ==========================

    def _corner_xy(self, i):
        # ------------------------------------------------------------------
        # TODO 3: Return corner i of the square
        #   - self.home = (hx, hy) is the start point and the first corner.
        #   - The four corners, in order, are:
        #     (hx, hy), (hx + side, hy), (hx + side, hy + side), (hx, hy + side)
        #     where side = self.side.
        #   - i can be larger than 3, so use i % 4.
        #   - Return a tuple (x, y).
        # ======================= ADD YOUR CODE BELOW =======================
        hx, hy = self.home
        return [(hx, hy), (hx + self.side, hy),
                (hx + self.side, hy + self.side), (hx, hy + self.side)][i % 4]
        # ======================= END OF YOUR CODE ==========================

    def _elapsed(self):
        """Seconds since the current phase started."""
        return (self.get_clock().now() - self.t_phase).nanoseconds * 1e-9

    def _enter(self, phase):
        """Switch to a new phase and restart the phase timer."""
        self.phase = phase
        self.t_phase = self.get_clock().now()
        self.get_logger().info(f"-> {phase}")

    def _request_every_second(self, fn):
        """Call fn() at most once per second.

        The autopilot needs time to answer a service request. Sending the same
        request 20 times a second does not help; it only fills the queue.
        """
        now = self.get_clock().now()
        if (now - self.last_request).nanoseconds * 1e-9 < 1.0:
            return
        self.last_request = now
        fn()

    # ----------------------------------------------------------------- loop
    def _tick(self):
        # Wait until MAVROS is connected to PX4 and we have a position.
        if self.pose is None or not self.state.connected:
            return

        if self.phase == "WAIT_FCU":
            # Remember the start point. The square is flown relative to it.
            self.home = (self.pose.x, self.pose.y)
            self._enter("STREAM")

        # Every phase below publishes a setpoint on every tick. If the
        # setpoints stop, PX4 leaves OFFBOARD.
        if self.phase == "STREAM":
            # Stream setpoints for 1 s before asking for OFFBOARD.
            self._send_position(self.home[0], self.home[1], self.alt)
            if self._elapsed() > 1.0:
                self._enter("MODE")

        elif self.phase == "MODE":
            self._send_position(self.home[0], self.home[1], self.alt)
            # ------------------------------------------------------------------
            # TODO 4: Switch to OFFBOARD, then arm, then go to TAKEOFF
            #   - self.state.mode is the current mode (a string) and
            #     self.state.armed is True or False.
            #   - If the mode is not "OFFBOARD", ask for it:
            #     self.set_mode.call_async(SetMode.Request(custom_mode="OFFBOARD"))
            #   - Else, if not armed, ask to arm:
            #     self.arming.call_async(CommandBool.Request(value=True))
            #   - Else call self._enter("TAKEOFF").
            #   - Wrap each service call in self._request_every_second(lambda: ...)
            #     so it is sent at most once per second.
            #   - The order matters. PX4 accepts OFFBOARD only because the
            #     STREAM phase already sent setpoints. If you arm before the
            #     mode change is accepted, the aircraft is armed in its old mode.
            # ======================= ADD YOUR CODE BELOW =======================
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
            # ======================= END OF YOUR CODE ==========================

        elif self.phase == "TAKEOFF":
            # Climb above the start point until we are close to the altitude.
            self._send_position(self.home[0], self.home[1], self.alt)
            if abs(self.pose.z - self.alt) < self.accept:
                self._enter("SQUARE")

        elif self.phase == "SQUARE":
            # Fly to each corner in turn. A corner counts as reached when we
            # are within accept_radius of it.
            x, y = self._corner_xy(self.corner)
            self._send_position(x, y, self.alt)
            if math.dist((self.pose.x, self.pose.y), (x, y)) < self.accept:
                self.corner += 1
                self.get_logger().info(f"corner {self.corner}/4 reached")
                if self.corner >= 4:
                    self._enter("VELOCITY")

        elif self.phase == "VELOCITY":
            # Fly one circle using velocity setpoints only. The horizontal
            # velocity turns at w rad/s, so one circle takes 2*pi/w seconds
            # (about 25 s). The vertical velocity is a simple P controller
            # that holds the altitude. There is no position feedback in x and
            # y, so the circle drifts: a velocity setpoint does not remember
            # a position.
            w = 0.25
            t = self._elapsed()
            self._send_velocity(self.cruise * math.cos(w * t),
                                self.cruise * math.sin(w * t),
                                0.6 * (self.alt - self.pose.z))
            if t > 2 * math.pi / w:
                self._enter("HOLD")

        elif self.phase == "HOLD":
            # Go back above the start point and hover.
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
