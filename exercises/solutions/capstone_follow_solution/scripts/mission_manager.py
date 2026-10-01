#!/usr/bin/env python3
"""Capstone reference solution -- the mission state machine.

Your tasks:
    TODO 1  In SEARCH, sweep the gimbal and switch to FOLLOW on a track
    TODO 2  In FOLLOW, switch to LOST when the track is lost
    TODO 3  In LOST, wait, then scan, then return home
Search this file for 'TODO' -- you only need to edit between the
ADD YOUR CODE BELOW / END OF YOUR CODE lines.

States:

    IDLE -> TAKEOFF -> SEARCH -> FOLLOW -> LOST -> RTL
                          ^________|          |
                                   |__________|  (reacquired)

This node is the only one that arms the aircraft, the only one that changes
the flight mode, and the only one that turns guidance on or off
(/guidance/enable is True only in FOLLOW). Only one node may publish
setpoints at a time: in FOLLOW it is follow_guidance; in every other state
it is this node.

What happens when the target is lost is decided in advance:

    target lost   -> hover for coast_seconds; the filter keeps predicting
                     and may find the target again
    still lost    -> sweep the gimbal left and right for scan_seconds
    still lost    -> return home (RTL) and land

Every step has a time limit and a next state, so the aircraft never waits
forever.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

# MAVROS publishes the pose as BEST_EFFORT, so we must subscribe that way.
SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

# type_mask: use position and yaw only (see Lab 3).
POSITION_ONLY = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                 PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                 PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                 PositionTarget.IGNORE_YAW_RATE)


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("search_altitude", 15.0)  # m
        self.declare_parameter("accept_radius", 1.0)     # m, "close enough" to a point
        self.declare_parameter("track_timeout", 1.0)     # s without a track before FOLLOW gives up
        self.declare_parameter("coast_seconds", 3.0)     # LOST: time to trust the filter
        self.declare_parameter("scan_seconds", 12.0)     # LOST: time to sweep the gimbal
        self.declare_parameter("scan_rate", 0.35)        # rad/s, frequency of the yaw sweep
        self.declare_parameter("scan_amplitude", 1.2)    # rad, inside the gimbal's +/-135 deg
        self.declare_parameter("search_pitch", -0.7)     # rad, look down while searching
        self.declare_parameter("mission_seconds", 0.0)   # 0 = no time limit
        self.declare_parameter("auto_start", True)       # take off without waiting

        g = lambda n: self.get_parameter(n).value    # short name for reading a parameter
        self.search_alt = float(g("search_altitude"))
        self.accept = float(g("accept_radius"))
        self.track_timeout = float(g("track_timeout"))
        self.coast_s = float(g("coast_seconds"))
        self.scan_s = float(g("scan_seconds"))
        self.scan_rate = float(g("scan_rate"))
        self.scan_amp = float(g("scan_amplitude"))
        self.search_pitch = float(g("search_pitch"))
        self.mission_s = float(g("mission_seconds"))
        self.auto_start = bool(g("auto_start"))

        self.state = State()       # latest autopilot state (mode, armed, connected)
        self.pose = None           # aircraft position (ENU, m)
        self.home = None           # (x, y) where the mission started
        self.track_t = None        # time the last /target/track message arrived

        self.pub_sp = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.pub_state = self.create_publisher(String, "/mission/state", 10)
        self.pub_enable = self.create_publisher(Bool, "/guidance/enable", 10)
        self.pub_gimbal = self.create_publisher(Vector3Stamped, "/gimbal/cmd/angle", 10)

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, SENSOR_QOS)
        self.create_subscription(Odometry, "/target/track", self._on_track, 10)

        self.arming = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")

        self.phase = "IDLE"                          # current state
        self.t_phase = self.get_clock().now()        # when the state started
        self.t_start = None                          # when the mission started (s)
        self.last_request = self.get_clock().now()   # last service request

        self.create_timer(0.05, self._tick)          # 20 Hz: above PX4's 2 Hz minimum
        self.create_timer(0.5, self._announce)       # repeat the state twice a second

    # ------------------------------------------------------------ callbacks
    def _now(self):
        """Current ROS time in seconds (sim time in simulation)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg):
        self.state = msg

    def _on_pose(self, msg):
        self.pose = msg.pose.position

    def _on_track(self, msg):
        self.track_t = self._now()

    def _announce(self):
        self.pub_state.publish(String(data=self.phase))

    def _have_track(self):
        """True if a /target/track message arrived within track_timeout."""
        return self.track_t is not None and (self._now() - self.track_t) < self.track_timeout

    def _elapsed(self):
        """Seconds since the current state started."""
        return (self.get_clock().now() - self.t_phase).nanoseconds * 1e-9

    def _enter(self, phase):
        """Switch to a new state. Publishes the state, and enables guidance
        only in FOLLOW."""
        if phase == self.phase:
            return
        self.phase = phase
        self.t_phase = self.get_clock().now()
        self.get_logger().info(f"-> {phase}")
        self.pub_state.publish(String(data=phase))
        self.pub_enable.publish(Bool(data=(phase == "FOLLOW")))

    def _request_every_second(self, fn):
        """Call fn() at most once per second (service requests; see Lab 3)."""
        now = self.get_clock().now()
        if (now - self.last_request).nanoseconds * 1e-9 < 1.0:
            return
        self.last_request = now
        fn()

    def _hold(self, x, y, z, yaw=0.0):
        """Publish a position setpoint (ENU values, see Lab 3)."""
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = POSITION_ONLY
        m.position.x, m.position.y, m.position.z = x, y, z
        m.yaw = yaw
        self.pub_sp.publish(m)

    def _point_gimbal(self, pitch, yaw):
        """Command the gimbal to an absolute angle (rad). Negative pitch looks
        down."""
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.vector.x = 0.0
        m.vector.y = pitch
        m.vector.z = yaw
        self.pub_gimbal.publish(m)

    # ----------------------------------------------------------------- loop
    def _tick(self):
        # Wait until MAVROS is connected to PX4 and we have a position.
        if self.pose is None or not self.state.connected:
            return
        if self.home is None:
            self.home = (self.pose.x, self.pose.y)
        if self.t_start is None:
            self.t_start = self._now()
        hx, hy = self.home

        # Optional time limit: go home when mission_seconds have passed.
        if self.mission_s > 0.0 and (self._now() - self.t_start) > self.mission_s \
                and self.phase not in ("RTL", "IDLE"):
            self.get_logger().info("mission time is up")
            self._enter("RTL")

        if self.phase == "IDLE":
            # Stream setpoints before asking for OFFBOARD: PX4 accepts the
            # mode only if setpoints already arrive faster than 2 Hz.
            # Then switch to OFFBOARD, arm, and take off (as in Lab 3).
            self._hold(hx, hy, self.search_alt)
            if not self.auto_start:
                return
            if self._elapsed() < 1.0:
                return
            if self.state.mode != "OFFBOARD":
                self._request_every_second(
                    lambda: self.set_mode.call_async(
                        SetMode.Request(custom_mode="OFFBOARD")))
            elif not self.state.armed:
                self._request_every_second(
                    lambda: self.arming.call_async(CommandBool.Request(value=True)))
            else:
                self._enter("TAKEOFF")

        elif self.phase == "TAKEOFF":
            # Climb above home, camera pointing forward and down.
            self._hold(hx, hy, self.search_alt)
            self._point_gimbal(self.search_pitch, 0.0)
            if abs(self.pose.z - self.search_alt) < self.accept:
                self._enter("SEARCH")

        elif self.phase == "SEARCH":
            # Hover above home while looking for the target.
            self._hold(hx, hy, self.search_alt)
            # ------------------------------------------------------------------
            # TODO 1: In SEARCH, sweep the gimbal and switch to FOLLOW on a track
            #   - Point the gimbal with self._point_gimbal(pitch, yaw), using
            #     pitch = self.search_pitch (look down) and
            #     yaw = self.scan_amp * sin(self.scan_rate * self._elapsed()),
            #     so it sweeps left and right.
            #   - If self._have_track() is True, call self._enter("FOLLOW").
            #   - We move the gimbal, not the aircraft: it is faster, and the
            #     aircraft stays still while we do not know where the target is.
            # ======================= ADD YOUR CODE BELOW =======================
            self._point_gimbal(self.search_pitch,
                               self.scan_amp * math.sin(self.scan_rate * self._elapsed()))
            if self._have_track():
                self._enter("FOLLOW")
            # ======================= END OF YOUR CODE ==========================

        elif self.phase == "FOLLOW":
            # follow_guidance publishes the setpoints in this state.
            # ------------------------------------------------------------------
            # TODO 2: In FOLLOW, switch to LOST when the track is lost
            #   - If self._have_track() is False, call self._enter("LOST").
            #   - Do not publish setpoints here. follow_guidance publishes them
            #     while it is enabled, and two nodes publishing on the same
            #     topic would send conflicting setpoints.
            # ======================= ADD YOUR CODE BELOW =======================
            if not self._have_track():
                self._enter("LOST")
            # ======================= END OF YOUR CODE ==========================

        elif self.phase == "LOST":
            # Hover where we are, at search altitude.
            self._hold(self.pose.x, self.pose.y, self.search_alt)
            # ------------------------------------------------------------------
            # TODO 3: In LOST, wait, then scan, then return home
            #   - e = self._elapsed() is the time spent in LOST so far.
            #   - Check these in order:
            #       track is back (self._have_track())  -> self._enter("FOLLOW")
            #       e < self.coast_s                   -> do nothing: the filter
            #                                             may still find it
            #       e < self.coast_s + self.scan_s     -> sweep the gimbal as in
            #                                             TODO 1, using the time
            #                                             e - self.coast_s
            #       otherwise                          -> log a warning and
            #                                             self._enter("RTL")
            #   - Every case must lead somewhere. Do not retry forever.
            # ======================= ADD YOUR CODE BELOW =======================
            e = self._elapsed()
            if self._have_track():
                self._enter("FOLLOW")
            elif e < self.coast_s:
                pass                                   # the filter may still find it
            elif e < self.coast_s + self.scan_s:
                self._point_gimbal(self.search_pitch,
                                   self.scan_amp * math.sin(self.scan_rate * (e - self.coast_s)))
            else:
                self.get_logger().warn("target not reacquired; breaking off")
                self._enter("RTL")
            # ======================= END OF YOUR CODE ==========================

        elif self.phase == "RTL":
            # Fly home at search altitude, then ask PX4 to land (AUTO.LAND).
            self._point_gimbal(self.search_pitch, 0.0)
            self._hold(hx, hy, self.search_alt)
            if math.dist((self.pose.x, self.pose.y), (hx, hy)) < self.accept:
                self._request_every_second(
                    lambda: self.set_mode.call_async(
                        SetMode.Request(custom_mode="AUTO.LAND")))


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
