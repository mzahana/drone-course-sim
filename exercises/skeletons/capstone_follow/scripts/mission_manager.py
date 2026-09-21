#!/usr/bin/env python3
"""Capstone skeleton -- the mission state machine.

    IDLE -> TAKEOFF -> SEARCH -> FOLLOW -> LOST -> RTL
                          ^________|          |
                                   |__________|  (reacquired)

This node is the **only** thing that arms the aircraft, the only thing that
asks for a mode, and the only thing that decides whether the guidance law is
allowed to drive. Guidance publishes setpoints; this decides whether guidance
is running at all. Two writers to one setpoint topic is a fight, and the one
that publishes last wins, which is not a design.

The failure responses are **designed here, in advance**, not discovered in
flight:

  target lost      -> coast on the filter for T_coast
  still lost       -> scan the gimbal across its yaw travel
  still lost       -> break off and return home

Each has a timeout and each has a next state. A state machine whose LOST branch
is "keep trying" is how an aircraft ends up somewhere nobody chose.
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

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

POSITION_ONLY = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                 PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                 PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                 PositionTarget.IGNORE_YAW_RATE)


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")

        self.declare_parameter("search_altitude", 15.0)
        self.declare_parameter("accept_radius", 1.0)
        self.declare_parameter("track_timeout", 1.0)     # s before FOLLOW gives up
        self.declare_parameter("coast_seconds", 3.0)     # LOST: trust the filter
        self.declare_parameter("scan_seconds", 12.0)     # LOST: sweep the gimbal
        self.declare_parameter("scan_rate", 0.35)        # rad/s of yaw sweep
        self.declare_parameter("scan_amplitude", 1.2)    # rad, inside +/-135 deg
        self.declare_parameter("search_pitch", -0.7)     # rad, look down while searching
        self.declare_parameter("mission_seconds", 0.0)   # 0 = no time limit
        self.declare_parameter("auto_start", True)

        g = lambda n: self.get_parameter(n).value
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

        self.state = State()
        self.pose = None
        self.home = None
        self.track_t = None

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

        self.phase = "IDLE"
        self.t_phase = self.get_clock().now()
        self.t_start = None
        self.last_request = self.get_clock().now()

        self.create_timer(0.05, self._tick)          # 20 Hz: above PX4's 2 Hz floor
        self.create_timer(0.5, self._announce)

    # ------------------------------------------------------------ callbacks
    def _now(self):
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
        return self.track_t is not None and (self._now() - self.track_t) < self.track_timeout

    def _elapsed(self):
        return (self.get_clock().now() - self.t_phase).nanoseconds * 1e-9

    def _enter(self, phase):
        if phase == self.phase:
            return
        self.phase = phase
        self.t_phase = self.get_clock().now()
        self.get_logger().info(f"-> {phase}")
        self.pub_state.publish(String(data=phase))
        self.pub_enable.publish(Bool(data=(phase == "FOLLOW")))

    def _request_every_second(self, fn):
        now = self.get_clock().now()
        if (now - self.last_request).nanoseconds * 1e-9 < 1.0:
            return
        self.last_request = now
        fn()

    def _hold(self, x, y, z, yaw=0.0):
        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        m.type_mask = POSITION_ONLY
        m.position.x, m.position.y, m.position.z = x, y, z
        m.yaw = yaw
        self.pub_sp.publish(m)

    def _point_gimbal(self, pitch, yaw):
        m = Vector3Stamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.vector.x = 0.0
        m.vector.y = pitch
        m.vector.z = yaw
        self.pub_gimbal.publish(m)

    # ----------------------------------------------------------------- loop
    def _tick(self):
        if self.pose is None or not self.state.connected:
            return
        if self.home is None:
            self.home = (self.pose.x, self.pose.y)
        if self.t_start is None:
            self.t_start = self._now()
        hx, hy = self.home

        if self.mission_s > 0.0 and (self._now() - self.t_start) > self.mission_s \
                and self.phase not in ("RTL", "IDLE"):
            self.get_logger().info("mission time is up")
            self._enter("RTL")

        if self.phase == "IDLE":
            # Stream before asking for OFFBOARD. PX4 will not accept the mode
            # unless setpoints are already arriving faster than 2 Hz.
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
            self._hold(hx, hy, self.search_alt)
            self._point_gimbal(self.search_pitch, 0.0)
            if abs(self.pose.z - self.search_alt) < self.accept:
                self._enter("SEARCH")

        elif self.phase == "SEARCH":
            self._hold(hx, hy, self.search_alt)
            # Sweep the gimbal across its yaw travel rather than yawing the
            # aircraft: it is faster, and it does not move the vehicle while
            # nobody knows where the target is.
            # TODO(student): sweep the gimbal in yaw while searching, and leave SEARCH for
            # FOLLOW once there is a live track. self.scan_amp and
            # self.scan_rate shape the sweep; self.search_pitch is how
            # far down to look.
            # Sweeping the gimbal beats yawing the aircraft: it is
            # faster, and it does not move the vehicle while nobody
            # knows where the target is.
            raise NotImplementedError

        elif self.phase == "FOLLOW":
            # follow_guidance owns the setpoint topic while enabled. This node
            # publishes nothing here on purpose -- two writers is a fight.
            # TODO(student): leave FOLLOW for LOST when the track goes stale.
            # Publish nothing else here: follow_guidance owns the
            # setpoint topic while it is enabled, and two writers to one
            # topic is a fight the later publisher wins.
            raise NotImplementedError

        elif self.phase == "LOST":
            self._hold(self.pose.x, self.pose.y, self.search_alt)
            # TODO(student): the LOST ladder, with a timeout on every rung.
            #   reacquired at any point      -> FOLLOW
            #   for the first coast_s        -> do nothing; the filter is
            #                                   still coasting
            #   then for scan_s              -> sweep the gimbal in yaw
            #   after that                   -> RTL
            # A LOST branch whose answer is 'keep trying' is how an
            # aircraft ends up somewhere nobody chose.
            raise NotImplementedError

        elif self.phase == "RTL":
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
