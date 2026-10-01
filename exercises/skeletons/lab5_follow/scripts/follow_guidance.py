#!/usr/bin/env python3
"""Lab 5 skeleton, part 2 -- the follow guidance law, with safety
limits.

Your tasks:
    TODO 1  Compute the standoff reference point
    TODO 2  Apply the safety limits
Search this file for 'TODO' -- you only need to edit between the
ADD YOUR CODE BELOW / END OF YOUR CODE lines.

The guidance law:

    p_des = p_T - d * v_hat_T + h * z_hat
    v_cmd = v_T + Kp * (p_des - p_drone)

    p_T, v_T   target position and velocity (from /target/track)
    v_hat_T    unit vector along the target's velocity (its heading)
    d, h       standoff distance behind the target, and altitude (m)
    z_hat      unit vector pointing up
    p_drone    the aircraft's position

So the reference point p_des is d metres behind the target and h metres up.

Velocity feedforward: with only the Kp term, following a target that moves
at constant speed leaves a constant lag of e_ss = v_T / Kp. Adding v_T to
the command removes that lag. PX4 computes the Kp term itself (see the
comment in _tick), so this node sends p_des as the position setpoint and
v_T as the velocity feedforward. Without the feedforward the lag is about
3.2 m behind a 3 m/s target; with it, about zero.

Safety limits (TODO 2) are applied AFTER the guidance law, so they can
override it:
  * altitude floor: the ground position estimate gets worse as the aircraft
    gets lower, and there is little room to recover;
  * speed limit: the detector runs at about 10 Hz, and an aircraft that
    moves too fast loses the target;
  * geofence: if the target leaves the area, the aircraft must not follow.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64

# MAVROS publishes the pose as BEST_EFFORT, so we must subscribe that way.
SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

# type_mask: use position AND velocity (and yaw). PX4 tracks the position
# and adds the velocity as a feedforward term. Acceleration and yaw rate are
# ignored.
POS_PLUS_VEL = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)


class FollowGuidance(Node):
    def __init__(self):
        super().__init__("follow_guidance")

        self.declare_parameter("standoff", 18.0)        # d, m behind the target
        # Why 12 m up and 18 m back: that is a 34 deg look-down angle at
        # 21.6 m slant range. The angle is chosen for the detector. Measured
        # detector confidence on this target, at 960 px input:
        #
        #   look-down   30    40    45    50    55    60    70
        #   confidence  0.49  0.51  0.40  0.27  0.13  none  none
        #
        # The detector's training set (COCO) has very few vehicles seen from
        # above, so past about 50 deg it stops finding the truck (at 60 deg it
        # reports an aeroplane instead). 34 deg is in the flat part of the
        # curve, with margin for the approach.
        # The cost is accuracy: the along-range error grows by 1/sin(theta),
        # which is 1.80 at 34 deg against 1.41 at 45 deg. A less accurate fix
        # is better than no fix, so this is the right choice.
        self.declare_parameter("altitude", 12.0)        # h, m above the target
        # kp is not used in the command: PX4's own gain MPC_XY_P does this job
        # (see the comment in _tick). It is kept because the law is written
        # with it, and a team may want to close the position loop themselves.
        self.declare_parameter("kp", 1.0)               # 1/s
        self.declare_parameter("feedforward", True)     # set False to see the lag
        self.declare_parameter("max_speed", 8.0)        # m/s, limited by the detector
        self.declare_parameter("altitude_floor", 5.0)   # m, never command lower
        self.declare_parameter("geofence_radius", 100.0)  # m from the map origin
        self.declare_parameter("track_timeout", 1.0)    # s, older tracks are ignored
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("min_speed_for_heading", 0.7)   # m/s
        # The target's heading is low-pass filtered with time constant
        # heading_tau. Without it, the reference point swings quickly around
        # the target every time the target turns, and the aircraft chases it.
        # 2 s of smoothing adds a little lag on real turns.
        self.declare_parameter("heading_tau", 2.0)      # s
        self.declare_parameter("start_enabled", True)   # the capstone starts it disabled

        g = lambda n: self.get_parameter(n).value    # short name for reading a parameter
        self.d = float(g("standoff"))
        self.h = float(g("altitude"))
        self.kp = float(g("kp"))
        self.use_ff = bool(g("feedforward"))
        self.vmax = float(g("max_speed"))
        self.floor = float(g("altitude_floor"))
        self.fence = float(g("geofence_radius"))
        self.track_timeout = float(g("track_timeout"))
        self.min_speed = float(g("min_speed_for_heading"))
        self.heading_tau = float(g("heading_tau"))
        self.enabled = bool(g("start_enabled"))

        self.track = None          # (t, x, y, vx, vy) of the target, in map
        self.pose = None           # aircraft position (ENU, m)
        self.heading = None        # last valid unit heading of the target (kept when it stops)
        self.yaw_rate_req = 0.0    # turn request from gimbal_pointer (not used here)

        self.sp = self.create_publisher(PositionTarget, "/mavros/setpoint_raw/local", 10)
        self.pub_des = self.create_publisher(PoseStamped, "/guidance/reference", 1)
        self.create_subscription(Odometry, "/target/track", self._on_track, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_pose, SENSOR_QOS)
        self.create_subscription(Bool, "/guidance/enable", self._on_enable, 10)
        self.create_subscription(Float64, "/guidance/yaw_rate", self._on_yaw_rate, 10)
        self.create_timer(1.0 / float(g("rate_hz")), self._tick)

        self.get_logger().info(
            f"standoff {self.d:.1f} m, altitude {self.h:.1f} m, "
            f"feedforward {'on' if self.use_ff else 'OFF'} "
            f"(the proportional term is PX4's MPC_XY_P)")

    def _now(self):
        """Current ROS time in seconds (sim time in simulation)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_track(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.track = (self._now(), p.x, p.y, v.x, v.y)

    def _on_pose(self, msg):
        self.pose = msg.pose.position

    def _on_enable(self, msg):
        # The mission manager turns guidance on and off with /guidance/enable.
        if msg.data != self.enabled:
            self.get_logger().info(f"guidance {'enabled' if msg.data else 'disabled'}")
        self.enabled = msg.data

    def _on_yaw_rate(self, msg):
        self.yaw_rate_req = msg.data

    def _tick(self):
        if not self.enabled or self.pose is None or self.track is None:
            return
        t, tx, ty, tvx, tvy = self.track
        if self._now() - t > self.track_timeout:
            return              # old track: publish nothing rather than guess

        # --- the reference point ------------------------------------------
        # ------------------------------------------------------------------
        # TODO 1: Compute the standoff reference point
        #   - The target is at (tx, ty) and moves at (tvx, tvy). Use self.d
        #     (standoff) and self.h (altitude). Produce px, py, pz.
        #   - Formula: p_des = p_T - d * v_hat_T + h * z_hat, where v_hat_T is
        #     the target's unit heading (velocity divided by speed).
        #   - Only compute a new heading when the speed is above
        #     self.min_speed. Below that the heading is not defined (you would
        #     divide by almost zero). Keep the last valid heading in
        #     self.heading and use it while the target is slow or stopped.
        #   - Optional: smooth the heading with time constant self.heading_tau.
        #   - If there has never been a heading (self.heading is None), stay on
        #     the line from the target to the aircraft, at distance d. Do NOT
        #     fly directly overhead: the look angle is lost, and the detector
        #     does not recognise vehicles seen from straight above.
        # ======================= ADD YOUR CODE BELOW =======================
        raise NotImplementedError("TODO 1 in follow_guidance.py: Compute the standoff reference point")  # delete this line and write your code here
        # ======================= END OF YOUR CODE ==========================

        # Velocity: only the target's velocity, as a feedforward.
        #
        # The Kp * (p_des - p_drone) term of the law is computed by PX4 from
        # the position setpoint we send, with its own gain MPC_XY_P (0.95 by
        # default). Do not add our own Kp term here as well: the total gain
        # roughly doubles and the aircraft overshoots. Measured: it flew 5 m
        # past a stationary target, the target ended up behind the camera,
        # and it was lost.
        #
        # With feedforward off, the steady lag is v_T / MPC_XY_P: about 3.2 m
        # behind a 3 m/s target. With it on, the lag is about zero.
        # Read the gain with: ros2 param get /mavros/param MPC_XY_P
        vx = tvx if self.use_ff else 0.0
        vy = tvy if self.use_ff else 0.0
        vz = 0.0

        # --- the safety limits, applied after the law so they always win ---
        # ------------------------------------------------------------------
        # TODO 2: Apply the safety limits
        #   - Speed: if the horizontal speed hypot(vx, vy) is above self.vmax,
        #     scale vx and vy by the SAME factor. Limiting each one on its own
        #     would also change the direction of travel.
        #   - Altitude floor: pz must not be below self.floor.
        #   - Geofence: if the reference (px, py) is farther than self.fence
        #     from the map origin, scale it back onto the circle of radius
        #     self.fence.
        #   - Keep this block after the guidance law, so the limits always win.
        # ======================= ADD YOUR CODE BELOW =======================
        raise NotImplementedError("TODO 2 in follow_guidance.py: Apply the safety limits")  # delete this line and write your code here
        # ======================= END OF YOUR CODE ==========================

        # Point the nose at the target. This keeps the target near the
        # middle of the gimbal's yaw range.
        yaw = math.atan2(ty - self.pose.y, tx - self.pose.x)

        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED   # MAVROS converts: values are ENU
        m.type_mask = POS_PLUS_VEL
        m.position.x, m.position.y, m.position.z = px, py, pz
        m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
        m.yaw = yaw
        self.sp.publish(m)

        # Also publish the reference point, for RViz.
        ref = PoseStamped()
        ref.header = m.header
        ref.header.frame_id = "map"
        ref.pose.position.x, ref.pose.position.y, ref.pose.position.z = px, py, pz
        # Yaw-only quaternion: z = sin(yaw / 2), w = cos(yaw / 2).
        ref.pose.orientation.z = math.sin(yaw / 2.0)
        ref.pose.orientation.w = math.cos(yaw / 2.0)
        self.pub_des.publish(ref)


def main():
    rclpy.init()
    node = FollowGuidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
