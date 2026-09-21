#!/usr/bin/env python3
"""Lab 5 reference solution, part 2 -- the standoff law, with the clamps.

    p_des = p_T - d * v_hat_T + h * z_hat
    v_cmd = v_T + Kp (p_des - p_drone)

The second term is the satisfying one, because students measure it in a single
run. Without the v_T feedforward, a proportional controller following a target
at constant speed settles at a permanent lag of

    e_ss = v_T / Kp

-- 3 m behind a 3 m/s target with Kp = 1. Adding v_T drives it to zero. Fly it
both ways; the difference is visible on one plot.

The clamps are not decoration. With the drone above and the target on the
ground, the risk is not collision -- it is **losing the target**, and every
clamp here is really about how well you can see:

  * an altitude floor, because the ground-plane fix degrades as h -> 0 and the
    aircraft has nowhere to go;
  * a speed limit, because a detector running at 10 Hz cannot keep up with an
    aircraft that outruns its own perception;
  * a geofence, because a target that drives out of the area must not take the
    aircraft with it.

Safety goes last, after guidance, and it is allowed to overrule it. That
ordering is the architectural lesson of the day -- on a real vehicle the flight
state machine holds this veto and nothing else may write to the controller.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

# Position plus velocity feedforward: PX4 takes the position as the reference
# and the velocity as a feedforward term, which is exactly what the law above
# produces. Acceleration and yaw rate are ignored.
POS_PLUS_VEL = (PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
                PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE)


class FollowGuidance(Node):
    def __init__(self):
        super().__init__("follow_guidance")

        self.declare_parameter("standoff", 18.0)        # d, m behind the target
        # 12 m up and 18 m back is a 34 degree look-down at 21.6 m slant
        # range. The look angle is chosen by the DETECTOR, not by the geometry:
        # measured on this target, at 960 px input,
        #
        #   look-down   30    40    45    50    55    60    70
        #   confidence  0.49  0.51  0.40  0.27  0.13  none  none
        #
        # COCO has almost no pictures of vehicles taken from above, so a
        # near-nadir view of a pickup truck is not a thing YOLO was ever taught
        # to recognise. Past about 50 degrees it stops finding it at all, and
        # at 60 it confidently reports an aeroplane instead.
        #
        # 34 degrees sits in the flat part of that curve with room for the
        # transients of an approach. It costs accuracy -- the along-range error
        # is amplified by 1/sin(theta), 1.80 here against 1.41 at 45 degrees --
        # and that is the right trade, because an accurate fix you never get is
        # worth nothing. This is the "how well you see determines how you are
        # allowed to fly" lesson, with numbers.
        self.declare_parameter("altitude", 12.0)        # h, m above it
        # Kept because the guidance law is written in terms of it, and because
        # a team may want to close the position loop themselves. PX4's own
        # MPC_XY_P plays this role by default; see the comment in _tick.
        self.declare_parameter("kp", 1.0)               # 1/s
        self.declare_parameter("feedforward", True)     # turn off to see e_ss
        self.declare_parameter("max_speed", 8.0)        # m/s, detector-limited
        self.declare_parameter("altitude_floor", 5.0)
        self.declare_parameter("geofence_radius", 100.0)
        self.declare_parameter("track_timeout", 1.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("min_speed_for_heading", 0.7)
        # The heading is low-pass filtered. Taking it raw from the filter makes
        # the standoff reference whip through a wide arc every time the target
        # turns, and the aircraft spends the turn chasing a point that is
        # moving faster than it is. Two seconds of smoothing costs a little lag
        # on a genuine turn and removes most of that.
        self.declare_parameter("heading_tau", 2.0)      # s
        self.declare_parameter("start_enabled", True)

        g = lambda n: self.get_parameter(n).value
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

        self.track = None          # (t, x, y, vx, vy)
        self.pose = None
        self.heading = None        # latched unit heading of the target
        self.yaw_rate_req = 0.0

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
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_track(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        self.track = (self._now(), p.x, p.y, v.x, v.y)

    def _on_pose(self, msg):
        self.pose = msg.pose.position

    def _on_enable(self, msg):
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
            return              # stale track: say nothing rather than guess

        # --- the reference ------------------------------------------------
        speed = math.hypot(tvx, tvy)
        if speed > self.min_speed:
            hx, hy = tvx / speed, tvy / speed
            if self.heading is None:
                self.heading = (hx, hy)
            else:
                a = min(1.0, (1.0 / 20.0) / max(self.heading_tau, 1e-3))
                bx = self.heading[0] + a * (hx - self.heading[0])
                by = self.heading[1] + a * (hy - self.heading[1])
                n = math.hypot(bx, by)
                if n > 1e-6:
                    self.heading = (bx / n, by / n)
        # When the target stops, v_hat is undefined. Latching the last valid
        # heading keeps the aircraft where it was rather than snapping
        # overhead, and a latched bearing is a decision -- dividing by a speed
        # of zero is a crash.
        if self.heading is None:
            # Hold the bearing we are already on, at the standoff distance.
            #
            # The tempting fallback -- sit directly overhead -- was tried and
            # is wrong twice over. It throws away the look angle the whole
            # error model is built on, and it puts the camera at nadir, where
            # a top-down truck stops looking like anything COCO was trained on:
            # measured, detections went to zero the moment the aircraft arrived
            # over a stationary target, and the mission went to LOST while
            # hovering directly above a truck in plain view.
            bx, by = self.pose.x - tx, self.pose.y - ty
            n = math.hypot(bx, by)
            if n > 0.5:
                px, py = tx + self.d * bx / n, ty + self.d * by / n
            else:
                px, py = tx + self.d, ty       # any bearing beats none
        else:
            px = tx - self.d * self.heading[0]
            py = ty - self.d * self.heading[1]
        pz = self.h

        # The velocity field is a FEEDFORWARD, so it carries the target's
        # velocity and nothing else. The proportional term of
        #
        #     v_cmd = v_T + Kp (p_des - p_drone)
        #
        # is computed by PX4, from the position setpoint we are already
        # sending, with its own gain MPC_XY_P (0.95 by default). Adding our own
        # Kp term on top does not make the loop tighter, it makes the gain
        # roughly double and the approach overshoot: measured, the aircraft ran
        # 5 m past a stationary target, put it behind the camera, and lost it.
        #
        # So the lag experiment is unchanged and still honest. With the
        # feedforward off, the steady-state lag is v_T / MPC_XY_P -- about 3.2 m
        # behind a 3 m/s target. With it on, it goes to zero. Read MPC_XY_P
        # with: ros2 param get /mavros/param MPC_XY_P
        vx = tvx if self.use_ff else 0.0
        vy = tvy if self.use_ff else 0.0
        vz = 0.0

        # --- the clamps, applied after the law and allowed to overrule it --
        sp_h = math.hypot(vx, vy)
        if sp_h > self.vmax:
            vx, vy = vx * self.vmax / sp_h, vy * self.vmax / sp_h
        pz = max(pz, self.floor)
        r = math.hypot(px, py)
        if r > self.fence:
            px, py = px * self.fence / r, py * self.fence / r

        # Point the nose at the target. It costs nothing and it keeps the
        # aircraft's own sensors, and the gimbal's yaw travel, centred.
        yaw = math.atan2(ty - self.pose.y, tx - self.pose.x)

        m = PositionTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.coordinate_frame = PositionTarget.FRAME_LOCAL_NED   # MAVROS: this is ENU
        m.type_mask = POS_PLUS_VEL
        m.position.x, m.position.y, m.position.z = px, py, pz
        m.velocity.x, m.velocity.y, m.velocity.z = vx, vy, vz
        m.yaw = yaw
        self.sp.publish(m)

        ref = PoseStamped()
        ref.header = m.header
        ref.header.frame_id = "map"
        ref.pose.position.x, ref.pose.position.y, ref.pose.position.z = px, py, pz
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
