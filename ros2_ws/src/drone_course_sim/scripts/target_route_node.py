#!/usr/bin/env python3
"""Drive the ground target along a scripted route.

The route is the course's difficulty dial. Each tier isolates one thing the
students' follower has to cope with:

 -1  idle                 publishes nothing, so something else can drive the
                         target. Used by the scenario test and by anyone who
                         wants manual control.
  0  stationary           does the loop close at all
  1  out and back         long straight legs for the steady-state lag lesson --
                         the velocity feedforward result -- with a U-turn at
                         each end
  2  turning and stopping gimbal saturation, keeping it in frame, and the
                         degeneracy when the target stops and its heading is
                         no longer defined
  3  tier 2 plus blackouts  prediction and graceful degradation

Every route is BOUNDED. A target driving in a straight line forever leaves the
arena in seconds, is out of detection range almost immediately, and makes the
scenario untestable and unteachable.

Tier 3 drives /detector/blackout directly, so the detector goes blind on a
schedule rather than a student having to stage it by hand.

The target is commanded in its own body frame: linear.x forward, angular.z yaw
rate. That is what a car does, and it keeps the route readable.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Int32


class TargetRoute(Node):
    def __init__(self):
        super().__init__("target_route")

        self.declare_parameter("tier", 1)
        self.declare_parameter("speed", 3.0)           # m/s
        self.declare_parameter("turn_rate", 0.25)      # rad/s in tier 2+
        self.declare_parameter("leg_seconds", 12.0)    # straight leg before a turn
        self.declare_parameter("turn_seconds", 6.0)
        self.declare_parameter("stop_seconds", 5.0)
        # The target waits for the aircraft to be AIRBORNE, and only then
        # counts start_delay.
        #
        # A fixed delay from this node's start does not work, and the way it
        # fails is invisible. The route starts when `course bringup` runs; the
        # student then builds a workspace, reads an error, builds again, and
        # launches the mission four minutes later. By the time the aircraft
        # reaches SEARCH the target has driven a fifty-metre circuit and is far
        # outside any plausible search pattern. The run then scores like a
        # follower that failed, when it never had anything to follow.
        #
        # Measured, before this was fixed: tier 2 spent 90 of its 180 scored
        # seconds in SEARCH, holding position over an empty field, and scored
        # 25.4/85 -- a number about a follower that was never given a target.
        self.declare_parameter("start_altitude", 5.0)    # m, aircraft airborne
        self.declare_parameter("start_delay", 5.0)       # s, after that
        self.declare_parameter("blackout_every", 25.0)
        self.declare_parameter("blackout_length", 3.0)

        self.tier = int(self.get_parameter("tier").value)
        self.speed = float(self.get_parameter("speed").value)
        self.turn_rate = float(self.get_parameter("turn_rate").value)
        self.leg = float(self.get_parameter("leg_seconds").value)
        self.turn = float(self.get_parameter("turn_seconds").value)
        self.stop = float(self.get_parameter("stop_seconds").value)
        self.start_delay = float(self.get_parameter("start_delay").value)
        self.start_altitude = float(self.get_parameter("start_altitude").value)
        self.bo_every = float(self.get_parameter("blackout_every").value)
        self.bo_len = float(self.get_parameter("blackout_length").value)

        self.pub = self.create_publisher(Twist, "/target/cmd_vel", 10)
        self.pub_bo = self.create_publisher(Float64, "/detector/blackout", 10)

        # Announce which tier is actually being driven, latched, so anything
        # that joins later still sees it.
        #
        # This exists because the tier is set in two places that look like one:
        # `course bringup tier:=N` chooses the ROUTE, and the capstone launch's
        # own tier:= argument only configures the SCORER. Set one and not the
        # other and everything runs, nothing warns, and four tiers of results
        # come back identical -- which is exactly how this was discovered. The
        # scoring node compares what it was told against what is published here
        # and says so out loud when they disagree.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub_tier = self.create_publisher(Int32, "/target/route_tier", qos)
        self.pub_tier.publish(Int32(data=self.tier))

        # Ground truth, used ONLY to know when to set off. Nothing that flies
        # may read this topic; the target is scenery, not a competitor.
        self.airborne_at = None
        self.create_subscription(Odometry, "/drone/ground_truth", self._drone, 10)

        self.t = 0.0
        self.dt = 0.05
        self.last_blackout = 0.0
        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"target route tier {self.tier}")

    def _t_start(self):
        """The simulated time at which the route began moving."""
        return (self.airborne_at or 0.0) + self.start_delay

    def _drone(self, msg: Odometry):
        if self.airborne_at is None and msg.pose.pose.position.z > self.start_altitude:
            self.airborne_at = self.t
            self.get_logger().info(
                f"aircraft above {self.start_altitude:.0f} m -- target sets off in "
                f"{self.start_delay:.0f} s")

    def _out_and_back(self):
        """Tier 1: long straight legs with a U-turn at each end."""
        u_turn = math.pi / self.turn_rate          # seconds for 180 degrees
        cycle = 2.0 * (self.leg + u_turn)
        u = (self.t - self._t_start()) % cycle
        if u < self.leg:
            return self.speed, 0.0
        u -= self.leg
        if u < u_turn:
            return self.speed * 0.4, self.turn_rate
        u -= u_turn
        if u < self.leg:
            return self.speed, 0.0
        return self.speed * 0.4, self.turn_rate

    def _phase(self):
        """Tier 2+: drive, corner, drive, stop -- a bounded circuit.

        The legs are shorter than tier 1's. Each cycle turns only 90 degrees,
        so with full-length legs the "circuit" is a square 80 m on a side and
        the target spends most of the run outside the area the aircraft can
        reasonably search. Measured: it reached 105 m from the origin.
        """
        leg = 0.6 * self.leg
        corner = (math.pi / 2.0) / self.turn_rate   # 90 degrees
        cycle = leg + corner + leg + self.stop
        u = (self.t - self._t_start()) % cycle
        if u < leg:
            return self.speed, 0.0
        u -= leg
        if u < corner:
            return self.speed * 0.5, self.turn_rate
        u -= corner
        if u < leg:
            return self.speed, 0.0
        return 0.0, 0.0          # stopped: v_hat is undefined, on purpose

    def update(self):
        self.t += self.dt

        if self.tier < 0:
            return               # idle: someone else owns /target/cmd_vel

        if self.airborne_at is None or self.t < self.airborne_at + self.start_delay:
            self.pub.publish(Twist())     # hold still until the aircraft is up
            return

        cmd = Twist()
        if self.tier == 0:
            pass                                   # stationary
        elif self.tier == 1:
            v, w = self._out_and_back()
            cmd.linear.x, cmd.angular.z = v, w
        else:
            v, w = self._phase()
            cmd.linear.x, cmd.angular.z = v, w

        self.pub.publish(cmd)

        if (self.tier >= 3 and self.t > self._t_start()
                and self.t - self.last_blackout >= self.bo_every):
            self.last_blackout = self.t
            self.pub_bo.publish(Float64(data=self.bo_len))
            self.get_logger().info(f"t={self.t:.0f}s: detector blackout {self.bo_len}s")


def main():
    rclpy.init()
    node = TargetRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
