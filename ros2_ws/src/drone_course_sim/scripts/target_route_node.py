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
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class TargetRoute(Node):
    def __init__(self):
        super().__init__("target_route")

        self.declare_parameter("tier", 1)
        self.declare_parameter("speed", 3.0)           # m/s
        self.declare_parameter("turn_rate", 0.25)      # rad/s in tier 2+
        self.declare_parameter("leg_seconds", 12.0)    # straight leg before a turn
        self.declare_parameter("turn_seconds", 6.0)
        self.declare_parameter("stop_seconds", 5.0)
        self.declare_parameter("blackout_every", 25.0)
        self.declare_parameter("blackout_length", 3.0)

        self.tier = int(self.get_parameter("tier").value)
        self.speed = float(self.get_parameter("speed").value)
        self.turn_rate = float(self.get_parameter("turn_rate").value)
        self.leg = float(self.get_parameter("leg_seconds").value)
        self.turn = float(self.get_parameter("turn_seconds").value)
        self.stop = float(self.get_parameter("stop_seconds").value)
        self.bo_every = float(self.get_parameter("blackout_every").value)
        self.bo_len = float(self.get_parameter("blackout_length").value)

        self.pub = self.create_publisher(Twist, "/target/cmd_vel", 10)
        self.pub_bo = self.create_publisher(Float64, "/detector/blackout", 10)

        self.t = 0.0
        self.dt = 0.05
        self.last_blackout = 0.0
        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"target route tier {self.tier}")

    def _out_and_back(self):
        """Tier 1: long straight legs with a U-turn at each end."""
        u_turn = math.pi / self.turn_rate          # seconds for 180 degrees
        cycle = 2.0 * (self.leg + u_turn)
        u = self.t % cycle
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
        """Tier 2+: drive, corner, drive, stop -- a bounded circuit."""
        corner = (math.pi / 2.0) / self.turn_rate   # 90 degrees
        cycle = self.leg + corner + self.leg + self.stop
        u = self.t % cycle
        if u < self.leg:
            return self.speed, 0.0
        u -= self.leg
        if u < corner:
            return self.speed * 0.5, self.turn_rate
        u -= corner
        if u < self.leg:
            return self.speed, 0.0
        return 0.0, 0.0          # stopped: v_hat is undefined, on purpose

    def update(self):
        self.t += self.dt

        if self.tier < 0:
            return               # idle: someone else owns /target/cmd_vel

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

        if self.tier >= 3 and self.t - self.last_blackout >= self.bo_every:
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
