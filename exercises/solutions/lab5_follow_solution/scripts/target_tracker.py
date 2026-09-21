#!/usr/bin/env python3
"""Lab 5 reference solution, part 1 -- a four-state constant-velocity filter.

    x = [x, y, vx, vy]      on the ground plane, in map

Four states, not six, because the target is on a known plane. Two reasons this
filter is not optional, both worth saying out loud:

  * It is the **only** source of the target's velocity, and the guidance law
    needs it. Differencing consecutive fixes gives you velocity with the
    measurement noise differentiated too -- at 10 Hz and 0.5 m of noise that is
    5 m/s of garbage on a 3 m/s target.
  * It **coasts through dropouts**, which are guaranteed the moment the target
    is small, shadowed, or clipped by the frame edge.

R comes from the locator's covariance, not from a constant. A filter told the
measurement is better than it is will chase noise; told it is worse, it will
ignore the measurement and drift. Both look like "the filter is badly tuned"
and neither is.

The Mahalanobis gate is what stops one false positive from dragging the track
across the field. It is three lines and it is the difference between a filter
and a liability.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def mat_mul(A, B):
    n, m, p = len(A), len(B), len(B[0])
    return [[sum(A[i][k] * B[k][j] for k in range(m)) for j in range(p)] for i in range(n)]


def mat_add(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def transpose(A):
    return [list(r) for r in zip(*A)]


def inv2(M):
    d = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    if abs(d) < 1e-12:
        return None
    return [[M[1][1] / d, -M[0][1] / d], [-M[1][0] / d, M[0][0] / d]]


class TargetTracker(Node):
    def __init__(self):
        super().__init__("target_tracker")

        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("accel_sigma", 1.5)     # m/s^2, process noise
        self.declare_parameter("gate_chi2", 9.21)      # 2 dof, 99%
        self.declare_parameter("max_coast", 5.0)       # s before the track dies
        self.declare_parameter("init_speed_var", 25.0)

        g = lambda n: self.get_parameter(n).value
        self.dt = 1.0 / float(g("rate_hz"))
        self.q_sigma = float(g("accel_sigma"))
        self.gate = float(g("gate_chi2"))
        self.max_coast = float(g("max_coast"))
        self.init_speed_var = float(g("init_speed_var"))

        self.x = None
        self.P = None
        self.t_last_update = None
        self.n_gated = 0
        self.n_upd = 0

        self.pub = self.create_publisher(Odometry, "/target/track", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/target/estimate",
                                 self._on_measurement, 10)
        self.create_timer(self.dt, self._tick)
        self.create_timer(10.0, self._report)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _report(self):
        if self.n_upd or self.n_gated:
            self.get_logger().info(
                f"{self.n_upd} updates, {self.n_gated} rejected by the gate")
            self.n_upd = self.n_gated = 0

    # ------------------------------------------------------------- predict
    def _predict(self, dt):
        # F = [[I, dt I], [0, I]]
        x = self.x
        self.x = [x[0] + dt * x[2], x[1] + dt * x[3], x[2], x[3]]
        F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        # Piecewise-constant white acceleration. Writing Q this way rather than
        # as a diagonal guess is what makes position and velocity uncertainty
        # grow together, which is what actually happens.
        s = self.q_sigma ** 2
        d4, d3, d2 = dt ** 4 / 4.0, dt ** 3 / 2.0, dt ** 2
        Q = [[d4 * s, 0, d3 * s, 0],
             [0, d4 * s, 0, d3 * s],
             [d3 * s, 0, d2 * s, 0],
             [0, d3 * s, 0, d2 * s]]
        self.P = mat_add(mat_mul(mat_mul(F, self.P), transpose(F)), Q)

    # -------------------------------------------------------------- update
    def _on_measurement(self, msg):
        z = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        c = msg.pose.covariance
        R = [[c[0], c[1]], [c[6], c[7]]]
        if R[0][0] <= 0.0 or R[1][1] <= 0.0:
            R = [[1.0, 0.0], [0.0, 1.0]]     # a locator that reports nothing

        now = self._now()
        if self.x is None:
            self.x = [z[0], z[1], 0.0, 0.0]
            self.P = [[R[0][0], R[0][1], 0, 0],
                      [R[1][0], R[1][1], 0, 0],
                      [0, 0, self.init_speed_var, 0],
                      [0, 0, 0, self.init_speed_var]]
            self.t_last_update = now
            self.get_logger().info(f"track initialised at ({z[0]:.1f}, {z[1]:.1f})")
            return

        # S = H P H' + R, with H = [I 0]
        S = [[self.P[0][0] + R[0][0], self.P[0][1] + R[0][1]],
             [self.P[1][0] + R[1][0], self.P[1][1] + R[1][1]]]
        Si = inv2(S)
        if Si is None:
            return
        nu = [z[0] - self.x[0], z[1] - self.x[1]]
        d2 = (nu[0] * (Si[0][0] * nu[0] + Si[0][1] * nu[1]) +
              nu[1] * (Si[1][0] * nu[0] + Si[1][1] * nu[1]))
        if d2 > self.gate:
            self.n_gated += 1
            return

        # K = P H' S^-1, H' picks the first two columns of P
        PHt = [[self.P[i][0], self.P[i][1]] for i in range(4)]
        K = mat_mul(PHt, Si)
        for i in range(4):
            self.x[i] += K[i][0] * nu[0] + K[i][1] * nu[1]
        # Joseph form is not needed at this size, but subtracting K H P must be
        # done against the ORIGINAL P, so build the new one rather than editing.
        HP = [[self.P[0][j] for j in range(4)], [self.P[1][j] for j in range(4)]]
        KHP = mat_mul(K, HP)
        self.P = [[self.P[i][j] - KHP[i][j] for j in range(4)] for i in range(4)]
        self.t_last_update = now
        self.n_upd += 1

    # ---------------------------------------------------------------- loop
    def _tick(self):
        if self.x is None:
            return
        now = self._now()
        age = now - self.t_last_update
        if age > self.max_coast:
            self.get_logger().warn(
                f"track dropped after coasting {age:.1f} s with no measurement")
            self.x = None
            self.P = None
            return

        self._predict(self.dt)

        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.child_frame_id = "map"
        m.pose.pose.position.x = self.x[0]
        m.pose.pose.position.y = self.x[1]
        m.pose.pose.position.z = 0.0
        m.pose.pose.orientation.w = 1.0
        m.twist.twist.linear.x = self.x[2]
        m.twist.twist.linear.y = self.x[3]
        cov = [0.0] * 36
        cov[0], cov[1] = self.P[0][0], self.P[0][1]
        cov[6], cov[7] = self.P[1][0], self.P[1][1]
        m.pose.covariance = cov
        tcov = [0.0] * 36
        tcov[0], tcov[1] = self.P[2][2], self.P[2][3]
        tcov[6], tcov[7] = self.P[3][2], self.P[3][3]
        m.twist.covariance = tcov
        self.pub.publish(m)


def main():
    rclpy.init()
    node = TargetTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()
