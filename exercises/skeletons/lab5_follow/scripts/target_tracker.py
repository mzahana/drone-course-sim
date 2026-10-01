#!/usr/bin/env python3
"""Lab 5 skeleton, part 1 -- a constant-velocity Kalman filter for
the target on the ground.

Your tasks:
    TODO 1  Predict step
    TODO 2  Update step, with a gate
Search this file for 'TODO' -- you only need to edit between the
ADD YOUR CODE BELOW / END OF YOUR CODE lines.

State:        x = [x, y, vx, vy]   position (m) and velocity (m/s), in map.
Model:        the target moves at constant velocity; its unknown
              accelerations are treated as noise (the process noise Q).
Measurement:  z = [x, y] from /target/estimate, with covariance R.
Output:       /target/track (nav_msgs/Odometry) with position and velocity.

The target is on the ground, so 4 states are enough (no height).

Why this filter is needed:
  * It is the only source of the target's velocity, and the guidance law
    needs that velocity. Subtracting two noisy positions gives a very noisy
    velocity: at 10 Hz with 0.5 m of position noise, that is about 5 m/s of
    noise on a 3 m/s target.
  * It keeps predicting (it "coasts") when detections stop. That happens
    whenever the target is small, hidden, or at the edge of the image.

Two design choices:
  * R is taken from the covariance the locator publishes, not a constant.
    If R is too small, the filter follows the noise. If R is too large, the
    filter ignores the measurements and drifts.
  * A Mahalanobis gate rejects a measurement that is too far from the
    prediction, so one false detection cannot pull the track away.

Matrices are plain Python lists of rows. The small helpers below do the
matrix algebra.
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def mat_mul(A, B):
    """Matrix product A * B."""
    n, m, p = len(A), len(B), len(B[0])
    return [[sum(A[i][k] * B[k][j] for k in range(m)) for j in range(p)] for i in range(n)]


def mat_add(A, B):
    """Matrix sum A + B."""
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]


def transpose(A):
    """Matrix transpose A^T."""
    return [list(r) for r in zip(*A)]


def inv2(M):
    """Inverse of a 2x2 matrix, or None if it cannot be inverted."""
    d = M[0][0] * M[1][1] - M[0][1] * M[1][0]     # determinant
    if abs(d) < 1e-12:
        return None
    return [[M[1][1] / d, -M[0][1] / d], [-M[1][0] / d, M[0][0] / d]]


class TargetTracker(Node):
    def __init__(self):
        super().__init__("target_tracker")

        self.declare_parameter("rate_hz", 20.0)        # predict and publish rate
        self.declare_parameter("accel_sigma", 1.5)     # m/s^2, std of the target's unknown acceleration
        # Gate threshold: 9.21 is the 99% chi-square value for 2 degrees of
        # freedom, so a correct measurement passes the gate 99% of the time.
        self.declare_parameter("gate_chi2", 9.21)
        self.declare_parameter("max_coast", 5.0)       # s without a measurement before the track is dropped
        self.declare_parameter("init_speed_var", 25.0) # (m/s)^2, initial velocity variance (std 5 m/s)

        g = lambda n: self.get_parameter(n).value    # short name for reading a parameter
        self.dt = 1.0 / float(g("rate_hz"))
        self.q_sigma = float(g("accel_sigma"))
        self.gate = float(g("gate_chi2"))
        self.max_coast = float(g("max_coast"))
        self.init_speed_var = float(g("init_speed_var"))

        self.x = None              # state [x, y, vx, vy]; None = no track yet
        self.P = None              # 4x4 state covariance
        self.t_last_update = None  # time of the last accepted measurement (s)
        self.n_gated = 0           # measurements rejected by the gate (for the log)
        self.n_upd = 0             # measurements accepted (for the log)

        self.pub = self.create_publisher(Odometry, "/target/track", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/target/estimate",
                                 self._on_measurement, 10)
        self.create_timer(self.dt, self._tick)
        self.create_timer(10.0, self._report)

    def _now(self):
        """Current ROS time in seconds (sim time in simulation)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _report(self):
        if self.n_upd or self.n_gated:
            self.get_logger().info(
                f"{self.n_upd} updates, {self.n_gated} rejected by the gate")
            self.n_upd = self.n_gated = 0

    # ------------------------------------------------------------- predict
    def _predict(self, dt):
        # ------------------------------------------------------------------
        # TODO 1: Predict step
        #   - Move the state forward by dt: x_new = F x, with
        #     F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]
        #     (position += dt * velocity; velocity stays the same).
        #   - Grow the covariance: P_new = F P F^T + Q.
        #   - Q models an unknown acceleration with standard deviation
        #     self.q_sigma. With s = q_sigma^2, for each axis (x and y):
        #       position variance         dt^4 / 4 * s
        #       position-velocity term    dt^3 / 2 * s
        #       velocity variance         dt^2 * s
        #     All other entries of Q are 0.
        #   - Store the results in self.x and self.P.
        #   - mat_mul, mat_add and transpose are defined at the top of the file.
        #   - Try a simple diagonal Q as well, and compare the velocity
        #     estimate during a detection dropout.
        # ======================= ADD YOUR CODE BELOW =======================
        raise NotImplementedError("TODO 1 in target_tracker.py: Predict step")  # delete this line and write your code here
        # ======================= END OF YOUR CODE ==========================

    # -------------------------------------------------------------- update
    def _on_measurement(self, msg):
        # z = measured position. R = its 2x2 covariance, taken from the
        # x-y block of the 6x6 ROS covariance (indices 0, 1, 6, 7).
        z = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        c = msg.pose.covariance
        R = [[c[0], c[1]], [c[6], c[7]]]
        if R[0][0] <= 0.0 or R[1][1] <= 0.0:
            R = [[1.0, 0.0], [0.0, 1.0]]     # no covariance was sent: use 1 m^2

        now = self._now()
        if self.x is None:
            # First measurement: start the track here, with zero velocity
            # and a large velocity variance (we do not know it yet).
            self.x = [z[0], z[1], 0.0, 0.0]
            self.P = [[R[0][0], R[0][1], 0, 0],
                      [R[1][0], R[1][1], 0, 0],
                      [0, 0, self.init_speed_var, 0],
                      [0, 0, 0, self.init_speed_var]]
            self.t_last_update = now
            self.get_logger().info(f"track initialised at ({z[0]:.1f}, {z[1]:.1f})")
            return

        # ------------------------------------------------------------------
        # TODO 2: Update step, with a gate
        #   - H = [I 0] picks the position out of the state. So H x is
        #     (self.x[0], self.x[1]) and H P H^T is the top-left 2x2 of P.
        #   - innovation:              nu = z - H x
        #   - innovation covariance:   S = H P H^T + R
        #     Invert it with inv2(S). If that returns None, return.
        #   - gate: d2 = nu^T S^-1 nu. If d2 > self.gate, reject the
        #     measurement: add 1 to self.n_gated and return.
        #   - gain:        K = P H^T S^-1   (4x2; P H^T = first two columns of P)
        #   - state:       x = x + K nu
        #   - covariance:  P = P - K H P    (H P = first two rows of P)
        #     Build the new P from the OLD P. Do not change P in place while
        #     you are still reading from it.
        #   - At the end, set self.t_last_update = now and add 1 to self.n_upd.
        #   - Without the gate, one false detection far away pulls the whole
        #     track across the field.
        # ======================= ADD YOUR CODE BELOW =======================
        raise NotImplementedError("TODO 2 in target_tracker.py: Update step, with a gate")  # delete this line and write your code here
        # ======================= END OF YOUR CODE ==========================

    # ---------------------------------------------------------------- loop
    def _tick(self):
        if self.x is None:
            return
        # Drop the track if no measurement was accepted for max_coast seconds.
        now = self._now()
        age = now - self.t_last_update
        if age > self.max_coast:
            self.get_logger().warn(
                f"track dropped after coasting {age:.1f} s with no measurement")
            self.x = None
            self.P = None
            return

        # Predict on every tick, with or without a measurement, so the track
        # keeps moving during a dropout.
        self._predict(self.dt)

        # Publish position and velocity, both in map.
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
        # 6x6 ROS covariances, stored row by row: index 0 = xx, 1 = xy,
        # 6 = yx, 7 = yy. The pose gets P's position block, the twist gets
        # P's velocity block.
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
