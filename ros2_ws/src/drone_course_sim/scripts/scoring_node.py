#!/usr/bin/env python3
"""Score a capstone run against Gazebo ground truth.

Run it alongside a student's mission; it watches for a fixed duration and
prints a scored breakdown plus a JSON file.

    ros2 run drone_course_sim scoring_node.py --ros-args \
        -p duration:=120.0 -p standoff:=8.0 -p altitude:=15.0

Design decisions worth not re-litigating:

* **Ground truth comes from two OdometryPublisher plugins**, one on the target
  and one on the aircraft, bridged to /target/ground_truth and
  /drone/ground_truth. The obvious alternative -- bridging the world's
  /pose/info -- loses every link name, because gz.msgs.Pose_V maps onto
  tf2_msgs/TFMessage with empty frame_id and child_frame_id.

* **"Kept in frame" is measured by projecting ground truth into the camera**,
  not by counting detections. A team whose detector is flaky but whose gimbal
  is pointed correctly should not be punished twice, and a team that loses the
  target out of frame should not be rescued by a lucky false positive.

* **The standoff reference is recomputed from truth at every sample**, not read
  from the student's node. Scoring what they were asked to achieve, not what
  they decided to aim at, is the whole point.

* Scores are continuous, not pass/fail. A team that closes the loop on a
  straight-line target and nothing else must still score meaningfully, or the
  tiers do not do their job.

Nothing here may ever be used by flight code. There is no ground truth on the
real aircraft; a mission that reads it cannot transfer, which is the one thing
this course exists to demonstrate.
"""
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64, String

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)

GRN = "\033[32m"; YEL = "\033[33m"; RED = "\033[31m"; RST = "\033[0m"; BLD = "\033[1m"


def quat_to_rot(x, y, z, w):
    """Rotation matrix as a flat tuple of rows. No scipy in the image."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
        (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)),
    )


def rot_t_mul(R, v):
    """R^T v -- world vector into the frame R describes."""
    return tuple(sum(R[k][i] * v[k] for k in range(3)) for i in range(3))


class Scorer(Node):
    def __init__(self):
        super().__init__("scoring")

        self.declare_parameter("duration", 120.0)
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("standoff", 18.0)      # d, metres behind
        self.declare_parameter("altitude", 12.0)      # h, metres above
        # 8 m, not 5. The tolerance has to be larger than the error a team
        # cannot remove: about 3.5 m RMS of estimate error (most of it the
        # simulator's own 5 degree yaw bias) plus the few metres the standoff
        # reference itself sweeps while the target is turning. Set tighter, the
        # metric measures the simulator rather than the student.
        self.declare_parameter("station_tol", 8.0)    # m, counts as on station
        self.declare_parameter("altitude_floor", 5.0)
        self.declare_parameter("geofence_radius", 120.0)
        self.declare_parameter("estimate_timeout", 1.0)
        # Full marks at 2 m, not at the 0.5 m the pinhole error model alone
        # would suggest. EKF2's yaw in this simulator sits 5 to 6 degrees off
        # truth (reproducible with stock PX4 and the stock world, so not
        # something a student can fix), and a yaw error rotates the bearing ray
        # about the vertical: at the default 8 m standoff and 15 m altitude the
        # target is about 17 m away on the ground, so that alone is ~1.6 m.
        # Setting the threshold below the achievable floor would grade everyone
        # on the simulator's compass rather than on their own work.
        self.declare_parameter("rms_full_marks", 2.0)  # m
        self.declare_parameter("rms_zero_marks", 8.0)  # m
        self.declare_parameter("coast_tolerance", 8.0)  # m during a blackout
        self.declare_parameter("airborne_altitude", 2.0)
        self.declare_parameter("output", "/tmp/score.json")
        self.declare_parameter("tier", 1)

        g = lambda n: self.get_parameter(n).value
        self.duration = float(g("duration"))
        self.period = 1.0 / float(g("rate"))
        self.d = float(g("standoff"))
        self.h = float(g("altitude"))
        self.station_tol = float(g("station_tol"))
        self.alt_floor = float(g("altitude_floor"))
        self.fence = float(g("geofence_radius"))
        self.est_timeout = float(g("estimate_timeout"))
        self.rms_full = float(g("rms_full_marks"))
        self.rms_zero = float(g("rms_zero_marks"))
        self.coast_tol = float(g("coast_tolerance"))
        self.airborne = float(g("airborne_altitude"))
        self.output = str(g("output"))
        self.tier = int(g("tier"))

        # latest inputs
        self.truth_t = None      # (t, x, y, z, vx, vy)
        self.truth_d = None      # (t, x, y, z)
        self.est = None          # (t, x, y)
        self.cam = None          # (K, width, height)
        self.state = None
        self.blackout_until = -1.0
        self.heading = None      # latched unit heading of the target

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, "/target/ground_truth", self._truth_t, SENSOR_QOS)
        self.create_subscription(Odometry, "/drone/ground_truth", self._truth_d, SENSOR_QOS)
        self.create_subscription(PoseWithCovarianceStamped, "/target/estimate",
                                 self._estimate, 10)
        # A team may publish the filtered track instead of the raw locate.
        self.create_subscription(Odometry, "/target/track", self._track, 10)
        self.create_subscription(CameraInfo, "/camera/camera_info", self._cam, SENSOR_QOS)
        self.create_subscription(String, "/mission/state", self._state, 10)
        self.create_subscription(Float64, "/detector/blackout", self._blackout, 10)

        # accumulators
        self.n = 0
        self.n_airborne = 0
        self.n_scored = 0
        self.n_on_station = 0
        self.station_err_sum = 0.0
        self.n_est = 0
        self.est_sq_sum = 0.0
        self.est_worst = 0.0
        self.n_frame_checked = 0
        self.n_in_frame = 0
        self.n_blackout = 0
        self.n_blackout_ok = 0
        self.floor_violations = 0
        self.fence_violations = 0
        self.min_altitude = float("inf")
        self.max_radius = 0.0
        self.states_seen = set()
        self.t0 = None

        self.timer = self.create_timer(self.period, self._sample)
        self.get_logger().info(
            f"scoring for {self.duration:.0f} s -- standoff {self.d:.1f} m, "
            f"altitude {self.h:.1f} m, tolerance {self.station_tol:.1f} m")

    # ------------------------------------------------------------- callbacks
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _truth_t(self, msg):
        p, v = msg.pose.pose.position, msg.twist.twist.linear
        # The target's OdometryPublisher reports twist in the body frame, so
        # rotate it into the world before treating it as a heading.
        q = msg.pose.pose.orientation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        vw = tuple(sum(R[i][k] * (v.x, v.y, v.z)[k] for k in range(3)) for i in range(3))
        self.truth_t = (self._now(), p.x, p.y, p.z, vw[0], vw[1])

    def _truth_d(self, msg):
        p = msg.pose.pose.position
        self.truth_d = (self._now(), p.x, p.y, p.z)

    def _estimate(self, msg):
        p = msg.pose.pose.position
        self.est = (self._now(), p.x, p.y)

    def _track(self, msg):
        p = msg.pose.pose.position
        self.est = (self._now(), p.x, p.y)

    def _cam(self, msg):
        self.cam = (list(msg.k), msg.width, msg.height)

    def _state(self, msg):
        self.state = msg.data
        self.states_seen.add(msg.data)

    def _blackout(self, msg):
        if msg.data > 0.0:
            self.blackout_until = self._now() + float(msg.data)

    # ---------------------------------------------------------------- sample
    def _sample(self):
        now = self._now()
        if self.t0 is None:
            if self.truth_t is None or self.truth_d is None:
                return          # nothing to score yet; the sim may still be coming up
            self.t0 = now
        if now - self.t0 > self.duration:
            self._finish()
            return

        self.n += 1
        if self.truth_t is None or self.truth_d is None:
            return
        _, tx, ty, tz, tvx, tvy = self.truth_t
        _, dx, dy, dz = self.truth_d

        self.min_altitude = min(self.min_altitude, dz)
        radius = math.hypot(dx, dy)
        self.max_radius = max(self.max_radius, radius)

        airborne = dz > self.airborne
        # A descent during RTL or landing, and the climb-out during TAKEOFF,
        # are the mission working -- not a clamp being violated. Scoring the
        # deliberate landing as a floor breach punished the reference solution
        # for ending correctly, and scoring the climb punished it for starting.
        descending_on_purpose = self.state in ("RTL", "LAND", "IDLE", "TAKEOFF")
        if airborne:
            self.n_airborne += 1
            if dz < self.alt_floor and not descending_on_purpose:
                self.floor_violations += 1
            if radius > self.fence:
                self.fence_violations += 1

        # --- 1. time on station -------------------------------------------
        speed = math.hypot(tvx, tvy)
        if speed > 0.5:
            self.heading = (tvx / speed, tvy / speed)
        if self.heading is None:
            # The target has not moved yet, so "behind it" is meaningless.
            # Score the standoff on whatever bearing the aircraft is holding,
            # which is what the guidance law is asked to do in that case.
            bx, by = dx - tx, dy - ty
            n = math.hypot(bx, by)
            if n > 0.5:
                px, py = tx + self.d * bx / n, ty + self.d * by / n
            else:
                px, py = tx + self.d, ty
        else:
            px = tx - self.d * self.heading[0]
            py = ty - self.d * self.heading[1]
        pz = tz + self.h
        err = math.sqrt((dx - px) ** 2 + (dy - py) ** 2 + (dz - pz) ** 2)
        # Count only the part of the run the team is being asked to fly. The
        # climb to altitude and the trip home are not following, and averaging
        # them in makes the metric mostly a function of how long the run was.
        scoring_now = airborne and self.state not in ("RTL", "LAND", "IDLE", "TAKEOFF")
        if scoring_now:
            self.n_scored += 1
            self.station_err_sum += err
            if err <= self.station_tol:
                self.n_on_station += 1

        # --- 2. estimation quality ----------------------------------------
        if scoring_now and self.est is not None and (now - self.est[0]) < self.est_timeout:
            e = math.hypot(self.est[1] - tx, self.est[2] - ty)
            self.n_est += 1
            self.est_sq_sum += e * e
            self.est_worst = max(self.est_worst, e)
            if now < self.blackout_until:
                self.n_blackout += 1
                if e <= self.coast_tol:
                    self.n_blackout_ok += 1
        elif scoring_now and now < self.blackout_until:
            # No estimate at all during a blackout is exactly the failure the
            # tier is designed to expose.
            self.n_blackout += 1

        # --- 3. target kept in the camera frame ---------------------------
        if scoring_now and self.cam is not None:
            self.n_frame_checked += 1
            if self._projects_inside(tx, ty, tz):
                self.n_in_frame += 1

    def _projects_inside(self, tx, ty, tz):
        try:
            tr = self.tf_buffer.lookup_transform(
                "camera_optical_frame", "map", rclpy.time.Time())
        except Exception:
            return False
        t = tr.transform.translation
        q = tr.transform.rotation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        # point in optical frame
        p = (R[0][0] * tx + R[0][1] * ty + R[0][2] * tz + t.x,
             R[1][0] * tx + R[1][1] * ty + R[1][2] * tz + t.y,
             R[2][0] * tx + R[2][1] * ty + R[2][2] * tz + t.z)
        if p[2] <= 0.1:
            return False        # behind the camera
        K, w, h = self.cam
        u = K[0] * p[0] / p[2] + K[2]
        v = K[4] * p[1] / p[2] + K[5]
        return 0.0 <= u < w and 0.0 <= v < h

    # ---------------------------------------------------------------- report
    def _finish(self):
        self.timer.cancel()
        frac = lambda a, b: (a / b) if b else 0.0

        on_station = frac(self.n_on_station, self.n_scored)
        s_station = 30.0 * on_station

        rms = math.sqrt(self.est_sq_sum / self.n_est) if self.n_est else None
        coverage = frac(self.n_est, self.n_scored)
        if rms is None:
            s_est = 0.0
        else:
            span = max(self.rms_zero - self.rms_full, 1e-6)
            quality = max(0.0, min(1.0, (self.rms_zero - rms) / span))
            # Accuracy is only worth what it covers: an estimate that is
            # perfect for two seconds out of sixty is not a working locator.
            s_est = 20.0 * quality * coverage

        in_frame = frac(self.n_in_frame, self.n_frame_checked)
        s_frame = 20.0 * in_frame

        s_floor = 5.0 if self.floor_violations == 0 else 0.0
        s_fence = 5.0 if self.fence_violations == 0 else 0.0
        if self.n_blackout == 0:
            s_coast, coast_note = 5.0, "no blackout occurred (tier < 3)"
            coast_frac = None
        else:
            coast_frac = frac(self.n_blackout_ok, self.n_blackout)
            s_coast = 5.0 * coast_frac
            coast_note = f"{coast_frac * 100:.0f}% of blackout samples within {self.coast_tol:.0f} m"
        s_robust = s_floor + s_fence + s_coast

        automated = s_station + s_est + s_frame + s_robust

        result = {
            "tier": self.tier,
            "duration_s": round(self.duration, 1),
            "samples": self.n,
            "airborne_samples": self.n_airborne,
            "scored_samples": self.n_scored,
            "standoff_m": self.d,
            "altitude_m": self.h,
            "time_on_station": {
                "fraction": round(on_station, 4),
                "mean_error_m": round(frac(self.station_err_sum, self.n_scored), 3),
                "score": round(s_station, 2), "weight": 30},
            "estimation": {
                "rms_error_m": round(rms, 3) if rms is not None else None,
                "worst_error_m": round(self.est_worst, 3) if self.n_est else None,
                "coverage": round(coverage, 4),
                "score": round(s_est, 2), "weight": 20},
            "kept_in_frame": {
                "fraction": round(in_frame, 4),
                "score": round(s_frame, 2), "weight": 20},
            "robustness": {
                "altitude_floor_violations": self.floor_violations,
                "min_altitude_m": round(self.min_altitude, 2)
                                  if self.min_altitude != float("inf") else None,
                "geofence_violations": self.fence_violations,
                "max_radius_m": round(self.max_radius, 2),
                "coasting": coast_note,
                "coast_fraction": round(coast_frac, 4) if coast_frac is not None else None,
                "score": round(s_robust, 2), "weight": 15},
            "mission_states_seen": sorted(self.states_seen),
            "automated_total": round(automated, 2),
            "automated_out_of": 85,
            "manual_remaining": "15 for code quality, reproducibility, report and demo",
        }

        try:
            with open(self.output, "w") as f:
                json.dump(result, f, indent=2)
        except OSError as exc:
            self.get_logger().warn(f"could not write {self.output}: {exc}")

        def line(name, score, weight, detail):
            colour = GRN if score >= 0.75 * weight else (YEL if score >= 0.4 * weight else RED)
            print(f"  {name:<24} {colour}{score:5.1f}{RST} / {weight:<3} {detail}")

        print()
        print(f"{BLD}Capstone score -- tier {self.tier}, {self.duration:.0f} s{RST}")
        print()
        line("Time on station", s_station, 30,
             f"{on_station*100:.0f}% within {self.station_tol:.0f} m "
             f"(mean error {frac(self.station_err_sum, self.n_scored):.1f} m)")
        line("Estimation quality", s_est, 20,
             (f"RMS {rms:.2f} m, worst {self.est_worst:.2f} m, "
              f"present {coverage*100:.0f}% of the time")
             if rms is not None
             else "no estimate published on /target/estimate or /target/track")
        line("Kept in camera frame", s_frame, 20, f"{in_frame*100:.0f}% of samples")
        line("Robustness", s_robust, 15,
             f"floor {self.floor_violations}, fence {self.fence_violations}, {coast_note}")
        print()
        print(f"  {BLD}{'Automated total':<24} {automated:5.1f} / 85{RST}"
              f"   (+15 manual: code, reproducibility, report, demo)")
        print()
        if self.n_scored == 0:
            print(f"  {YEL}The aircraft never got above {self.airborne:.0f} m. "
                  f"Nothing flight-related could be scored.{RST}")
        if self.n_est == 0:
            print(f"  {YEL}Nothing was published on /target/estimate "
                  f"(PoseWithCovarianceStamped) or /target/track (Odometry).{RST}")
        print(f"  written to {self.output}")
        print()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = Scorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
