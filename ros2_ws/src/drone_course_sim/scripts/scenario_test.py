#!/usr/bin/env python3
"""End-to-end check of the course scenario.

Asserts the things a lab actually depends on, in the order they would break:
the transform chain, the camera, that the detector really finds the target
vehicle, that ground truth is available, that the target drives, and that a
blackout silences detection.

The detection check is the important one. Everything else can pass while the
detector quietly sees nothing -- an empty world, a camera pointing the wrong
way, a confidence threshold that is too high -- and the first person to notice
would be a student in the middle of Lab 4.

Run it against a live sim launched with `tier:=-1`, so the route driver stays
idle and this test owns /target/cmd_vel. With a route running, the target has
usually driven out of frame before the detector check even starts -- which is
how the "drives to infinity" bug in tier 1 was found.

    ros2 launch drone_course_sim sim_bringup.launch.py tier:=-1
    ros2 run drone_course_sim scenario_test.py
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from vision_msgs.msg import Detection2DArray

VEHICLE_CLASSES = {"car", "truck", "bus"}


class ScenarioTest(Node):
    def __init__(self):
        super().__init__("scenario_test")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)

        self.images = 0
        self.det_msgs = 0
        self.vehicle_hits = 0
        self.best_conf = 0.0
        self.odom = None
        self.odom_first = None

        self.create_subscription(Image, "/camera/image_raw",
                                 lambda m: self._bump("images"), qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "/detections", self.on_det, 10)
        self.create_subscription(Odometry, "/target/ground_truth", self.on_odom, 10)
        self.pub_bo = self.create_publisher(Float64, "/detector/blackout", 10)
        self.pub_cmd = self.create_publisher(Twist, "/target/cmd_vel", 10)

        self.failures = []
        self.checks = 0

    def _bump(self, attr):
        setattr(self, attr, getattr(self, attr) + 1)

    def on_det(self, msg: Detection2DArray):
        self.det_msgs += 1
        for d in msg.detections:
            for r in d.results:
                if r.hypothesis.class_id in VEHICLE_CLASSES:
                    self.vehicle_hits += 1
                    self.best_conf = max(self.best_conf, r.hypothesis.score)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, p.z)
        if self.odom_first is None:
            self.odom_first = self.odom

    # -- harness -----------------------------------------------------------
    def spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def check(self, ok, label, detail=""):
        self.checks += 1
        print(f"  {'ok  ' if ok else 'FAIL'} {label}" + (f"  [{detail}]" if detail else ""))
        if not ok:
            self.failures.append(label)

    def run(self):
        print("Scenario test\n")

        # 1. transform chain
        deadline = time.monotonic() + 25
        chain = False
        while rclpy.ok() and time.monotonic() < deadline and not chain:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                self.buf.lookup_transform("map", "camera_optical_frame", rclpy.time.Time())
                chain = True
            except tf2_ros.TransformException:
                pass
        self.check(chain, "map -> camera_optical_frame resolves")

        # 2/3. camera and detector, watched together
        print("\n  watching camera and detector for 20 s ...")
        self.spin(20.0)
        self.check(self.images > 0, "camera publishing", f"{self.images} frames")
        self.check(self.det_msgs > 0, "detector publishing", f"{self.det_msgs} messages")
        self.check(self.vehicle_hits > 0,
                   "detector FINDS the target vehicle",
                   f"{self.vehicle_hits} hits, best conf {self.best_conf:.2f}")

        # 4. ground truth
        self.check(self.odom is not None, "ground truth odometry available",
                   f"{self.odom}" if self.odom else "")

        # 5. the target can be driven. We command it ourselves rather than
        #    relying on a route, so the check is deterministic.
        if self.odom is not None:
            print("\n  commanding the target for 6 s ...")
            start = self.odom
            cmd = Twist()
            cmd.linear.x = 3.0
            end = time.monotonic() + 6.0
            while rclpy.ok() and time.monotonic() < end:
                self.pub_cmd.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
            moved = math.dist(self.odom[:2], start[:2])
            self.pub_cmd.publish(Twist())      # stop
            self.spin(1.0)
            self.check(moved > 3.0, "target responds to cmd_vel",
                       f"moved {moved:.1f} m in 6 s at 3 m/s")

        # 6. blackout silences detection
        print("\n  testing blackout ...")
        before = self.vehicle_hits
        self.pub_bo.publish(Float64(data=5.0))
        self.spin(1.0)
        mid = self.vehicle_hits
        self.spin(3.0)
        during = self.vehicle_hits - mid
        self.check(during == 0, "blackout silences detection",
                   f"{during} hits during blackout")
        self.spin(4.0)
        self.check(self.vehicle_hits > mid, "detection recovers after blackout",
                   f"{self.vehicle_hits - mid} hits after")

        print()
        if self.failures:
            print(f"{len(self.failures)}/{self.checks} checks FAILED: "
                  + ", ".join(self.failures))
            return 1
        print(f"All {self.checks} checks passed.")
        return 0


def main():
    rclpy.init()
    node = ScenarioTest()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
