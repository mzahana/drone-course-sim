#!/usr/bin/env python3
"""Check that base_link -> camera_optical_frame is actually correct.

Getting this wrong is invisible: nothing crashes, the TF tree looks fine in
RViz, and geolocation results are simply biased. So it is asserted rather than
eyeballed.

With every gimbal joint at zero the camera looks straight ahead, so expressed
in base_link (FLU) the optical axes must be:

    optical z (view direction) = +x   forward
    optical x (image right)    = -y   aircraft right
    optical y (image down)     = -z   aircraft down

Run it against a live sim:  ros2 run drone_course_sim verify_camera_frame.py
"""
import sys
import math

import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import JointState

TOL = 0.02  # ~1.1 deg

EXPECTED = {
    "optical x (image right)": ((0.0, -1.0, 0.0), "aircraft right"),
    "optical y (image down)": ((0.0, 0.0, -1.0), "aircraft down"),
    "optical z (view dir)": ((1.0, 0.0, 0.0), "aircraft forward"),
}


def quat_to_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_camera_frame")
        self.buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buf, self)
        self.joints = None
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)

    def _on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def run(self, timeout=30.0):
        deadline = self.get_clock().now().nanoseconds + timeout * 1e9
        tf = None
        while rclpy.ok() and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                tf = self.buf.lookup_transform(
                    "base_link", "camera_optical_frame", rclpy.time.Time())
                if self.joints:
                    break
            except tf2_ros.TransformException:
                continue

        if tf is None:
            print("FAIL: no base_link -> camera_optical_frame transform. "
                  "Is the sim running and sim_bringup launched?")
            return 1

        if self.joints:
            worst = max((abs(v) for v in self.joints.values()), default=0.0)
            print("gimbal joints:", {k: round(v, 4) for k, v in self.joints.items()})
            if worst > 0.05:
                print(f"NOTE: a joint is at {math.degrees(worst):.1f} deg, not zero. "
                      "Centre the gimbal before trusting this check.")

        q = tf.transform.rotation
        R = quat_to_matrix((q.x, q.y, q.z, q.w))
        # Columns of R are the optical frame's axes expressed in base_link.
        axes = {
            "optical x (image right)": (R[0][0], R[1][0], R[2][0]),
            "optical y (image down)": (R[0][1], R[1][1], R[2][1]),
            "optical z (view dir)": (R[0][2], R[1][2], R[2][2]),
        }

        print(f"\ntranslation: ({tf.transform.translation.x:+.4f}, "
              f"{tf.transform.translation.y:+.4f}, {tf.transform.translation.z:+.4f})\n")

        failed = 0
        for label, (exp, meaning) in EXPECTED.items():
            got = axes[label]
            err = max(abs(a - b) for a, b in zip(got, exp))
            ok = err < TOL
            failed += 0 if ok else 1
            mark = "ok  " if ok else "FAIL"
            print(f"  {mark} {label:24s} = ({got[0]:+.3f}, {got[1]:+.3f}, {got[2]:+.3f})"
                  f"  expected ({exp[0]:+.0f}, {exp[1]:+.0f}, {exp[2]:+.0f})  {meaning}")

        print()
        if failed:
            print(f"{failed} axis/axes wrong. Geolocation WILL be biased -- fix the URDF "
                  "before using this for anything.")
            return 1
        print("Camera optical frame is correct.")
        return 0


def main():
    rclpy.init()
    node = Verifier()
    try:
        rc = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
