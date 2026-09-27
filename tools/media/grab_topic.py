#!/usr/bin/env python3
"""Save one frame from an image topic.

Screenshotting the RViz image panel would work and would be worse: it carries
RViz's chrome, it is at whatever size the panel happened to be, and it is not
reproducible. This takes the image itself, at full resolution, exactly as the
detector published it.

    python3 grab_topic.py /detector/image_annotated out.png [timeout_s]
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def main():
    topic = sys.argv[1]
    out = sys.argv[2]
    timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0

    rclpy.init()
    node = Node("grab_topic")
    br = CvBridge()
    got = {}

    def cb(msg):
        got["img"] = br.imgmsg_to_cv2(msg, "bgr8")

    node.create_subscription(Image, topic, cb, qos_profile_sensor_data)
    end = node.get_clock().now().nanoseconds + timeout * 1e9
    while "img" not in got and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    if "img" not in got:
        print(f"no frame on {topic} in {timeout:.0f}s")
        sys.exit(1)
    cv2.imwrite(out, got["img"])
    print(f"{out}  {got['img'].shape[1]}x{got['img'].shape[0]}")


if __name__ == "__main__":
    main()
