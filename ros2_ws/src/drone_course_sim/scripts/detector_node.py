#!/usr/bin/env python3
"""YOLO11n on the gimbal camera stream.

Course-provided infrastructure: students do not write this. What they write is
the node that turns these detections into a world position (Lab 5) and the one
that keeps the gimbal on the target (Lab 4).

Two details here matter more than the inference:

  * Detections carry the IMAGE's timestamp, never "now". Everything downstream
    looks the camera transform up by that stamp; restamping would silently
    reintroduce the pointing error the course spends a lecture on.
  * Inference is throttled. YOLO11n on CPU cannot keep up with a 30 Hz stream,
    and a node that queues frames it will never catch up on produces detections
    describing where the target used to be.

Topics
  in   /camera/image_raw            sensor_msgs/Image
  out  /detections                  vision_msgs/Detection2DArray
  out  /detector/image_annotated    sensor_msgs/Image   (optional)
  in   /detector/blackout           std_msgs/Float64    seconds to go blind

The blackout input exists for the robustness lab: publish a duration and the
detector silently stops reporting, so students can see their tracker coast on
prediction -- or fail to.
"""
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from vision_msgs.msg import (Detection2D, Detection2DArray,
                             ObjectHypothesisWithPose)


class Detector(Node):
    def __init__(self):
        super().__init__("detector")

        self.declare_parameter("model", os.environ.get("YOLO_WEIGHTS", "/opt/models/yolo11n.pt"))
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("max_rate", 10.0)          # Hz
        self.declare_parameter("publish_annotated", True)
        # COCO classes worth chasing on the ground. Empty list = keep everything.
        self.declare_parameter("classes", ["person", "car", "truck", "bus", "motorcycle"])

        self.conf = self.get_parameter("confidence").value
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.min_period = 1.0 / max(self.get_parameter("max_rate").value, 0.1)
        self.annotate = self.get_parameter("publish_annotated").value
        wanted = list(self.get_parameter("classes").value)

        from ultralytics import YOLO          # imported late: it is slow
        model_path = self.get_parameter("model").value
        self.model = YOLO(model_path)
        names = self.model.names
        self.class_filter = (
            [i for i, n in names.items() if n in wanted] if wanted else None)

        self.bridge = CvBridge()
        self.last_infer = 0.0
        self.blackout_until = 0.0
        self.n_frames = 0
        self.dt_sum = 0.0

        self.pub_det = self.create_publisher(Detection2DArray, "/detections", 10)
        self.pub_img = (self.create_publisher(Image, "/detector/image_annotated", 1)
                        if self.annotate else None)
        self.create_subscription(Image, "/camera/image_raw", self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Float64, "/detector/blackout", self.on_blackout, 10)
        self.create_timer(10.0, self._report)

        dev = "GPU" if self._cuda() else "CPU"
        self.get_logger().info(
            f"{os.path.basename(model_path)} on {dev}, <= {1/self.min_period:.0f} Hz, "
            f"conf {self.conf}, classes {wanted or 'all'}")

    @staticmethod
    def _cuda():
        try:
            import torch
            return torch.cuda.is_available()
        except Exception:
            return False

    def on_blackout(self, msg: Float64):
        self.blackout_until = time.monotonic() + max(0.0, msg.data)
        self.get_logger().warn(f"detector blind for {msg.data:.1f}s")

    def _report(self):
        if self.n_frames:
            self.get_logger().info(
                f"{self.n_frames} frames, mean inference {1000*self.dt_sum/self.n_frames:.0f} ms")
            self.n_frames = 0
            self.dt_sum = 0.0

    def on_image(self, msg: Image):
        now = time.monotonic()
        if now - self.last_infer < self.min_period:
            return                      # drop, do not queue
        self.last_infer = now

        out = Detection2DArray()
        # The image's stamp and frame, not the clock's. This is what makes the
        # TF lookup downstream correct.
        out.header = msg.header

        if now < self.blackout_until:
            self.pub_det.publish(out)   # still publish, just empty
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        t0 = time.monotonic()
        res = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf,
                                 classes=self.class_filter, verbose=False)[0]
        self.dt_sum += time.monotonic() - t0
        self.n_frames += 1

        for box in res.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            d = Detection2D()
            d.header = msg.header
            d.bbox.center.position.x = (x1 + x2) / 2.0
            d.bbox.center.position.y = (y1 + y2) / 2.0
            d.bbox.size_x = x2 - x1
            d.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.model.names[int(box.cls[0])]
            hyp.hypothesis.score = float(box.conf[0])
            d.results.append(hyp)
            out.detections.append(d)

        self.pub_det.publish(out)

        if self.pub_img is not None:
            img = self.bridge.cv2_to_imgmsg(res.plot(), encoding="bgr8")
            img.header = msg.header
            self.pub_img.publish(img)


def main():
    rclpy.init()
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
