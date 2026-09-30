#!/usr/bin/env python3
"""Draw chosen TF frames as labelled axes, for a screenshot.

Used by `make_media.sh frames` for the Day 3 slide "Where these frames sit on
the aircraft". RViz's own TF display cannot make that picture:

  * it draws frame names in white, invisible on a slide's light background;
  * it did not apply a per-frame filter loaded from a config file, so every
    frame in the tree came up (tried in Jazzy);
  * three of the four frames that matter sit within a few centimetres of
    another, so names drawn at the origins land on top of each other.

Nothing here is drawn by hand. Every arrow is a frame-locked marker in the
frame it belongs to, so RViz places it through the live /tf exactly as the TF
display would. The labels are the only thing positioned for legibility: each
one is looked up from /tf, pushed out by a fixed offset, and tied back to its
origin by a leader line.

    python3 frame_axes.py          # publishes /media/frame_axes until killed
"""
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

FIXED = "base_link"

# frame, axis length (m), label offset from the origin in base_link (m), and
# whether the name sits above or below the end of its leader line.
# The offsets are for the front-left view in frames_on_aircraft.rviz: labels
# go outward, away from the airframe, and at about the same depth as each
# other so perspective does not print one name larger than the rest. Axis lengths are long enough to clear
# the mesh the origin sits inside (the top plate, the camera ball).
FRAMES = [
    ("base_link",              0.25, (-0.12, 0.16, 0.14), +1),
    ("cgo3_vertical_arm_link", 0.15, (0.14, -0.20, 0.16), +1),
    ("camera_link",            0.15, (-0.18, 0.08, -0.21), -1),
    ("camera_optical_frame",   0.15, (0.26, -0.02, -0.12), -1),
]

RGB = [(0.85, 0.10, 0.10), (0.10, 0.62, 0.15), (0.10, 0.25, 0.85)]   # x y z
INK = (0.13, 0.15, 0.17)
SHAFT, HEAD, HEAD_LEN = 0.008, 0.018, 0.025
TEXT_H = 0.042      # sized for a projector: the slide shows this at about half width


def colour(rgb, a=1.0):
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=a)


class FrameAxes(Node):
    def __init__(self):
        super().__init__("frame_axes")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/media/frame_axes", latched)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_timer(0.5, self.publish)

    def publish(self):
        out = MarkerArray()
        mid = 0
        for frame, length, offset, side in FRAMES:
            try:
                t = self.tf.lookup_transform(FIXED, frame, Time()).transform.translation
            except Exception:           # not in the tree yet; try again next tick
                return
            for axis in range(3):
                m = Marker(type=Marker.ARROW, action=Marker.ADD, id=mid, ns="axes")
                m.header.frame_id = frame          # placed by /tf, not by us
                m.frame_locked = True
                tip = [0.0, 0.0, 0.0]
                tip[axis] = length
                m.points = [Point(), Point(x=tip[0], y=tip[1], z=tip[2])]
                m.scale.x, m.scale.y, m.scale.z = SHAFT, HEAD, HEAD_LEN
                m.color = colour(RGB[axis])
                m.pose.orientation.w = 1.0
                out.markers.append(m)
                mid += 1
            origin = Point(x=t.x, y=t.y, z=t.z)
            anchor = Point(x=t.x + offset[0], y=t.y + offset[1], z=t.z + offset[2])
            lead = Marker(type=Marker.LINE_LIST, action=Marker.ADD, id=mid, ns="leaders")
            lead.header.frame_id = FIXED
            lead.points = [origin, anchor]
            lead.scale.x = 0.002
            lead.color = colour(INK, 0.6)
            lead.pose.orientation.w = 1.0
            out.markers.append(lead)
            mid += 1
            text = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, id=mid, ns="names")
            text.header.frame_id = FIXED
            # clear of the leader's end, so the line never runs through the name
            text.pose.position = Point(x=anchor.x, y=anchor.y, z=anchor.z + side * 0.75 * TEXT_H)
            text.pose.orientation.w = 1.0
            text.scale.z = TEXT_H
            text.color = colour(INK)
            text.text = frame
            out.markers.append(text)
            mid += 1
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(FrameAxes())


if __name__ == "__main__":
    main()
