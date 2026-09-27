#!/usr/bin/env python3
"""Measure detector confidence against look-down angle. Repeatably.

The course chooses its flight geometry from this measurement, so the
measurement has to be trustworthy. The obvious way to take it -- fly the
aircraft, point the gimbal, read the confidence -- is not: the aircraft is
never exactly where you asked, the gimbal is stabilising against a body that
is still settling, and a target half out of frame scores whatever it scores.
Two runs of that disagree by 0.5 confidence, and then you are measuring your
own station-keeping rather than the detector.

So: no aircraft. A world of STATIC camera rigs, one per angle, all looking at
one parked vehicle, all at the same slant range. Apparent size is therefore
identical in every frame and the only variable is the angle the vehicle is
seen from. Same lens, same resolution, same weights, same imgsz as the flying
detector.

  python3 rig.py                      # confidence against look-down angle
  python3 rig.py --mode aspect        # confidence against which way it faces
  python3 rig.py --save DIR           # also write one annotated frame each

Requires a Gazebo with the course models on GZ_SIM_RESOURCE_PATH, i.e. run it
inside the course container.
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
import time

ANGLES = [20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80]
# Aspect: how the vehicle is turned relative to the camera. 0 is its rear --
# which is what a follower actually sees -- 90 is broadside, 180 is head on.
ASPECTS = [0, 30, 45, 60, 90, 120, 135, 150, 180]
ASPECT_ANGLE = 34.0     # the capstone's look-down angle
PITCH_SEP = 200.0       # metres between rigs, so none appears in another's frame
SLANT = 21.0          # metres, camera to target centre
TARGET_H = 0.93       # target centre height, from its collision box
FRAMES = 12           # frames averaged per angle
WORLD = "rig_world"

CAM = """
    <model name="cam{a}">
      <static>true</static>
      <pose>{x:.4f} {y} {z:.4f} 0 {p:.6f} 0</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <gz_frame_id>cam{a}</gz_frame_id>
          <camera>
            <horizontal_fov>1.414</horizontal_fov>
            <image><format>R8G8B8</format><width>1280</width><height>720</height></image>
            <clip><near>0.05</near><far>15000</far></clip>
          </camera>
          <always_on>1</always_on>
          <update_rate>30</update_rate>
        </sensor>
      </link>
    </model>
"""


def build_aspect_world(aspects):
    """One camera and one vehicle per aspect, far enough apart not to overlap.

    The look angle is fixed at the capstone's, so the only variable is which
    way the vehicle is pointing. This is the measurement that matters most and
    is the easiest to forget to take: a follower sits BEHIND its target, and
    the rear of a pickup is not the view COCO is full of.
    """
    th = math.radians(ASPECT_ANGLE)
    z = TARGET_H + SLANT * math.sin(th)
    dx = -SLANT * math.cos(th)
    body = ""
    for i, psi in enumerate(aspects):
        y = i * PITCH_SEP
        body += f"""
    <include>
      <uri>model://course_target_vehicle</uri>
      <name>veh{psi}</name>
      <pose>0 {y} 0 0 0 {math.radians(psi):.6f}</pose>
    </include>
"""
        body += CAM.format(a=psi, x=dx, y=y, z=z, p=th)
    return body


def build_world(angles):
    cams = ""
    for a in angles:
        th = math.radians(a)
        # Slant range is held constant, so the vehicle subtends the same angle
        # in every frame: only the viewing direction changes.
        z = TARGET_H + SLANT * math.sin(th)
        x = -SLANT * math.cos(th)
        # Gazebo cameras look down the link's +x. Positive pitch swings +x
        # toward -z, i.e. downward, so pitch is the look-down angle itself.
        cams += CAM.format(a=a, x=x, y=0, z=z, p=th)

    return world_shell(cams, with_target=True)


def world_shell(body, with_target):
    target = """
    <!-- Broadside to the cameras, which all sit on -x. This is the aspect the
         follower does NOT normally get, and it is the best case: the rear
         three-quarter view a follower actually sees scores lower. -->
    <include>
      <uri>model://course_target_vehicle</uri>
      <name>course_target_vehicle</name>
      <pose>0 0 0 0 0 1.5707963267949</pose>
    </include>
""" if with_target else ""
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{WORLD}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <scene>
      <ambient>0.45 0.45 0.47 1</ambient>
      <background>0.62 0.72 0.85 1</background>
      <shadows>true</shadows>
      <sky></sky>
    </scene>
    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 0 0</pose>
      <cast_shadows>true</cast_shadows>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation><range>2000</range><linear>0</linear>
        <constant>1</constant><quadratic>0</quadratic></attenuation>
      <direction>0.4 0.2 -0.9</direction>
    </light>
    <include><uri>model://course_ground</uri></include>{target}{body}
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", metavar="DIR")
    ap.add_argument("--mode", choices=["angle", "aspect"], default="angle")
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--conf", type=float, default=0.20)
    ap.add_argument("--weights", default=os.environ.get(
        "YOLO_WEIGHTS", "/opt/models/yolo11n.pt"))
    args = ap.parse_args()

    import cv2
    import numpy as np
    from ultralytics import YOLO

    if args.mode == "aspect":
        sweep, world = ASPECTS, world_shell(build_aspect_world(ASPECTS), False)
        label, unit = "aspect", "deg from rear"
    else:
        sweep, world = ANGLES, build_world(ANGLES)
        label, unit = "angle", "deg below horizontal"
    path = os.path.join(tempfile.mkdtemp(), f"{WORLD}.sdf")
    with open(path, "w") as f:
        f.write(world)

    gz = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "1", path],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(12)          # let rendering come up and the sky settle
        model = YOLO(args.weights)
        keep = {"car", "truck", "bus", "motorcycle", "person"}

        print(f"slant range {SLANT:.0f} m, imgsz {args.imgsz}, conf {args.conf}, "
              f"{FRAMES} frames each; {label} in {unit}")
        if args.mode == "aspect":
            print(f"look-down fixed at {ASPECT_ANGLE:.0f} deg "
                  f"(0 = rear view, 90 = broadside, 180 = head on)")
        print(f"{label:>6} {'alt':>6} {'range':>6}  {'best':>5} {'mean':>5}  "
              f"{'hit':>4}  class")
        rows = []
        for a in sweep:
            topic = f"/world/{WORLD}/model/cam{a}/link/link/sensor/camera/image"
            frames = grab(topic, FRAMES)
            if not frames:
                print(f"{a:>6}  -- no frames --")
                continue
            best, scores, cls = 0.0, [], "-"
            for img in frames:
                res = model.predict(img, imgsz=args.imgsz, conf=args.conf,
                                    verbose=False)[0]
                s = 0.0
                for b in res.boxes:
                    n = model.names[int(b.cls[0])]
                    if n in keep and float(b.conf[0]) > s:
                        s, c = float(b.conf[0]), n
                        if s > best:
                            best, cls = s, c
                scores.append(s)
            th = math.radians(a if args.mode == "angle" else ASPECT_ANGLE)
            hit = sum(1 for s in scores if s > 0) / len(scores)
            print(f"{a:>6} {TARGET_H + SLANT*math.sin(th):>6.1f} "
                  f"{SLANT*math.cos(th):>6.1f}  {best:>5.2f} "
                  f"{np.mean(scores):>5.2f}  {hit:>4.0%}  {cls}")
            rows.append((a, best, float(np.mean(scores)), hit))
            if args.save:
                os.makedirs(args.save, exist_ok=True)
                res = model.predict(frames[len(frames)//2], imgsz=args.imgsz,
                                    conf=args.conf, verbose=False)[0]
                tag = "look" if args.mode == "angle" else "aspect"
                cv2.imwrite(os.path.join(args.save, f"{tag}{a:02d}.png"), res.plot())
        return rows
    finally:
        gz.terminate()
        try:
            gz.wait(timeout=10)
        except subprocess.TimeoutExpired:
            gz.kill()


def grab(topic, n):
    """Pull n raw images off a gz camera topic."""
    import numpy as np
    out = []
    p = subprocess.Popen(["gz", "topic", "-e", "-t", topic, "-n", str(n),
                          "--json-output"], stdout=subprocess.PIPE, text=True)
    import json
    import base64
    try:
        for line in p.stdout:
            try:
                m = json.loads(line)
            except json.JSONDecodeError:
                continue
            data = base64.b64decode(m["data"])
            h, w = int(m["height"]), int(m["width"])
            img = np.frombuffer(data, np.uint8)[:h * w * 3].reshape(h, w, 3)
            out.append(img[:, :, ::-1].copy())     # RGB -> BGR for cv2/YOLO
            if len(out) >= n:
                break
    finally:
        p.terminate()
    return out


if __name__ == "__main__":
    main()
