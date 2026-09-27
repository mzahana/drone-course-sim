#!/usr/bin/env python3
"""Crop captured frames to what the slide actually needs.

The captures are 1500x860 of a Gazebo viewport. A slide wants the subject, not
the four hundred pixels of empty field around it -- at slide size that field
is what the projector spends its resolution on.

The crops live here, as a table, rather than being done by hand in an image
editor. That is the whole point: `make_media.sh` and then this script
reproduce every image in the deck from a running simulator, and a frame that
gets re-shot next year gets the same treatment without anyone remembering
what they did the first time.

Boxes are (left, top, right, bottom) in the 1500x860 capture. Re-derive them
by eye when the camera moves; there is no cleverness here and there should
not be, because an automatic subject-finder that is wrong once is worse than
a number someone can see and change.

    python3 crop_for_slides.py ~/media ~/src/drone_courses/slides/media
"""
import os
import sys

from PIL import Image

CROPS = {
    # the aircraft on the ground, three-quarter view: airframe and gimbal
    "drone-three-quarter": (470, 320, 1030, 635),
    # head on, low: the gimbal ball under the nose is the subject
    "drone-gimbal":        (440, 330, 1020, 656),
    # from above and behind, showing the prop layout
    "drone-from-above":    (430, 250, 1070, 610),
    # airborne, gimbal pointed down
    "drone-in-flight":     (240, 430, 800, 745),
    # the whole geometry: aircraft, standoff, target
    "standoff-geometry":   (0, 120, 1500, 764),
    # the world, for "this is what you should be looking at"
    "world-overview":      (0, 90, 1500, 700),
}

# The look-angle rig writes 1280x720 frames with the vehicle dead centre.
CENTRE_CROPS = {"look": (620, 350)}


def main():
    src, dst = sys.argv[1], sys.argv[2]
    os.makedirs(dst, exist_ok=True)

    for name, box in CROPS.items():
        p = os.path.join(src, f"{name}.png")
        if not os.path.exists(p):
            print(f"  -- {name}: not captured")
            continue
        im = Image.open(p)
        box = (box[0], box[1], min(box[2], im.width), min(box[3], im.height))
        out = im.crop(box)
        out.save(os.path.join(dst, f"{name}.png"))
        print(f"  {name}: {im.width}x{im.height} -> {out.width}x{out.height}")

    look = os.path.join(src, "look")
    if os.path.isdir(look):
        w, h = CENTRE_CROPS["look"]
        for f in sorted(os.listdir(look)):
            if not f.endswith(".png"):
                continue
            im = Image.open(os.path.join(look, f))
            cx, cy = im.width // 2, im.height // 2
            im.crop((cx - w // 2, cy - h // 2,
                     cx + w // 2, cy + h // 2)).save(os.path.join(dst, f))
        print(f"  look/: centre {w}x{h}")


if __name__ == "__main__":
    main()
