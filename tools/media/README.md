# Making the course's screenshots and video

Everything in the slides' `media/` directory is produced by these scripts, from
a running simulator, inside the course container. Nothing is cropped by hand in
an image editor.

That is not fussiness. Most of these frames have numbers burnt into them — a
detection confidence, a real-time factor, a bounding box — and a hand-made
replacement stops matching the deck that cites it without anyone noticing. A
year from now, someone re-shooting one of these should get the same framing
without having to remember what was done the first time.

## The three scripts

| | |
|---|---|
| `capture.sh` | The primitives: `shot`, `rec`, `rec-bg`, `closeup`, `follow`, `frame`, `term`, `type`, `rviz`, `qgc`, `gazebo-gui` |
| `make_media.sh` | One target per deliverable: `closeups`, `capstone`, `offboard`, `blackout`, `rviz`, `frames`, `qgc`, `firstrun`, `ekf2`, or `all` |
| `frame_axes.py` | Draws chosen TF frames as labelled axes for `frames`, placed by the live `/tf` |
| `crop_for_slides.py` | Crops the captures to what a slide needs, from a table of named boxes |

## Running them

```bash
course desktop                              # the virtual X server
bash $COURSE_TOOLS/media/make_media.sh all  # about forty minutes
python3 $COURSE_TOOLS/media/crop_for_slides.py ~/media ~/shared_volume/media
```

Every target restarts the simulator first, with `course stop --all`. That is
deliberate: a clip recorded on top of a previous run's leftover nodes is a clip
of two mission managers arguing, and the camera bridge in particular leaks one
process per `course bringup` if it is not cleaned up properly. See spike note
40 — twenty-nine of them accumulated once, and everything recorded during that
window was unusable.

## Two things that did not work

**A scene-only GUI config.** `gz sim -g --gui-config` with a layout containing
nothing but `MinimalScene` is the tidy way to get a clean frame. It silently
did not apply — the stock panels came back — so captures crop to the render
area instead (`GZ_CROP`, default `1500x860+0+148`). A deterministic crop beats
an elegant configuration that does not take.

**Shooting the aircraft from 3 m.** Gazebo's camera has an 80° horizontal field
of view, so at 3 m an X500 is a speck and you cannot tell it has a gimbal. The
ground close-ups are taken from 1.2–1.7 m.

## What cannot be produced here

The Day 4 hardware bench test. That one needs the real aircraft, and its
`\dcvideoplaceholder` in the deck stays until someone records it at rehearsal.
