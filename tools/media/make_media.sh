#!/usr/bin/env bash
# Produce every screenshot and clip the slides use, from a clean simulator.
#
# Run inside the course container, with the virtual desktop up:
#
#   course desktop
#   bash tools/media/make_media.sh all
#
# Each target is independent and restarts the simulator first, because a clip
# recorded on top of a previous run's leftover nodes is a clip of two mission
# managers arguing -- which has happened, see spike note 36.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAP="bash $HERE/capture.sh"
export MEDIA="${MEDIA:-$HOME/media}"
export DISPLAY="${DISPLAY:-:99}"
WS="$HOME/shared_volume/ros2_ws"
mkdir -p "$MEDIA"

# Restart the simulator and bring the stack up on a given route tier.
#
# `course stop --all` rather than a hand-rolled pkill: the camera bridge is a
# separate executable and a cleanup that misses it leaves a publisher behind
# on /camera/image_raw. Twenty-nine of those accumulated once (spike note 40)
# and every clip recorded afterwards was of a simulator being strangled.
fresh() {          # fresh TIER
    course stop --all > /dev/null 2>&1
    sleep 5
    ( export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
      export __GLX_VENDOR_LIBRARY_NAME=nvidia
      cd /opt/PX4-Autopilot && nohup script -qc \
        "make px4_sitl gz_x500_course_gimbal_course_world" /tmp/sim.log \
        > /dev/null 2>&1 & )
    sleep 50
    nohup course bringup "tier:=$1" > /tmp/bringup.log 2>&1 &
    sleep 35
}

capstone() {       # capstone TIER
    # colcon's setup.bash reads unbound variables, so -u must be off for it
    set +u
    # shellcheck disable=SC1090
    source "$WS/install/setup.bash"
    set -u
    nohup ros2 launch capstone_follow_solution capstone.launch.py "tier:=$1" \
        > /tmp/capstone.log 2>&1 &
}

# ---------------------------------------------------------------- the targets

# The capstone, seen from behind and above. This is the clip the course opens
# with, so it is the one worth the most care: tier 2 rather than tier 1,
# because 90-degree corners read as deliberate manoeuvring on screen while
# tier 1's U-turns read as the aircraft losing the plot.
m_capstone() {
    fresh 2
    $CAP gazebo-gui
    capstone 2
    sleep 25                       # through TAKEOFF and into FOLLOW
    # Close. The first version of this shot used a -14,-9,7 offset and the
    # aircraft came out twelve pixels across -- you could not see that it had
    # a gimbal, let alone which way the gimbal was pointing, which is the
    # entire subject of the clip.
    $CAP follow x500_course_gimbal_0 -7 -5 3
    sleep 3
    $CAP rec capstone-follow 150
    $CAP frame /detector/image_annotated detector-follow
}

# The aircraft itself, close enough to see what it is made of. These are the
# stills for Day 1 -- "this is the machine" -- and no amount of wide shot of
# a field substitutes for them.
m_closeups() {
    fresh 0
    $CAP gazebo-gui
    # On the ground: three-quarter front, then a low angle that puts the
    # gimbal and the lens against the sky.
    $CAP closeup 0 0 0.30  1.7  135 14 ; sleep 2 ; $CAP shot drone-three-quarter
    $CAP closeup 0 0 0.28  1.2  175  4 ; sleep 2 ; $CAP shot drone-gimbal
    $CAP closeup 0 0 0.30  2.4   45 30 ; sleep 2 ; $CAP shot drone-from-above
    # In flight, with the gimbal pointed at the target: the geometry the whole
    # course is about, in one frame.
    capstone 0
    sleep 45
    $CAP follow x500_course_gimbal_0 -3 -2 1.2
    sleep 4
    $CAP shot drone-in-flight
    $CAP follow x500_course_gimbal_0 -16 -11 8
    sleep 4
    $CAP shot standoff-geometry
    $CAP frame /detector/image_annotated detector-annotated
}

# What the detector is looking at while that happens, and the estimate with
# its covariance. RViz rather than Gazebo: the point here is the track, not
# the scenery.
m_rviz() {
    fresh 2
    capstone 2
    sleep 40
    $CAP rviz rviz-follow full
}

# The blackout. Recorded in RViz for the same reason: the interesting thing is
# the estimate continuing to move while the detections stop.
m_blackout() {
    fresh 3
    capstone 3
    sleep 40
    course rviz > /tmp/rviz.log 2>&1 &
    sleep 15
    $CAP rec detector-blackout 100 full
}

# Day 3, "Where these frames sit on the aircraft": the aircraft in RViz with
# the four frames the slide names, at the capstone's 35 degree look-down. It
# replaces a generated picture that had the axis directions wrong, so the one
# thing that matters is that every arrow comes from the live /tf; see
# frame_axes.py for why it does not use RViz's own TF display.
#
# Its own 4:3 X server rather than the desktop, so the shot does not depend on
# the size of whatever desktop happens to be up, and RViz --fullscreen fills
# it with nothing but the 3D view: no panels to crop.
m_frames() {
    local disp="${FRAMES_DISPLAY:-:97}" out="$MEDIA/day3-frames-on-aircraft.png" share
    fresh 0
    # ROS's and colcon's setup.bash read unbound variables, so -u must be off
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    # shellcheck disable=SC1091
    source /opt/course_ws/install/setup.bash
    set -u
    share="$(ros2 pkg prefix drone_course_sim)/share/drone_course_sim"
    # Held for the whole shot. Pitch negative is down, same as the A8 mini.
    timeout 120 ros2 topic pub -r 2 /gimbal/cmd/angle geometry_msgs/msg/Vector3Stamped \
        "{vector: {x: 0.0, y: -0.6109, z: 0.0}}" > /dev/null 2>&1 &
    python3 "$HERE/frame_axes.py" --ros-args -p use_sim_time:=true \
        > /tmp/frame_axes.log 2>&1 &
    Xvfb "$disp" -screen 0 1600x1200x24 > /tmp/xvfb_frames.log 2>&1 &
    sleep 2
    DISPLAY="$disp" ros2 run rviz2 rviz2 -d "$share/rviz/frames_on_aircraft.rviz" \
        --fullscreen --ros-args -p use_sim_time:=true > /tmp/rviz_frames.log 2>&1 &
    sleep 25                       # gimbal settled, meshes loaded, markers in
    DISPLAY="$disp" import -window root "$out"
    # By pattern, not PID: `ros2 run` leaves its child behind when killed.
    # Brackets so the pattern cannot match this shell's own command line.
    pkill -f "[f]rames_on_aircraft.rviz"
    pkill -f "[f]rame_axes.py"
    pkill -f "[X]vfb $disp"
    pkill -f "[g]imbal/cmd/angle"
    echo "$out"
}

# Lab 3: the OFFBOARD square, which is the first thing a student flies.
m_offboard() {
    fresh 0
    $CAP gazebo-gui
    # colcon's setup.bash reads unbound variables, so -u must be off for it
    set +u
    # shellcheck disable=SC1090
    source "$WS/install/setup.bash"
    set -u
    nohup ros2 run lab3_offboard_solution offboard_square.py \
        > /tmp/lab3.log 2>&1 &
    sleep 8
    $CAP follow x500_course_gimbal_0 -16 -12 9
    $CAP rec offboard-square 110
}

# The console, for the two clips that are really about what PX4 prints.
m_first_run() {
    course stop --all > /dev/null 2>&1
    sleep 5
    $CAP term 'cd /opt/PX4-Autopilot && course sim' 150 42
    $CAP rec course-sim-first-run 75 full
}

# EKF2's own view of itself, typed into the PX4 console. `ekf2 status` is the
# command students are told to reach for when the estimate looks wrong, and
# the aiding flags are the part of its output that actually answers the
# question -- so the clip is of someone typing it and reading the flags, not
# of a tidy prepared screen.
m_ekf2() {
    course stop --all > /dev/null 2>&1
    sleep 5
    $CAP term 'cd /opt/PX4-Autopilot && make px4_sitl gz_x500_course_gimbal_course_world' 150 42
    sleep 60
    $CAP rec-bg ekf2-status 95 full > /dev/null
    sleep 3
    $CAP type 'ekf2 status'      ; sleep 14
    $CAP type 'ekf2 status'      ; sleep 12
    $CAP type 'commander status' ; sleep 12
    $CAP type 'listener vehicle_local_position 1' ; sleep 14
    $CAP type 'listener estimator_status_flags 1' ; sleep 20
    wait
}

m_qgc() {
    fresh 1
    $CAP qgc qgc-connected full
}

case "${1:-help}" in
    capstone)  m_capstone ;;
    closeups)  m_closeups ;;
    rviz)      m_rviz ;;
    frames)    m_frames ;;
    blackout)  m_blackout ;;
    offboard)  m_offboard ;;
    firstrun)  m_first_run ;;
    ekf2)      m_ekf2 ;;
    qgc)       m_qgc ;;
    all)       for t in closeups capstone rviz frames blackout offboard qgc firstrun; do
                   echo "=== $t ==="; "m_${t/firstrun/first_run}"; done ;;
    *) sed -n '2,10p' "${BASH_SOURCE[0]}" ;;
esac
