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
fresh() {          # fresh TIER
    bash ~/restart.sh
    sleep 50
    nohup course bringup "tier:=$1" > /tmp/bringup.log 2>&1 &
    sleep 35
}

capstone() {       # capstone TIER
    # shellcheck disable=SC1090
    source "$WS/install/setup.bash"
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
    $CAP follow x500_course_gimbal_0 -12 -8 6
    sleep 3
    $CAP rec capstone-follow 150
    $CAP frame /detector/image_annotated detector-follow
}

# What the detector is looking at while that happens, and the estimate with
# its covariance. RViz rather than Gazebo: the point here is the track, not
# the scenery.
m_rviz() {
    fresh 2
    capstone 2
    sleep 40
    $CAP rviz rviz-follow
}

# The blackout. Recorded in RViz for the same reason: the interesting thing is
# the estimate continuing to move while the detections stop.
m_blackout() {
    fresh 3
    capstone 3
    sleep 40
    course rviz > /tmp/rviz.log 2>&1 &
    sleep 15
    $CAP rec detector-blackout 100
}

# Lab 3: the OFFBOARD square, which is the first thing a student flies.
m_offboard() {
    fresh 0
    $CAP gazebo-gui
    # shellcheck disable=SC1090
    source "$WS/install/setup.bash"
    nohup ros2 run lab3_offboard_solution offboard_square.py \
        > /tmp/lab3.log 2>&1 &
    sleep 8
    $CAP follow x500_course_gimbal_0 -16 -12 9
    $CAP rec offboard-square 110
}

# The console, for the two clips that are really about what PX4 prints.
m_first_run() {
    bash ~/restart.sh
    $CAP term 'cd /opt/PX4-Autopilot && course sim' 150 42
    $CAP rec course-sim-first-run 75
}

# EKF2's own view of itself, typed into the PX4 console. `ekf2 status` is the
# command students are told to reach for when the estimate looks wrong, and
# the aiding flags are the part of its output that actually answers the
# question -- so the clip is of someone typing it and reading the flags, not
# of a tidy prepared screen.
m_ekf2() {
    bash ~/restart.sh
    $CAP term 'cd /opt/PX4-Autopilot && make px4_sitl gz_x500_course_gimbal_course_world' 150 42
    sleep 60
    $CAP rec-bg ekf2-status 95 > /dev/null
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
    $CAP qgc qgc-connected
}

case "${1:-help}" in
    capstone)  m_capstone ;;
    rviz)      m_rviz ;;
    blackout)  m_blackout ;;
    offboard)  m_offboard ;;
    firstrun)  m_first_run ;;
    ekf2)      m_ekf2 ;;
    qgc)       m_qgc ;;
    all)       for t in capstone rviz blackout offboard qgc firstrun; do
                   echo "=== $t ==="; "m_${t/firstrun/first_run}"; done ;;
    *) sed -n '2,10p' "${BASH_SOURCE[0]}" ;;
esac
