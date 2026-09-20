#!/usr/bin/env bash
# PX4 SITL build dependencies for Ubuntu 24.04 (noble).
# We deliberately do NOT run PX4's Tools/setup/ubuntu.sh: it pulls the NuttX
# cross-toolchain and a second Gazebo. We only need the SITL host build.
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
    astyle build-essential ccache cmake cppcheck file g++ gcc gdb git \
    lcov libfuse2 libxml2-dev libxml2-utils make ninja-build \
    python3 python3-dev python3-pip python3-venv \
    rsync shellcheck unzip zip wget curl gnupg lsb-release ca-certificates \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-gl \
    libimage-exiftool-perl
rm -rf /var/lib/apt/lists/*
