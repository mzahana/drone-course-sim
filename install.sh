#!/usr/bin/env bash
# One-time setup on a student machine.
set -euo pipefail

IMAGE="${IMAGE:-drone-course-sim:jazzy}"
SHARED="${SHARED:-$HOME/drone_course_shared_volume}"

echo "Drone course — setup"
echo

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker is not installed. See https://docs.docker.com/engine/install/ and re-run."
    exit 1
fi
if ! docker info >/dev/null 2>&1; then
    echo "Docker is installed but not usable by this user."
    echo "Try:  sudo usermod -aG docker \$USER    then log out and back in."
    exit 1
fi
echo "  ok    docker"

mkdir -p "${SHARED}/ros2_ws/src"
echo "  ok    workspace at ${SHARED}/ros2_ws/src"

if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "  ok    image ${IMAGE} already present"
else
    echo "  ..    pulling ${IMAGE} (this is a few GB, once)"
    docker pull "${IMAGE}"
fi

echo
echo "Done. Start the container with:  ./run.sh"
echo "Then inside it run:             course doctor"
