#!/usr/bin/env bash
# One-time setup on a student machine.
set -euo pipefail

# Override IMAGE to pull from somewhere else. The default is the published
# course image; a locally built one is tagged drone-course-sim:jazzy and is
# found first by the check below.
IMAGE="${IMAGE:-ghcr.io/mzahana/drone-course-sim:jazzy}"
LOCAL_TAG="drone-course-sim:jazzy"
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

if docker image inspect "${LOCAL_TAG}" >/dev/null 2>&1; then
    echo "  ok    image ${LOCAL_TAG} already present"
elif docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "  ok    image ${IMAGE} already present"
    docker tag "${IMAGE}" "${LOCAL_TAG}"
else
    echo "  ..    pulling ${IMAGE}"
    echo "        About 3.4 GB to download and about 13 GB on disk. Once."
    if docker pull "${IMAGE}"; then
        docker tag "${IMAGE}" "${LOCAL_TAG}"
    else
        echo
        echo "Could not pull ${IMAGE}."
        echo "Build it locally instead (slow, about 30 minutes):"
        echo "    docker build -t ${LOCAL_TAG} -f docker/Dockerfile ."
        exit 1
    fi
fi

# Ask for a lot of disk before finding out the hard way at 90%.
AVAIL_GB=$(df -BG --output=avail /var/lib/docker 2>/dev/null | tail -1 | tr -dc '0-9' || echo 0)
if [ "${AVAIL_GB:-0}" -gt 0 ] && [ "${AVAIL_GB}" -lt 20 ]; then
    echo "  warn  only ${AVAIL_GB} GB free where Docker stores images; 20 GB is comfortable"
fi

echo
echo "Done. Start the container with:  ./run.sh"
echo "Then inside it run:             course doctor"
