#!/usr/bin/env bash
# Build the course image locally, choosing CPU or GPU for this machine.
#
#   ./build.sh              detect: GPU image if Docker can use an NVIDIA GPU here
#   ./build.sh --gpu        GPU image regardless (e.g. to push it)
#   ./build.sh --cpu        CPU image regardless
#   ./build.sh --no-cache   any of the above, from scratch
#
# Most students never need this: install.sh pulls a ready-made CPU image. Build
# when you want YOLO on your GPU, or when you changed something in this repo.
#
# The two images are tagged separately -- drone-course-sim:jazzy (CPU) and
# drone-course-sim:jazzy-gpu -- and run.sh picks the right one. A container
# keeps the image it was created from, so after a rebuild start a new one
# with ./run.sh --fresh.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source docker/gpu.sh

usage() { sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; }

NO_CACHE=()
while [ $# -gt 0 ]; do
    case "$1" in
        --gpu) GPU=1 ;;
        --cpu) GPU=0 ;;
        --no-cache) NO_CACHE=(--no-cache) ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1"; usage; exit 2 ;;
    esac
    shift
done

detect_gpu
case "$GPU_STATE" in
    ok)
        VARIANT="${TORCH_VARIANT:-cu128}"; TAG="$GPU_TAG"
        echo "[build] NVIDIA GPU usable from Docker -- building the GPU image (torch ${VARIANT})" ;;
    driver)
        VARIANT=cpu; TAG="$CPU_TAG"
        echo "[build] building the CPU image"
        gpu_toolkit_hint
        echo "        Then run ./build.sh again to get the GPU image." ;;
    *)
        VARIANT=cpu; TAG="$CPU_TAG"
        echo "[build] no NVIDIA GPU -- building the CPU image" ;;
esac
echo "[build] tag ${TAG}. About 30 minutes the first time."
echo

docker build "${NO_CACHE[@]}" -t "$TAG" \
    --build-arg TORCH_VARIANT="$VARIANT" \
    --build-arg PX4_VERSION="${PX4_VERSION:-v1.17.0}" \
    -f docker/Dockerfile .

echo
echo "[build] built ${TAG}"
NAME="${NAME:-drone-course}"
if [ -n "$(docker ps -aq -f name="^${NAME}$")" ]; then
    echo "[build] your container '${NAME}' was created from an older or different image"
    echo "        ($(docker inspect -f '{{.Config.Image}}' "${NAME}")). Switch to ${TAG} with:"
    echo "            ./run.sh --fresh"
    echo "        (your work in the shared volume is kept)"
else
    echo "[build] start it with:  ./run.sh"
fi
