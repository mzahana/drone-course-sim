#!/usr/bin/env bash
# Start (or re-enter) the course container.
#
#   ./run.sh            start the container, or open another shell in it
#   ./run.sh --fresh    delete the container and start a new one from the
#                       current image -- after ./build.sh, or to start clean.
#                       Your shared volume is kept; anything else inside the
#                       old container is not.
#   ./run.sh --gpu      with a new container: require the NVIDIA GPU
#   ./run.sh --cpu      with a new container: ignore any GPU
#
# The image is chosen when the container is created: drone-course-sim:jazzy-gpu
# if Docker can use an NVIDIA GPU here and that image exists, otherwise
# drone-course-sim:jazzy. IMAGE=... overrides the choice.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${HERE}/docker/gpu.sh"

usage() { sed -n '4,14p' "$0" | sed 's/^# \{0,1\}//'; }

FRESH=0
while [ $# -gt 0 ]; do
    case "$1" in
        -f|--fresh) FRESH=1 ;;
        --gpu) GPU=1 ;;
        --cpu) GPU=0 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1"; usage; exit 2 ;;
    esac
    shift
done

NAME="${NAME:-drone-course}"
SHARED="${SHARED:-$HOME/drone_course_shared_volume}"

mkdir -p "${SHARED}/ros2_ws/src"

if [ "${FRESH}" = "1" ] && [ -n "$(docker ps -aq -f name="^${NAME}$")" ]; then
    echo "[run] --fresh: removing container '${NAME}' (${SHARED} is kept)"
    docker rm -f "${NAME}" >/dev/null
fi

# Re-enter if it already exists. A container keeps the image it was created
# from, so a rebuild changes nothing here until --fresh -- which is the single
# most confusing thing about Docker for a newcomer. Say so when it applies.
if [ -n "$(docker ps -aq -f name="^${NAME}$")" ]; then
    if [ -n "${GPU:-}" ]; then
        echo "[run] --gpu/--cpu only apply to a new container; add --fresh"
    fi
    CUR_IMAGE="$(docker inspect -f '{{.Config.Image}}' "${NAME}")"
    CUR_ID="$(docker inspect -f '{{.Image}}' "${NAME}")"
    TAG_ID="$(docker image inspect -f '{{.Id}}' "${CUR_IMAGE}" 2>/dev/null || true)"
    if [ -n "${TAG_ID}" ] && [ "${TAG_ID}" != "${CUR_ID}" ]; then
        echo "[run] ${CUR_IMAGE} has been rebuilt since this container was created."
        echo "      ./run.sh --fresh switches to it (your shared volume is kept)."
    elif [ "${CUR_IMAGE}" = "${CPU_TAG}" ] && docker image inspect "${GPU_TAG}" >/dev/null 2>&1 \
         && [ "${GPU:-auto}" != "0" ] && [ "${GPU:-auto}" != "cpu" ]; then
        echo "[run] this container runs the CPU image, and ${GPU_TAG} exists."
        echo "      ./run.sh --fresh switches to it (your shared volume is kept)."
    fi
    if [ -z "$(docker ps -q -f name="^${NAME}$")" ]; then
        docker start "${NAME}" >/dev/null
    fi
    exec docker exec -it "${NAME}" bash
fi

# ---------------------------------------------------------------------------
# GPU. Optional: the course runs on CPU, with slower inference and, without
# GPU rendering, a simulator that may not reach real time (spike note 38).
# ---------------------------------------------------------------------------
detect_gpu
if [ -z "${IMAGE:-}" ]; then
    if [ "${GPU_STATE}" = "ok" ] && docker image inspect "${GPU_TAG}" >/dev/null 2>&1; then
        IMAGE="${GPU_TAG}"
    elif docker image inspect "${CPU_TAG}" >/dev/null 2>&1; then
        IMAGE="${CPU_TAG}"
    elif docker image inspect "${GPU_TAG}" >/dev/null 2>&1; then
        IMAGE="${GPU_TAG}"
    else
        echo "[run] no course image on this machine. Run ./install.sh (or ./build.sh) first."
        exit 1
    fi
fi

GPU_ARGS=()
case "${GPU_STATE}" in
    ok)
        GPU_ARGS=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all)
        if [ "${IMAGE}" = "${GPU_TAG}" ]; then
            echo "[run] NVIDIA GPU enabled -- rendering and YOLO on the GPU"
        else
            echo "[run] NVIDIA GPU enabled for rendering; YOLO stays on the CPU in ${IMAGE}."
            echo "      ./build.sh builds the GPU image, then ./run.sh --fresh."
        fi ;;
    driver)
        echo "[run] running on CPU"
        gpu_toolkit_hint ;;
    *)
        echo "[run] no NVIDIA GPU -- running on CPU" ;;
esac

# WSL2 has no /dev/nvidia*. The GPU is reached through /dev/dxg, and OpenGL
# through Mesa's d3d12 driver and the libraries Windows puts in /usr/lib/wsl.
# This also accelerates Gazebo on an Intel or AMD GPU, so it does not wait for
# NVIDIA. `course sim` selects the d3d12 driver when /dev/dxg is present.
# Written to Microsoft's and NVIDIA's documentation; not yet tested on WSL.
if [ "${IS_WSL}" = "1" ] && [ -e /dev/dxg ]; then
    GPU_ARGS+=(--device /dev/dxg
               --volume /usr/lib/wsl:/usr/lib/wsl
               --env LD_LIBRARY_PATH=/usr/lib/wsl/lib)
    # A laptop has an integrated GPU too; d3d12 should pick the NVIDIA one.
    [ "${GPU_STATE}" = "ok" ] && GPU_ARGS+=(--env MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA)
    echo "[run] WSL2: GPU rendering through /dev/dxg"
fi
# The GPU's render node. On a Wayland desktop, X11 windows from the container
# go through XWayland, and NVIDIA's OpenGL there hands frames to the
# compositor through /dev/dri/renderD128 -- which is mode 660, group `render`
# on the host. The container user is not in that group (its gid differs per
# machine, 992 here), so every OpenGL window -- Gazebo, QGroundControl, RViz --
# opens and stays BLACK, with no error beyond a libEGL warning in a log nobody
# reads. Add the device's own gid, numerically; the name need not exist inside.
DRI_GIDS=()
for dev in /dev/dri/renderD* /dev/dri/card*; do
    [ -e "$dev" ] || continue
    gid="$(stat -c %g "$dev")"
    [ "$gid" != "0" ] && [[ " ${DRI_GIDS[*]} " != *" --group-add ${gid} "* ]] \
        && DRI_GIDS+=(--group-add "$gid")
done
GPU_ARGS+=("${DRI_GIDS[@]}")
echo "[run] new container '${NAME}' from ${IMAGE}"

xhost +local:docker >/dev/null 2>&1 || true

if [ -z "${DISPLAY:-}" ]; then
    echo "[run] no DISPLAY on this machine — inside the container run 'course desktop'"
    echo "[run] and open http://localhost:${VNC_PORT:-6080}/vnc.html"
fi

# ---------------------------------------------------------------------------
# Isolation. This matters more than it looks.
#
# gz-transport discovers peers by multicast and is NOT scoped by ROS_DOMAIN_ID.
# With host networking and no partition, a PX4 instance will happily find ANY
# other Gazebo reachable on the network and spawn its model into that world.
# In a classroom that means thirty students sharing one simulation.
#
# So: no host networking by default, a per-user gz partition, a per-user ROS
# domain, and DDS discovery confined to this machine.
# ---------------------------------------------------------------------------
GZ_PARTITION="${GZ_PARTITION:-course_$(id -un)}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$(( (UID % 100) + 1 ))}"
echo "[run] gz partition '${GZ_PARTITION}', ROS_DOMAIN_ID ${ROS_DOMAIN_ID}"

# Set USE_HOST_NETWORK=1 only if you know why you need it.
NET_ARGS=(--network bridge)
if [ "${USE_HOST_NETWORK:-0}" = "1" ]; then
    echo "[run] WARNING: host networking — this container can reach other simulations"
    NET_ARGS=(--network host)
fi

# The browser desktop needs one port out. It costs nothing when unused, and
# publishing it after the fact means recreating the container -- which on a
# student laptop means losing whatever they had running.
VNC_PORT="${VNC_PORT:-6080}"

# ---------------------------------------------------------------------------
# UID alignment.
#
# The shared volume is a bind mount, so it keeps the HOST's numeric owner. The
# image's user is uid 1000, which is what almost every Linux and WSL2 account
# is. If yours is not, the container cannot write to your own workspace -- and
# it fails silently, several commands later, as a colcon permission error. So
# check, say so, and fix it once when the container is created.
# ---------------------------------------------------------------------------
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

fix_uid() {
    [ "${HOST_UID}" = "1000" ] && [ "${HOST_GID}" = "1000" ] && return 0
    echo "[run] your uid:gid is ${HOST_UID}:${HOST_GID}, the image ships 1000:1000"
    echo "[run] aligning the container user so the shared volume is writable (one time, ~1 min)"
    docker exec -u root "${NAME}" bash -c "
        groupmod -g ${HOST_GID} user 2>/dev/null || true
        usermod -u ${HOST_UID} -g ${HOST_GID} user
        chown -R ${HOST_UID}:${HOST_GID} /home/user /opt/PX4-Autopilot /opt/course_ws /opt/course_exercises
    " || echo "[run] WARNING: could not align uid; 'course doctor' will tell you if it matters"
}

docker run \
    --name "${NAME}" \
    --publish "127.0.0.1:${VNC_PORT}:6080" \
    "${NET_ARGS[@]}" \
    --ipc host \
    --privileged \
    --env GZ_PARTITION="${GZ_PARTITION}" \
    --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    "${GPU_ARGS[@]}" \
    --env DISPLAY="${DISPLAY:-:0}" \
    --env QT_X11_NO_MITSHM=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "${SHARED}:/home/user/shared_volume:rw" \
    --detach --tty "${IMAGE}" bash > /dev/null

fix_uid
exec docker exec -it "${NAME}" bash
