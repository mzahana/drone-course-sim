#!/usr/bin/env bash
# Start (or re-enter) the course container.
set -euo pipefail

IMAGE="${IMAGE:-drone-course-sim:jazzy}"
NAME="${NAME:-drone-course}"
SHARED="${SHARED:-$HOME/drone_course_shared_volume}"

mkdir -p "${SHARED}/ros2_ws/src"

# Re-enter if it is already up.
if [ "$(docker ps -q -f name="^${NAME}$")" ]; then
    exec docker exec -it "${NAME}" bash
fi
if [ "$(docker ps -aq -f name="^${NAME}$")" ]; then
    docker start "${NAME}" >/dev/null
    exec docker exec -it "${NAME}" bash
fi

# GPU is optional: the course runs on CPU, just slower at inference.
GPU_ARGS=()
if command -v nvidia-smi >/dev/null 2>&1 && docker info 2>/dev/null | grep -qi nvidia; then
    GPU_ARGS=(--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all)
    echo "[run] NVIDIA runtime detected — GPU enabled"
else
    echo "[run] no NVIDIA runtime — running on CPU"
fi

xhost +local:docker >/dev/null 2>&1 || true

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

docker run -it \
    --name "${NAME}" \
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
    "${IMAGE}" bash
