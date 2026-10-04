# Sourced by install.sh, build.sh and run.sh. Answers one question: can a
# container on THIS machine use an NVIDIA GPU?
#
# It asks Docker for real -- `docker run --gpus all ... true` -- rather than
# reading `docker info`. Docker Desktop and CDI-only setups hand out GPUs
# without listing an `nvidia` runtime, and a runtime that is listed can still
# fail to start a container. The probe costs about 0.3 s.
#
# After detect_gpu:
#   GPU_STATE  none    no NVIDIA driver on this machine (or GPU=0)
#              driver  the driver works, but Docker cannot pass the GPU to a
#                      container: the NVIDIA Container Toolkit is missing or
#                      not configured. Worth saying loudly -- it is fixable.
#              ok      `docker run --gpus all` works
#   IS_WSL     1 under WSL2, else 0
#
# GPU=0 forces none; GPU=1 forces ok without probing (for building a GPU image
# on a machine that cannot run one). Default: auto.

CPU_TAG="drone-course-sim:jazzy"
GPU_TAG="drone-course-sim:jazzy-gpu"

detect_gpu() {    # detect_gpu [probe-image]
    IS_WSL=0
    grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null && IS_WSL=1

    GPU_STATE=none
    case "${GPU:-auto}" in
        0|no|off|cpu) return 0 ;;
        1|yes|on|gpu) GPU_STATE=ok; return 0 ;;
    esac

    # Under WSL2 the Windows driver provides nvidia-smi in /usr/lib/wsl/lib,
    # which is not always on PATH.
    local smi
    smi="$(command -v nvidia-smi 2>/dev/null || true)"
    [ -z "$smi" ] && [ -x /usr/lib/wsl/lib/nvidia-smi ] && smi=/usr/lib/wsl/lib/nvidia-smi
    [ -n "$smi" ] && "$smi" -L >/dev/null 2>&1 || return 0
    GPU_STATE=driver

    # Probe with an image that is already here if possible; any image with
    # `true` in it will do. Otherwise ubuntu:24.04, which is small.
    local probe="${1:-}"
    if [ -z "$probe" ]; then
        for probe in "$GPU_TAG" "$CPU_TAG" ubuntu:24.04; do
            docker image inspect "$probe" >/dev/null 2>&1 && break
        done
    fi
    docker run --rm --gpus all --entrypoint true "$probe" >/dev/null 2>&1 && GPU_STATE=ok
    return 0
}

gpu_toolkit_hint() {
    echo "        An NVIDIA GPU is present, but Docker cannot give it to a container."
    echo "        Install the NVIDIA Container Toolkit -- docs/prework.md, section 4 -- then:"
    echo "            docker run --rm --gpus all ubuntu:24.04 nvidia-smi"
    echo "        must print your GPU. Until then everything runs on the CPU, which works."
}
