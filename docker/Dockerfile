# syntax=docker/dockerfile:1
#
# Drone Course simulation image
#   Ubuntu 24.04 · ROS 2 Jazzy (desktop-full) · Gazebo Harmonic · PX4 v1.17.0 · MAVROS
#
# Layer order is deliberate:
#   * the slow, stable layers (apt, ROS, torch) come first so that iterating on
#     PX4 or on the course scripts does not re-download 2 GB of wheels;
#   * PX4 is cloned and built *as the course user*, not as root. SITL writes into
#     its own build tree at run time, so a root-owned PX4 tree fails at the first
#     `make px4_sitl` with an unhelpful symlink error. Chowning afterwards would
#     work but would duplicate the whole 2 GB tree into another layer.
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full
ARG ROS_DISTRO
ARG PX4_VERSION=v1.17.0
ARG USERNAME=user
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------- 1. PX4 build deps
COPY docker/scripts/px4_deps.sh /tmp/px4_deps.sh
RUN /tmp/px4_deps.sh && rm /tmp/px4_deps.sh

# ---------------------------------------------------------------- 2. ROS packages
# ros_gz pulls the gz_*_vendor packages, i.e. Gazebo Harmonic under
# /opt/ros/$ROS_DISTRO/opt. That is the ONLY Gazebo in this image: PX4 is built
# against it below, so the bridge can never be ABI-skewed against the simulator.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-image \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-mavros \
      ros-${ROS_DISTRO}-mavros-extras \
      ros-${ROS_DISTRO}-mavros-msgs \
      ros-${ROS_DISTRO}-vision-msgs \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-rqt-image-view \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-xacro \
      sudo \
    && rm -rf /var/lib/apt/lists/*

# MAVROS refuses to start without the GeographicLib geoid dataset. Doing this at
# build time is what stops it being every student's first error message.
RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

# ---------------------------------------------------------------- 3. Perception
# Two constraints here, and both are easy to get wrong:
#
#   * numpy MUST stay on the 1.x that ROS Jazzy was built against. Letting pip
#     resolve numpy 2.x breaks cv_bridge at import time -- and pip cannot even
#     replace it, because the system numpy came from dpkg and has no RECORD file.
#   * torch comes from the CPU index. The default PyPI wheel drags in
#     cuda-toolkit, cudnn, cublas and triton -- about 3 GB that most student
#     laptops cannot use. A CUDA variant of this image is a separate build.
RUN echo "numpy<2" > /tmp/constraints.txt \
    && pip3 install --no-cache-dir --break-system-packages -c /tmp/constraints.txt \
         --index-url https://download.pytorch.org/whl/cpu torch torchvision \
    && pip3 install --no-cache-dir --break-system-packages -c /tmp/constraints.txt \
         ultralytics onnxruntime \
    && rm /tmp/constraints.txt

# Guard: if these imports ever fail the image is broken for every perception lab,
# so fail the build here rather than in front of the class.
RUN python3 -c "import numpy, cv2, torch, ultralytics; \
    assert numpy.__version__.startswith('1.'), numpy.__version__; \
    print('numpy', numpy.__version__, '| torch', torch.__version__)"
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && python3 -c "import cv_bridge; print('cv_bridge ok')"

# Bake the weights in so no student ever waits on a download mid-lab.
# Ultralytics downloads into the working directory, so cd there first.
RUN mkdir -p /opt/models && cd /opt/models \
    && python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')" \
    && test -f /opt/models/yolo11n.pt \
    && curl -fsSL -o /opt/models/detector_selftest.jpg https://ultralytics.com/images/bus.jpg

# Prove the detector actually detects, rather than merely that it imports.
# A model file that loads but returns nothing is a silent failure that would
# otherwise surface in the middle of the perception lab.
RUN python3 -c "\
from ultralytics import YOLO; \
r = YOLO('/opt/models/yolo11n.pt').predict('/opt/models/detector_selftest.jpg', verbose=False)[0]; \
n = len(r.boxes); \
assert n >= 3, f'detector self-test found only {n} objects'; \
print('detector self-test ok:', n, 'objects')"
ENV YOLO_WEIGHTS=/opt/models/yolo11n.pt

# ---------------------------------------------------------------- 4. Course user
RUN useradd -ms /bin/bash ${USERNAME} && echo "${USERNAME}:${USERNAME}" | chpasswd \
    && usermod -aG sudo,dialout,video ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

# ---------------------------------------------------------------- 5. PX4 v1.17.0
ENV PX4_DIR=/opt/PX4-Autopilot
RUN mkdir -p ${PX4_DIR} && chown ${USERNAME}:${USERNAME} ${PX4_DIR} \
    && git config --system --add safe.directory ${PX4_DIR}

USER ${USERNAME}
RUN git clone -b ${PX4_VERSION} --depth 1 --recursive --shallow-submodules \
      https://github.com/PX4/PX4-Autopilot.git ${PX4_DIR} \
    && echo "${PX4_VERSION}" > ${PX4_DIR}/PX4_COURSE_VERSION

# ---------------------------------------------------------------- 5b. Course model
# The course flies its own gimbal model (A8-mini-like optics, joint state
# feedback). PX4 spawns models from its own models directory and generates make
# targets from the airframe list, so both are installed into the PX4 tree here
# -- before the build, so the target exists. See models/README.md for what
# changed and, more importantly, what deliberately did not.
# Every asset the course flies is vendored in this repository -- airframe,
# gimbal and world -- so the simulation does not silently inherit anything from
# whatever PX4 happens to ship. PX4 spawns models from PX4_GZ_MODELS and globs
# PX4_GZ_WORLDS to generate make targets, and gz_env.sh overwrites both at run
# time, so the assets are installed into the PX4 tree here rather than pointed
# at from outside.
# The world must be here too, not with the models below: PX4 generates its make
# targets from airframes x worlds at cmake configure time, so a world that
# arrives after the build yields `ninja: unknown target gz_<model>_<world>`.
COPY --chown=${USERNAME}:${USERNAME} worlds/course_world.sdf ${PX4_DIR}/Tools/simulation/gz/worlds/course_world.sdf
COPY --chown=${USERNAME}:${USERNAME} px4/airframes/4090_gz_x500_course_gimbal ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4090_gz_x500_course_gimbal
RUN AF=${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt \
    && sed -i 's|^\t4019_gz_x500_gimbal$|\t4019_gz_x500_gimbal\n\t4090_gz_x500_course_gimbal|' $AF \
    && grep -q 4090_gz_x500_course_gimbal $AF \
    && chmod +x ${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4090_gz_x500_course_gimbal

USER root
RUN pip3 install --no-cache-dir --break-system-packages \
      -r ${PX4_DIR}/Tools/setup/requirements.txt

# Build SITL against the vendored Gazebo. The prefix path is what lets PX4's
# find_package(gz-transport13) resolve without a second system-wide Gazebo.
USER ${USERNAME}
ENV GZ_VENDOR_PREFIX=/opt/ros/${ROS_DISTRO}/opt
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && export CMAKE_PREFIX_PATH="${GZ_VENDOR_PREFIX}/gz_transport_vendor:${GZ_VENDOR_PREFIX}/gz_msgs_vendor:${GZ_VENDOR_PREFIX}/gz_math_vendor:${GZ_VENDOR_PREFIX}/gz_utils_vendor:${GZ_VENDOR_PREFIX}/gz_cmake_vendor:${CMAKE_PREFIX_PATH}" \
    && cd ${PX4_DIR} && make px4_sitl_default

# ---------------------------------------------------------------- 5b2. Sim assets
# Installed AFTER the PX4 build on purpose: only the airframe has to exist
# before it (make targets are generated from the airframe list), so iterating on
# a model no longer costs a five minute PX4 rebuild.
USER ${USERNAME}
COPY --chown=${USERNAME}:${USERNAME} models/course_x500_base ${PX4_DIR}/Tools/simulation/gz/models/course_x500_base
COPY --chown=${USERNAME}:${USERNAME} models/course_x500 ${PX4_DIR}/Tools/simulation/gz/models/course_x500
COPY --chown=${USERNAME}:${USERNAME} models/course_gimbal ${PX4_DIR}/Tools/simulation/gz/models/course_gimbal
COPY --chown=${USERNAME}:${USERNAME} models/x500_course_gimbal ${PX4_DIR}/Tools/simulation/gz/models/x500_course_gimbal
RUN M=${PX4_DIR}/Tools/simulation/gz/models \
    && python3 -c "import xml.dom.minidom as m, sys; \
[m.parse(f) for f in sys.argv[1:]]; print('course SDF xml ok')" \
         $M/course_x500_base/model.sdf $M/course_x500/model.sdf \
         $M/course_gimbal/model.sdf $M/x500_course_gimbal/model.sdf \
         ${PX4_DIR}/Tools/simulation/gz/worlds/course_world.sdf \
    && ! grep -rhoE 'model://[a-z0-9_]+' $M/course_x500_base $M/course_x500 \
         $M/course_gimbal $M/x500_course_gimbal \
         | sort -u | grep -v '^model://course' \
    && echo 'course models are self-contained'

# ---------------------------------------------------------------- 5c. Course ROS package
# Pre-built into the image so students never wait on it, and so a broken course
# package fails the build rather than the lab. Their OWN code lives in the
# shared volume and is built there.
ENV COURSE_WS=/opt/course_ws
USER root
RUN mkdir -p ${COURSE_WS}/src && chown -R ${USERNAME}:${USERNAME} ${COURSE_WS}
USER ${USERNAME}
COPY --chown=${USERNAME}:${USERNAME} ros2_ws/src/drone_course_sim ${COURSE_WS}/src/drone_course_sim
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && cd ${COURSE_WS} && colcon build --packages-select drone_course_sim

# ---------------------------------------------------------------- 6. Entrypoint
USER root
COPY docker/scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
COPY docker/scripts/course /usr/local/bin/course
RUN chmod +x /usr/local/bin/entrypoint.sh /usr/local/bin/course

# `docker exec` bypasses the entrypoint, and run.sh uses exec to open extra
# terminals -- so without this every second terminal a student opens has no ROS
# on it, and the px4 binary cannot find the vendored Gazebo libraries.
# Static environment as ENV so it reaches EVERY exec session, whatever shell
# form is used -- `docker exec bash -c` reads no init file at all.
ENV GZ_PARTITION=course_local \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    GZ_SIM_RESOURCE_PATH=/opt/PX4-Autopilot/Tools/simulation/gz/models:/opt/PX4-Autopilot/Tools/simulation/gz/worlds

# Interactive terminals additionally get ROS sourced. This must be PREPENDED:
# Ubuntu's stock .bashrc returns early for non-interactive shells, so anything
# appended after that guard silently never runs.
USER ${USERNAME}
RUN printf '%s\n' \
      'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash' \
      '[ -f /opt/course_ws/install/setup.bash ] && source /opt/course_ws/install/setup.bash' \
      '[ -f $HOME/shared_volume/ros2_ws/install/setup.bash ] && source $HOME/shared_volume/ros2_ws/install/setup.bash' \
      '' > /tmp/course_rc \
    && cat /home/${USERNAME}/.bashrc >> /tmp/course_rc \
    && mv /tmp/course_rc /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
