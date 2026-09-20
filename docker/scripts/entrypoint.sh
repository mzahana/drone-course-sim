#!/usr/bin/env bash
set -e
source /opt/ros/${ROS_DISTRO}/setup.bash
export PX4_DIR=${PX4_DIR:-/opt/PX4-Autopilot}
export GZ_SIM_RESOURCE_PATH="${PX4_DIR}/Tools/simulation/gz/models:${PX4_DIR}/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
# Never let a simulation escape onto the class network. See run.sh for why.
export GZ_PARTITION="${GZ_PARTITION:-course_local}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
if [ -f "/opt/course_ws/install/setup.bash" ]; then
  source "/opt/course_ws/install/setup.bash"
fi
if [ -f "$HOME/shared_volume/ros2_ws/install/setup.bash" ]; then
  source "$HOME/shared_volume/ros2_ws/install/setup.bash"
fi
exec "$@"
