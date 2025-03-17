#!/bin/bash
set -e

# Source ROS 2 setup
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the workspace setup
if [ -f "/camera_ws/install/setup.bash" ]; then
  source "/camera_ws/install/setup.bash"
fi

exec "$@"
