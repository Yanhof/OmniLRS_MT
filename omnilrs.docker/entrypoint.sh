#!/bin/bash
set -e

# setup ros2 environment
#source "/opt/ros/$ROS_DISTRO/setup.bash" --
#cd /workspace/omnilrs
#exec "$@"

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Fix für ROS 2 / spdlog Konflikt
export LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:$LD_LIBRARY_PATH

cd /workspace/omnilrs
exec "$@"