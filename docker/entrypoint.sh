#!/bin/bash
set -e

# Source ROS and your workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros2_ws/install/setup.bash

exec "$@"
