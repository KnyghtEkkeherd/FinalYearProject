#!/bin/bash

# Get tf out if any failures
set -e

# Set ROS distro
ROS_DISTRO="jazzy"

# Source setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Move into ws
cd /root/fyp/ros2_ws

# Install deps
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Build
colcon build
source install/setup.bash

