#!/bin/bash

# Set ROS distro
ROS_DISTRO="jazzy"

# Create temp dir for gui apps
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Create ROS ws
ROS_WS="/root/fyp/ros2_ws"

# Shared ROS files
SHARED_ROS="/root/fyp/shared/"

# Source ROS setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Source ws setup
source $ROS_WS/install/setup.bash

# Source .bashrc
source /root/.bashrc

# Move into ws
cd $ROS_WS

# Build with colcon
colcon build

# Return to root
cd

# Re-source .bashrc
source /root/.bashrc

# Any command post-haste runs via this script
exec "$@"