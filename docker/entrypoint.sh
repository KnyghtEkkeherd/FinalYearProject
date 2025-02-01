#!/bin/bash

# Set ROS distro
ROS_DISTRO="jazzy"

# Create temp dir for gui apps
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Set ROS ws
ROS_WS_ROOT="/home/ros/FinalYearProject"

# Source ROS setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Source ws setup
source $ROS_WS_ROOT/install/setup.bash

# Source .bashrc
source /home/ros/.bashrc

# Move into ws
cd $ROS_WS_ROOT

# Build with colcon
colcon build

# Return to root
cd

# Re-source .bashrc
source /home/ros/.bashrc

# Any command post-entrypoint runs via this script
exec "$@"