#!/bin/bash

# Create temp dir for gui apps
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 setup for ${ROS_DISTRO}."

# Source workspace setup
if [ -f "/fyp_ws/install/setup.bash" ]; then
    source /fyp_ws/install/setup.bash
    echo "Sourced workspace setup."
else
    echo "No workspace setup found."
fi

# Execute any commands passed to the entrypoint
exec "$@"