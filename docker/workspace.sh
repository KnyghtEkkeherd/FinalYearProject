#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Set ROS distro
ROS_DISTRO="jazzy"

echo "Starting workspace setup script..."

# Source ROS setup
source /opt/ros/$ROS_DISTRO/setup.bash
echo "Sourced ROS setup for $ROS_DISTRO."

# Move into workspace
WORKSPACE_DIR="/root/FinalYearProject"
echo "Moving to workspace directory: $WORKSPACE_DIR"
cd $WORKSPACE_DIR

# Install dependencies
echo "Updating rosdep..."
rosdep update
echo "Installing dependencies from src..."
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Build the workspace
echo "Building the workspace..."
colcon build

# Source the install setup
echo "Sourcing the install setup..."
source install/setup.bash

echo "Workspace setup completed successfully."
