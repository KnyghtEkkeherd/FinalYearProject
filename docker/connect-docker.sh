#!/bin/bash

# Set project directory
PROJECT_DIR=$(pwd)

echo "Connecting to the ROS2 Ubuntu container in the project directory: $PROJECT_DIR"

# Enable GUI
xhost +

# Build the Docker image
SCRIPT_PATH=$(dirname $(realpath "$0"))
PARENT_PATH=$(dirname "$SCRIPT_PATH")

echo "Building the Docker image..."
sudo docker image build --no-cache=false -f $SCRIPT_PATH/Dockerfile -t ros2ubuntu:latest $PARENT_PATH

# Remove all containers (excluding the built images)
echo "Removing all containers..."
docker container prune -f

# Start the container
echo "Starting the container..."
docker-compose -f $PROJECT_DIR/docker-compose.yml up -d

# Connect to the container
docker-compose -f $PROJECT_DIR/docker-compose.yml exec ros2ubuntu bash
