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

# Stop and remove the old container if it exists
if [ "$(docker ps -q -f name=ros2ubuntu)" ]; then
    echo "Stopping and removing the old container..."
    docker stop ros2ubuntu
    docker rm ros2ubuntu
fi

# Remove old images
echo "Removing old images..."
docker image prune -f

# Start the container
echo "Starting the container..."
docker-compose -f $PROJECT_DIR/docker-compose.yml up -d

# Connect to the container
docker-compose -f $PROJECT_DIR/docker-compose.yml exec ros2ubuntu bash
