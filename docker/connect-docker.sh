#!/bin/bash

# Set project directory
PROJECT_DIR=$(pwd)

echo "Connecting to the ROS2 Ubuntu container in the project directory: $PROJECT_DIR"

# Enable GUI
xhost +

# Check if the container is running
if [ "$(docker-compose -f $PROJECT_DIR/docker-compose.yml ps | grep 'ros2ubuntu.*Up')" ]; then
    echo "The container is already running. Connecting to the container..."
    docker-compose -f $PROJECT_DIR/docker-compose.yml exec ros2ubuntu bash
else
    echo "The container is not running. Starting the container..."
    docker-compose -f $PROJECT_DIR/docker-compose.yml up -d
    docker-compose -f $PROJECT_DIR/docker-compose.yml exec ros2ubuntu bash
fi
