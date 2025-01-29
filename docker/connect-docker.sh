#!/bin/bash

# Set project directory
PROJECT_DIR=$(pwd)

# Enable GUI
xhost +

# Check if the container is running
if [ "$(docker-compose -f $PROJECT_DIR/docker-compose.yml ps | grep 'ros2ubuntu.*Up')" ]; then
    docker-compose -f $PROJECT_DIR/docker-compose.yml exec ros2ubuntu bash
else
    echo "Starting the container..."
    docker-compose -f $PROJECT_DIR/docker-compose.yml up -d
    docker-compose -f $PROJECT_DIR/docker-compose.yml exec ros2ubuntu bash
fi
