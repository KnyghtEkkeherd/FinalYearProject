#!/bin/bash

# Create shared ros2 folder (files to be shared between container and host)
if [ ! -d "$HOME/$USER/shared/" ]; then
  echo "CREATING $HOME/$USER/shared/..."
  mkdir -p $HOME/$USER/shared/
fi

# Build docker img
SCRIPT_PATH=$(dirname $(realpath "$0"))
PARENT_PATH=$(dirname "$SCRIPT_PATH")
sudo docker image build -f $SCRIPT_PATH/Dockerfile -t ros2ubuntu:latest $PARENT_PATH #--no-cache