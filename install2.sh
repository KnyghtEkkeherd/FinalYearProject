#!/bin/bash

# Check if the docker group exists, and create if it doesn't
if ! getent group docker > /dev/null; then
  sudo groupadd docker
fi

# Add self to docker group
sudo usermod -aG docker $USER

# Activate changes
newgrp docker

# Test without sudo
docker run hello-world

# Run on start
sudo systemctl enable docker.service
sudo systemctl enable containerd.service
