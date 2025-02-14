#!/bin/bash

set -e
cd $(dirname "$0")
cd ../docker

# Allow the container to connect to the host's X server for display passthrough
sudo xhost +si:localuser:root

# Run the containers~
ROS_DISTRO=humble
docker-compose up -d --build

# Attach an interactive terminal to the container
sh ../scripts/attach-terminal.sh tfm17-mapper
