#!/bin/bash

shopt -s expand_aliases
set -e

cd $(dirname "$0")
cd ../docker

# Alias the compose command for compatibility between Docker installations
alias docker-compose="docker compose"

# Allow the container to connect to the host's X server for display passthrough
sudo xhost +si:localuser:root

# Run the containers
export ROS_DISTRO=humble
docker-compose up -d --build

# Attach an interactive terminal to the container
bash ../scripts/attach-terminal.sh tfm17-mapper
