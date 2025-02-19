#!/bin/bash

shopt -s expand_aliases
set -e

cd "$(dirname "$0")" || exit
cd ../docker

# Alias the compose command for compatibility between Docker installations
alias docker-compose="docker compose"

# Allow the container to connect to the host's X server for display passthrough
sudo xhost +si:localuser:root

# Default action
CREATE_CONTAINERS=false

# Parse arguments
while [[ "$#" -gt 0 ]]; do
  case $1 in
    --create) CREATE_CONTAINERS=true ;;
    *) echo "Unknown parameter passed: $1"; exit 1 ;;
  esac
  shift
done

# Choose between creating containers or just starting them
if [ "$CREATE_CONTAINERS" = true ]; then
  echo "Creating and starting containers..."
  docker-compose up -d --build
else
  echo "Starting existing containers..."
  if ! docker-compose start; then
    echo "Failed to start containers. Attempting to create and start with 'docker-compose up'..."
    docker-compose up -d
  fi
fi

# Attach an interactive terminal to the container
CONTAINER_NAME="tfm17-mapper"
ROS_DISTRO="humble"
bash ../scripts/attach-terminal.sh $CONTAINER_NAME $ROS_DISTRO
