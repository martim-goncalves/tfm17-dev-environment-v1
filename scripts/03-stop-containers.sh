#!/bin/bash

shopt -s expand_aliases
set -e

cd "$(dirname "$0")" || exit
cd ../docker

# Alias the compose command for compatibility between Docker installations
alias docker-compose="docker compose"

# Stop containers belonging to the service/stack in the docker directory
docker-compose stop
