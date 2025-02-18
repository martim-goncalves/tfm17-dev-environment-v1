#!/bin/bash
cd "$(dirname "$0")" || exit
bash ./docker-startup.sh
docker start portainer
