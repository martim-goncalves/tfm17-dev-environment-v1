#!/bin/bash
cd "$(dirname "$0")" || exit
bash ./docker-startup.sh
docker start portainer >/dev/null 2>&1
echo "[+] Running Portainer at: http://localhost:9000/"
