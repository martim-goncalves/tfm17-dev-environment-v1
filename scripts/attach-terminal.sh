#!/bin/bash

function usage() {
    echo "Attach an interactive terminal to a container."
    echo "Args:"
    echo "    container     - Name of the running container."
    echo "    ros-distro    - Name of the ROS 2 distribution."
}

if [ "$#" -lt 2 ]; then
    usage
    exit 1
fi

CONTAINER_NAME=$1
ROS_DISTRO=$2

echo -e "\n[+] Attaching interactive terminal to container..."
echo -e "\t- Container \t.. \t${CONTAINER_NAME}"
echo -e "\t- ROS Distro \t.. \t${ROS_DISTRO}\n"

docker exec -it "$CONTAINER_NAME" bash -c " \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /root/ros2_ws/install/local_setup.bash && \
    exec bash"
