#!/bin/bash

function usage() {
    echo "Attach an interactive terminal to a container."
    echo "Args:"
    echo "    container - Name of the running container."
}

if [ "$#" -lt 1 ]; then
    usage
    exit 1
fi

# echo "Attaching interactive terminal to container '$1'."

docker exec -it "$1" bash -c " \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /root/ros2_ws/install/local_setup.bash && \
    exec bash"
