#!/bin/bash
set -e

# Source ROS environment
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
else
    echo "⚠️ Could not find ROS setup.bash"
    exit 1
fi

# Source workspace if it exists
if [ -f "/ros2_ws/install/setup.bash" ]; then
    # shellcheck source=/ros2_ws/install/setup.bash
    source "/ros2_ws/install/setup.bash"
else
    echo "🔹 No workspace to source at /ros2_ws/install/setup.bash"
fi

# Show RealSense devices
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    echo "🔎 RealSense Devices:"
    rs-enumerate-devices
else
    echo "⚠️ rs-enumerate-devices not found"
fi

if [ $# -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi
