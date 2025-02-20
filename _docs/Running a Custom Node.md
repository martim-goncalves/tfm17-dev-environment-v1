# Running a Custom Node
This document serves as a step-by-step guide on how to run a custom node in the development environment's `tfm17-mapper` container. Part of the steps performed in it may already have been performed for packages installed via the Dockerfile or by using volumes instead of file transfer, but will be shown as a complete workflow for introducing custom code.

First and foremost, it is assumed that you are operating from a terminal attached to the container. Starting from the project root, `tfm17-dev-environment-v1`:

## Run only once (package creation)
```bash
cd scripts/
./attach-terminal.sh tfm17-mapper humble
```

Create a new package:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake colored_octomap_pkg
```

## Run each time the code is updated
Open a new terminal. Place the source code inside the package and replace the default `CMakeLists.txt`:
```bash
cd ../custom-ros2-pkgs/ros-humble-colored-octomap
docker cp colored_octomap_node.cpp tfm17-mapper:/root/ros2_ws/src/colored_octomap_pkg/src/
docker cp ./CMakeLists.txt tfm17-mapper:/root/ros2_ws/src/colored_octomap_pkg/
```

Back on the attached terminal, build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select colored_octomap_pkg
source install/setup.bash
```

```bash
ros2 run colored_octomap_pkg colored_octomap_node --ros-args -r /cloud_in:=/zed/zed_node/point_cloud/cloud_registered -p resolution_m:=0.025 -p timer_period_seconds:=20
```