# Testing zed-ros2-wrapper

There are a few useful commands to verify if the dependencies are properly set up.
```bash
nvidia-smi
nvcc --version
lsusb | grep STEREOLABS ZED
ls /dev/video* # Once with the camera connected and once with it disconnected to probe for changes in output
ros2 pkg list | grep zed
ros2 pkg list | grep octomap
```

When first testing the development environment, the `zed_display_rviz2` package was missing. While manually installing it inside the container's ROS 2 workspace is an option, this should be moved to the Dockerfile, along with `ros-humble-depthimage-to-laserscan` which is reportedly missing a dependency.
```bash
cd /root/ros2_ws/src && \ 
git clone https://github.com/stereolabs/zed-ros2-examples.git && \
cd /root/ros2_ws && \
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release && \
source install/local_setup.bash
```

This command will launch a sample with a ZED 2 camera and RViz2 for visualization. By setting the `sim_mode` to `false` we ensure that the real camera is used.
```bash
ros2 launch zed_display_rviz2 display_zed_cam.launch.py camera_model:=zed2 sim_mode:=false
```
