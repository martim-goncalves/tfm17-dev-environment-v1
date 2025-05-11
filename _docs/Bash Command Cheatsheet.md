# Bash Command Cheatsheet

## Running Nodes

### Stereolabs' ZED Camera Wrapper (ZED 2)
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

### ⛅ Point Cloud Accumulator Node
Building the package:
```bash
colcon build --packages-select point_cloud_accumulator_pkg && \
    source install/setup.bash
```

Running the node with custom parameters (e.g. min/max point threshold for interpolation):
```bash
ros2 launch point_cloud_accumulator_pkg point_cloud_accumulator.launch.py min_points_thr:=1000 max_points_thr:=100000 enable_logging:=false
```

### Colored Octomap Node
```bash
ros2 run colored_octomap_pkg colored_octomap_node --ros-args -r /cloud_in:=/accumulator/cloud_frame -p resolution_m:=0.05 -p timer_period_seconds:=0 -p savefolder:=/root/ros2_ws/src/artifacts/ -p savefile:=colored_octomap
```
