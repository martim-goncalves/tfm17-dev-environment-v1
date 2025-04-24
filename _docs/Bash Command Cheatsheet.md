# Bash Command Cheatsheet

## Running Nodes

### Stereolabs' ZED Camera Wrapper (ZED 2)
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
```

### ⛅ Point Cloud Accumulator Node
```bash
ros2 run point_cloud_accumulator_pkg point_cloud_accumulator_node --ros-args -r /cloud_in:=/zed/zed_node/point_cloud/cloud_registered -p savefolder:=/root/ros2_ws/src/artifacts/ -p savefile:=accumulated_cloud -p min_points_thr:=1000 -p max_points_thr:=100000 -p min_voxel_size_m:=0.025 -p max_voxel_size_m:=0.1 -p num_neighbors:=20 -p std_ratio:=2.0 -p save_interval_seconds:=0 --log-level INFO
```

### Colored Octomap Node
```bash
ros2 run colored_octomap_pkg colored_octomap_node --ros-args -r /cloud_in:=/accumulator/cloud_frame -p resolution_m:=0.05 -p timer_period_seconds:=0 -p savefolder:=/root/ros2_ws/src/artifacts/ -p savefile:=colored_octomap
```
