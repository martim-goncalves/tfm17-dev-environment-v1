# Bash Command Cheatsheet

## Running Nodes

### ⛅ Point Cloud Accumulator Node
```bash
ros2 run point_cloud_accumulator_pkg point_cloud_accumulator_node --ros-args -r /cloud_in:=/zed/zed_node/point_cloud/cloud_registered -p savefolder:=/root/ros2_ws/src/artifacts/ -p savefile:=accumulated_cloud -p voxel_size_m:=0.1 -p num_neighbors:=20 -p std_ratio:=2.0 -p save_interval_seconds:=0
```
