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
ros2 launch point_cloud_accumulator_pkg point_cloud_accumulator.launch.py min_points_thr:=1000 max_points_thr:=100000 enable_logging:=true
```

### Colored Octomap Node
```bash
colcon build --packages-select hybrid_mapper_pkg && \
    source install/setup.bash
```

```bash
ros2 launch hybrid_mapper_pkg hybrid_mapper.launch.py fused_cloud_topic:=/accumulator/cloud_frame resolution_m:=0.05 enable_logging:=true
```

### RTAB-Map
```bash
ros2 launch rtabmap_launch rtabmap.launch.py rtabmap_args:="--delete_db_on_start" \
  stereo:=true \
  subscribe_depth:=false \
  subscribe_stereo:=true \
  approx_sync:=false \
  frame_id:=zed_left_camera_optical_frame \
  left_image_topic:=/zed/zed_node/left/image_rect_color \
  right_image_topic:=/zed/zed_node/right/image_rect_color \
  left_camera_info_topic:=/zed/zed_node/left/camera_info \
  right_camera_info_topic:=/zed/zed_node/right/camera_info \
  odom_topic:=zed/zed_node/odom \
  rtabmap_viz:=true
```
