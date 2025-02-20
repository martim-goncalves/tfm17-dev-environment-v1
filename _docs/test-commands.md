```bash
docker exec -it tfm17-mapper bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/local_setup.bash && exec bash"
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2 publish_tf:=true slam_enabled:=false
ros2 run octomap_server octomap_server_node --ros-args -r /frame_id:=map /cloud_in:=/zed/zed_node/point_cloud/cloud_registered
rviz2
```

```bash
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
```

```bash
ros2 run octomap_server octomap_server_node --ros-args -r /cloud_in:=/zed/zed_node/point_cloud/cloud_registered -p frame_id:=map -p base_frame_id:=base_link -p sensor_frame_id:=zed_left_camera_frame -p height_map:=false -p resolution:=0.025 -p max_range:=5.0 -p raycast_range:=5.0 -p latch:=true -p filter_ground:=false # --log-level debug
```
