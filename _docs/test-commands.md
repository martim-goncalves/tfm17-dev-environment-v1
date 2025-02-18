```bash
docker exec -it tfm17-mapper bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/local_setup.bash && exec bash"
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2
ros2 run octomap_server octomap_server_node --ros-args -r /cloud_in:=/zed/zed_node/point_cloud/cloud_registered
rviz2
```

```bash
ros2 topic hz /zed/zed_node/point_cloud/cloud_registered
```
