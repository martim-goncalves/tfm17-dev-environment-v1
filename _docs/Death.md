New container. Who this?

```bash
docker build -t tfm17-dev-env-v1-realsense -f Dockerfile.tfm17-dev-env-v1.realsense.v1 .
```

```bash
docker run -it \
    --privileged \
    --device=/dev/bus/usb \
    --device-cgroup-rule='c 81:* rmw' \
    --device-cgroup-rule='c 189:* rmw' \
    -v /dev:/dev \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -e DISPLAY=$DISPLAY \
    --gpus all \
    --name tfm17-mapper-realsense tfm17-dev-env-v1-realsense
```

```bash
source /opt/ros/humble/setup.bash 
source install/setup.bash 
```

```bash
apt-get update && apt-get install -y usbutils lsof

```

Check `realsense-passthrough.sh`.

May need to `sudo systemctl restart systemd-udevd` on the host and restart container:
```bash
sudo systemctl restart systemd-udevd
docker start -ai tfm17-mapper-realsense
```

Or just `sudo udevadm trigger` on the container
```bash
sudo udevadm trigger
```

```bash
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
