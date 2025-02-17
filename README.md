# TFM17 - Development Environment V1
This project encapsulates the configuration of a portable development environment for TFM17. It aims to facilitate work on a functional prototype for mapping caves using stereoscopy. The stereoscopic sensor used to achieve this goal was initially determined to be the ZED 2 camera. This choice was made due to equipment availability. While that may be subject to change in future iterations of the project, this environment will always be configured to support the ZED SDK as a baseline.

## Cloning the repository

When cloning the repository for the first time:
```bash
git clone --recurse-submodules https://github.com/martim-goncalves/tfm17-dev-environment-v1.git
```

Working with an already cloned repository:
```bash
git clone https://github.com/martim-goncalves/tfm17-dev-environment-v1.git
cd tfm17-dev-environment-v1
git submodule update --init --recursive
```

If the submodule is not correctly marked, try:
```bash
git submodule deinit -f -- ./zed-ros2-wrapper
git submodule update --init --recursive
```

Don't forget to ensure all scripts inside it are made executable! Otherwise, permission will be denied.
```bash
chmod -R +x ~/tfm17-dev-environment-v1
```

## Checking for installations

```bash
nvidia-smi
ros2 pkg list | grep octomap
```


## Sources
1. [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
2. [Stereolabs ZED ROS 2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper)