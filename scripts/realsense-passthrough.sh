#!/bin/bash
set -e

sudo apt-get update
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev build-essential cmake

if [ ! -d librealsense ]; then
    git clone https://github.com/IntelRealSense/librealsense.git
fi

cd librealsense

git checkout v2.55.1 || echo "Using default branch"

rm -rf build
mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

sudo make install

wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

rs-enumerate-devices --compact
