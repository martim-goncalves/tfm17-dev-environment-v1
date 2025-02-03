#!/bin/bash

function usage() {
    echo -e "Build an image for ZED SDK with ROS 2."
    echo -e "Args:"
    echo -e "    device - Target device for the image. Defaults to 'desktop'. ( desktop | jetson )"
}

if [ "$#" -lt 1 ]; then
    usage
    exit 1
fi

echo -e ""
echo -e "Running multi-step Docker image build process:"
echo -e "    1. Build the 'zed-ros2-wrapper' image."
echo -e "    2. Build the 'tfm17-dev-env-v1' image from the 'zed-ros2-wrapper' image as base."
echo -e ""

# _____________________________________________________________________________
# Build `zed-ros2-wrapper` image

cd $(dirname "$0")
PWD=$(pwd)
echo -e "[Step 1] (WKDIR) :: ${PWD}"
echo -e ""

DOCKERDIR=../zed-ros2-wrapper/docker
DESKTOP_SCRIPT="desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh"
JETSON_SCRIPT="jetson_build_dockerfile_from_sdk_and_l4T_version.sh"

if [ "$1" = "desktop" ]; then
    UBUNTU="ubuntu-22.04"
    CUDA="cuda-12.6.3"
    ZEDSDK="zedsdk-4.2.3"
    SCRIPT="${DOCKERDIR}/${DESKTOP_SCRIPT}"
    $SCRIPT $UBUNTU $CUDA $ZEDSDK
elif [ "$1" = "jetson" ]; then
    L4T="l4t-r36.3.0"
    ZEDSDK="zedsdk-4.2.3" 
    SCRIPT="${DOCKERDIR}/${JETSON_SCRIPT}"
    $SCRIPT $L4T $ZEDSDK
else 
    echo -e "Invalid argument: $1"
    usage
fi

# _____________________________________________________________________________
# Build tfm17-dev-env-v1 image 

cd $(dirname "$0")
PWD=$(pwd)
echo -e ""
echo -e "[Step 2] (WKDIR) :: ${PWD}"
echo -e ""

DOCKERDIR="../docker"
DOCKERFILE="Dockerfile.tfm17-dev-env-v1"

docker build -t tfm17-dev-env-v1 -f "${DOCKERDIR}/${DOCKERFILE}" .
