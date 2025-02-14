#!/bin/bash

function usage() {
    echo -e "\nBuild an image for ZED SDK with ROS 2."
    echo "Args:"
    echo "    device - Target device for the image. Defaults to 'desktop'. ( desktop | jetson )"
}

# Set PWD to the scripts directory
cd $(dirname "$0")
PWD=$(pwd)

# Save the main script's arguments
main_args=("$@")
set --

# Check dependencies for the environment setup
source ./00-check-dependencies.sh
if [[ "$ALL_CHECKS_PASSED" = false ]]; then 
    echo "Missing dependencies!"
    exit 1
else
    echo "All dependencies checked!"
fi

# Restore the main script's arguments
set -- "${main_args[@]}"

# Check args
if [ "$#" -lt 1 ]; then
    usage
    exit 1
fi

echo ""
echo "Running multi-step Docker image build process:"
echo "    1. Build the 'zed-ros2-wrapper' image."
echo "    2. Build the 'tfm17-dev-env-v1' image from the 'zed-ros2-wrapper' image as base."
echo ""

# _____________________________________________________________________________
# :: [STEP 1] ::
# :: Build `zed-ros2-wrapper` image

echo "[Step 1] (WKDIR) :: ${PWD}"
echo ""

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
    echo "Invalid argument: $1"
    usage
fi

# _____________________________________________________________________________
# :: [STEP 2] ::
# :: Build tfm17-dev-env-v1 image 

cd $(dirname "$0")
PWD=$(pwd)
echo ""
echo "[Step 2] (WKDIR) :: ${PWD}"
echo ""

DOCKERDIR="../docker"
DOCKERFILE="Dockerfile.tfm17-dev-env-v1"

docker build -t tfm17-dev-env-v1 -f "${DOCKERDIR}/${DOCKERFILE}" .
