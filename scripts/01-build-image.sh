#!/bin/bash

function usage() {
    echo -e "\nBuild an image for ZED SDK with ROS 2."
    echo "Usage:"
    echo "  $0 <device> [--ubuntu <version_number>] [--cuda <version_number>] [--zed-sdk <version_number>]"
    echo "Args:"
    echo "  device         - Target device for the image. (desktop | jetson)"
    echo "  --ubuntu       - Ubuntu version number (e.g., 22.04). Defaults to 22.04."
    echo "  --cuda         - CUDA version number (e.g., 11.8.0). Defaults to 12.6.3 (desktop only)."
    echo "                   If you pass just two numbers (e.g., 11.8), .0 will be appended."
    echo "  --zed-sdk      - ZED SDK version number (e.g., 4.2.3). Defaults to 4.2.3."
}

# Set working directory to the scripts directory
cd "$(dirname "$0")" || exit
PWD=$(pwd)

# Default version numbers (without prefixes)
UBUNTU_VERSION="22.04"
CUDA_VERSION="12.6.3"
ZED_SDK_VERSION="4.2.3"

# Parse options using getopt
OPTS=$(getopt -o '' --long ubuntu:,cuda:,zed-sdk: -n 'parse-options' -- "$@")
if [ $? != 0 ]; then
    echo "Failed parsing options." >&2
    usage
    exit 1
fi
eval set -- "$OPTS"

while true; do
    case "$1" in
        --ubuntu)
            UBUNTU_VERSION="$2"
            shift 2 ;;
        --cuda)
            CUDA_VERSION="$2"
            shift 2 ;;
        --zed-sdk)
            ZED_SDK_VERSION="$2"
            shift 2 ;;
        --)
            shift
            break ;;
        *)
            break ;;
    esac
done

# Normalize the Ubuntu version:
# Ensure it follows the pattern "XX.XX" (e.g., "22.04")
dot_count=$(echo "$UBUNTU_VERSION" | awk -F. '{print NF-1}')
if [ "$dot_count" -eq 0 ]; then
    UBUNTU_VERSION="${UBUNTU_VERSION}.04"
fi

# Normalize the CUDA version:
# If the version contains only one dot (e.g., "11.8"), append ".0" to form "11.8.0"
dot_count=$(echo "$CUDA_VERSION" | awk -F. '{print NF-1}')
if [ "$dot_count" -eq 1 ]; then
    CUDA_VERSION="${CUDA_VERSION}.0"
fi

# Normalize the ZED SDK version:
# If the version contains only two numbers (e.g., "4.2"), append ".0" to form "4.2.0"
dot_count=$(echo "$ZED_SDK_VERSION" | awk -F. '{print NF-1}')
if [ "$dot_count" -eq 1 ]; then
    ZED_SDK_VERSION="${ZED_SDK_VERSION}.0"
fi

# Now, the remaining positional arguments should contain the device
if [ "$#" -lt 1 ]; then
    usage
    exit 1
fi

DEVICE="$1"
shift

echo ""
echo "Running multi-step Docker image build process:"
echo "    1. Build the 'zed-ros2-wrapper' image for ${DEVICE}."
echo "    2. Build the 'tfm17-dev-env-v1' image from the 'zed-ros2-wrapper' image as base."
echo ""

# _____________________________________________________________________________
# :: [STEP 1] :: Build the `zed-ros2-wrapper` image
echo "[Step 1] (WKDIR) :: ${PWD}"
echo ""

DOCKERDIR=../zed-ros2-wrapper/docker
DESKTOP_SCRIPT="desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh"
JETSON_SCRIPT="jetson_build_dockerfile_from_sdk_and_l4T_version.sh"

if [ "$DEVICE" = "desktop" ]; then
    UBUNTU="ubuntu-${UBUNTU_VERSION}"
    ZEDSDK="zedsdk-${ZED_SDK_VERSION}"
    CUDA="cuda-${CUDA_VERSION}"
    SCRIPT="${DOCKERDIR}/${DESKTOP_SCRIPT}"
    $SCRIPT "$UBUNTU" "$CUDA" "$ZEDSDK"
elif [ "$DEVICE" = "jetson" ]; then
    L4T="l4t-r36.3.0"
    ZEDSDK="zedsdk-${ZED_SDK_VERSION}" 
    SCRIPT="${DOCKERDIR}/${JETSON_SCRIPT}"
    $SCRIPT "$L4T" "$ZEDSDK"
else 
    echo "Invalid argument: $DEVICE"
    usage
    exit 1
fi

# _____________________________________________________________________________
# :: [STEP 2] :: Build the tfm17-dev-env-v1 image 
cd "$(dirname "$0")" || exit
PWD=$(pwd)
echo ""
echo "[Step 2] (WKDIR) :: ${PWD}"
echo ""

DOCKERDIR="../docker"
DOCKERFILE="Dockerfile.tfm17-dev-env-v1"

docker build \
  --build-arg TARGET_DEVICE="${DEVICE}" \
  --build-arg UBUNTU_VERSION="${UBUNTU_VERSION}" \
  --build-arg ZED_SDK_VERSION="${ZED_SDK_VERSION}" \
  --build-arg CUDA_VERSION="${CUDA_VERSION}" \
  -t tfm17-dev-env-v1 \
  -f "${DOCKERDIR}/${DOCKERFILE}" .


# #!/bin/bash

# function usage() {
#     echo -e "\nBuild an image for ZED SDK with ROS 2."
#     echo "Usage:"
#     echo "  $0 <device> [--cuda <version_number>]"
#     echo "Args:"
#     echo "  device         - Target device for the image. (desktop | jetson)"
#     echo "  --cuda         - CUDA version number (e.g., 11.8.0). Defaults to 12.6.3 (desktop only)."
#     echo "                   If you pass just two numbers (e.g., 11.8), .0 will be appended."
# }

# # Set working directory to the scripts directory
# cd "$(dirname "$0")"
# PWD=$(pwd)

# # Default CUDA version number (without prefix)
# CUDA_VERSION="12.6.3"

# # Parse options using getopt
# OPTS=$(getopt -o '' --long cuda: -n 'parse-options' -- "$@")
# if [ $? != 0 ]; then
#     echo "Failed parsing options." >&2
#     usage
#     exit 1
# fi
# eval set -- "$OPTS"

# while true; do
#     case "$1" in
#         --cuda)
#             CUDA_VERSION="$2"
#             shift 2 ;;
#         --)
#             shift
#             break ;;
#         *)
#             break ;;
#     esac
# done

# # Normalize the CUDA version:
# # If the version contains only one dot (e.g., "11.8"), append ".0" to form "11.8.0"
# dot_count=$(echo "$CUDA_VERSION" | awk -F. '{print NF-1}')
# if [ "$dot_count" -eq 1 ]; then
#     CUDA_VERSION="${CUDA_VERSION}.0"
# fi

# # Now, the remaining positional arguments should contain the device
# if [ "$#" -lt 1 ]; then
#     usage
#     exit 1
# fi

# DEVICE="$1"
# shift

# echo ""
# echo "Running multi-step Docker image build process:"
# echo "    1. Build the 'zed-ros2-wrapper' image."
# echo "    2. Build the 'tfm17-dev-env-v1' image from the 'zed-ros2-wrapper' image as base."
# echo ""

# # _____________________________________________________________________________
# # :: [STEP 1] :: Build the `zed-ros2-wrapper` image
# echo "[Step 1] (WKDIR) :: ${PWD}"
# echo ""

# DOCKERDIR=../zed-ros2-wrapper/docker
# DESKTOP_SCRIPT="desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh"
# JETSON_SCRIPT="jetson_build_dockerfile_from_sdk_and_l4T_version.sh"

# if [ "$DEVICE" = "desktop" ]; then
#     UBUNTU="ubuntu-22.04"
#     CUDA="cuda-${CUDA_VERSION}"
#     ZEDSDK="zedsdk-4.2.3"
#     SCRIPT="${DOCKERDIR}/${DESKTOP_SCRIPT}"
#     $SCRIPT "$UBUNTU" "$CUDA" "$ZEDSDK"
# elif [ "$DEVICE" = "jetson" ]; then
#     L4T="l4t-r36.3.0"
#     ZEDSDK="zedsdk-4.2.3" 
#     SCRIPT="${DOCKERDIR}/${JETSON_SCRIPT}"
#     $SCRIPT "$L4T" "$ZEDSDK"
# else 
#     echo "Invalid argument: $DEVICE"
#     usage
#     exit 1
# fi

# # _____________________________________________________________________________
# # :: [STEP 2] :: Build the tfm17-dev-env-v1 image 
# cd "$(dirname "$0")"
# PWD=$(pwd)
# echo ""
# echo "[Step 2] (WKDIR) :: ${PWD}"
# echo ""

# DOCKERDIR="../docker"
# DOCKERFILE="Dockerfile.tfm17-dev-env-v1"

# docker build -t tfm17-dev-env-v1 -f "${DOCKERDIR}/${DOCKERFILE}" .


# #!/bin/bash

# function usage() {
#     echo -e "\nBuild an image for ZED SDK with ROS 2."
#     echo "Args:"
#     echo "    device - Target device for the image. Defaults to 'desktop'. ( desktop | jetson )"
# }

# # Set PWD to the scripts directory
# cd $(dirname "$0")
# PWD=$(pwd)

# # Save the main script's arguments
# main_args=("$@")
# set --

# # Check dependencies for the environment setup
# source ./00-check-dependencies.sh
# if [[ "$ALL_CHECKS_PASSED" = false ]]; then 
#     echo "Missing dependencies!"
#     exit 1
# else
#     echo "All dependencies checked!"
# fi

# # Restore the main script's arguments
# set -- "${main_args[@]}"

# # Check args
# if [ "$#" -lt 1 ]; then
#     usage
#     exit 1
# fi

# echo ""
# echo "Running multi-step Docker image build process:"
# echo "    1. Build the 'zed-ros2-wrapper' image."
# echo "    2. Build the 'tfm17-dev-env-v1' image from the 'zed-ros2-wrapper' image as base."
# echo ""

# # _____________________________________________________________________________
# # :: [STEP 1] ::
# # :: Build `zed-ros2-wrapper` image

# echo "[Step 1] (WKDIR) :: ${PWD}"
# echo ""

# DOCKERDIR=../zed-ros2-wrapper/docker
# DESKTOP_SCRIPT="desktop_build_dockerfile_from_sdk_ubuntu_and_cuda_version.sh"
# JETSON_SCRIPT="jetson_build_dockerfile_from_sdk_and_l4T_version.sh"

# if [ "$1" = "desktop" ]; then
#     UBUNTU="ubuntu-22.04"
#     CUDA="cuda-12.6.3"
#     ZEDSDK="zedsdk-4.2.3"
#     SCRIPT="${DOCKERDIR}/${DESKTOP_SCRIPT}"
#     $SCRIPT $UBUNTU $CUDA $ZEDSDK
# elif [ "$1" = "jetson" ]; then
#     L4T="l4t-r36.3.0"
#     ZEDSDK="zedsdk-4.2.3" 
#     SCRIPT="${DOCKERDIR}/${JETSON_SCRIPT}"
#     $SCRIPT $L4T $ZEDSDK
# else 
#     echo "Invalid argument: $1"
#     usage
# fi

# # _____________________________________________________________________________
# # :: [STEP 2] ::
# # :: Build tfm17-dev-env-v1 image 

# cd $(dirname "$0")
# PWD=$(pwd)
# echo ""
# echo "[Step 2] (WKDIR) :: ${PWD}"
# echo ""

# DOCKERDIR="../docker"
# DOCKERFILE="Dockerfile.tfm17-dev-env-v1"

# docker build -t tfm17-dev-env-v1 -f "${DOCKERDIR}/${DOCKERFILE}" .
