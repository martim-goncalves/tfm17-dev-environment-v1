#!/bin/bash

function usage() {
    echo -e "Build an image for ZED SDK with ROS 2."
    echo -e "Args:"
    echo -e "    device - Target device for the image. Defaults to 'desktop'. ( desktop | jetson )"
}

usage()