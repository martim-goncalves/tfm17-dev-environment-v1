#!/bin/bash

set -euo pipefail

NVIDIA_DRIVERS_AVAILABLE=false
NVIDIA_RUNTIME_AVAILABLE=false
DOCKER_ENGINE_RUNNING=false
VIRTUALIZATION_SUPPORTED=false

# Verbosity control
VERBOSE=false
SILENT=false

# Parse arguments
for arg in "$@"; do
    case $arg in
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -s|--silent)
            SILENT=true
            shift
            ;;
        *)
            echo "Unknown argument: $arg"
            echo "Usage: $0 [-v|--verbose] [-s|--silent]"
            exit 1
            ;;
    esac
done

check_nvidia_drivers() {
    
    $VERBOSE && echo "::: NVIDIA Drivers Check :::"
    
    if command -v nvidia-smi >/dev/null 2>&1; then
        $VERBOSE && echo "nvidia-smi is available."
        $VERBOSE && echo "Driver and CUDA info:"
        nvidia_driver_version=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n 1)
        NVIDIA_DRIVERS_AVAILABLE=true
        $VERBOSE && echo "Detected NVIDIA driver version: $nvidia_driver_version"
    else
        $VERBOSE && echo "NVIDIA drivers are NOT installed."
        NVIDIA_DRIVERS_AVAILABLE=false
    fi
    $VERBOSE && echo
    true
}

check_virtualization() {
    
    $VERBOSE && echo "::: CPU Virtualization Check :::"
    
    if command -v lscpu >/dev/null 2>&1; then
        $VERBOSE && echo "lscpu output (showing virtualization):"
        virtualization_info=$(lscpu | grep -i 'virtualization' | xargs)
        if [[ -n "$virtualization_info" ]]; then
            $VERBOSE && echo "$virtualization_info"
            VIRTUALIZATION_SUPPORTED=true
        else
            $VERBOSE && echo "No virtualization support detected."
            VIRTUALIZATION_SUPPORTED=false
        fi
    else
        $VERBOSE && echo "Virtualization flags (from /proc/cpuinfo):"
        if grep -Eq 'vmx|svm' /proc/cpuinfo; then
            $VERBOSE && echo "Virtualization flags detected."
            VIRTUALIZATION_SUPPORTED=true
        else
            $VERBOSE && echo "No virtualization flags found."
            VIRTUALIZATION_SUPPORTED=false
        fi
    fi
    $VERBOSE && echo
    true
}

check_docker_engine() {

    $VERBOSE && echo "::: Docker Engine Check :::"

    if ! command -v docker >/dev/null 2>&1; then 
        $VERBOSE && echo "Docker engine is NOT installed."
        DOCKER_ENGINE_RUNNING=false
        $VERBOSE && echo
        return
    fi
    $VERBOSE && echo "Docker command is available."

    # Try a simple docker version command to see if the daemon is reachable.
    if ! docker version >/dev/null 2>&1; then
        $VERBOSE && echo "Docker daemon is NOT running."
        DOCKER_ENGINE_RUNNING=false
        $VERBOSE && echo
        return
    fi

    $VERBOSE && echo "Docker daemon is running."
    DOCKER_ENGINE_RUNNING=true
    docker_client_version=$(docker version --format '{{.Client.Version}}' 2>/dev/null || "N/A")
    docker_server_version=$(docker version --format '{{.Server.Version}}' 2>/dev/null || "N/A")
    $VERBOSE && echo "Docker Client Version: $docker_client_version"
    $VERBOSE && echo "Docker Server Version: $docker_server_version"
    $VERBOSE && echo
    true

}

check_nvidia_container_runtime() {
    
    $VERBOSE && echo "::: NVIDIA Container Runtime Check :::"
    
    if docker info 2>/dev/null | grep -qi "nvidia"; then
        $VERBOSE && echo "Docker reports an NVIDIA runtime available."
        NVIDIA_RUNTIME_AVAILABLE=true
    else
        $VERBOSE && echo "NVIDIA runtime is NOT listed in Docker info."
        NVIDIA_RUNTIME_AVAILABLE=false
    fi
    
    if command -v nvidia-container-runtime >/dev/null 2>&1; then
        # Capture the output (ignore the error exit code)
        nct_output=$(nvidia-container-runtime --version 2>&1 || true)
        # Extract the version line only (if present)
        nct_version=$(grep -m1 "^NVIDIA Container Runtime version" <<< "$nct_output" || "N/A")
        $VERBOSE && echo "$nct_version"
    else
        $VERBOSE && echo "nvidia-container-runtime command not found."
    fi
    $VERBOSE && echo
    true

}

$VERBOSE && echo 
$VERBOSE && echo -e "Starting system checks...\n"
check_nvidia_drivers
check_virtualization
check_docker_engine
check_nvidia_container_runtime
$VERBOSE && echo "System checks completed!"
$VERBOSE && echo 

[[ "$SILENT" = false ]] && echo
[[ "$SILENT" = false ]] && echo "::: Summary of Checks :::"
[[ "$SILENT" = false ]] && echo "NVIDIA_DRIVERS_AVAILABLE: $NVIDIA_DRIVERS_AVAILABLE"
[[ "$SILENT" = false ]] && echo "VIRTUALIZATION_SUPPORTED: $VIRTUALIZATION_SUPPORTED"
[[ "$SILENT" = false ]] && echo "DOCKER_ENGINE_RUNNING: $DOCKER_ENGINE_RUNNING"
[[ "$SILENT" = false ]] && echo "NVIDIA_RUNTIME_AVAILABLE: $NVIDIA_RUNTIME_AVAILABLE"
[[ "$SILENT" = false ]] && echo 

if $NVIDIA_DRIVERS_AVAILABLE && $VIRTUALIZATION_SUPPORTED && $DOCKER_ENGINE_RUNNING && $NVIDIA_RUNTIME_AVAILABLE; then 
    ALL_CHECKS_PASSED=true
else
    ALL_CHECKS_PASSED=false
fi

export NVIDIA_DRIVERS_AVAILABLE
export NVIDIA_RUNTIME_AVAILABLE
export DOCKER_ENGINE_RUNNING
export VIRTUALIZATION_SUPPORTED
export ALL_CHECKS_PASSED
