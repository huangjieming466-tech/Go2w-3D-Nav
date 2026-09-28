#!/usr/bin/env bash
# Source the ROS workspace together with the locally built PCT dependencies.

source /opt/ros/humble/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/setup.bash"

export CMAKE_PREFIX_PATH="/home/hjj/PCT-SCAN-ROS2/.deps/install/gtsam-4.2:/home/hjj/PCT-SCAN-ROS2/.deps/install/osqp-1.0.0${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export CUDA_PATH="$HOME/.local/cuda"
export LD_LIBRARY_PATH="/home/hjj/PCT-SCAN-ROS2/.deps/install/gtsam-4.2/lib:/home/hjj/PCT-SCAN-ROS2/.deps/install/osqp-1.0.0/lib:$HOME/.local/lib/python3.10/site-packages/nvidia/cuda_nvrtc/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
