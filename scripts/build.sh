#!/bin/bash
set -e
# Set the default build type
BUILD_TYPE=RelWithDebInfo
export MAKEFLAGS="-j16"
colcon build \
        --merge-install \
        --symlink-install \
        --packages-ignore px4 \
        --cmake-args "-DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" \
        -Wall -Wextra -Wpedantic