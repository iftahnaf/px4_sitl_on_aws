#!/bin/bash

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, Iftach Naftaly <iftahnaf@gmail.com>

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