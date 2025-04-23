#!/bin/bash

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, Iftach Naftaly <iftahnaf@gmail.com>

# Get current timestamp
timestamp=$(date +"%Y%m%d%H%M")

# Define image name
image="ghcr.io/iftahnaf/dev:$timestamp"

# Build image with timestamp
docker build -t "$image" -f ./dockers/Dockerfile.dev .

# Push both timestamped and latest tags
docker push "$image"
docker tag "$image" ghcr.io/iftahnaf/dev:latest
docker push ghcr.io/iftahnaf/dev:latest
