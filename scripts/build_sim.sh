#!/bin/bash

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, Iftach Naftaly <iftahnaf@gmail.com>
set -e

registry="718459739973.dkr.ecr.eu-west-1.amazonaws.com"
repo="px4_sitl_on_aws"
timestamp=$(date +"%Y%m%d%H%M")

image="$registry/$repo:$timestamp"

echo "Building image: $image"

docker buildx create --use --name builder || docker buildx use builder

docker buildx build \
  --cache-from=type=local,src=/tmp/.buildx-cache \
  --cache-to=type=local,dest=/tmp/.buildx-cache,mode=max \
  --push \
  -t "$image" \
  -t "$registry/$repo:latest" \
  -f ./dockers/Dockerfile.simulation .

echo "Image pushed successfully: $image"
