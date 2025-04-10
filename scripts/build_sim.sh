#!/bin/bash
set -e

registry="718459739973.dkr.ecr.eu-west-1.amazonaws.com"
repo="px4_sitl_on_aws"
timestamp=$(date +"%Y%m%d%H%M")

image="$registry/$repo:$timestamp"

echo "Building image: $image"

docker build -t "$image" -f ./dockers/Dockerfile.simulation .

echo "Pushing image: $image"
docker push "$image"

echo "Tagging and pushing 'latest'"
docker tag "$image" "$registry/$repo:latest"
docker push "$registry/$repo:latest"
