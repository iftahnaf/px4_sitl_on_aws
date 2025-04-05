#!/bin/bash
set -e

registry=$1
timestamp=$(date +"%Y%m%d%H%M")
image="$registry/simulation:$timestamp"

docker build -t "$image" -f ./dockers/Dockerfile.simulation .
docker push "$image"

docker tag "$image" "$registry/simulation:latest"
docker push "$registry/simulation:latest"
