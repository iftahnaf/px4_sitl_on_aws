#!/bin/bash

# Get current timestamp
timestamp=$(date +"%Y%m%d%H%M")

# Define image name
image="ghcr.io/iftahnaf/simulation:$timestamp"

# Build image with timestamp
docker build -t "$image" -f ./dockers/Dockerfile.simulation .

# Push both timestamped and latest tags
docker push "$image"
docker tag "$image" ghcr.io/iftahnaf/simulation:latest
docker push ghcr.io/iftahnaf/simulation:latest
