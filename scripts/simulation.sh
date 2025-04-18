#!/bin/bash
set -e

# Source ROS and workspace setup
source /opt/ros/jazzy/setup.bash
source /workspaces/px4_sitl_on_aws/install/local_setup.bash

# Install rosbags (once)
pip install rosbags --break-system-packages

# Read input args from workflow
RADIUS="$1"
ALTITUDE="$2"
OMEGA="$3"
TIMEOUT_S="$4"

# Echo input args
echo "Radius: $RADIUS"
echo "Altitude: $ALTITUDE"
echo "Omega: $OMEGA"
echo "Timeout: $TIMEOUT_S"

# Launch the ROS 2 simulation
ros2 launch px4_ci_aws ci.launch.py \
  radius:=$RADIUS \
  altitude:=$ALTITUDE \
  omega:=$OMEGA \
  timeout_s:=$TIMEOUT_S
