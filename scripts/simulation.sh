#!/bin/bash
set -e

# Source ROS and workspace setup
source /opt/ros/jazzy/setup.bash
source /workspaces/px4_sitl_on_aws/install/local_setup.bash

# Install rosbags (once)
pip install rosbags --break-system-packages

echo "Radius: $RADIUS"
echo "Altitude: $ALTITUDE"
echo "Omega: $OMEGA"
echo "Timeout: $TIMEOUT_S"

ros2 launch px4_ci_aws ci.launch.py