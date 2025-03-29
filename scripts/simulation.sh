#/bin/bash

# Source the required setup files
source /opt/ros/jazzy/setup.bash
source /workspaces/px4_sitl_on_aws/install/local_setup.bash

pip install rosbags --break-system-packages

# Launch the ROS2 simulation with the provided offsets
ros2 launch px4_ci_aws ci.launch.py