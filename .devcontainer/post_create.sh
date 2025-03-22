#!/bin/bash
WORKSPACE_DIR="/workspaces/px4_workspace/"
PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"; export PYTHONWARNINGS

cd $WORKSPACE_DIR

## Updateing and installing dependencies
sudo chown -R ros:ros $WORKSPACE_DIR/PX4-Autopilot
cd PX4-Autopilot
echo "Installing dependencies"
git submodule update --init --recursive

## Installing all PX4 deps and build SITL
echo "Installing PX4 dependencies"
./Tools/setup/ubuntu.sh --no-nuttx

## build
echo "Building PX4"
git remote add upstream https://github.com/PX4/PX4-Autopilot.git
git fetch upstream
git fetch upstream --tags
DONT_RUN=1 make px4_sitl
echo "source ${WORKSPACE_DIR}PX4-Autopilot/install/local_setup.bash" >> ~/.bashrc

# mavros dependencies
sudo geographiclib-get-geoids egm96-5
sudo geographiclib-get-gravity egm96
sudo geographiclib-get-magnetic emm2010
sudo geographiclib-get-magnetic emm2015

## compile ROS workspace
echo "Building ROS workspace"
cd $WORKSPACE_DIR/src
git submodule update --init --recursive
colcon build --packages-select px4_msgs
sleep 1
colcon build --packages-select px4_msgs
echo "source ${WORKSPACE_DIR}install/local_setup.bash" >> ~/.bashrc

sudo mkdir /run/user/1000
sudo chown ros:ros /run/user/1000
sudo chmod 0700 /run/user/1000

pip install pyvectorguidance

echo "Done installing, ready to develop!"
