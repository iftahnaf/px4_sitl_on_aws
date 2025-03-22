#!/bin/bash
WORKSPACE_DIR="/workspaces/px4_sitl_on_aws/"
PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"; export PYTHONWARNINGS

cd $WORKSPACE_DIR
git submodule update --init --recursive

## Updateing and installing dependencies
sudo chown -R ros:ros $WORKSPACE_DIR/PX4-Autopilot
cd PX4-Autopilot
echo "Installing dependencies"

## Installing all PX4 deps and build SITL
echo "Installing PX4 dependencies"
./Tools/setup/ubuntu.sh --no-nuttx

## build
echo "Building PX4"
git remote add upstream https://github.com/PX4/PX4-Autopilot.git
git fetch upstream
git fetch upstream --tags
make px4_sitl

## compile ROS workspace
echo "Building ROS workspace"
cd $WORKSPACE_DIR/src
git submodule update --init --recursive
colcon build
echo "source ${WORKSPACE_DIR}install/local_setup.bash" >> ~/.bashrc

sudo mkdir /run/user/1000
sudo chown ros:ros /run/user/1000
sudo chmod 0700 /run/user/1000

echo "Done installing, ready to develop!"
