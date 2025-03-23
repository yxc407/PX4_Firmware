#! /bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TC_SRC_DIR=${DIR}/..

source /opt/ros/${ROS_DISTRO:-kinetic}/setup.bash
source ${TC_SRC_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${TC_SRC_DIR} ${TC_SRC_DIR}/build/tc_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${TC_SRC_DIR}:${TC_SRC_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic

export ROS_LOG_DIR="$HOME/.ros/ros_logs"
mkdir -p "$ROS_LOG_DIR"

rostest tc "$@"
