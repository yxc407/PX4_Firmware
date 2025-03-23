#! /bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
TC_SRC_DIR=${DIR}/..

source /opt/ros/${ROS_DISTRO:-kinetic}/setup.bash
mkdir -p ${TC_SRC_DIR}/catkin_ws/src
cd ${TC_SRC_DIR}/catkin_ws/
git clone -b 0.3.1 --single-branch --depth 1 https://github.com/TC/avoidance.git src/avoidance

catkin init
catkin build local_planner safe_landing_planner --cmake-args -DCMAKE_BUILD_TYPE=Release

source ${TC_SRC_DIR}/catkin_ws/devel/setup.bash
source /usr/share/gazebo/setup.sh

export CATKIN_SETUP_UTIL_ARGS=--extend
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${TC_SRC_DIR}/catkin_ws/src/avoidance/avoidance/sim/models

source $DIR/rostest_tc_run.sh "$@"
