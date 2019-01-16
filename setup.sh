#!/usr/bin/env bash

if [ ! $ROS_PACKAGE_PATH ] || [ ! -d $ROS_PACKAGE_PATH ]; then
  echo "ROS must be installed"
  exit 0
fi

if [ ! -f ./CMakeLists.txt ]; then
  ln -s $ROS_PACKAGE_PATH/catkin/cmake/toplevel.cmake CMakeLists.txt
fi

pushd rosdeps
sh setup.sh
popd