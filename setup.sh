#!/usr/bin/env bash

. ./bash/logging.sh

if [ ! $ROS_PACKAGE_PATH ] ; then
  error "ROS must be installed"
  exit 0
fi

if [ ! -f ./CMakeLists.txt ]; then
  ln -s $ROS_PACKAGE_PATH/catkin/cmake/toplevel.cmake CMakeLists.txt
fi

pushd rosdeps
  ./setup.sh
popd
