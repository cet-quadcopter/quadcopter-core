#!/usr/bin/env bash

sudo sh -c "echo 'yaml file://$(pwd)/osx.yaml osx' > /etc/ros/rosdep/sources.list.d/10-ros-drone.list"

echo "Updating rosdep source list"
rosdep update

pushd ../../
  echo "Downloading dependencies"
  rosdep install --from-paths src --ignore-src -r -y --as-root 'pip:false'
popd