#!/usr/bin/env bash

. ../bash/logging.sh

sudo sh -c "echo 'yaml file://$(pwd)/osx.yaml osx' > /etc/ros/rosdep/sources.list.d/10-ros-drone.list"
sudo sh -c "echo 'yaml file://$(pwd)/python.yaml' >> /etc/ros/rosdep/sources.list.d/10-ros-drone.list"

info "Updating rosdep source list"
rosdep update

pushd ../../
  info "Downloading dependencies"
  rosdep install --from-paths src --ignore-src -r -y --as-root 'pip:false'
popd