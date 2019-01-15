#!/usr/bin/env sh

sudo sh -c "echo 'yaml file://$(pwd)/osx.yaml osx' > /etc/ros/rosdep/sources.list.d/10-ros-drone.list"

rosdep update