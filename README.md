Quadcopter design, simulation and control
====

# Setup
## Prerequisites
1. ROS
2. Python
3. Pip


## Create a catkin workspace
``` sh
mkdir drone_workspace
cd drone_workspace
git clone https://github.com/cet-quadcopter/quadcopter-core.git src
```

## Update rosdep registry with custom repositories and download dependencies
``` sh
cd src
./setup.sh
```

## Build and source workspace
```sh
catkin_make
```