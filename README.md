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

## Update rosdep registry with custom repositories
``` sh
cd src
./setup.sh
```

## Download dependencies
``` sh
cd ..
rosdep install --from-paths src --ignore-src -r -y --as-root 'pip:false'
```

If you are getting permission denied error when running above command, run

``` sh
rosdep install --from-paths src --ignore-src -r -y
```

## Build and source workspace
```sh
catkin_make
```