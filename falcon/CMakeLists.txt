###################################
##   FALCON FLIGHT CONTROLLER    ##
##   CET TVM                     ##
###################################

cmake_minimum_required(VERSION 2.8.3)
project(falcon)

## Compile as C++11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES falcon
 CATKIN_DEPENDS roscpp
 DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/falcon.cpp
  src/${PROJECT_NAME}/state/attitude.cpp
  src/${PROJECT_NAME}/state/velocity.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(${PROJECT_NAME}_node src/falcon_node.cpp)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


#############
## Install ##
#############


#############
## Testing ##
#############

