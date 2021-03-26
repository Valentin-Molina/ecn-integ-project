cmake_minimum_required(VERSION 2.8.3)
project(suivi_traj)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  gkd_models
)

## System dependencies are found with CMake's conventions
#find_package(OpenCV REQUIRED)
#find_package(VISP REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  LIBRARIES suivi_traj
  CATKIN_DEPENDS roscpp sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(suivi_traj include/control_node.h include/control_node_pid.h src/control_node.cpp src/control_node_pid.cpp)
target_link_libraries(suivi_traj ${catkin_LIBRARIES})

add_executable(control_ctc src/main_ctc.cpp)
add_executable(control_hand src/main_Hand.cpp)
add_executable(control_finger src/main_Finger.cpp)

target_link_libraries(control_ctc suivi_traj)
target_link_libraries(control_hand suivi_traj)

target_link_libraries(control_finger suivi_traj)



