cmake_minimum_required(VERSION 2.8.3)
project(target)

find_package(   
    catkin REQUIRED COMPONENTS
    roscpp
    gazebo_msgs
    std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(move_cible src/move_cible.cpp)
target_link_libraries(move_cible ${catkin_LIBRARIES})

