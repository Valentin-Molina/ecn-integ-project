cmake_minimum_required(VERSION 3.0.0)
project(trapezoidal_planning)

find_package(   
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    )

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
    )

include_directories(include ${catkin_INCLUDE_DIRS})

# Set the name to use for the executable.
set(BINNAME1 trapezoidal_planning)

# Set the source files to use with the executable.
set(SRCS1 ${SRCS1} src/trapezoidalNode.cpp)
set(SRCS1 ${SRCS1} src/trapezoidal.cpp)

# Build the executable that will be used to run this node.
add_executable(${BINNAME1} ${SRCS1})
add_dependencies(${BINNAME1} ${BINNAME1}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})