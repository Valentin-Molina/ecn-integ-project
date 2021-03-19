cmake_minimum_required(VERSION 3.0.0)
project(suivi_traj_ek)

find_package(   
    catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    message_generation
    gkd_models
    )

# Si jamais on a besoin d'un message personnalise    
#add_message_files(
#FILES
#Commande.msg
#)

#generate_messages(
#    DEPENDENCIES
#    std_msgs
#    )

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs message_runtime
    )

include_directories(include ${catkin_INCLUDE_DIRS})

# Set the name to use for the executable.
set(BINNAME suivi_traj_ek)
set(BINNAME1 Commande_PID)
set(BINNAME2 Commande_CTC)

# Set the source files to use with the executable.
set(SRCS1 ${SRCS1} src/PID_controller.cpp)
set(SRCS2 ${SRCS2} src/CTC_controller.cpp)

# Build the executable that will be used to run this node.
add_executable(${BINNAME1} ${SRCS1})
add_executable(${BINNAME2} ${SRCS2})

add_dependencies(${BINNAME1}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})
#${BINNAME}_generate_messages_cpp

add_dependencies(${BINNAME2} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${BINNAME2} ${catkin_LIBRARIES})
#${BINNAME}_generate_messages_cpp
