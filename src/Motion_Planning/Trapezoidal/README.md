# Integ_description

## Table of Contents
1. [General Info](#general-info)
2. [Installation](#installation)

### General Info
***
In this package you will find:
* A node which generates a trajectory from a serie of waypoints.
* A bash script which allows you to test the trajectory generation.
* A launch file which launches the pkg integ_gazebo and the node.

The pkg integ_gazebo (or integ_description) needs to be launch before 
running the node because the node generate a trajectory in the joint 
space starting at the current joint state which is recevered on the 
topic /joint_states.
When the node is running you could use the service /Waypoint_serv to
generate a trajectory. (You cannot use the service when a trajectory is
already in process).
The trajectory is then sent on the topic /Trajectoire.

## Installation
***
Download the projet to your user home using the following commands.
The repository will be a ros workspace.
```
$ cd ~
$ git clone https://github.com/Valentin-Molina/ecn-integ-project
$ cd ecn-integ-project
$ catkin build trapezoidal_planning
$ source devel/setup.bash
$ roslaunch trapezoidal_planning motion.launch
'''
