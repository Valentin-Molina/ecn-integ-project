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
already in process). The waypoints are geometry_msgs/Pose2D, the x value
is used for the first angular joint (theta1) and the y value is used for
the second angular joint (theta2) (the value of theta is not used).
You can see and use the scrip TestMotionPlanning.sh to test the node.
The trajectory is then sent on the topic /Trajectoire.

When running the trapezoidal_planning node, you could set 4 parameters 
which are the two maximum joint velocitiers and the two maximum joint 
accelerations. These parameters are called vMax1, vMax2, aMax1 and 
aMax2.

The frequency used for publishing (and computing) the trajectory on 
/Trajectoire is set to 100Hz by default. It can be changed on the source 
code if necessary.

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
