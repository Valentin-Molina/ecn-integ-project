#!/bin/bash
#This script will publish 3 Waypoints on the topic /trapeziodal_planning/Waypoints with the last one being x=0,y=0 to trigger the compute fonction

rostopic pub /trapezoidal_planning/Waypoints geometry_msgs/Pose2D "x: 6.0
y: 7.0
theta: 0.0"


rostopic pub /trapezoidal_planning/Waypoints geometry_msgs/Pose2D "x: 5.0
y: -1.0
theta: 0.0"


rostopic pub /trapezoidal_planning/Waypoints geometry_msgs/Pose2D "x: 0.0
y: 0.0
theta: 0.0"
