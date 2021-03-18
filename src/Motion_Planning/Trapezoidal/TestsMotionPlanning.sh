#!/bin/bash
# This script will add 3 Waypoints via the /Waypoint_serv service with 
# the last one being x=0,y=0 to trigger the compute fonction.

rosservice call /Waypoint_serv "
waypoints:
  - {x: 1.0, y: 2.0, theta: 0.0}
  - {x: 5.0, y: 7.0, theta: 0.0}
  - {x: 0.0, y: 0.0, theta: 0.0}
"
