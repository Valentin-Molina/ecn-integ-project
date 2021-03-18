#!/usr/bin/env python

import rospy
import yaml

from __future__ import print_function
from math import cos, sin
from sensor_msgs.msg import JointState
from Kinematic.srv import Kinematic,KinematicResponse


# load robot parameters from yaml file
with open('~/ros/src/integ_gcd_models_pkg/RobotParam.yml') as f :
	yaml_dict = yaml.safe_load(f)
	l1 = yaml_dict.get("l1")
	l2 = yaml_dict.get("l2")


def handle_Kinematic(req):
	
	# recover joint velocities from request
	theta_dot=req.input.velocity
	
	# compute end effector velocity
    velocity[0]  = -req.l1*theta_dot[0]*sin(theta[0])-req.l2*(theta_dot[0]+theta_dot[1])*sin(theta[0]+theta[1])
    velocity[1]  = req.l1*theta_dot[0]*cos(theta[0])+req.l2*(theta_dot[0]+theta_dot[1])*cos(theta[0]+theta[1])

    return KinematicResponse(velocity)

def Kinematic_server():
    rospy.init_node('Kinematic_server')
    s = rospy.Service('Kinematic', Kinematic, handle_Kinematic)
    print("Kinematic Direct Model")
    rospy.spin()

if __name__ == "__main__":
    Kinematic_server()
