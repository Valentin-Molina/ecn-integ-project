#!/usr/bin/env python

import rospy
import yaml

from __future__ import print_function
from math import cos, sin, atan2
from sensor_msgs.msg import JointState
from Kinematic.srv import Kinematic,KinematicResponse


# load robot parameters from yaml file
with open('~/ros/src/integ_gcd_models_pkg/RobotParam.yml') as f :
	yaml_dict = yaml.safe_load(f)
	l1 = yaml_dict.get("l1")
	l2 = yaml_dict.get("l2")


def handle_MGI(req):
	
	x=req.pos[0]
	y=req.pos[1]
	
	c2=(x**2+y**2-l1**2-l2**2)/(2*l1*l2)
	if req.high_elbow :
		s2=-sqrt(1-c2**2)
	else :
		s2=sqrt(1-c2**2)
		
	s1=((l1+l2*c2)*y-l2*s2*x)/(x**2+y**2)
	c1=((l1+l2*c2)*x+l2*s2*y)/(x**2+y**2)
	
	output.position[1]=atan2(s2,c2)
	output.position[0]=atan2(s1,c1)	
	
    return MGIResponse(output)

def MGI_server():
    rospy.init_node('MGI_server')
    s = rospy.Service('MGI', MGI, handle_MGI)
    print("MGI")
    rospy.spin()

if __name__ == "__main__":
    MGI_server()
