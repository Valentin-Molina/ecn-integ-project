#!/usr/bin/env python
from __future__ import print_function

import rospy
import yaml
import numpy as np #np.dot
import os.path


from math import cos, sin
from sensor_msgs.msg import JointState
from gkd_models.srv import Dynamic,DynamicResponse

# load robot parameters :
	# mi = mass 
	# li = arm lenght 
	# Izi = z axis inertia
	# ci = arm joint to center of mass distance
	
path=os.path.dirname(__file__)

with open(os.path.join(path,'RobotParam.yml')) as f :
	yaml_dict = yaml.safe_load(f)
	l1 = yaml_dict.get("l1")
	l2 = yaml_dict.get("l2")
	m0 = yaml_dict.get("m0")
	m1 = yaml_dict.get("m1")
	m2 = yaml_dict.get("m2")
	Iz1 = yaml_dict.get("Iz1")
	Iz2 = yaml_dict.get("Iz2")
	c1 = yaml_dict.get("c1")
	c2 = yaml_dict.get("c2")
	g = yaml_dict.get("g")


def handle_Dynamic(req):
	
	# recover joint positions, velocities and acceleration from request
	theta = req.input.position
	theta_d = req.input.velocity
	theta_d_d = req.input.effort
	
	# compute intermediate variables
	Z1=Iz2+m2*c2**2
	Z2=m2*l2*c2
	Z3=m2*l1*c2
	Z4=Iz1+m1*c1**2
	Z5=m2*l1**2
	Z6=m2*l1*(2*c2-l2)
	Z7=m2*l1*l2
	Z8=m2*l1*(l2-c2)
	Z9=(m2*c2+m0*l2)*g
	Z10=(m0*l1+m1*c1+m2*l1)*g
	
	# compute final matrixes
	# T = D.qdotdot + C.qdot + G
	D=[[Z1-Z2+Z3*cos(theta[1]) , Z1],[Z1+Z4-Z2+Z5+Z6*cos(theta[1]) , Z1+Z3*cos(theta[1])]]
	
	C=[[-Z3*theta_d[0]*sin(theta[1]) , 0],[(-Z7*theta_d[0]-Z8*theta_d[1])*sin(theta[1]) , (Z3*theta_d[1]-Z8*theta_d[0])*sin(theta[1])]]
	
	G=[-Z9*cos(theta[0]+theta[1]),-Z9*cos(theta[0]+theta[1])+Z10*cos(theta[0])]
	
	output=JointState()
	print(np.dot(D,req.input.effort)+np.dot(C,theta_d)+G)
	output.effort=(np.dot(D,req.input.effort)+np.dot(C,theta_d)+G).tolist()
	
	return DynamicResponse(output)

def Dynamic_server():
    rospy.init_node('Dynamic_server')
    s = rospy.Service('Dynamic', Dynamic, handle_Dynamic)
    print("Dynamic Model Direct")
    rospy.spin()

if __name__ == "__main__":
    Dynamic_server()
