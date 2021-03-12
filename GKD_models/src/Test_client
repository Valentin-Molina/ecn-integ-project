#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from Kinematic.srv import Kinematic,KinematicResponse

def test_callback():
	S = JointState()
	S.name[0] = 'fake_joint'
	S.position[0] = 2
	S.velocity[0] = 1

	rospy.wait_for_service('Kinematic')
	dynamic = rospy.ServiceProxy('Kinematic', KinematicResponse)

	try:
		resp = Kinematic(S)
		print(resp)
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))


if __name__ == '__main__':
    try:
        test_callback()
    except rospy.ROSInterruptException:
        pass
