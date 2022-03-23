#! /usr/bin/env python

import rospy
import math
from robotics_cswk_kin.srv import IKinMsg, IKinMsgRequest
from geometry_msgs.msg import Point

from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition

rospy.init_node("ikintest")
proxy = rospy.ServiceProxy("/inv_kin", IKinMsg)
request = IKinMsgRequest()
request.position = Point()
request.position.x = 0.15
request.position.y = 0.0
request.position.z = 0.1
request.angle.data = math.pi / 3

response = proxy(request)
r = [x.data for x in response.joint_positions]
print(r)

if not response.success.data:
    print('Failed inv kin')
else:
    setPose = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)

    rospy.wait_for_service('/goal_joint_space_path')

    jointRequest=JointPosition()
    jointRequest.joint_name=["joint1","joint2","joint3","joint4"]  
    jointRequest.position=[x.data for x in response.joint_positions]
    setPose(str(), jointRequest, 4.0)
