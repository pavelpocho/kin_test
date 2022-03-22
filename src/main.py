#! /usr/bin/env python3

import rospy
from cswk_kin.srv import IKinMsg, IKinMsgRequest
from geometry_msgs.msg import Point

rospy.init_node("ikintest")
proxy = rospy.ServiceProxy("/inv_kin", IKinMsg)
request = IKinMsgRequest()
request.position = Point()
request.position.x = 0.15
request.position.y = 0.0
request.position.z = 0.15

response = proxy(request)
r = [x.data for x in response.joint_positions]
print(r)
